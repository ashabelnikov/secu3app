/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   contacts:
              http://secu-3.org
              email: shabelnikov@secu-3.org
*/

/** \file ckps.c
 * Implementation of crankshaft position sensor's processing.
 * (Реализация обработки датчика положения коленвала).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "ckps.h"
#include "magnitude.h"

#include "secu3.h"
#include "knock.h"

#ifndef WHEEL_36_1 //60-2
/** Number of teeth (including absent) (количество зубьев (включая отсутствующие)) */
 #define WHEEL_COGS_NUM   60
/** Number of absent teeth (количество отсутствующих зубьев) */
 #define WHEEL_COGS_LACK  2
/**Number of teeth before t.d.c which determines moment of advance angle latching, start of measurements from sensors,
 * latching of settings into HIP9011 (кол-во зубьев до в.м.т определяющие момент загрузки УОЗ, старт измерения датчиков,
 * загрузку настроек в HIP)  */
 #define WHEEL_LATCH_BTDC 11
/** p * 2.5,  barrier for detecting of synchro-label (барьер для селекции синхрометки) = 2.5 */
 #define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p) >> 1))
#else //36-1
 #define WHEEL_COGS_NUM   36
 #define WHEEL_COGS_LACK  1
 #define WHEEL_LATCH_BTDC 7   //70 degree (70 градусов)
 #define CKPS_GAP_BARRIER(p) ((p) + ((p) >> 1))  // p * 1.5
#endif

/** Number of degrees which corresponds to one rotation (количество градусов приходящееся на один зуб диска) */
#define CKPS_DEGREES_PER_COG (360 / WHEEL_COGS_NUM)

/** Number of last(present) tooth, numeration begins from 1! (номер последнего(существующего) зуба, нумерация ничинается с 1!) */
#define WHEEL_LAST_COG (WHEEL_COGS_NUM - WHEEL_COGS_LACK)

/** number of teeth that will be skipped at the start before synchronization
 * (количество зубов которое будет пропускаться при старте перед синхронизацией) */
#define CKPS_ON_START_SKIP_COGS      5

/** Access Input Capture Register */
#define GetICR() (ICR1)

/** 4 channels of ignition maximum (максимум 4 канала зажигания) */
#define IGN_CHANNELS_MAX     4

/** Used to indicate that none from ignition channels are selected
 * (используется для указания того что ни один канал зажигания не выбран) */
#define CKPS_CHANNEL_MODENA  255

/** Flags */
typedef struct
{
 uint8_t  ckps_error_flag:1;                 //!< CKP error flag, set in the CKP's interrupt, reset after processing (признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки)
 uint8_t  ckps_is_valid_half_turn_period:1;  //!< used to indicate that measured period is valid (actually measured)
 uint8_t  ckps_is_synchronized:1;            //!< indicates that synchronization has been completed (syncho-label found)
 uint8_t  ckps_new_engine_cycle_happen:1;    //!< flag for synchronization with rotation (флаг синхронизации с вращением)
 uint8_t  ckps_use_knock_channel:1;          //!< flag which indicates using of knock channel (признак использования канала детонации)
 uint8_t  ckps_need_to_set_channel:1;        //!< indicates that it is necessary to set channel
#ifdef COIL_REGULATION
 uint8_t  ckps_need_to_set_channel_b:1;      //!< Indicates that it is necessary to set channel (COMPB)
#endif
 uint8_t  ckps_ign_enabled:1;                //!< Ignition enabled/disabled
}ckpsflags_t;

/** State variables */
typedef struct
{
 uint16_t icr_prev;                   //!< previous value if Input Capture Register (предыдущее значение регистра захвата)
 volatile uint16_t period_curr;       //!< last measured inter-tooth period (последнй измеренный межзубный период)
 uint16_t period_prev;                //!< previous value of inter-tooth period (предыдущее значение межзубного периода)
 volatile uint8_t  cog;               //!< counts teeth from synchro-label, begins from 1 (считает зубья после выреза, начинает считать с 1)
 uint16_t measure_start_value;        //!< remembers the value of the capture register to measure the half-turn (запоминает значение регистра захвата для измерения периода полуоборота)
 uint16_t current_angle;              //!< counts out given advance angle during the passage of each tooth (отсчитывает заданный УОЗ при прохождении каждого зуба)
 volatile uint16_t half_turn_period;  //!< stores the last measurement of the passage of teeth n (хранит последнее измерение времени прохождения n зубьев)
 int16_t  advance_angle;              //!< required adv.angle * ANGLE_MULTIPLAYER (требуемый УОЗ * ANGLE_MULTIPLAYER)
 volatile int16_t advance_angle_buffered;//!< buffered value of advance angle (to ensure correct latching)
 uint8_t  ignition_cogs;              //!< number of teeth determine the duration of ignition drive pulse (кол-во зубьев определяющее длительность импульсов запуска коммутаторов)
 uint8_t  starting_mode;              //!< state of state machine processing of teeth at the startup (состояние конечного автомата обработки зубьев на пуске)
 uint8_t  channel_mode;               //!< determines which channel of the ignition to run at the moment (определяет какой канал зажигания нужно запускать в данный момент)
 volatile uint8_t  cogs_btdc;         //!< number of teeth from synchro-label to t.d.c of the first cylinder (кол-во зубьев от синхрометки до в.м.т первого цилиндра)
 int8_t   knock_wnd_begin_abs;        //!< begin of the phase selection window of detonation in the teeth of wheel, relatively to t.d.c (начало окна фазовой селекции детонации в зубьях шкива относительно в.м.т)
 int8_t   knock_wnd_end_abs;          //!< end of the phase selection window of detonation in the teeth of wheel, relatively to t.d.c (конец окна фазовой селекции детонации в зубьях шкива относительно в.м.т)
 volatile uint8_t chan_number;        //!< number of ignition channels (кол-во каналов зажигания)
 uint32_t frq_calc_dividend;          //!< divident for calculating RPM (делимое для расчета частоты вращения)
#ifdef COIL_REGULATION
 volatile uint16_t cr_acc_time;       //!< accumulation time for coil requlation (timer's ticks)
 uint8_t  channel_mode_b;             //!< determines which channel of the ignition to start accumulate at the moment (определяет какой канал зажигания будет накапливать энергию в данный момент)
 uint32_t acc_delay;                  //!< delay between last ignition and next accumulation
 uint16_t tmrval_saved;               //!< value of timer at the moment of each spark
 uint16_t period_saved;               //!< inter-tooth period at the moment of each spark
 uint8_t  add_period_taken;           //!< indicates that time between spark and next cog was taken into account
#endif
 volatile uint8_t chan_mask;          //!< mask used to disable multi-channel mode and use single channel
}ckpsstate_t;
 
/**Precalculated data (reference points) and state data for a single channel plug (a couple of cylinders)
 * Предрасчитанные данные(опорные точки) и данные состояния для отдельного канала зажигания (пара цилиндров)
 * 2cyl: cylstate_t[0]
 * 4cyl: cylstate_t[0], cylstate_t[1]
 * 6cyl: cylstate_t[0], cylstate_t[1], cylstate_t[2]
 * 8cyl: cylstate_t[0], cylstate_t[1], cylstate_t[2], cylstate_t[3]
 */
typedef struct
{
#ifndef COIL_REGULATION
 /** Counts out teeth for ignition pulse (отсчитывает зубья импульса зажигания) */
 volatile uint8_t ignition_pulse_cogs;
#endif

 /** Determines number of tooth (relatively to t.d.c) at which "latching" of data is performed (определяет номер зуба (относительно в.м.т.) на котором происходит "защелкивание" данных) */
 volatile uint8_t cogs_latch;
 /** Determines number of tooth at which measurement of rotation period is performed (определяет номер зуба на котором производится измерение периода вращения коленвала (между раб. циклами)) */
 volatile uint8_t cogs_btdc;
 /** Determines number of tooth at which phase selection window for knock detection is opened (определяет номер зуба на котором открывается окно фазовой селекции сигнала ДД (начало интегрирования)) */
 volatile uint8_t knock_wnd_begin;
 /** Determines number of tooth at which phase selection window for knock detection is closed (определяет номер зуба на котором закрывается окно фазовой селекции сигнала ДД (конец интегрирования)) */
 volatile uint8_t knock_wnd_end;
}chanstate_t;

ckpsstate_t ckps;                         //!< instance of state variables
chanstate_t chanstate[IGN_CHANNELS_MAX];  //!< instance of array of channel's state variables

/** Arrange in the free I/O registers (размещаем в свободных регистрах ввода/вывода) */
#define flags ( (volatile ckpsflags_t*)(&TWAR) ) //note: may be not effective on other MCUs

/** Supplement timer/counter 0 up to 16 bits, use R15 (для дополнения таймера/счетчика 0 до 16 разрядов, используем R15) */
#ifdef __ICCAVR__
 __no_init __regvar uint8_t TCNT0_H@15;
#else //GCC
 uint8_t TCNT0_H __attribute__((section (".noinit")));
#endif

void ckps_init_state_variables(void)
{
#ifndef COIL_REGULATION
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 //at the first interrupt will generate an end trigger pulse ignition
 //(при первом же прерывании будет сгенерирован конец импульса запуска зажигания) 
 for(i = 0; i < IGN_CHANNELS_MAX; i++)
  chanstate[i].ignition_pulse_cogs = 0;
#else
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = 0;
 ckps.channel_mode_b = CKPS_CHANNEL_MODENA;
 flags->ckps_need_to_set_channel_b = 0;
#endif

 ckps.cog = 0;
 ckps.half_turn_period = 0xFFFF;
 ckps.advance_angle = 0;
 ckps.advance_angle_buffered = 0;
 ckps.starting_mode = 0;
 ckps.channel_mode = CKPS_CHANNEL_MODENA;

 flags->ckps_need_to_set_channel = 0;
 flags->ckps_new_engine_cycle_happen = 0;
 flags->ckps_is_synchronized = 0;
 flags->ckps_ign_enabled = 1;
 TCCR0 = 0; //timer is stopped (останавливаем таймер0)
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps_init_state_variables();
 flags->ckps_error_flag = 0;

 //Compare channels do not connected to lines of ports (normal port mode)
 //(Каналы Compare неподключены к линиям портов (нормальный режим портов))
 TCCR1A = 0; 

 //(Noise reduction(подавление шума), rising edge of capture(передний фронт захвата), clock = 250kHz)
 TCCR1B = _BV(ICNC1)|_BV(ICES1)|_BV(CS11)|_BV(CS10);

 //enable input capture and Compare A interrupts of timer 1, also overflow interrupt of timer 0
 //(разрешаем прерывание по захвату и сравнению А таймера 1, а также по переполнению таймера 0)
 TIMSK|= _BV(TICIE1)/*|_BV(OCIE1A)*/|_BV(TOIE0);
 _END_ATOMIC_BLOCK();
}

void ckps_set_advance_angle(int16_t angle)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.advance_angle_buffered = angle;
 _END_ATOMIC_BLOCK();
}

void ckps_init_ports(void)
{
 //after ignition is on, igniters must not be in the accumulation mode,
 //therefore set low level on their inputs
 //(после включения зажигания коммутаторы недолжны быть в режиме накопления,
 //поэтому устанавливаем на их входах низкий уровень)
#ifndef INVERSE_IGN_OUTPUTS
 PORTD|= _BV(PD5)|_BV(PD4)|_BV(PD6); // 1st and 2nd ignition channels, pullup for ICP1 (1-й и 2-й каналы зажигания, подтяжка для ICP1)
 PORTC|= _BV(PC1)|_BV(PC0); //3rd and 4th ignition channels (3-й и 4-й каналы зажигания)
#else //outputs inversion mode (режим инверсии выходов)
 PORTD&= ~(_BV(PD5)|_BV(PD4));
 PORTC&= ~(_BV(PC1)|_BV(PC0));
 PORTD|= _BV(PD6);
#endif

 //PD5,PD4,PC1,PC0 must be configurated as outputs (должны быть сконфигурированы как выходы)
 DDRD|= _BV(DDD5)|_BV(DDD4); //1-2 ignition channels - for 2 and 4 cylinder engines (1-2 каналы зажигания - для 2 и 4 ц. двигателей)
 DDRC|= _BV(DDC1)|_BV(DDC0); //3-4 ignition channels - for 6 and 8 cylinder engines (3-4 каналы зажигания - для 6 и 8 ц. двигателей)
}

//Calculation of instantaneous frequency of crankshaft rotation from the measured period between the cycles of the engine 
//(for example for 4-cylinder, 4-stroke it is 180 degrees) 
//Period measured in the discretes of timer (one discrete = 4us), one minute = 60 seconds, one second has 1,000,000 us.
//Высчитывание мгновенной частоты вращения коленвала по измеренному периоду между тактами двигателя 
//(например для 4-цилиндрового, 4-х тактного это 180 градусов)
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс.
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period;
 _DISABLE_INTERRUPT();
 period = ckps.half_turn_period;           //ensure atomic acces to variable (обеспечиваем атомарный доступ к переменной)
 _ENABLE_INTERRUPT();

 //if equal to minimum, this means engine is stopped (если самый минимум, значит двигатель остановился)
 if (period!=0xFFFF)
  return (ckps.frq_calc_dividend)/(period);
 else
  return 0;
}

void ckps_set_edge_type(uint8_t edge_type)
{
 _BEGIN_ATOMIC_BLOCK();
 if (edge_type)
  TCCR1B|= _BV(ICES1);
 else
  TCCR1B&=~_BV(ICES1);
 _END_ATOMIC_BLOCK();
}

/**
 * Ensures that tooth number will be in the allowed range.
 * Tooth number should not be greater than WHEEL_COGS_NUM or less than zero
 */
uint8_t _normalize_tn(int8_t i_tn)
{
 if (i_tn > WHEEL_COGS_NUM)
  return i_tn - WHEEL_COGS_NUM;
 if (i_tn < 0)
  return i_tn + WHEEL_COGS_NUM;
 return i_tn;
}

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 uint8_t _t, i;
 // pre-compute and store the reference points (teeth) (заранее вычисляем и сохраняем опорные точки (зубья))
 // cogs_per_cycle - number of wheel teeth attributable to a single cycle of engine (количество зубьев шкива приходящиеся на один такт двигателя)
 uint8_t cogs_per_cycle = (WHEEL_COGS_NUM) / ckps.chan_number;
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (cogs_btdc + i * cogs_per_cycle);
  chanstate[i].cogs_btdc = _normalize_tn(tdc);
  chanstate[i].cogs_latch = _normalize_tn(tdc - WHEEL_LATCH_BTDC);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
 }
 ckps.cogs_btdc = cogs_btdc;
 _RESTORE_INTERRUPT(_t);
}

#ifndef COIL_REGULATION
void ckps_set_ignition_cogs(uint8_t cogs)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.ignition_cogs = cogs;
 _END_ATOMIC_BLOCK();
}
#else
void ckps_set_acc_time(uint16_t i_acc_time)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = i_acc_time;
 _END_ATOMIC_BLOCK();
}
#endif

uint8_t ckps_is_error(void)
{
 return flags->ckps_error_flag;
}

void ckps_reset_error(void)
{
 flags->ckps_error_flag = 0;
}

void ckps_use_knock_channel(uint8_t use_knock_channel)
{
 flags->ckps_use_knock_channel = use_knock_channel;
}

uint8_t ckps_is_cycle_cutover_r()
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = flags->ckps_new_engine_cycle_happen;
 flags->ckps_new_engine_cycle_happen = 0;
 _END_ATOMIC_BLOCK();
 return result;
}

uint8_t ckps_get_current_cog(void)
{
 return ckps.cog;
}

uint8_t ckps_is_cog_changed(void)
{
 static uint8_t prev_cog = 0;
 uint8_t value = ckps.cog;
 if (prev_cog != value)
 {
  prev_cog = value;
  return 1;
 }
 return 0;
}

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps.chan_number = i_cyl_number >> 1; //single ignition channel for two cylinders (один канал зажигания на 2 цилиндра)
 _END_ATOMIC_BLOCK();

 switch(i_cyl_number)
 {
 case 2: ckps.frq_calc_dividend = 15000000L;
  break;
 case 4: ckps.frq_calc_dividend = 7500000L;
  break;
 case 6: ckps.frq_calc_dividend = 5000000L;
  break;
 case 8: ckps.frq_calc_dividend = 3750000L;
  break;
 }
 //TODO: calculations previosly made by ckps_set_cogs_btdc()|ckps_set_knock_window() becomes invalid!
 //So, ckps_set_cogs_btdc() must be called again. Do it here or in place where this function called.
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 uint8_t _t, i, cogs_per_cycle;
 //translate from degree to teeth (переводим из градусов в зубья)
 ckps.knock_wnd_begin_abs = begin / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);
 ckps.knock_wnd_end_abs = end / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);

 cogs_per_cycle = (WHEEL_COGS_NUM) / ckps.chan_number;
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (ckps.cogs_btdc + i * cogs_per_cycle);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
 }
 _RESTORE_INTERRUPT(_t);
}

void ckps_enable_ignition(uint8_t i_cutoff)
{
 flags->ckps_ign_enabled = i_cutoff;
}

void ckps_set_merge_outs(uint8_t i_merge)
{
 ckps.chan_mask = i_merge ? 0x00 : 0xFF;
}

/**Helpful macro.
 * Generates end of accumulation pulse (moment of spark) for 1st,2nd,3rd,4th channels correspondingly
 * (Вспомогательный макрос. Конец импульса накачки (момент искры) для 1-го,2-го,3-го,4-го каналов соответственно). */
#define TURNON_IGN_CHANNELS(){\
  case 0: PORTD |= _BV(PORTD4);\
   break;\
  case 1: PORTD |= _BV(PORTD5);\
   break;\
  case 2: PORTC |= _BV(PORTC0);\
   break;\
  case 3: PORTC |= _BV(PORTC1);\
   break;}

/**Helpful macro. 
 * Generates end of ignition drive pulse for 1st,2nd,3rd,4th channels correspondingly
 * (Вспомогательный макрос. Конец импульса запуска зажигания для 1-го,2-го,3-го,4-го каналов соответственно) */
#define TURNOFF_IGN_CHANNELS(){\
 case 0: PORTD &= ~_BV(PORTD4);\
  break;\
 case 1: PORTD &= ~_BV(PORTD5);\
  break;\
 case 2: PORTC &= ~_BV(PORTC0);\
  break;\
 case 3: PORTC &= ~_BV(PORTC1);\
  break;}

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
INLINE
void turn_off_ignition_channel(uint8_t i_channel)
{
 if (!flags->ckps_ign_enabled)
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes 
 //the igniter go to the regime of energy accumulation
 //Завершение импульса запуска коммутатора, перевод линии порта в низкий уровень - заставляем
 //коммутатор перейти в режим накопления энергии
 switch(i_channel)
 {
#ifndef INVERSE_IGN_OUTPUTS
  TURNOFF_IGN_CHANNELS();
#else
  TURNON_IGN_CHANNELS();
#endif
 }
}

/**Interrupt handler for Compare/Match channel A of timer T1
 * вектор прерывания по совпадению канала А таймера Т1
 */                                                                        
ISR(TIMER1_COMPA_vect)
{
#ifdef COIL_REGULATION
 ckps.tmrval_saved = TCNT1;
#endif

 TIMSK&= ~_BV(OCIE1A); //disable interrupt (запрещаем прерывание)

 //line of port in the low level, now set it into a high level - makes the igniter to stop 
 //the accumulation of energy and close the transistor (spark)
 //(линия порта в низком уровне, теперь переводим её в высокий уровень - заставляем коммутатор прекратить 
 //накопление энергии и закрыть транзистор (искра)).
 switch(ckps.channel_mode)
 {
#ifndef INVERSE_IGN_OUTPUTS
  TURNON_IGN_CHANNELS();
#else
  TURNOFF_IGN_CHANNELS();
#endif
  default:
   return; //none of channels selected (никакой канал не выбран) - CKPS_CHANNEL_MODENA
 }

#ifdef COIL_REGULATION
 ckps.acc_delay = ((uint32_t)ckps.period_curr) * (WHEEL_COGS_NUM / ckps.chan_number);
 if (ckps.cr_acc_time > ckps.acc_delay-120)
  ckps.cr_acc_time = ckps.acc_delay-120;  //restrict accumulation time. Dead band = 500us   
 ckps.acc_delay-= ckps.cr_acc_time;
 ckps.period_saved = ckps.period_curr; //remember current inter-tooth period

 ckps.channel_mode_b = (ckps.channel_mode < ckps.chan_number-1) ? ckps.channel_mode + 1 : 0 ;
 ckps.channel_mode_b&= ckps.chan_mask;
 ckps.add_period_taken = 0;
 flags->ckps_need_to_set_channel_b = 1;

 //We remembered value of TCNT1 at the top of of this function. But input capture event
 //may occur when interrupts were already disabled (by hardware) but value of timer is still
 //not saved. Another words, ICR1 must not be less than tmrval_saved.
 if (TIFR & _BV(ICF1))
  ckps.tmrval_saved = ICR1;
#else
 //start counting the duration of pulse in the teeth (начинаем отсчет длительности импульса в зубьях)
 chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
#endif
}

#ifdef COIL_REGULATION
/**Interrupt handler for Compare/Match channel B of timer T1. Used for coil regulation.
 * вектор прерывания по совпадению канала B таймера Т1. Используется для управления накоплением энергии КЗ.
 */
ISR(TIMER1_COMPB_vect)
{
 TIMSK&= ~_BV(OCIE1B); //запрещаем прерывание
 //start accumulation
 turn_off_ignition_channel(ckps.channel_mode_b);
 ckps.channel_mode_b = CKPS_CHANNEL_MODENA;
}
#endif

/**Initialization of timer 0 using specified value and start, clock = 250kHz
 * It is assumed that this function called when all interrupts are disabled
 * (Инициализация таймера 0 указанным значением и запуск, clock = 250kHz.
 * Предполагается что вызов этой функции будет происходить при запрещенных прерываниях)
 * \param value value for load into timer (значение для загрузки в таймер)
 */
INLINE
void set_timer0(uint16_t value)
{
 TCNT0_H = GETBYTE(value, 1);
 TCNT0 = 255 - GETBYTE(value, 0);
 TCCR0  = _BV(CS01)|_BV(CS00);
}

/**Helpful function, used at the startup of engine
 * (Вспомогательная функция, используется во время пуска)
 * \return 1 when synchronization is finished, othrewise 0 (1 когда синхронизация окончена, иначе 0)
 */
uint8_t sync_at_startup(void)
{
 switch(ckps.starting_mode)
 {
  case 0: //skip certain number of teeth (пропуск определенного кол-ва зубьев)
   /////////////////////////////////////////
   flags->ckps_is_valid_half_turn_period = 0;
   /////////////////////////////////////////
   if (ckps.cog >= CKPS_ON_START_SKIP_COGS) 
    ckps.starting_mode = 1;
   break;
  case 1: //find out synchro-label (поиск синхрометки)
   if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)) 
   {
    flags->ckps_is_synchronized = 1;
    ckps.period_curr = ckps.period_prev;  //exclude value of synchro-label's period
    ckps.cog = 1; //first tooth (1-й зуб)
    return 1; //finish process of synchronization (конец процесса синхронизации)
   }
   break;
  }
 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;
 ++ckps.cog;
 return 0; //continue process of synchronization (продолжение процесса синхронизации)
}

/**This procedure called for all teeth (including recovered teeth)
 * Процедура. Вызывается для всех зубьев шкива (включительно с восстановленными)
 */
void process_ckps_cogs(void)
{
 uint16_t diff;
 uint8_t i;
   
#ifdef COIL_REGULATION
 if (ckps.channel_mode_b != CKPS_CHANNEL_MODENA)
 {
  //We must take into account time elapsed between last spark and following tooth.
  //Because ICR1 can not be less than tmrval_saved we are using addition modulo 65535
  //to calculate difference (elapsed time).
  if (!ckps.add_period_taken)
  {
   ckps.acc_delay-=((uint16_t)(~ckps.tmrval_saved)) + 1 + GetICR();
   ckps.add_period_taken = 1;
  }
  //Correct our prediction on each cog 
  ckps.acc_delay+=((int32_t)ckps.period_curr - ckps.period_saved);

  //Do we have to set COMPB ? (We have to set COMPB if less than 2 periods remain)
  if (flags->ckps_need_to_set_channel_b && ckps.acc_delay <= (ckps.period_curr << 1))
  {
   OCR1B = GetICR() + ckps.acc_delay + ((int32_t)ckps.period_curr - ckps.period_saved);
   TIFR = _BV(OCF1B);
   TIMSK|= _BV(OCIE1B);
   flags->ckps_need_to_set_channel_b = 0; // To avoid entering into setup mode (чтобы не войти в режим настройки ещё раз)
  }
  ckps.acc_delay-=ckps.period_curr;
 }
#endif 

#ifdef COOLINGFAN_PWM
 //CKP processing creates a big delay which negatively affects cooling fan's PWM. We
 //need to enable T/C 2 interrupts. TODO: it is bad idea to enable all interrupts 
 //here. We need only OCIE2 and TOIE2.
 _ENABLE_INTERRUPT();
#endif

 if (flags->ckps_use_knock_channel)
 {
  for(i = 0; i < ckps.chan_number; ++i)
  {
   //start listening a detonation (opening the window)
   //начинаем слушать детонацию (открытие окна)
   if (ckps.cog == chanstate[i].knock_wnd_begin)
    knock_set_integration_mode(KNOCK_INTMODE_INT);

   //finish listening a detonation (closing the window) and start the process of measuring integrated value
   //заканчиваем слушать детонацию (закрытие окна) и запускаем процесс измерения накопленного значения
   if (ckps.cog == chanstate[i].knock_wnd_end)
   {
    knock_set_integration_mode(KNOCK_INTMODE_HOLD);
    adc_begin_measure_knock();
   }
  }
 }

 for(i = 0; i < ckps.chan_number; ++i)
 {
  //for 66° before t.d.c (before working cycle) establish new advance angle to be actuated, 
  //before this moment value was stored in a temporary buffer.
  //за 66 градусов до в.м.т перед рабочим циклом устанавливаем новый УОЗ для реализации, УОЗ
  //до этого хранился во временном буфере.
  if (ckps.cog == chanstate[i].cogs_latch)
  {
   ckps.channel_mode = (i & ckps.chan_mask); //remember number of channel (запоминаем номер канала)
   flags->ckps_need_to_set_channel = 1;      //establish an indication that needed to count advance angle (устанавливаем признак того, что нужно отсчитывать УОЗ)
   //start counting of advance angle (начинаем отсчет угла опережения)
   ckps.current_angle = ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * WHEEL_LATCH_BTDC; // those same 66° (те самые 66°)
   ckps.advance_angle = ckps.advance_angle_buffered; //advance angle with all the adjustments (say, 15°)(опережение со всеми корректировками (допустим, 15°))
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (запускаем процесс загрузки настроек в HIP)
   adc_begin_measure();            //start the process of measuring analog input values (запуск процесса измерения значений аналоговых входов)
  }

  //teeth of end/beginning of the measurement of rotation period - t.d.c Read and save the measured period,
  //then remember current value of count for the next measurement
  //(зубья завершения/начала измерения периодов вращения  - в.м.т. считывание и сохранение измеренного периода,
  //затем запоминание текущего значения счетчика для следующего измерения)
  if (ckps.cog==chanstate[i].cogs_btdc) 
  { 
   //if it was overflow, then set the maximum possible time
   //если было переполнение то устанавливаем максимально возможное время
   if (((ckps.period_curr > 1250) || !flags->ckps_is_valid_half_turn_period))
    ckps.half_turn_period = 0xFFFF;
   else
    ckps.half_turn_period = (GetICR() - ckps.measure_start_value);

   ckps.measure_start_value = GetICR();
   flags->ckps_is_valid_half_turn_period = 1;
   /////////////////////////////////////////
   flags->ckps_new_engine_cycle_happen = 1; //set the cycle-synchronozation event (устанавливаем событие цикловой синхронизации)
   /////////////////////////////////////////
  }
 }

 //Preparing to start the ignition for the current channel (if the right moment became)
 //подготовка к запуску зажигания для текущего канала (если наступил нужный момент)
 if (flags->ckps_need_to_set_channel && ckps.channel_mode!= CKPS_CHANNEL_MODENA)
 {
  diff = ckps.current_angle - ckps.advance_angle;
  if (diff <= (ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) << 1))
  {
   //before starting the ignition it is left to count less than 2 teeth. It is necessary to prepare the compare module
   //(до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения)
   //TODO: replace heavy division by multiplication with magic number. This will reduce up to 40uS !
   OCR1A = GetICR() + ((uint32_t)diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
   TIFR = _BV(OCF1A);
   flags->ckps_need_to_set_channel = 0; // For avoiding to enter into setup mode (чтобы не войти в режим настройки ещё раз)
   TIMSK|= _BV(OCIE1A); //enable Compare A interrupt (разрешаем прерывание)
#ifndef COIL_REGULATION
   chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
#endif
  }
 }

#ifndef COIL_REGULATION
 //finish the ignition trigger pulses for igniter(s) and immediately increase the number of tooth for processed channel
 //заканчиваем импульсы запуска коммутатора(ов) и сразу увеличиваем номер зуба для обработанного канала
 for(i = 0; i < ckps.chan_number; ++i)
 {
  if (chanstate[i].ignition_pulse_cogs >= ckps.ignition_cogs)
   turn_off_ignition_channel(i);
  ++(chanstate[i].ignition_pulse_cogs);
 }
#endif

 //tooth passed - angle before t.d.c. decriased by 6 degr. (for 60-2) or 10 degr. (for 36-1).
 //(прошел зуб - угол до в.м.т. уменьшился на 6 град (для 60-2) или 10 град (для 36-1))
 ckps.current_angle-= ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
 ++ckps.cog;
}

/**Input capture interrupt of timer 1 (called at passage of each tooth)
 * прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
 */
ISR(TIMER1_CAPT_vect)
{
 ckps.period_curr = GetICR() - ckps.icr_prev;

 //At the start of engine, skipping a certain number of teeth for initializing 
 //the memory of previous periods. Then look for synchro-label.
 //при старте двигателя, пропускаем определенное кол-во зубьев для инициализации 
 //памяти предыдущих периодов. Затем ищем синхрометку.
 if (!flags->ckps_is_synchronized)
 {
  if (sync_at_startup())
   goto synchronized_enter;
  return;
 }

 //Each period, check for synchro-label, and if, after discovering of synchro-label
 //count of teeth being found incorrect, then set error flag.
 //(каждый период проверяем на синхрометку, и если после обнаружения синхрометки
 //оказалось что кол-во зубьев неправильное, то устанавливаем признак ошибки).
 if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev))
 {
  if ((ckps.cog != (WHEEL_COGS_NUM + 1))) //also taking into account recovered teeth (учитываем также восстановленные зубья)
   flags->ckps_error_flag = 1; //ERROR
  ckps.cog = 1; //first tooth (1-й зуб)
  ckps.period_curr = ckps.period_prev;  //exclude value of synchro-label's period
 }

synchronized_enter:
 //If the last tooth before synchro-label, we begin the countdown for
 //the restoration of missing teeth, as the initial data using the last
 //value of inter-teeth period.
 //(Если последний зуб перед синхрометкой, то начинаем отсчет времени для
 //восстановления отсутствующих зубов, в качестве исходных данных используем 
 //последнее значение межзубного периода).
 if (ckps.cog == WHEEL_LAST_COG)
  set_timer0(ckps.period_curr);

 //call handler for normal teeth (вызываем обработчик для нормальных зубьев)
 process_ckps_cogs();

 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;
}

/**Purpose of this interrupt handler is to supplement timer up to 16 bits and call procedure
 * for processing teeth when set 16 bit timer expires
 * (Задача этого обработчика дополнять таймер до 16-ти разрядов и вызывать процедуру
 * обработки зубьев по истечении установленного 16-ти разряюного таймера). */
ISR(TIMER0_OVF_vect)
{
 if (TCNT0_H!=0)  //Did high byte exhaust (старший байт не исчерпан) ?
 {
  TCNT0 = 0;
  --TCNT0_H;
 }
 else
 {//the countdown is over (отсчет времени закончился)
  ICR1 = TCNT1;  //simulate input capture 
  TCCR0 = 0;     //stop timer (останавливаем таймер)

#ifndef WHEEL_36_1 //60-2
  //start timer to recover 60th tooth (запускаем таймер чтобы восстановить 60-й зуб)
  if (ckps.cog == 59)
   set_timer0(ckps.period_curr);
#endif

  //Call handler for absent teeth (вызываем обработчик для отсутствующих зубьев)
  process_ckps_cogs();
 }
}
