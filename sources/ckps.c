/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

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

#if !defined(HALL_SYNC) && !defined(CKPS_2CHIGN)

#include <stdlib.h>
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "camsens.h"
#include "ckps.h"
#include "ioconfig.h"
#include "injector.h"   //inject_start_inj()
#include "magnitude.h"
#include "tables.h"     //fnptr_t

#include "knock.h"

//PHASED_IGNITION can't be used without PHASE_SENSOR
#if defined(PHASED_IGNITION) && !defined(PHASE_SENSOR)
 #error "You can not use phased ignition without phase sensor. Define PHASE_SENSOR if it is present in the system or do not use phased ignition!"
#endif

/**Maximum number of ignition channels */
#define IGN_CHANNELS_MAX      8

/** Barrier for detecting of missing teeth (барьер для селекции синхрометки) 
 * e.g. for 60-2 crank wheel, p * 2.5
 *      for 36-1 crank wheel, p * 1.5
 */
#define CKPS_GAP_BARRIER(p) ( ((p) << (ckps.miss_cogs_num==2)) + ((p) >> 1) )

/** number of teeth that will be skipped at the start before synchronization
 * (количество зубьев которое будет пропускаться при старте перед синхронизацией) */
#define CKPS_ON_START_SKIP_COGS      5

/** Access Input Capture Register */
#define GetICR() (ICR1)

/** Used to indicate that none from ignition channels are selected
 * (используется для указания того что ни один канал зажигания не выбран) */
#define CKPS_CHANNEL_MODENA  255

//Define values for controlling of outputs
#define IGN_OUTPUTS_INIT_VAL 1        //!< value used for initialization
#define IGN_OUTPUTS_ON_VAL   1        //!< value used to turn on ignition channel
#define IGN_OUTPUTS_OFF_VAL  0        //!< value used to turn off ignition channel

/**Задержка входа в прерывание COMPA и установки уровня на соотв. линии порта в тиках таймера.
 * Используется для компенсации времени. */
#define COMPA_VECT_DELAY 2

// Flags (see flags variable)
#define F_ERROR     0                 //!< CKP error flag, set in the CKP's interrupt, reset after processing (признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки) 
#define F_VHTPER    1                 //!< used to indicate that measured period is valid (actually measured)
#define F_ISSYNC    2                 //!< indicates that synchronization has been completed (missing teeth found)
#define F_STROKE    3                 //!< flag for synchronization with rotation (флаг синхронизации с вращением)
#define F_USEKNK    4                 //!< flag which indicates using of knock channel (признак использования канала детонации)
#define F_NTSCHA    5                 //!< indicates that it is necessary to set channel
#ifdef DWELL_CONTROL
 #define F_NTSCHB   6                 //!< Indicates that it is necessary to set channel (COMPB) 
#endif
#define F_IGNIEN    7                 //!< Ignition enabled/disabled

//Additional flags (see flags2 variable)
#ifdef DWELL_CONTROL
 #define F_ADDPTK    0                //!< indicates that time between spark and next cog was taken into account
#endif
#ifdef PHASED_IGNITION
 #define F_CAMISS    1                //!< Indicates that system has already obtained event from a cam sensor
#endif
#define F_CALTIM     2                //!< Indicates that time calculation is started before the spark
#define F_SPSIGN     3                //!< Sign of the measured stroke period (time between TDCs)

/** State variables */
typedef struct
{
 uint16_t icr_prev;                   //!< previous value if Input Capture Register (предыдущее значение регистра захвата)
 volatile uint16_t period_curr;       //!< last measured inter-tooth period (последнй измеренный межзубный период)
 uint16_t period_prev;                //!< previous value of inter-tooth period (предыдущее значение межзубного периода)
 volatile uint16_t cog;               //!< counts teeth starting from missing teeth (2 revolutions), begins from 1 (считает зубья после выреза, начинает считать с 1)
 volatile uint8_t cog360;             //!< counts teeth starting from missing teeth (1 revolution).
 uint16_t measure_start_value;        //!< remembers the value of the capture register to measure the half-turn (запоминает значение регистра захвата для измерения периода полуоборота)
 uint16_t current_angle;              //!< counts out given advance angle during the passage of each tooth (отсчитывает заданный УОЗ при прохождении каждого зуба)
 volatile uint16_t stroke_period;     //!< stores the last measurement of the passage of teeth n (хранит последнее измерение времени прохождения n зубьев)
 int16_t  advance_angle;              //!< required adv.angle * ANGLE_MULTIPLAYER (требуемый УОЗ * ANGLE_MULTIPLAYER)
 volatile int16_t advance_angle_buffered;//!< buffered value of advance angle (to ensure correct latching)
 uint8_t  ignition_cogs;              //!< number of teeth determining the duration of ignition drive pulse (кол-во зубьев определяющее длительность импульсов запуска коммутаторов)
 uint8_t  starting_mode;              //!< state of state machine processing of teeth at the startup (состояние конечного автомата обработки зубьев на пуске)
 uint8_t  channel_mode;               //!< determines which channel of the ignition to run at the moment (определяет какой канал зажигания нужно запускать в данный момент)
 volatile uint8_t cogs_btdc;          //!< number of teeth from missing teeth to TDC of the first cylinder (кол-во зубьев от синхрометки до в.м.т первого цилиндра)
 int8_t   knock_wnd_begin_abs;        //!< begin of the phase selection window of detonation in the teeth of wheel, relatively to TDC (начало окна фазовой селекции детонации в зубьях шкива относительно в.м.т)
 int8_t   knock_wnd_end_abs;          //!< end of the phase selection window of detonation in the teeth of wheel, relatively to TDC (конец окна фазовой селекции детонации в зубьях шкива относительно в.м.т)
 volatile uint8_t chan_number;        //!< number of ignition channels (кол-во каналов зажигания)
 uint32_t frq_calc_dividend;          //!< divident for calculating RPM (делимое для расчета частоты вращения)
#ifdef DWELL_CONTROL
 volatile uint16_t cr_acc_time;       //!< accumulation time for dwell control (timer's ticks)
 uint8_t  channel_mode_b;             //!< determines which channel of the ignition to start accumulate at the moment (определяет какой канал зажигания будет накапливать энергию в данный момент)
 uint32_t acc_delay;                  //!< delay between last ignition and next accumulation
 uint16_t tmrval_saved;               //!< value of timer at the moment of each spark
 uint16_t period_saved;               //!< inter-tooth period at the moment of each spark
#endif
 volatile uint8_t chan_mask;          //!< mask used to disable multi-channel mode and use single channel
#ifdef HALL_OUTPUT
 int8_t   hop_offset;                 //!< Hall output: start of pulse in tooth of wheel relatively to TDC
 uint8_t  hop_duration;               //!< Hall output: duration of pulse in tooth of wheel
#endif

 volatile uint8_t wheel_cogs_num;     //!< Number of teeth, including absent (количество зубьев, включая отсутствующие)
 volatile uint8_t wheel_cogs_nump1;   //!< wheel_cogs_num + 1
 volatile uint8_t wheel_cogs_numm1;   //!< wheel_cogs_num - 1
 volatile uint16_t wheel_cogs_num2;   //!< Number of teeth which corresponds to 720° (2 revolutions)
 volatile uint16_t wheel_cogs_num2p1; //!< wheel_cogs_num2 + 1
 volatile uint8_t miss_cogs_num;      //!< Count of crank wheel's missing teeth (количество отсутствующих зубьев)
 volatile uint8_t wheel_last_cog;     //!< Number of last(present) tooth, numeration begins from 1! (номер последнего(существующего) зуба, нумерация ничинается с 1!)
 /**Number of teeth before TDC which determines moment of advance angle latching, start of measurements from sensors,
  * latching of settings into HIP9011 (кол-во зубьев до в.м.т определяющие момент загрузки УОЗ, старт измерения датчиков,
  * загрузку настроек в HIP)
  */
 volatile uint8_t  wheel_latch_btdc;
 volatile uint16_t degrees_per_cog;   //!< Number of degrees which corresponds to the 1 tooth (количество градусов приходящееся на один зуб диска)
 volatile uint16_t cogs_per_chan;     //!< Number of teeth per 1 ignition channel (it is fractional number * 256)
 volatile int16_t start_angle;        //!< Precalculated value of the advance angle at 66° (at least) BTDC
#ifdef STROBOSCOPE
 uint8_t strobe;                      //!< Flag indicates that strobe pulse must be output on pending ignition stroke
#endif

 volatile uint8_t t1oc;               //!< Timer 1 overflow counter
 volatile uint8_t t1oc_s;             //!< Contains value of t1oc synchronized with stroke_period value
}ckpsstate_t;
 
/**Precalculated data (reference points) and state data for a single channel plug
 * Предрасчитанные данные(опорные точки) и данные состояния для отдельного канала зажигания
 */
typedef struct
{
#ifndef DWELL_CONTROL
 /** Counts out teeth for ignition pulse (отсчитывает зубья импульса зажигания) */
 volatile uint8_t ignition_pulse_cogs;
#endif

 /**Address of callback which will be used for settiong of I/O */
 volatile fnptr_t io_callback1;
#ifdef PHASED_IGNITION
 /**Second callback used only in semi-sequential ignition mode */
 volatile fnptr_t io_callback2;
#endif

#ifdef HALL_OUTPUT
 volatile uint16_t hop_begin_cog;      //!< Hall output: tooth number that corresponds to the beginning of pulse
 volatile uint16_t hop_end_cog;        //!< Hall output: tooth number that corresponds to the end of pulse
#endif

 /** Determines number of tooth (relatively to TDC) at which "latching" of data is performed (определяет номер зуба (относительно в.м.т.) на котором происходит "защелкивание" данных) */
 volatile uint16_t cogs_latch;
 /** Determines number of tooth at which measurement of rotation period is performed (определяет номер зуба на котором производится измерение периода вращения коленвала (между раб. тактами)) */
 volatile uint16_t cogs_btdc;
 /** Determines number of tooth at which phase selection window for knock detection is opened (определяет номер зуба на котором открывается окно фазовой селекции сигнала ДД (начало интегрирования)) */
 volatile uint16_t knock_wnd_begin;
 /** Determines number of tooth at which phase selection window for knock detection is closed (определяет номер зуба на котором закрывается окно фазовой селекции сигнала ДД (конец интегрирования)) */
 volatile uint16_t knock_wnd_end;
}chanstate_t;

ckpsstate_t ckps;                         //!< instance of state variables
chanstate_t chanstate[IGN_CHANNELS_MAX];  //!< instance of array of channel's state variables

// Arrange flags in the free I/O register (размещаем в свободном регистре ввода/вывода)
//  note: may be not effective on other MCUs or even case bugs! Be aware.
#ifdef _PLATFORM_M644_
#define flags  GPIOR0                 //!< ATmega644 has one general purpose I/O register and we use it for first flags variable
#else
#define flags  TWAR                   //!< First flags variable in I/O register (ATMega16/32)
#endif
#define flags2 TWBR                   //!< Second flags variable in I/O register

/** Supplement timer/counter 0 up to 16 bits, use R15 (для дополнения таймера/счетчика 0 до 16 разрядов, используем R15) */
#ifdef __ICCAVR__
 __no_init __regvar uint8_t TCNT0_H@15;
#else //GCC
 uint8_t TCNT0_H __attribute__((section (".noinit")));
#endif

/**Accessor macro for RPM dividents table*/
#define FRQ_CALC_DIVIDEND(channum) PGM_GET_DWORD(&frq_calc_dividend[channum])
/**Table srtores dividends for calculating of RPM */
PGM_DECLARE(uint32_t frq_calc_dividend[1+IGN_CHANNELS_MAX]) =
#ifdef _PLATFORM_M644_
 //     1          2          3          4         5         6         7         8
 {0, 37500000L, 18750000L, 12500000L, 9375000L, 7500000L, 6250000L, 5357143L, 4687500L};
#else
 {0, 30000000L, 15000000L, 10000000L, 7500000L, 6000000L, 5000000L, 4285714L, 3750000L};
#endif

void ckps_init_state_variables(void)
{
#ifndef DWELL_CONTROL
 uint8_t i;
 _BEGIN_ATOMIC_BLOCK();
 //Set to value that means to do nothing with outputs
 //(Установка в хначение изначающее ничего не делать с выходами) 
 for(i = 0; i < IGN_CHANNELS_MAX; ++i)
  chanstate[i].ignition_pulse_cogs = 255;
#else
 _BEGIN_ATOMIC_BLOCK();
 ckps.cr_acc_time = 0;
 ckps.channel_mode_b = CKPS_CHANNEL_MODENA;
 CLEARBIT(flags, F_NTSCHB);
#endif

 ckps.cog = ckps.cog360 = 0;
 ckps.stroke_period = 0xFFFF;
 ckps.advance_angle = ckps.advance_angle_buffered = 0;
 ckps.starting_mode = 0;
 ckps.channel_mode = CKPS_CHANNEL_MODENA;
#ifdef PHASED_IGNITION
 CLEARBIT(flags2, F_CAMISS);
#endif
 CLEARBIT(flags, F_NTSCHA);
 CLEARBIT(flags, F_STROKE);
 CLEARBIT(flags, F_ISSYNC);
 SETBIT(flags, F_IGNIEN);
 CLEARBIT(flags2, F_CALTIM);
 CLEARBIT(flags2, F_SPSIGN);

#ifdef _PLATFORM_M644_
 TCCR0B = 0;
 TIMSK1|=_BV(TOIE1);
#else
 TCCR0 = 0;                           //timer is stopped (останавливаем таймер0)
 TIMSK|=_BV(TOIE1);                   //enable Timer 1 overflow interrupt. Used for correct calculation of very low RPM
#endif

#ifdef STROBOSCOPE
 ckps.strobe = 0;
#endif

 ckps.t1oc = 0;                       //reset overflow counter
 ckps.t1oc_s = 255;                   //RPM is very low
 _END_ATOMIC_BLOCK();
}

void ckps_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 ckps_init_state_variables();
 CLEARBIT(flags, F_ERROR);

 //Compare channels do not connected to lines of ports (normal port mode)
 //(Каналы Compare не подключены к линиям портов (нормальный режим портов))
 TCCR1A = 0;

 //(Noise reduction(подавление шума), rising edge of capture(передний фронт захвата), clock = 250kHz)
 TCCR1B = _BV(ICNC1)|_BV(ICES1)|_BV(CS11)|_BV(CS10);

 //enable input capture and Compare A interrupts of timer 1, also overflow interrupt of timer 0
 //(разрешаем прерывание по захвату и сравнению А таймера 1, а также по переполнению таймера 0)
#ifdef _PLATFORM_M644_
 TIMSK1|= _BV(ICIE1);
 TIMSK0|= _BV(TOIE0);
#else
 TIMSK|= _BV(TICIE1)|_BV(TOIE0);
#endif
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
 PORTD|= _BV(PD6); // pullup for ICP1 (подтяжка для ICP1)

 //after ignition is on, igniters must not be in the accumulation mode,
 //therefore set low level on their inputs
 //(после включения зажигания коммутаторы не должны быть в режиме накопления,
 //поэтому устанавливаем на их входах низкий уровень)
 IOCFG_INIT(IOP_IGN_OUT1, IGN_OUTPUTS_INIT_VAL);        //init 1-st (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT2, IGN_OUTPUTS_INIT_VAL);        //init 2-nd (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT3, IGN_OUTPUTS_INIT_VAL);        //init 3-rd (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT4, IGN_OUTPUTS_INIT_VAL);        //init 4-th (can be remapped)
 IOCFG_INIT(IOP_ADD_IO1, IGN_OUTPUTS_INIT_VAL);         //init 5-th (can be remapped)
 IOCFG_INIT(IOP_ADD_IO2, IGN_OUTPUTS_INIT_VAL);         //init 6-th (can be remapped)
 IOCFG_INIT(IOP_IGN_OUT7, IGN_OUTPUTS_INIT_VAL);        //init 7-th (for maniacs)
 IOCFG_INIT(IOP_IGN_OUT8, IGN_OUTPUTS_INIT_VAL);        //init 8-th (for maniacs)

 //init I/O for Hall output if it is enabled
#ifdef HALL_OUTPUT
 IOCFG_INIT(IOP_HALL_OUT, 1);
#endif

 //init I/O for stroboscope
#ifdef STROBOSCOPE
 IOCFG_INIT(IOP_STROBE, 0);
#endif
}

//Instantaneous frequency calculation of crankshaft rotation from the measured period between the engine strokes
//(for example for 4-cylinder, 4-stroke it is 180°)
//Period measured in the discretes of timer (one discrete = 4us), one minute = 60 seconds, one second has 1,000,000 us.
//Высчитывание мгновенной частоты вращения коленвала по измеренному периоду между тактами двигателя
//(например для 4-цилиндрового, 4-х тактного это 180 градусов)
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс.
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period; uint8_t ovfcnt, sign;
 //ensure atomic acces to variable (обеспечиваем атомарный доступ к переменной)
 _DISABLE_INTERRUPT();
 period = ckps.stroke_period;        //stroke period
 ovfcnt = ckps.t1oc_s;               //number of timer overflows
 sign = CHECKBIT(flags2, F_SPSIGN);  //sign of stroke period
 _ENABLE_INTERRUPT();

 //We know period and number of timer overflows, so we can calculate correct value of RPM even if RPM is very low
 if (sign && ovfcnt > 0)
  return ckps.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) - (65536-period));
 else
  return ckps.frq_calc_dividend / ((((int32_t)ovfcnt) * 65536) + period);
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
 * Tooth number should not be greater than cogs number * 2 or less than zero
 */
static uint16_t _normalize_tn(int16_t i_tn)
{
 if (i_tn > (int16_t)ckps.wheel_cogs_num2)
  return i_tn - (int16_t)ckps.wheel_cogs_num2;
 if (i_tn <= 0)
  return i_tn + ckps.wheel_cogs_num2;
 return i_tn;
}

void ckps_set_cogs_btdc(uint8_t cogs_btdc)
{
 uint8_t _t, i;
 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].cogs_btdc = _normalize_tn(tdc);
  chanstate[i].cogs_latch = _normalize_tn(tdc - ckps.wheel_latch_btdc);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
#ifdef HALL_OUTPUT
  //update Hall output pulse parameters because they depend on ckps.cogs_btdc parameter
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
#endif
 }
 ckps.cogs_btdc = cogs_btdc;
 _RESTORE_INTERRUPT(_t);
}

#ifndef DWELL_CONTROL
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
 return CHECKBIT(flags, F_ERROR) > 0;
}

void ckps_reset_error(void)
{
 CLEARBIT(flags, F_ERROR);
}

void ckps_use_knock_channel(uint8_t use_knock_channel)
{
 WRITEBIT(flags, F_USEKNK, use_knock_channel);
}

uint8_t ckps_is_stroke_event_r()
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = CHECKBIT(flags, F_STROKE) > 0;
 CLEARBIT(flags, F_STROKE);
 _END_ATOMIC_BLOCK();
 return result;
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

/** Get value of I/O callback by index. This function is necessary for supporting of 7,8 ign. channels
 * \param index Index of callback */
INLINE
static fnptr_t get_callback(uint8_t index)
{
 return (index < IOP_ECF) ? IOCFG_CB(index) : IOCFG_CB(index + 24);
}

#ifndef PHASED_IGNITION
/**Tune channels' I/O for semi-sequential ignition mode (wasted spark) */
static void set_channels_ss(void)
{
 uint8_t _t, i = 0, chan = ckps.chan_number / 2;
 for(; i < chan; ++i)
 {
  fnptr_t value = IOCFG_CB(i);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback1 = value;
  chanstate[i + chan].io_callback1 = value;
  _RESTORE_INTERRUPT(_t);
 }
}

#else
/**Tune channels' I/O for full sequential ignition mode */
static void set_channels_fs(uint8_t fs_mode)
{
 uint8_t _t, i = 0, ch2 = fs_mode ? 0 : ckps.chan_number / 2, iss;
 for(; i < ckps.chan_number; ++i)
 {
  iss = (i + ch2);
  if (iss >= ckps.chan_number)
   iss-=ckps.chan_number;

  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  chanstate[i].io_callback1 = get_callback(i);
  chanstate[i].io_callback2 = get_callback(iss);
  _RESTORE_INTERRUPT(_t);
 }
}
#endif

void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 uint8_t i = ckps.chan_number;
 _BEGIN_ATOMIC_BLOCK();
 ckps.chan_number = i_cyl_number;
 _END_ATOMIC_BLOCK();

 ckps.frq_calc_dividend = FRQ_CALC_DIVIDEND(i_cyl_number);

 //We have to retune I/O configuration after changing of cylinder number
#ifndef PHASED_IGNITION
 set_channels_ss();  // Tune for semi-sequential mode
#else //phased ignition
 //Tune for full sequential mode if cam sensor works, otherwise tune for semi-sequential mode
 set_channels_fs(cams_is_ready());
#endif

 //unused channels must be turned off
 if (i > i_cyl_number)
  for(i = i_cyl_number; i < IGN_CHANNELS_MAX; ++i)
   ((iocfg_pfn_set)get_callback(i))(IGN_OUTPUTS_ON_VAL);

 //TODO: calculations previosly made by ckps_set_cogs_btdc()|ckps_set_knock_window()|ckps_set_hall_pulse() becomes invalid!
 //So, ckps_set_cogs_btdc() must be called again. Do it here or in place where this function called.
}

void ckps_set_knock_window(int16_t begin, int16_t end)
{
 uint8_t _t, i;
 //translate from degrees to teeth (переводим из градусов в зубья)
 ckps.knock_wnd_begin_abs = begin / ckps.degrees_per_cog;
 ckps.knock_wnd_end_abs = end / ckps.degrees_per_cog;

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)ckps.cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs);
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs);
 }
 _RESTORE_INTERRUPT(_t);
}

void ckps_enable_ignition(uint8_t i_cutoff)
{
 WRITEBIT(flags, F_IGNIEN, i_cutoff);
}

void ckps_set_merge_outs(uint8_t i_merge)
{
 ckps.chan_mask = i_merge ? 0x00 : 0xFF;
}

#ifdef HALL_OUTPUT
void ckps_set_hall_pulse(int8_t i_offset, uint8_t i_duration)
{
 uint8_t _t, i;
 //save values because we will access them from other function
 ckps.hop_offset = i_offset;
 ckps.hop_duration = i_duration;

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint16_t tdc = (((uint16_t)ckps.cogs_btdc) + ((i * ckps.cogs_per_chan) >> 8));
  chanstate[i].hop_begin_cog = _normalize_tn(tdc - ckps.hop_offset);
  chanstate[i].hop_end_cog = _normalize_tn(chanstate[i].hop_begin_cog + ckps.hop_duration);
 }
 _RESTORE_INTERRUPT(_t);
}
#endif

void ckps_set_cogs_num(uint8_t norm_num, uint8_t miss_num)
{
 div_t dr; uint8_t _t;
#ifdef PHASE_SENSOR
 uint16_t err_thrd = (norm_num * 2) + (norm_num >> 3); //+ 12.5%
#endif
 uint16_t cogs_per_chan, degrees_per_cog;

 //precalculate number of cogs per 1 ignition channel, it is fractional number multiplied by 256
 cogs_per_chan = (((uint32_t)(norm_num * 2)) << 8) / ckps.chan_number;

 //precalculate value of degrees per 1 cog, it is fractional number multiplied by ANGLE_MULTIPLAYER
 degrees_per_cog = (((((uint32_t)360) << 8) / norm_num) * ANGLE_MULTIPLAYER) >> 8;

 //precalculate value and round it always to the upper bound,
 //e.g. for 60-2 crank wheel result = 11 (66°), for 36-1 crank wheel result = 7 (70°)
 dr = div(ANGLE_MAGNITUDE(66), degrees_per_cog);

 _t=_SAVE_INTERRUPT();
 _DISABLE_INTERRUPT();
 //calculate number of last cog
 ckps.wheel_last_cog = norm_num - miss_num;
 //set number of teeth (normal and missing)
 ckps.wheel_cogs_num = norm_num;
 ckps.wheel_cogs_nump1 = norm_num + 1;
 ckps.wheel_cogs_numm1 = norm_num - 1;
 ckps.miss_cogs_num = miss_num;
 ckps.wheel_cogs_num2 = norm_num * 2;
 ckps.wheel_cogs_num2p1 = (norm_num * 2) + 1;
 //set other precalculated values
 ckps.wheel_latch_btdc = dr.quot + (dr.rem > 0);
 ckps.degrees_per_cog = degrees_per_cog;
 ckps.cogs_per_chan = cogs_per_chan;
 ckps.start_angle = ckps.degrees_per_cog * ckps.wheel_latch_btdc;
#ifdef PHASE_SENSOR
 cams_set_error_threshold(err_thrd);
#endif
 _RESTORE_INTERRUPT(_t);
}

/** Turn OFF specified ignition channel
 * \param i_channel number of ignition channel to turn off
 */
INLINE
void turn_off_ignition_channel(uint8_t i_channel)
{
 if (!CHECKBIT(flags, F_IGNIEN))
  return; //ignition disabled
 //Completion of igniter's ignition drive pulse, transfer line of port into a low level - makes 
 //the igniter go to the regime of energy accumulation
 //Завершение импульса запуска коммутатора, перевод линии порта в низкий уровень - заставляем
 //коммутатор перейти в режим накопления энергии
 ((iocfg_pfn_set)chanstate[i_channel].io_callback1)(IGN_OUTPUTS_OFF_VAL);
#ifdef PHASED_IGNITION
 ((iocfg_pfn_set)chanstate[i_channel].io_callback2)(IGN_OUTPUTS_OFF_VAL);
#endif
}

/**Forces ignition spark if corresponding interrupt is pending*/
#ifdef _PLATFORM_M644_
#ifdef PHASED_IGNITION
#define force_pending_spark() \
 if ((TIFR1 & _BV(OCF1A)) && (CHECKBIT(flags2, F_CALTIM)))\
 { \
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGN_OUTPUTS_ON_VAL);\
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback2)(IGN_OUTPUTS_ON_VAL);\
 }
#else
#define force_pending_spark() \
 if ((TIFR1 & _BV(OCF1A)) && (CHECKBIT(flags2, F_CALTIM)))\
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGN_OUTPUTS_ON_VAL);
#endif

#else
#ifdef PHASED_IGNITION
#define force_pending_spark() \
 if ((TIFR & _BV(OCF1A)) && (CHECKBIT(flags2, F_CALTIM)))\
 { \
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGN_OUTPUTS_ON_VAL);\
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback2)(IGN_OUTPUTS_ON_VAL);\
 }
#else
#define force_pending_spark() \
 if ((TIFR & _BV(OCF1A)) && (CHECKBIT(flags2, F_CALTIM)))\
  ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGN_OUTPUTS_ON_VAL);
#endif
#endif
/**Interrupt handler for Compare/Match channel A of timer T1
 * вектор прерывания по совпадению канала А таймера Т1
 */
ISR(TIMER1_COMPA_vect)
{
#ifdef COOLINGFAN_PWM
#ifdef _PLATFORM_M644_
 uint8_t timsk_sv0, timsk_sv1;
#else
 uint8_t timsk_sv;
#endif
 uint8_t ucsrb_sv = UCSRB;
#endif

#ifdef DWELL_CONTROL
 ckps.tmrval_saved = TCNT1;
#endif

#ifdef _PLATFORM_M644_
 TIMSK1&= ~_BV(OCIE1A);
#else
 TIMSK&= ~_BV(OCIE1A);  //disable interrupt (запрещаем прерывание)
#endif

 //line of port in the low level, now set it into a high level - makes the igniter to stop 
 //the accumulation of energy and close the transistor (spark)
 //(линия порта в низком уровне, теперь переводим её в высокий уровень - заставляем коммутатор прекратить
 //накопление энергии и закрыть транзистор (искра)).
 if (CKPS_CHANNEL_MODENA == ckps.channel_mode)
  return; //none of channels selected (никакой канал не выбран)

#ifdef STROBOSCOPE
 if (1==ckps.strobe)
 {
  IOCFG_SET(IOP_STROBE, 1); //start pulse
  ckps.strobe = 2;          //and set flag to next state
#ifdef _PLATFORM_M644_
  OCR1A = TCNT1 + 31;
  TIMSK1|= _BV(OCIE1A);
#else
  OCR1A = TCNT1 + 25;       //We will generate 100uS pulse
  TIMSK|= _BV(OCIE1A);      //pulse will be ended in the next interrupt
#endif
 }
 else if (2==ckps.strobe)
 {
  IOCFG_SET(IOP_STROBE, 0); //end pulse
  ckps.strobe = 0;          //and reset flag
  return;
 }
#endif

 ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback1)(IGN_OUTPUTS_ON_VAL);
#ifdef PHASED_IGNITION
 ((iocfg_pfn_set)chanstate[ckps.channel_mode].io_callback2)(IGN_OUTPUTS_ON_VAL);
#endif

 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 //ADCSRA&=~_BV(ADIE);            //??
 UCSRB&=~(_BV(RXCIE)|_BV(UDRIE)); //mask UART interrupts
#ifdef _PLATFORM_M644_
 timsk_sv0 = TIMSK0, timsk_sv1 = TIMSK1;
 TIMSK0=0, TIMSK1=0;              //mask all timer interrupts except OCF2 and TOV2
#else
 timsk_sv = TIMSK;
 TIMSK&= _BV(OCIE2)|_BV(TOIE2);   //mask all timer interrupts except OCF2 and TOV2
#endif
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------

#ifdef DWELL_CONTROL
 ckps.acc_delay = (((uint32_t)ckps.period_curr) * ckps.cogs_per_chan) >> 8;
 if (ckps.cr_acc_time > ckps.acc_delay-120)
  ckps.cr_acc_time = ckps.acc_delay-120;  //restrict accumulation time. Dead band = 500us
 ckps.acc_delay-= ckps.cr_acc_time;    //apply dwell time
 ckps.period_saved = ckps.period_curr; //remember current inter-tooth period

 ckps.channel_mode_b = (ckps.channel_mode < ckps.chan_number-1) ? ckps.channel_mode + 1 : 0 ;
 ckps.channel_mode_b&= ckps.chan_mask;
 CLEARBIT(flags2, F_ADDPTK);
 SETBIT(flags, F_NTSCHB);

 //if less than 2 teeth remains to the accumulation beginning we have to program compare channel in advance, otherwise
 //we may loose spark in some cases
 if (ckps.acc_delay < (ckps.period_curr << 1))
 {
  OCR1B = GetICR() + ckps.acc_delay;
#ifdef _PLATFORM_M644_
  TIFR1 = _BV(OCF1B);
  TIMSK1|= _BV(OCIE1B);
#else
  TIFR = _BV(OCF1B);
  TIMSK|= _BV(OCIE1B);
#endif
 }

 //We remembered value of TCNT1 at the top of of this function. But input capture event
 //may occur when interrupts were already disabled (by hardware) but value of timer is still
 //not saved. Another words, ICR1 must not be less than tmrval_saved.
#ifdef _PLATFORM_M644_
 if (TIFR1 & _BV(ICF1))
#else
 if (TIFR & _BV(ICF1))
#endif
  ckps.tmrval_saved = ICR1;

#else
 //start counting the duration of pulse in the teeth (начинаем отсчет длительности импульса в зубьях)
 chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
#endif

 CLEARBIT(flags2, F_CALTIM); //we already output the spark, so calculation of time is finished
 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 _DISABLE_INTERRUPT();
 //ADCSRA|=_BV(ADIE);
 UCSRB = ucsrb_sv;
#ifdef _PLATFORM_M644_
 TIMSK0 = timsk_sv0, TIMSK1 = timsk_sv1;
#else
 TIMSK = timsk_sv;
#endif
#endif
 //-----------------------------------------------------
}

#ifdef DWELL_CONTROL
/**Interrupt handler for Compare/Match channel B of timer T1. Used for dwell control.
 * вектор прерывания по совпадению канала B таймера Т1. Используется для управления накоплением энергии КЗ.
 */
ISR(TIMER1_COMPB_vect)
{
#ifdef _PLATFORM_M644_
 TIMSK1&= ~_BV(OCIE1B);
#else
 TIMSK&= ~_BV(OCIE1B); //запрещаем прерывание
#endif
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
 TCNT0_H = _AB(value, 1);
 TCNT0 = ~(_AB(value, 0));  //One's complement is faster than 255 - low byte
#ifdef _PLATFORM_M644_
 TCCR0B  = _BV(CS01)|_BV(CS00);
#else
 TCCR0  = _BV(CS01)|_BV(CS00);
#endif
}

/**Helpful function, used at the startup of engine
 * (Вспомогательная функция, используется во время пуска)
 * \return 1 when synchronization is finished, othrewise 0 (1 когда синхронизация окончена, иначе 0)
 */
static uint8_t sync_at_startup(void)
{
 switch(ckps.starting_mode)
 {
  case 0: //skip certain number of teeth (пропуск определенного кол-ва зубьев)
   CLEARBIT(flags, F_VHTPER);
   if (ckps.cog >= CKPS_ON_START_SKIP_COGS)
#ifdef PHASED_IGNITION
    //if cylinder number is even, then cam synchronization will be performed later
    ckps.starting_mode = (ckps.chan_number & 1) ? 1 : 2;
#else
    ckps.starting_mode = 2; //even number of cylinders only
#endif
   break;

#ifdef PHASED_IGNITION
  case 1: //we fall into this state only if number of cylinders is odd
   cams_detect_edge();
   if (cams_is_event_r())
   {
    set_channels_fs(1);     //set full sequential mode
    SETBIT(flags2, F_CAMISS);
    ckps.starting_mode = 2;
   }
   break;
#endif

  case 2: //find out missing teeth (поиск синхрометки)
   //if missing teeth = 0, then reference will be identified by additional VR sensor (REF_S input)
   if (
#ifdef SECU3T
   ((0==ckps.miss_cogs_num) ? cams_vr_is_event_r() : (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)))
#else
   (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev))
#endif
   )
   {
    SETBIT(flags, F_ISSYNC);
    ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period
    ckps.cog = ckps.cog360 = 1; //first tooth (1-й зуб)
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
static void process_ckps_cogs(void)
{
 uint8_t i;
#ifdef _PLATFORM_M644_
 uint8_t timsk_sv0 = TIMSK0, timsk_sv1 = TIMSK1;
#else
 uint8_t timsk_sv = TIMSK;
#endif

 //-----------------------------------------------------
 //Software PWM is very sensitive even to small delays. So, we need to allow OCF2 and TOV2
 //interrupts occur during processing of this handler.
#ifdef COOLINGFAN_PWM
 //remember current state, mask all not urgent interrupts and enable interrupts
 uint8_t ucsrb_sv = UCSRB;
 //ADCSRA&=~_BV(ADIE);            //??
 UCSRB&=~(_BV(RXCIE)|_BV(UDRIE)); //mask UART interrupts
#ifdef _PLATFORM_M644_
 TIMSK0=0,TIMSK1=0;
#else
 TIMSK&= _BV(OCIE2)|_BV(TOIE2);   //mask all timer interrupts except OCF2 and TOV2
#endif
 _ENABLE_INTERRUPT();
#endif
 //-----------------------------------------------------
 force_pending_spark();

#ifdef DWELL_CONTROL
 if (ckps.channel_mode_b != CKPS_CHANNEL_MODENA)
 {
  //We must take into account time elapsed between last spark and following tooth.
  //Because ICR1 can not be less than tmrval_saved we are using addition modulo 65535
  //to calculate difference (elapsed time).
  if (!CHECKBIT(flags2, F_ADDPTK))
  {
   ckps.acc_delay-=((uint16_t)(~ckps.tmrval_saved)) + 1 + GetICR();
   SETBIT(flags2, F_ADDPTK);
  }
  //Correct our prediction on each cog
  ckps.acc_delay+=((int32_t)ckps.period_curr - ckps.period_saved);

  //Do we have to set COMPB ? (We have to set COMPB if less than 2 periods remain)
  if (CHECKBIT(flags, F_NTSCHB) && ckps.acc_delay <= (ckps.period_curr << 1))
  {
   OCR1B = GetICR() + ckps.acc_delay + ((int32_t)ckps.period_curr - ckps.period_saved);
#ifdef _PLATFORM_M644_
   TIFR1 = _BV(OCF1B);
   timsk_sv1|= _BV(OCIE1B);
#else
   TIFR = _BV(OCF1B);
   timsk_sv|= _BV(OCIE1B);
#endif
   CLEARBIT(flags, F_NTSCHB); // To avoid entering into setup mode (чтобы не войти в режим настройки ещё раз)
  }
  ckps.acc_delay-=ckps.period_curr;
 }
#endif

 force_pending_spark();

 for(i = 0; i < ckps.chan_number; ++i)
 {
  if (CHECKBIT(flags, F_USEKNK))
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
    adc_begin_measure_knock(_AB(ckps.stroke_period, 1) < 4);
   }
  }

  //for 66° before TDC (before working stroke) establish new advance angle to be actuated,
  //before this moment value was stored in a temporary buffer.
  //за 66 градусов до в.м.т перед рабочим тактом устанавливаем новый УОЗ для реализации, УОЗ
  //до этого хранился во временном буфере.
  if (ckps.cog == chanstate[i].cogs_latch)
  {
   ckps.channel_mode = (i & ckps.chan_mask); //remember number of channel (запоминаем номер канала)
   SETBIT(flags, F_NTSCHA);                  //establish an indication that it is need to count advance angle (устанавливаем признак того, что нужно отсчитывать УОЗ)
   //start counting of advance angle (начинаем отсчет угла опережения)
   ckps.current_angle = ckps.start_angle; // those same 66° (те самые 66°)
   ckps.advance_angle = ckps.advance_angle_buffered; //advance angle with all the adjustments (say, 15°)(опережение со всеми корректировками (допустим, 15°))
   knock_start_settings_latching();//start the process of downloading the settings into the HIP9011 (запускаем процесс загрузки настроек в HIP)
   adc_begin_measure(_AB(ckps.stroke_period, 1) < 4);//start the process of measuring analog input values (запуск процесса измерения значений аналоговых входов)
#ifdef STROBOSCOPE
   if (0==i)
    ckps.strobe = 1; //strobe!
#endif
  }

  force_pending_spark();

  //teeth of end/beginning of the measurement of rotation period - TDC Read and save the measured period,
  //then remember current value of count for the next measurement
  //(зубья завершения/начала измерения периодов вращения  - в.м.т. считывание и сохранение измеренного периода,
  //затем запоминание текущего значения счетчика для следующего измерения)
  if (ckps.cog==chanstate[i].cogs_btdc)
  {
   //save period value if it is correct
   if (CHECKBIT(flags, F_VHTPER))
   {
    ckps.stroke_period = (GetICR() - ckps.measure_start_value);
    WRITEBIT(flags2, F_SPSIGN, GetICR() < ckps.measure_start_value); //save sign
    ckps.t1oc_s = ckps.t1oc, ckps.t1oc = 0; //save value and reset counter
   }

   ckps.measure_start_value = GetICR();
   SETBIT(flags, F_VHTPER);
   SETBIT(flags, F_STROKE); //set the stroke-synchronozation event (устанавливаем событие тактовой синхронизации)
#ifdef FUEL_INJECT
   inject_start_inj();      //start fuel injection
#endif
  }

#ifdef HALL_OUTPUT
  if (ckps.cog == chanstate[i].hop_begin_cog)
   IOCFG_SET(IOP_HALL_OUT, 1);
  if (ckps.cog == chanstate[i].hop_end_cog)
   IOCFG_SET(IOP_HALL_OUT, 0);
#endif
 }

 force_pending_spark();

 //Preparing to start the ignition for the current channel (if the right moment became)
 //подготовка к запуску зажигания для текущего канала (если наступил нужный момент)
 if (CHECKBIT(flags, F_NTSCHA) && ckps.channel_mode!= CKPS_CHANNEL_MODENA)
 {
  uint16_t diff = ckps.current_angle - ckps.advance_angle;
  if (diff <= (ckps.degrees_per_cog << 1))
  {
   //before starting the ignition it is left to count less than 2 teeth. It is necessary to prepare the compare module
   //(до запуска зажигания осталось отсчитать меньше 2-x зубьев. Необходимо подготовить модуль сравнения)
   //TODO: replace heavy division by multiplication with magic number. This will reduce up to 40uS !
   if (ckps.period_curr < 128)
    OCR1A = GetICR() + ((diff * (ckps.period_curr)) / ckps.degrees_per_cog) - COMPA_VECT_DELAY;
   else
    OCR1A = GetICR() + (((uint32_t)diff * (ckps.period_curr)) / ckps.degrees_per_cog) - COMPA_VECT_DELAY;
#ifdef _PLATFORM_M644_
   TIFR1 = _BV(OCF1A);
   timsk_sv1|= _BV(OCIE1A);   // enable Compare A interrupt (разрешаем прерывание)
#else
   TIFR = _BV(OCF1A);
   timsk_sv|= _BV(OCIE1A);    // enable Compare A interrupt (разрешаем прерывание)
#endif
   CLEARBIT(flags, F_NTSCHA); // For avoiding to enter into setup mode (чтобы не войти в режим настройки ещё раз)
   SETBIT(flags2, F_CALTIM);  // Set indication that we begin to calculate the time
  }
 }

#ifndef DWELL_CONTROL
 //finish the ignition trigger pulses for igniter(s) and immediately increase the number of tooth for processed channel
 //заканчиваем импульсы запуска коммутатора(ов) и сразу увеличиваем номер зуба для обработанного канала
 for(i = 0; i < ckps.chan_number; ++i)
 {
  if (chanstate[i].ignition_pulse_cogs == 255)
   continue;

  if (chanstate[i].ignition_pulse_cogs >= ckps.ignition_cogs)
  {
   turn_off_ignition_channel(i);
   chanstate[i].ignition_pulse_cogs = 255; //set indication that channel has finished to work
  }
  else
   ++(chanstate[i].ignition_pulse_cogs);
 }
#endif

 //tooth passed - angle before TDC decriased (e.g 6° per tooth for 60-2).
 //(прошел зуб - угол до в.м.т. уменьшился (например 6° на зуб для 60-2)).
 ckps.current_angle-= ckps.degrees_per_cog;
 ++ckps.cog;

#ifdef PHASE_SENSOR
 //search for level's toggle from camshaft sensor on each cog
 cams_detect_edge();
#endif
#ifdef PHASED_IGNITION
 force_pending_spark();
 if (cams_is_event_r())
 {
  //Synchronize. We rely that cam sensor event (e.g. falling edge) coming before missing teeth
  if (ckps.cog < ckps.wheel_cogs_nump1)
   ckps.cog+= ckps.wheel_cogs_num;

  //Turn on full sequential mode because Cam sensor is OK
  if (!CHECKBIT(flags2, F_CAMISS))
  {
   set_channels_fs(1);
   SETBIT(flags2, F_CAMISS);
  }
 }
 if (cams_is_error())
 {
  //Turn off full sequential mode because cam semsor is not OK
  if (CHECKBIT(flags2, F_CAMISS))
  {
   set_channels_fs(0); //<--valid only for engines with even number of cylinders
   CLEARBIT(flags2, F_CAMISS);
  }
 }
#endif

 //-----------------------------------------------------
#ifdef COOLINGFAN_PWM
 //disable interrupts and restore previous states of masked interrupts
 _DISABLE_INTERRUPT();
 //ADCSRA|=_BV(ADIE);
 UCSRB = ucsrb_sv;
#endif
#ifdef _PLATFORM_M644_
 TIMSK0 = timsk_sv0, TIMSK1 = timsk_sv1;
#else
 TIMSK = timsk_sv;
#endif
 //-----------------------------------------------------

 force_pending_spark();
}

/**Input capture interrupt of timer 1 (called at passage of each tooth)
 * прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
 */
ISR(TIMER1_CAPT_vect)
{
 force_pending_spark();

 ckps.period_curr = GetICR() - ckps.icr_prev;

 //At the start of engine, skipping a certain number of teeth for initializing
 //the memory of previous periods. Then look for missing teeth.
 //при старте двигателя, пропускаем определенное кол-во зубьев для инициализации
 //памяти предыдущих периодов. Затем ищем синхрометку.
 if (!CHECKBIT(flags, F_ISSYNC))
 {
  if (sync_at_startup())
   goto sync_enter;
  return;
 }

 //if missing teeth = 0, then reference will be identified by additional VR sensor (REF_S input),
 //Otherwise:
 //Each period, check for missing teeth, and if, after discovering of missing teeth
 //count of teeth being found incorrect, then set error flag.
 //(каждый период проверяем на синхрометку, и если после обнаружения синхрометки
 //оказалось что кол-во зубьев неправильное, то устанавливаем признак ошибки).
#ifdef SECU3T
 if ((0==ckps.miss_cogs_num) ? cams_vr_is_event_r() : (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)))
#else
 if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev))
#endif
 {
  if ((ckps.cog360 != ckps.wheel_cogs_nump1)) //also taking into account recovered teeth (учитываем также восстановленные зубья)
  {
   SETBIT(flags, F_ERROR); //ERROR
   ckps.cog = 1;
   //TODO: maybe we need to turn off full sequential mode
  }
  //Reset 360° tooth counter to the first tooth (1-й зуб)
  ckps.cog360 = 1;
  //Also reset 720° tooth counter
  if (ckps.cog == ckps.wheel_cogs_num2p1)
   ckps.cog = 1;
  ckps.period_curr = ckps.period_prev;  //exclude value of missing teeth's period (for missing teeth only)
 }

sync_enter:
 //If the last tooth before missing teeth, we begin the countdown for
 //the restoration of missing teeth, as the initial data using the last
 //value of inter-teeth period.
 //(Если последний зуб перед синхрометкой, то начинаем отсчет времени для
 //восстановления отсутствующих зубьев, в качестве исходных данных используем
 //последнее значение межзубного периода).
 if (ckps.miss_cogs_num && ckps.cog360 == ckps.wheel_last_cog)
  set_timer0(ckps.period_curr);

 //call handler for normal teeth (вызываем обработчик для нормальных зубьев)
 process_ckps_cogs();
 ++ckps.cog360;

 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;

 force_pending_spark();
}

/**Purpose of this interrupt handler is to supplement timer up to 16 bits and call procedure
 * for processing teeth when set 16 bit timer expires
 * (Задача этого обработчика дополнять таймер до 16-ти разрядов и вызывать процедуру
 * обработки зубьев по истечении установленного 16-ти разрядного таймера). */
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
#ifdef _PLATFORM_M644_
  TCCR0B = 0;
#else
  TCCR0 = 0;     //stop timer (останавливаем таймер)
#endif

  if (ckps.miss_cogs_num > 1)
  {
   //start timer to recover 60th tooth (запускаем таймер чтобы восстановить 60-й (последний) зуб)
   if (ckps.cog360 == ckps.wheel_cogs_numm1)
    set_timer0(ckps.period_curr);
  }

  //Call handler for missing teeth (вызываем обработчик для отсутствующих зубьев)
  process_ckps_cogs();
  ++ckps.cog360;
 }
}

/** Timer 1 overflow interrupt.
 * Used to count timer 1 overflows to obtain correct revolution period at very low RPMs (10...400 min-1)
 */
ISR(TIMER1_OVF_vect)
{
 ++ckps.t1oc;
}

#endif //!HALL_SYNC && !CKPS_2CHIGN
