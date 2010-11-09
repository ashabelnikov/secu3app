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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#include <inavr.h>
#include <ioavr.h>
#include "ckps.h"
#include "bitmask.h"
#include "adc.h"
#include "magnitude.h"

#include "secu3.h"  
#include "knock.h"

//#ifdef COIL_REGULATION todo: implement

#ifndef WHEEL_36_1 //60-2
 #define WHEEL_COGS_NUM   60  //количество зубьев (включая отсутствующие)
 #define WHEEL_COGS_LACK  2   //количество отсутствующих зубьев
 #define WHEEL_LATCH_BTDC 11  //кол-во зубьев до в.м.т определяющие момент загрузки УОЗ, старт измерения датчиков, загрузку настроек в HIP  
 #define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p)>>1))  // p * 2.5,  барьер для селекции синхрометки = 2.5
#else //36-1
 #define WHEEL_COGS_NUM   36  
 #define WHEEL_COGS_LACK  1   
 #define WHEEL_LATCH_BTDC 7   //70 градусов  
 #define CKPS_GAP_BARRIER(p) ((p) + ((p)>>1))  // p * 1.5
#endif
  
//количество градусов приходящееся на один зуб диска
#define CKPS_DEGREES_PER_COG (360 / WHEEL_COGS_NUM)                           
    
//номер последнего (существующего) зуба, нумерация ничинается с 1! 
#define WHEEL_LAST_COG (WHEEL_COGS_NUM - WHEEL_COGS_LACK)

//количество зубов которое будет пропускатся при старте перед синхронизацией
#define CKPS_ON_START_SKIP_COGS      30

#define GetICR() (ICR1)

//максимум 4 канала зажигания.
#define IGN_CHANNELS_MAX     4

//используется для указания что ни один канал зажигания не выбран
#define CKPS_CHANNEL_MODENA  255

typedef struct
{
 uint8_t  ckps_error_flag:1;                   //признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки
 uint8_t  ckps_is_valid_half_turn_period:1;
 uint8_t  ckps_is_synchronized:1;
 uint8_t  ckps_new_engine_cycle_happen:1;      //флаг синхронизации с вращением  
 uint8_t  ckps_use_knock_channel:1;            //признак использования канала детонации
 uint8_t  ckps_need_to_set_channel:1;                      
}ckpsflags_t;

typedef struct
{
 uint16_t icr_prev;                   //предыдущее значение регистра захвата
 volatile uint16_t period_curr;       //последнй измеренный межзубный период
 uint16_t period_prev;                //предыдущее значение межзубного периода
 volatile uint8_t  cog;               //считает зубья после выреза, начинает считать с 1
 uint16_t measure_start_value;        //запоминает значение регистра захвата для измерения периода полуоборота
 uint16_t current_angle;              //отсчитывает заданный УОЗ при прохождении каждого зуба
 volatile uint16_t half_turn_period;  //хранит последнее измерение времени прохождения n зубьев  
 int16_t  advance_angle;              //требуемый УОЗ * ANGLE_MULTIPLAYER
 volatile int16_t advance_angle_buffered;     
 uint8_t  ignition_cogs;              //кол-во зубьев определяющее длительность импульсов запуска коммутаторов
 uint8_t  starting_mode;              //состояние конечного автомата обработки зубьев на пуске
 uint8_t  channel_mode;               //определяет какой канал зажигания нужно запускать в данный момент
 volatile uint8_t  cogs_btdc;         //кол-во зубьев от синхрометки до в.м.т первого цилиндра 
 int8_t   knock_wnd_begin_abs;        //начало окна фазовой селекции детонации в зубьях шкива относительно в.м.т 
 int8_t   knock_wnd_end_abs;          //конец окна фазовой селекции детонации в зубьях шкива относительно в.м.т  
 volatile uint8_t chan_number;        //кол-во каналов зажигания
 uint32_t frq_calc_dividend;          //делимое для расчета частоты вращения
}ckpsstate_t;
 
//Предрасчитанные данные(опорные точки) и данные состояния для отдельного канала зажигания (пара цилиндров)
//2ц: cylstate_t[0]
//4ц: cylstate_t[0], cylstate_t[1]
//6ц: cylstate_t[0], cylstate_t[1], cylstate_t[2]
//8ц: cylstate_t[0], cylstate_t[1], cylstate_t[2], cylstate_t[3]
typedef struct
{
 volatile uint8_t ignition_pulse_cogs;//отсчитывает зубья импульса зажигания для цилиндров 1-4
 volatile uint8_t cogs_latch;         //определяет номер зуба (относительно в.м.т.) на котором происходит "защелкивание" данных
 volatile uint8_t cogs_btdc;          //определяет номер зуба на котором производится измерение периода вращения коленвала (между раб. циклами)
 volatile uint8_t knock_wnd_begin;    //определяет номер зуба на котором открывается окно фазовой селекции сигнала ДД (начало интегрирования) 
 volatile uint8_t knock_wnd_end;      //определяет номер зуба на котором закрывается окно фазовой селекции сигнала ДД (конец интегрирования)
}chanstate_t[IGN_CHANNELS_MAX]; 
  
ckpsstate_t ckps;
chanstate_t chanstate;

//размещаем в свободных регистрах ввода/вывода
__no_init volatile ckpsflags_t flags@0x22;

//для дополнения таймера/счетчика 0 до 16 разрядов, используем R15
__no_init __regvar uint8_t TCNT0_H@15;

//Инициализирует переменные состояния ДПКВ
__monitor
void ckps_init_state_variables(void)
{
 //при первом же прерывании будет сгенерирован конец импульса запуска зажигания для цилиндров 1-4   
 uint8_t i;
 for(i = 0; i < IGN_CHANNELS_MAX; i++)
  chanstate[i].ignition_pulse_cogs = 0;

 ckps.cog = 0; 
 ckps.half_turn_period = 0xFFFF;                 
 ckps.advance_angle = 0;
 ckps.advance_angle_buffered = 0;
 ckps.starting_mode = 0;
 ckps.channel_mode = CKPS_CHANNEL_MODENA;
 
 flags.ckps_need_to_set_channel = 0;   
 flags.ckps_new_engine_cycle_happen = 0;
 flags.ckps_is_synchronized = 0;  
 TCCR0 = 0; //останавливаем таймер0  
}

//инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится 
__monitor
void ckps_init_state(void)
{
 ckps_init_state_variables();
 flags.ckps_error_flag = 0; 
  
 //Каналы Compare неподключены к линиям портов (нормальный режим портов) 
 TCCR1A = 0; 
  
 //подавление шума, передний фронт захвата, clock = 250kHz
 TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11)|(1<<CS10);       

 //разрешаем прерывание по захвату и сравнению А таймера 1, а также по переполнению таймера 0
 TIMSK|= (1<<TICIE1)/*|(1<<OCIE1A)*/|(1<<TOIE0);
}

//устанавливает УОЗ для реализации в алгоритме
__monitor
void ckps_set_advance_angle(int16_t angle)
{
 ckps.advance_angle_buffered = angle;
}

void ckps_init_ports(void)
{ 
 //после включения зажигания коммутаторы недолжны быть в режиме накопления,
 //поэтому устанавливаем на их входах низкий уровень.
#ifndef INVERSE_IGN_OUTPUTS
 PORTD|= (1<<PD5)|(1<<PD4)|(1<<PD6); //1-й и 2-й каналы зажигания, подтяжка для ICP1
 PORTC|= (1<<PC1)|(1<<PC0); //3-й и 4-й каналы зажигания
#else //режим инверсии выходов
 PORTD&= ~((1<<PD5)|(1<<PD4));
 PORTC&= ~((1<<PC1)|(1<<PC0));
 PORTD|=  (1<<PD6); 
#endif 

 //PD5,PD4,PC1,PC0 должны быть сконфигурированы как выходы
 DDRD|= (1<<DDD5)|(1<<DDD4); //1-2 каналы зажигания (для 2 и 4 ц. двигателей)
 DDRC|= (1<<DDC1)|(1<<DDC0); //3-4 каналы зажигания (для 6 и 8 ц. двигателей)
}

//Высчитывание мгновенной частоты вращения коленвала по измеренному времени прохождения 30 зубьев шкива.
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс, значит:
uint16_t ckps_calculate_instant_freq(void)
{
 uint16_t period;
 __disable_interrupt();
 period = ckps.half_turn_period;           //обеспечиваем атомарный доступ к переменной  
 __enable_interrupt();                           

 //если самый минимум, значит двигатель остановился 
 if (period!=0xFFFF)  
  return (ckps.frq_calc_dividend)/(period);
 else
  return 0;
}

//устанавливает тип фронта ДПКВ (0 - отрицательный, 1 - положительный)
__monitor
void ckps_set_edge_type(uint8_t edge_type)
{
 if (edge_type)
  TCCR1B|= (1<<ICES1);
 else
  TCCR1B&=~(1<<ICES1);
}

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
 // заранее вычисляем и сохраняем опорные точки (зубья)
 // cogs_per_cycle - количество зубьев шкива приходящееся на один такт двигателя 
 uint8_t cogs_per_cycle = (WHEEL_COGS_NUM) / ckps.chan_number;
  _t=__save_interrupt();
 __disable_interrupt();
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (cogs_btdc + i * cogs_per_cycle);
  chanstate[i].cogs_btdc = _normalize_tn(tdc);
  chanstate[i].cogs_latch = _normalize_tn(tdc - WHEEL_LATCH_BTDC);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs); 
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs); 
 }
 ckps.cogs_btdc = cogs_btdc;
 __restore_interrupt(_t); 
}

//устанавливает длительность импульса зажигания в зубьях
__monitor
void ckps_set_ignition_cogs(uint8_t cogs)
{
 ckps.ignition_cogs = cogs;
}

uint8_t ckps_is_error(void)
{
 return flags.ckps_error_flag;
}

void ckps_reset_error(void)
{
 flags.ckps_error_flag = 0;
}

void ckps_use_knock_channel(uint8_t use_knock_channel)
{
 flags.ckps_use_knock_channel = use_knock_channel;
}

//эта функция возвращает 1 если был новый цикл зажигания и сразу сбрасывает событие!
__monitor
uint8_t ckps_is_cycle_cutover_r()
{
 uint8_t result;
 result = flags.ckps_new_engine_cycle_happen;
 flags.ckps_new_engine_cycle_happen = 0;
 return result;
}

__monitor
uint8_t ckps_get_current_cog(void)
{
 return ckps.cog;
}

__monitor
uint8_t ckps_is_cog_changed(void)
{
 static uint8_t prev_cog = 0;
 if (prev_cog!=ckps.cog)
 {
  prev_cog = ckps.cog; 
  return 1;
 }
 return 0;
}

__monitor
void ckps_set_cyl_number(uint8_t i_cyl_number)
{
 ckps.chan_number = i_cyl_number >> 1; //один канал зажигания на 2 цилиндра
 
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
 //переводим из градусов в зубья
 ckps.knock_wnd_begin_abs = begin / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);
 ckps.knock_wnd_end_abs = end / (CKPS_DEGREES_PER_COG * ANGLE_MULTIPLAYER);

 cogs_per_cycle = (WHEEL_COGS_NUM) / ckps.chan_number;
 _t=__save_interrupt();
 __disable_interrupt(); 
 for(i = 0; i < ckps.chan_number; ++i)
 {
  uint8_t tdc = (ckps.cogs_btdc + i * cogs_per_cycle);
  chanstate[i].knock_wnd_begin = _normalize_tn(tdc + ckps.knock_wnd_begin_abs); 
  chanstate[i].knock_wnd_end = _normalize_tn(tdc + ckps.knock_wnd_end_abs); 
 }  
 __restore_interrupt(_t);
}

//Вспомогательный макрос
//конец импульса накачки (момент искры) для 1-го,2-го,3-го,4-го каналов соответственно
#define TURNON_IGN_CHANNELS(){\
  case 0: PORTD |= (1<<PORTD4);\
   break;\
  case 1: PORTD |= (1<<PORTD5);\
   break;\
  case 2: PORTC |= (1<<PORTC0);\
   break;\
  case 3: PORTC |= (1<<PORTC1);\
   break;}

//Вспомогательный макрос
//конец импульса запуска зажигания для 1-го,2-го,3-го,4-го каналов соответственно
#define TURNOFF_IGN_CHANNELS(){\
 case 0: PORTD &= ~(1<<PORTD4);\
  break;\
 case 1: PORTD &= ~(1<<PORTD5);\
  break;\
 case 2: PORTC &= ~(1<<PORTC0);\
  break;\
 case 3: PORTC &= ~(1<<PORTC1);\
  break;}  

#pragma vector=TIMER1_COMPA_vect //вектор прерывания по совпадению канала А таймера Т1
__interrupt void timer1_compa_isr(void)
{
  TIMSK&= ~(1<<OCIE1A); //запрещаем прерывание

 //линия порта в низком уровне, теперь переводим её в высокий уровень - заставляем коммутатор прекратить 
 //накопление энергии и закрыть транзистор (искра).
 switch(ckps.channel_mode)
 {
#ifndef INVERSE_IGN_OUTPUTS 
  TURNON_IGN_CHANNELS();
#else
  TURNOFF_IGN_CHANNELS();
#endif   
  default:
   return; //никакой канал не выбран - CKPS_CHANNEL_MODENA
 }
 //начинаем отсчет длительности импульса в зубьях
 chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
}

#pragma inline
void turn_off_ignition_channel(uint8_t i_channel)
{
 //Завершение импульса запуска коммутатора, перевод линии порта в низкий уровень - заставляем
 //коммутатор перейти в режим накопления энегрии
 switch(i_channel)
 {
#ifndef INVERSE_IGN_OUTPUTS 
  TURNOFF_IGN_CHANNELS();
#else
  TURNON_IGN_CHANNELS();
#endif 
 }
}
  
//Инициализация таймера 0 указанным значением и запуск, clock = 250kHz.
//Предполагается что вызов этой функции будет происходить при запрещенных прерываниях.
#pragma inline
void set_timer0(uint16_t value)
{                            
 TCNT0_H = GETBYTE(value, 1);            
 TCNT0 = 255 - GETBYTE(value, 0);      
 TCCR0  = (1<<CS01)|(1<<CS00);
}    

//Вспомогательная функция, используется во время пуска
//возвращает 1 когда синхронизация окончена, иначе 0.
uint8_t sync_at_startup(void) 
{
 switch(ckps.starting_mode)
 {
  case 0: //пропуск определенного кол-ва зубьев
   /////////////////////////////////////////
   flags.ckps_is_valid_half_turn_period = 0;
   /////////////////////////////////////////
   if (ckps.cog >= CKPS_ON_START_SKIP_COGS) 
    ckps.starting_mode = 1;
   break;    
  case 1: //поиск синхрометки
   if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)) 
   {
    flags.ckps_is_synchronized = 1;
    ckps.cog = 1; //1-й зуб
    return 1; //конец процесса синхронизации
   }
   break;
  }
 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;  
 ++ckps.cog; 
 return 0; //продолжение процесса синхронизации
} 

//Процедура. Вызывается для всех зубьев шкива (включительно с восстановленными)
void process_ckps_cogs(void)
{
 uint16_t diff; 
 uint8_t i;  

#ifdef VENTILATOR_PWM
 //CKP processing creates a big delay which negatively affects ventilator's PWM. We
 //need to enable T/C 2 interrupts. TODO: it is bad idea to enable all interrupts 
 //here. We need only OCIE2 and TOIE2.
 __enable_interrupt();
#endif 

 if (flags.ckps_use_knock_channel)
 {
  for(i = 0; i < ckps.chan_number; ++i)
  {
   //начинаем слушать детонацию (открытие окна)
   if (ckps.cog == chanstate[i].knock_wnd_begin)
    knock_set_integration_mode(KNOCK_INTMODE_INT);
      
   //заканчиваем слушать детонацию (закрытие окна) и запускаем процесс измерения 
   if (ckps.cog == chanstate[i].knock_wnd_end)
   {
    knock_set_integration_mode(KNOCK_INTMODE_HOLD); 
    adc_begin_measure_knock(); 
   }
  }  
 }
  
 for(i = 0; i < ckps.chan_number; ++i)
 {
  //за 66 градусов до в.м.т перед рабочим циклом устанавливаем новый УОЗ для реализации, УОЗ
  //до этого хранился во временном буфере.
  if (ckps.cog == chanstate[i].cogs_latch)
  {
   ckps.channel_mode = i;          //запоминаем номер канала
   flags.ckps_need_to_set_channel = 1; //устанавливаем признак того, что нужно отсчитывать УОЗ
   //начинаем отсчет угла опережения
   ckps.current_angle = ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * WHEEL_LATCH_BTDC; // те самые 66°
   ckps.advance_angle = ckps.advance_angle_buffered; // опережение со всеми корректировками (допустим, 15°)
   knock_start_settings_latching();//запускаем процесс загрузки настроек в HIP  
   adc_begin_measure();            //запуск процесса измерения значений аналоговых входов    
  }
  
  //зубья завершения/начала измерения периодов вращения  - в.м.т. считывание и сохранение измеренного периода, 
  //затем запоминаение текущего значения счетчика для следующего измерения 
  if (ckps.cog==chanstate[i].cogs_btdc) 
  { 
   //если было переполнение то устанавливаем максимально возможное время
   if (((ckps.period_curr > 1250) || !flags.ckps_is_valid_half_turn_period))
    ckps.half_turn_period = 0xFFFF;
   else       
    ckps.half_turn_period = (GetICR() - ckps.measure_start_value);
  
   ckps.measure_start_value = GetICR();
   flags.ckps_is_valid_half_turn_period = 1;
   /////////////////////////////////////////
   flags.ckps_new_engine_cycle_happen = 1; //устанавливаем событие цикловой синхронизации 
   /////////////////////////////////////////
  }  
 }  
 
 //подготовка к запуску зажигания для текущего канала (если наступил нужный момент)
 if (flags.ckps_need_to_set_channel && ckps.channel_mode!= CKPS_CHANNEL_MODENA)
 {
  diff = ckps.current_angle - ckps.advance_angle;
  if (diff <= (ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * 2))
  {
   //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
   //TODO: replace heavy division by multiplication with magic number. This will reduce up to 40uS !
   OCR1A = GetICR() + ((uint32_t)diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);
   TIFR = (1 << OCF1A);
   chanstate[ckps.channel_mode].ignition_pulse_cogs = 0;
   flags.ckps_need_to_set_channel = 0; // чтобы не войти в режим настройки ещё раз
   TIMSK|= (1<<OCIE1A); //разрешаем прерывание
  }
 }

 //заканчиваем импульсы запуска коммутатора(ов) и сразу увеличиваем номер зуба для обработанного канала
 for(i = 0; i < ckps.chan_number; ++i)
 {
  if (chanstate[i].ignition_pulse_cogs >= ckps.ignition_cogs)
   turn_off_ignition_channel(i);  
  ++(chanstate[i].ignition_pulse_cogs);
 }

 //прошел зуб - угол до в.м.т. уменьшился на 6 град.
 ckps.current_angle-= ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);  
 ++ckps.cog;
}

//прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
#pragma vector=TIMER1_CAPT_vect
__interrupt void timer1_capt_isr(void)
{    
 ckps.period_curr = GetICR() - ckps.icr_prev;

 //при старте двигателя, пропускаем определенное кол-во зубьев для инициализации 
 //памяти предыдущих периодов. Затем ищем синхрометку.
 if (!flags.ckps_is_synchronized)
 {
  if (sync_at_startup())
   goto synchronized_enter;   
  return;
 }

 //каждый период проверяем на синхрометку, и если после обнаружения синхрометки
 //оказалось что кол-во зубьев неправильное, то устанавливаем признак ошибки.
 if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)) 
 {
  if ((ckps.cog != (WHEEL_COGS_NUM + 1))) //учитываем также восстановленные зубья
   flags.ckps_error_flag = 1; //ERROR               
  ckps.cog = 1; //1-й зуб           
 }
  
synchronized_enter:   
 //Если последний зуб перед синхрометкой, то начинаем отсчет времени для 
 //восстановления отсутствующих зубов, в качестве исходных данных используем 
 //последнее значение межзубного периода.
 if (ckps.cog == WHEEL_LAST_COG)
  set_timer0(ckps.period_curr); 
    
 //вызываем обработчик для нормальных зубьев
 process_ckps_cogs(); 
  
 ckps.icr_prev = GetICR();
 ckps.period_prev = ckps.period_curr;  
}

//Задача этого обработчика дополнять таймер до 16-ти разрядов и вызывать процедуру
//обработки зубьев по истечении установленного 16-ти разряюного таймера.                  
#pragma vector=TIMER0_OVF_vect
__interrupt void timer0_ovf_isr(void)
{
 if (TCNT0_H!=0)  //старший байт не исчерпан ?
 {
  TCNT0 = 0;
  --TCNT0_H;         
 }  
 else  
 {//отсчет времени закончился    
  TCCR0 = 0; //останавливаем таймер
 
#ifndef WHEEL_36_1 //60-2
  //запускаем таймер чтобы восстановить 60-й зуб
  if (ckps.cog == 59)
   set_timer0(ckps.period_curr);
#endif
 
  //вызываем обработчик для отсутствующих зубьев
  process_ckps_cogs();
 }
}
