 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <inavr.h>
#include <iom16.h>
#include "ckps.h"
#include "bitmask.h"
#include "adc.h"
#include "magnitude.h"

#include "secu3.h"  

// p * 2.5,  барьер для селекции синхрометки = 2.5 
#define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p)>>1))  

#define GetICR() (ICR1)

#define CKPS_CHANNEL_MODE14  0
#define CKPS_CHANNEL_MODE23  1
#define CKPS_CHANNEL_MODENA  2

typedef struct
{
  unsigned char  ckps_error_flag:1;                   //признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки
  unsigned char  ckps_is_valid_half_turn_period:1;
  unsigned char  ckps_is_synchronized:1;
  unsigned char  ckps_new_engine_cycle_happen:1;      //флаг синхронизации с вращением  
}CKPSFLAGS;

typedef struct
{
  unsigned int  icr_prev;                   //предыдущее значение регистра захвата
  unsigned int  period_curr;                //последнй измеренный межзубный период
  unsigned int  period_prev;                //предыдущее значение межзубного периода
  unsigned char cog;                        //считает зубья после выреза, начинает считать с 1
  unsigned int  measure_start_value;        //запоминает значение регистра захвата для измерения периода полуоборота
  unsigned int  current_angle;              //отсчитывает заданный УОЗ при прохождении каждого зуба
  unsigned char ignition_pulse_cogs_14;     //отсчитывает зубья импульса зажигания для цилиндров 1-4
  unsigned char ignition_pulse_cogs_23;     //отсчитывает зубья импульса зажигания для цилиндров 2-3
  unsigned int  half_turn_period;           //хранит последнее измерение времени прохождения n зубьев  
  signed   int  advance_angle;              //требуемый УОЗ * ANGLE_MULTIPLAYER
  signed   int  advance_angle_buffered;
  unsigned char ignition_cogs;              //кол-во зубьев определяющее длительность импульсов запуска коммутаторов
  unsigned char cogs_latch14;
  unsigned char cogs_latch23;
  unsigned char cogs_btdc14;
  unsigned char cogs_btdc23;
  unsigned char starting_mode;
  unsigned char channel_mode;
}CKPSSTATE;
 
CKPSSTATE ckps;

//размещаем в свободных регистрах ввода/вывода
__no_init volatile CKPSFLAGS flags@0x22;

//для дополнения таймера/счетчика 0 до 16 разрядов, используем R15
__no_init __regvar unsigned char TCNT0_H@15;

//Инициализирует переменные состояния ДПКВ
__monitor
void ckps_init_state_variables(void)
{
  ckps.cog = 0;
  //при первом же прерывании будет сгенерирован конец импульса запуска зажигания для цилиндров 1-4
  ckps.ignition_pulse_cogs_14 = 128; 
  ckps.ignition_pulse_cogs_23 = 128; 
  ckps.half_turn_period = 0xFFFF;                 
  ckps.advance_angle = 0;
  ckps.advance_angle_buffered = 0;
  ckps.starting_mode = 0;
  ckps.channel_mode = CKPS_CHANNEL_MODENA;
  
  flags.ckps_error_flag = 0;
  flags.ckps_new_engine_cycle_happen = 0;
  flags.ckps_is_synchronized = 0;
  TCCR0 = 0; //останавливаем таймер0  
}


//инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится 
__monitor
void ckps_init_state(void)
{
  ckps_init_state_variables(); 
  
  //OC1А(PD5) и OC1В(PD4) должны быть сконфигурированы как выходы
  DDRD|= (1<<DDD5)|(1<<DDD4); 

  //при совпадении будет устанавливатся низкий уровень и заставляем его установиться прямо сейчас 
  TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<FOC1A)|(1<<FOC1B); 
  
  //подавление шума, передний фронт захвата, clock = 250kHz
  TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11)|(1<<CS10);       

  //разрешаем прерывание по захвату и сравнению А и В таймера 1, а также по переполнению таймера 0
  TIMSK|= (1<<TICIE1)|(1<<OCIE1A)|(1<<OCIE1B)|(1<<TOIE0);
}

//устанавливает УОЗ для реализации в алгоритме
__monitor
void ckps_set_dwell_angle(signed int angle)
{
  ckps.advance_angle_buffered = angle;
}

//Высчитывание мгновенной частоты вращения коленвала по измеренному времени прохождения 30 зубьев шкива.
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс, значит:
unsigned int ckps_calculate_instant_freq(void)
{
  unsigned int period;
  __disable_interrupt();
   period = ckps.half_turn_period;           //обеспечиваем атомарный доступ к переменной  
  __enable_interrupt();                           

  //если самый минимум, значит двигатель остановился 
  if (period!=0xFFFF)  
    return (7500000L)/(period);
  else
    return 0;
}

//устанавливает тип фронта ДПКВ (0 - отрицательный, 1 - положительный)
__monitor
void ckps_set_edge_type(unsigned char edge_type)
{
  if (edge_type)
    TCCR1B|= (1<<ICES1);
  else
    TCCR1B&=~(1<<ICES1);
}

__monitor
void ckps_set_cogs_btdc(unsigned char cogs_btdc)
{
  //11 зубьев = 66 град. до в.м.т. 
  ckps.cogs_latch14 = cogs_btdc - 11;
  ckps.cogs_latch23 = cogs_btdc + 19;
  ckps.cogs_btdc14  = cogs_btdc;
  ckps.cogs_btdc23  = cogs_btdc + 30;
}

//устанавливает длительность импульса зажигания в зубьях
__monitor
void ckps_set_ignition_cogs(unsigned char cogs)
{
  ckps.ignition_cogs = cogs;
}

unsigned char ckps_is_error(void)
{
 return flags.ckps_error_flag;
}

void ckps_reset_error(void)
{
 flags.ckps_error_flag = 0;
}

//эта функция возвращает 1 если был новый цикл зажигания и сразу сбрасывает событие!
__monitor
unsigned char ckps_is_cycle_cutover_r()
{
 unsigned char result;
 result = flags.ckps_new_engine_cycle_happen;
 flags.ckps_new_engine_cycle_happen = 0;
 return result;
}

__monitor
unsigned char ckps_get_current_cog(void)
{
 return ckps.cog;
}

__monitor
unsigned char ckps_is_cog_changed(void)
{
 static unsigned char prev_cog = 0;
 if (prev_cog!=ckps.cog)
  {
  prev_cog = ckps.cog; 
  return 1;
  }
 return 0;
}

#pragma vector=TIMER1_COMPA_vect
__interrupt void timer1_compa_isr(void)
{
 //линия в высоком уровне, теперь настраиваем линию на переход в низкий уровень по следующему событию.
 //Начинаем отсчет длительности импульса по зубъям  
 TCCR1A&= (~(1<<COM1A0));
 TCCR1A|= (1<<COM1A1);   
 ckps.ignition_pulse_cogs_23 = 0;
}

#pragma vector=TIMER1_COMPB_vect
__interrupt void timer1_compb_isr(void)
{
 //линия в высоком уровне, теперь настраиваем линию на переход в низкий уровень по следующему событию.  
 //Начинаем отсчет длительности импульса по зубъям
 TCCR1A&= (~(1<<COM1B0));
 TCCR1A|= (1<<COM1B1);   
 ckps.ignition_pulse_cogs_14 = 0;
}

//сбрасывает флаг прерывания, включает режим установки линии B в высокий уровень при совпадении
#define prepare_channel14(value)     \
  {                                  \
   OCR1B = value;                    \
   SETBIT(TIFR,OCF1B);               \
   TCCR1A|= (1<<COM1B1)|(1<<COM1B0); \
   }

//сбрасывает флаг прерывания, включает режим установки линии A в высокий уровень при совпадении
#define prepare_channel23(value)     \
  {                                  \
   OCR1A = value;                    \
   SETBIT(TIFR,OCF1A);               \
   TCCR1A|= (1<<COM1A1)|(1<<COM1A0); \
  }
  
//Инициализация таймера 0 указанным значением и запуск, clock = 250kHz.
//Предполагается что вызов этой функции будет происходить при запрещенных прерываниях.
#pragma inline
void set_timer0(unsigned int value)
 {                            
  TCNT0_H = GETBYTE(value, 1);            
  TCNT0 = 255 - GETBYTE(value, 0);      
  TCCR0  = (1<<CS01)|(1<<CS00);
 }    

//Вспомогательная функция, используется во время пуска
//возвращает 1 когда синхронизация окончена, иначе 0.
unsigned char sync_at_startup(void) 
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
 ckps.cog++; 
 return 0; //продолжение процесса синхронизации
} 

//Процедура. Вызывается для зубьев шкива 1-60, включительно (58 + 2 восстановленных)
void process_ckps_cogs(void)
{
  unsigned int diff;
  
  //за 66 градусов до в.м.т перед рабочим циклом устанавливаем новый УОЗ для реализации, УОЗ
  //до этого хранился во временном буфере.
  if (ckps.cog == ckps.cogs_latch14)
  {
   //начинаем отсчет угла опережения
   ckps.current_angle = ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * 11;
   ckps.advance_angle = ckps.advance_angle_buffered;
   ckps.channel_mode = CKPS_CHANNEL_MODE14;
   //knock_start_settings_latching();    nearest future!!!
   adc_begin_measure();            //запуск процесса измерения значений аналоговых входов   
  }
  if (ckps.cog == ckps.cogs_latch23)
  {
   //начинаем отсчет угла опережения
   ckps.current_angle = ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * 11;
   ckps.advance_angle = ckps.advance_angle_buffered;
   ckps.channel_mode = CKPS_CHANNEL_MODE23;
   //knock_start_settings_latching();    nearest future!!!
   adc_begin_measure();            //запуск процесса измерения значений аналоговых входов
  }

  
  //диаметральные зубья завершения/начала измерения периодов вращения  - в.м.т.   
  //считывание и сохранение измеренного периода, затем запоминаение текущего значения счетчика
  //для следующего измерения 
  if (ckps.cog==ckps.cogs_btdc14 || ckps.cog==ckps.cogs_btdc23) 
  {
   //если было переполнение то устанавливаем максимально возможное время
   if (((ckps.period_curr > 1250) || !flags.ckps_is_valid_half_turn_period))
    ckps.half_turn_period = 0xFFFF;
   else
    {     
     ckps.half_turn_period = (GetICR() - ckps.measure_start_value);
    }
   ckps.measure_start_value = GetICR();
   flags.ckps_is_valid_half_turn_period = 1;
   /////////////////////////////////////////
   flags.ckps_new_engine_cycle_happen = 1; //устанавливаем событие цикловой синхронизации 
   /////////////////////////////////////////
  }

  //подготовка к запуску зажигания для текущего канала
 switch(ckps.channel_mode)
  {
  case CKPS_CHANNEL_MODE14:
   diff = ckps.current_angle - ckps.advance_angle;
   if (diff <= (ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * 2))
   {
   //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
   prepare_channel14(GetICR() + ((unsigned long)diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG));       
   ckps.ignition_pulse_cogs_14 = 0;   
   ckps.channel_mode = CKPS_CHANNEL_MODENA;
   }
   break;
   
  case CKPS_CHANNEL_MODE23:
   diff = ckps.current_angle - ckps.advance_angle;
   if (diff <= (ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG) * 2))
   {
    //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
    prepare_channel23(GetICR() + ((unsigned long)diff * (ckps.period_curr)) / ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG));    
    ckps.ignition_pulse_cogs_23 = 0;   
   ckps.channel_mode = CKPS_CHANNEL_MODENA;
   }
   break;
  }

  if (ckps.ignition_pulse_cogs_14 >= ckps.ignition_cogs)
    TCCR1A|= (1<<FOC1B); //конец импульса запуска зажигания для цилиндров 1-4

  if (ckps.ignition_pulse_cogs_23 >= ckps.ignition_cogs)
    TCCR1A|= (1<<FOC1A); //конец импульса запуска зажигания для цилиндров 2-3

  //прошел зуб - угол до в.м.т. уменьшился на 6 град.
  ckps.current_angle-= ANGLE_MAGNITUDE(CKPS_DEGREES_PER_COG);  
  ckps.cog++;
  ckps.ignition_pulse_cogs_14++;
  ckps.ignition_pulse_cogs_23++;
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
   if ((ckps.cog != 61)) //учитываем 2 виртуальных зуба
     flags.ckps_error_flag = 1; //ERROR               
   ckps.cog = 1; //1-й зуб           
  }
  
synchronized_enter:   
  //Если последний зуб перед синхрометкой, то начинаем отсчет времени для 
  //восстановления отсутствующих зубов, в качестве исходных данных используем 
  //последнее значение межзубного периода.
  if (ckps.cog == 58)
    set_timer0(ckps.period_curr); 
    
  //для 1-58 зубъев 
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
  TCNT0_H--;         
 }  
 else  
 {//отсчет времени закончился    
 TCCR0 = 0; //останавливаем таймер
 
 //запускаем таймер чтобы восстановить 60-й зуб
 if (ckps.cog == 59)
   set_timer0(ckps.period_curr);
 
 //59,60
 process_ckps_cogs();
 }
}

