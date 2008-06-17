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

#include "secu3.h"  

/* (c < p*2.5)&&(c > p*1.5) */
//#define CKPS_CHECK_FOR_BAD_GAP(c,p) (((c) < (((p) << 1) + ((p) >> 1))) && ((c) > ((p) + ((p) >> 1))))

/* p * 2, двухкратный барьер для селекции синхрометки*/
//#define CKPS_GAP_BARRIER(p) ((p) * 2)               

/* p * 2.5,  барьер для селекции синхрометки = 2.5 */
#define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p)>>1))  

#define GetICR() (ICR1)
//#define GetICR() ((ICR1 >> 8)||(ICR1 << 8))

typedef struct
{
  unsigned char  ckps_new_engine_cycle_happen:1;      //флаг синхронизации с вращением
  unsigned char  ckps_returned_to_gap_search:1;       //признак того что были отсчитаны все зубья и КА вновь был переведен в режим поиска синхрометки
  unsigned char  ckps_error_flag:1;                   //признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки
  unsigned char  ckps_is_initialized_half_turn_period23:1;
  unsigned char  ckps_delay_prepared:1;
  unsigned char  ckps_gap_occured:1;
}ckps_flags;

typedef struct
{
  unsigned char sm_state;                   //текущее состояние конечного автомата (КА) 
  unsigned int  icr_prev;                   //предыдущее значение регистра захвата
  unsigned int  period_curr;                //последнй измеренный межзубный период
  unsigned int  period_prev;                //предыдущее значение межзубного периода
  unsigned char cog;                        //считает зубья после выреза, начинает считать с 1
  unsigned int  measure_start_value;        //запоминает значение регистра захвата для измерения периода полуоборота
  unsigned int  current_angle;              //отсчитывает заданный УОЗ при прохождении каждого зуба
  unsigned char ignition_pulse_cogs_14;     //отсчитывает зубья импульса зажигания для цилиндров 1-4
  unsigned char ignition_pulse_cogs_23;     //отсчитывает зубья импульса зажигания для цилиндров 2-3
  unsigned int  half_turn_period;           //хранит последнее измерение времени прохождения n зубьев  
  signed   int  ignition_dwell_angle;       //требуемый УОЗ * ANGLE_MULTIPLAYER
  unsigned char ignition_cogs;
}DPKVSTATE;
 
DPKVSTATE ckps;

//размещаем в свободных регистрах ввода/вывода
__no_init volatile ckps_flags f1@0x22;

//инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится 
void ckps_init_state(void)
{
  ckps.sm_state = 0;
  ckps.cog = 0;
  ckps.ignition_pulse_cogs_14 = CKPS_IGNITION_PULSE_COGS;
  ckps.ignition_pulse_cogs_23 = CKPS_IGNITION_PULSE_COGS;
  ckps.half_turn_period = 0xFFFF;                 
  ckps.ignition_dwell_angle = 0;
  ckps.ignition_cogs = CKPS_IGNITION_PULSE_COGS-1;

  //OC1А(PD5) и OC1В(PD4) должны быть сконфигурированы как выходы
  DDRD|= (1<<DDD5)|(1<<DDD4); 

  //при совпадении будет устанавливатся низкий уровень и заставляем его установиться прямо сейчас 
  TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<FOC1A)|(1<<FOC1B); 
  
  //подавление шума, передний фронт захвата, clock = 250kHz
  TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11)|(1<<CS10);       

  //разрешаем прерывание по захвату и сравнению А и В таймера 1
  TIMSK|= (1<<TICIE1)|(1<<OCIE1A)|(1<<OCIE1B);
}

//устанавливает УОЗ для реализации в алгоритме
void ckps_set_dwell_angle(signed int angle)
{
  __disable_interrupt();    
  ckps.ignition_dwell_angle = angle;
  __enable_interrupt();                
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
void ckps_set_edge_type(unsigned char edge_type)
{
  if (edge_type)
    TCCR1B|= (1<<ICES1);
  else
    TCCR1B&=~(1<<ICES1);
}

//устанавливает длительность импульса зажигания в зубьях
void ckps_set_ignition_cogs(unsigned char cogs)
{
 ckps.ignition_cogs = cogs - 1;
}

unsigned char ckps_is_error(void)
{
 return f1.ckps_error_flag;
}

void ckps_reset_error(void)
{
 f1.ckps_error_flag = 0;
}

//эта функция возвращает 1 если был новый цикл зажигания и сразу сбрасывает событие!
unsigned char ckps_is_cycle_cutover_r()
{
 unsigned char result;
  __disable_interrupt();
 result = f1.ckps_new_engine_cycle_happen;
 f1.ckps_new_engine_cycle_happen = 0;
  __enable_interrupt();                
 return result;
}

//эта функция возвращает 1 если произошло изменение положения коленвала на один оборот и сразу сбрасывает событие
unsigned char ckps_is_rotation_cutover_r(void)
{
 unsigned char result;
  __disable_interrupt();
 result = f1.ckps_gap_occured;
 f1.ckps_gap_occured = 0;
  __enable_interrupt();                
 return result;
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

//прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
#pragma vector=TIMER1_CAPT_vect
__interrupt void timer1_capt_isr(void)
{  
  unsigned int diff;
 
  ckps.period_curr = GetICR() - ckps.icr_prev;
  
  //конечный автомат для синхронизации, измерения скорости вращения коленвала, запуска зажигания в нужное время  
  switch(ckps.sm_state)
  {
   case 0://----------------пусковой режим (пропускаем несколько зубов)------------------- 
     f1.ckps_gap_occured = 0;
     if (ckps.cog >= CKPS_ON_START_SKIP_COGS) 
      {
       ckps.sm_state = 1;
       f1.ckps_returned_to_gap_search = 0;
       f1.ckps_is_initialized_half_turn_period23 = 0;
      }
     break;

   case 1://-----------------поиск синхрометки--------------------------------------------
     if (ckps.period_curr > CKPS_GAP_BARRIER(ckps.period_prev)) 
     {
      f1.ckps_gap_occured = 1; //устанавливаем событие нахождения синхрометки

      if (f1.ckps_returned_to_gap_search)
      {
       if ((ckps.cog != 58)/*||CKPS_CHECK_FOR_BAD_GAP(ckps.period_curr,ckps.period_prev)*/)
         f1.ckps_error_flag = 1; //ERROR             
       f1.ckps_returned_to_gap_search = 0;
      }

      ckps.cog = 0;
      ckps.sm_state = 2;
      ckps.ignition_pulse_cogs_14+=2;
      ckps.ignition_pulse_cogs_23+=2;
      f1.ckps_delay_prepared = 0;
    
      //начинаем отсчет угла опережения
      ckps.current_angle = (ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * (CKPS_COGS_BEFORE_TDC - 1);
     }
     break;

   case 2: //--------------реализация УОЗ для 1-4-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     ckps.current_angle-= ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG;

     if (!f1.ckps_delay_prepared)
     {
      diff = ckps.current_angle - ckps.ignition_dwell_angle;
      if (diff <= ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2) )
      {
       //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
       OCR1B = GetICR() + ((unsigned long)diff * (ckps.period_curr/* * 2*/)) / ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG)/* * 2*/);  
     
       //сбрасываем флаг прерывания, включаем режим установки линии B в высокий уровень при совпадении
       SETBIT(TIFR,OCF1B);
       TCCR1A|= (1<<COM1B1)|(1<<COM1B0);     
       ckps.ignition_pulse_cogs_14 = 0;   
       f1.ckps_delay_prepared = 1;
      }
     }

     if (ckps.cog==2) //диаметральный зуб завершения измерения периода вращения для 2-3
     {
      //если было переполнение то устанавливаем максимально возможное время
      ckps.half_turn_period = ((ckps.period_curr > 1250) || !f1.ckps_is_initialized_half_turn_period23) 
                              ? 0xFFFF : (GetICR() - ckps.measure_start_value);             
      ckps.measure_start_value = GetICR();
      f1.ckps_new_engine_cycle_happen = 1;      //устанавливаем событие цикловой синхронизации 
      adc_begin_measure();                 //запуск процесса измерения значений аналоговых входов        
     }

     if (ckps.cog == 30) //переход в режим реализации УОЗ для 2-3
     {
      //начинаем отсчет угла опережения
      ckps.current_angle = (ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * (CKPS_COGS_BEFORE_TDC - 1);
      ckps.sm_state = 3;
      f1.ckps_delay_prepared = 0;
     }

     if (ckps.period_curr > 12500) //обороты опустилисть ниже нижнего порога - двигатель остановился
       ckps.sm_state = 0;

     break;

   case 3: //--------------реализация УОЗ для 2-3-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     ckps.current_angle-= ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG;

     if (!f1.ckps_delay_prepared)
     {
      diff = ckps.current_angle - ckps.ignition_dwell_angle;
      if (diff <= ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2) )
      {
       //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
       OCR1A = GetICR() + ((unsigned long)diff * (ckps.period_curr/* * 2*/)) / ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) /** 2*/);    

       //сбрасываем флаг прерывания, включаем режим установки линии А в высокий уровень при совпадении
       SETBIT(TIFR,OCF1A);
       TCCR1A|= (1<<COM1A1)|(1<<COM1A0);      
       ckps.ignition_pulse_cogs_23 = 0;   
       f1.ckps_delay_prepared = 1;
      }
     }

     if (ckps.cog == 32) //диаметральный зуб завершения измерения периода вращения для 1-4
     {
      //если было переполнение то устанавливаем максимально возможное время
      ckps.half_turn_period = (ckps.period_curr > 1250) ? 0xFFFF : (GetICR() - ckps.measure_start_value);             
      ckps.measure_start_value = GetICR();    
      f1.ckps_new_engine_cycle_happen = 1;      //устанавливаем событие цикловой синхронизации 
      f1.ckps_is_initialized_half_turn_period23 = 1;
      adc_begin_measure();                 //запуск процесса измерения значений аналоговых входов
     } 

     if (ckps.cog > 55) //переход в режим поиска синхрометки
     {
      ckps.sm_state = 1;
      f1.ckps_returned_to_gap_search = 1; 
      f1.ckps_delay_prepared = 0;
     }

     if (ckps.period_curr > 12500) //обороты опустилисть ниже нижнего порога - двигатель остановился
       ckps.sm_state = 0;

     break;
  }
   
  if (ckps.ignition_pulse_cogs_14 >= ckps.ignition_cogs)
    TCCR1A|= (1<<FOC1B); //конец импульса запуска зажигания для цилиндров 1-4

  if (ckps.ignition_pulse_cogs_23 >= ckps.ignition_cogs)
    TCCR1A|= (1<<FOC1A); //конец импульса запуска зажигания для цилиндров 2-3
      
  ckps.icr_prev = GetICR();
  ckps.period_prev = ckps.period_curr;  
  ckps.cog++; 
  ckps.ignition_pulse_cogs_14++; 
  ckps.ignition_pulse_cogs_23++; 
}
