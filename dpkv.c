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
#include "dpkv.h"
#include "bitmask.h"
#include "adc.h"

#include "secu3.h"  //только для флагов!

extern unsigned char force_measure_timeout_counter;
extern unsigned char engine_stop_timeout_counter;


typedef struct
{
  unsigned char sm_state;                   //текущее состояние конечного автомата (КА) 
  unsigned int icr_prev;                    //предыдущее значение регистра захвата
  unsigned int period_curr;                 //последнй измеренный межзубный период
  unsigned int period_prev;                 //предыдущее значение межзубного периода
  unsigned char cog;                        //считает зубья после выреза, начинает считать с 1
  unsigned int measure_start_value;         //запоминает значение регистра захвата для измерения периода полуоборота
  unsigned int current_angle;               //отсчитывает заданный УОЗ при прохождении каждого зуба
  unsigned char ignition_pulse_teeth;       //отсчитывает зубья импульса зажигания 
  unsigned int  half_turn_period;           //хранит последнее измерение времени прохождения n зубьев  
  signed   int  ignition_dwell_angle;       //требуемый УОЗ * ANGLE_MULTIPLAYER
}DPKVSTATE;
 
DPKVSTATE dpkv;

//инициализирет структуру данных/состояния ДПКВ 
void dpkv_init_state(void)
{
  dpkv.sm_state = 0;
  dpkv.cog = 0;
  dpkv.ignition_pulse_teeth = DPKV_IGNITION_PULSE_COGS;
  dpkv.half_turn_period = 0xFFFF;                 
  dpkv.ignition_dwell_angle = 0;
}

//устанавливает УОЗ для реализации в алгоритме
void dpkv_set_dwell_angle(signed int angle)
{
  __disable_interrupt();    
  dpkv.ignition_dwell_angle = angle;
  __enable_interrupt();                
}

//Высчитывание мгновенной частоты вращения коленвала по измеренному времени прохождения 30 зубьев шкива.
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс, значит:
unsigned int dpkv_calculate_instant_freq(void)
{
  unsigned int period;
  __disable_interrupt();
   period = dpkv.half_turn_period;           //обеспечиваем атомарный доступ к переменной
  __enable_interrupt();                           

  //если самый минимум, значит двигатель остановился 
  if (period!=0xFFFF)  
    return (7500000L)/(period);
  else
    return 0;
}

#pragma vector=TIMER1_COMPA_vect
__interrupt void timer1_compa_isr(void)
{
 //линия в высоком уровне, теперь настраиваем обе линии на переход в низкий уровень по следующему событию.
 //Начинаем отсчет длительности импульса по зубъям  
  TCCR1A = (1<<COM1A1)|(1<<COM1B1);   
  dpkv.ignition_pulse_teeth = 0;
}

#pragma vector=TIMER1_COMPB_vect
__interrupt void timer1_compb_isr(void)
{
 //линия в высоком уровне, теперь настраиваем обе линии на переход в низкий уровень по следующему событию.  
 //Начинаем отсчет длительности импульса по зубъям
  TCCR1A = (1<<COM1A1)|(1<<COM1B1); 
  dpkv.ignition_pulse_teeth = 0;
}

//прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
#pragma vector=TIMER1_CAPT_vect
__interrupt void timer1_capt_isr(void)
{  
  unsigned int diff;
 
  dpkv.period_curr = ICR1 - dpkv.icr_prev;
  
  //конечный автомат для синхронизации, измерения скорости вращения коленвала, запуска зажигания в нужное время  
  switch(dpkv.sm_state)
  {
   case 0://----------------пусковой режим (пропускаем несколько зубов)------------------- 
     if (dpkv.cog >= DPKV_ON_START_SKIP_COGS)
      {
       dpkv.sm_state = 1;
       f1.dpkv_returned_to_gap_search = 0;
      }
     break;

   case 1://-----------------поиск синхрометки--------------------------------------------
     if (dpkv.period_curr > dpkv.period_prev)
     {
     force_measure_timeout_counter = FORCE_MEASURE_TIMEOUT_VALUE;  
     engine_stop_timeout_counter = ENGINE_STOP_TIMEOUT_VALUE;
     if (f1.dpkv_returned_to_gap_search)
     {
      if (dpkv.cog != 58)
        f1.dpkv_error_flag = 1; //ERROR             
      f1.dpkv_returned_to_gap_search = 0;
     }

     dpkv.cog = 1;
     dpkv.sm_state = 2;
     dpkv.ignition_pulse_teeth+=2;
    
     //начинаем отсчет угла опережения
     dpkv.current_angle = (ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * (DPKV_COGS_BEFORE_TDC - 1);
     }
     break;

   case 2: //--------------реализация УОЗ для 1-4-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     dpkv.current_angle-= ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG;

     diff = dpkv.current_angle - dpkv.ignition_dwell_angle;
     if (diff <= ((ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * 2) )
     {
     //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
     OCR1B = ICR1 + ((unsigned long)diff * (dpkv.period_curr * 2)) / ((ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * 2);  
     
     //сбрасываем флаг прерывания, включаем режим установки линии B в высокий уровень, а A в низкий
     SETBIT(TIFR,OCF1B);
     TCCR1A = (1<<COM1B1)|(1<<COM1B0)|(1<<COM1A1);        
     }

     if (dpkv.cog==2) //диаметральный зуб измерения периода вращения для 2-3
     {
     //если было переполнение то устанавливаем максимально возможное время
     dpkv.half_turn_period = (dpkv.period_curr > 1250) ? 0xFFFF : (ICR1 - dpkv.measure_start_value);             
     dpkv.measure_start_value = ICR1;
     f1.dpkv_new_engine_cycle_happen = 1;      //устанавливаем событие цикловой синхронизации 
     adc_begin_measure();                 //запуск процесса измерения значений аналоговых входов        
     }

     if (dpkv.cog == 30) //переход в режим реализации УОЗ для 2-3
     {
     //начинаем отсчет угла опережения
     dpkv.current_angle = (ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * (DPKV_COGS_BEFORE_TDC - 1);
     dpkv.sm_state = 3;
     }
     break;

   case 3: //--------------реализация УОЗ для 2-3-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     dpkv.current_angle-= ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG;

     diff = dpkv.current_angle - dpkv.ignition_dwell_angle;
     if (diff <= ((ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * 2) )
     {
     //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
     OCR1A = ICR1 + ((unsigned long)diff * (dpkv.period_curr * 2)) / ((ANGLE_MULTIPLAYER * DPKV_DEGREES_PER_COG) * 2);    

     //сбрасываем флаг прерывания, включаем режим установки линии А в высокий уровень, а В в низкий
     SETBIT(TIFR,OCF1A);
     TCCR1A = (1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1);      
     }

     if (dpkv.cog == 32) //диаметральный зуб измерения периода вращения для 1-4
     {
     //если было переполнение то устанавливаем максимально возможное время
     dpkv.half_turn_period = (dpkv.period_curr > 1250) ? 0xFFFF : (ICR1 - dpkv.measure_start_value);             
     dpkv.measure_start_value = ICR1;    
     f1.dpkv_new_engine_cycle_happen = 1;      //устанавливаем событие цикловой синхронизации 
     adc_begin_measure();                 //запуск процесса измерения значений аналоговых входов
     } 

     if (dpkv.cog > 55) //переход в режим поиска синхрометки
     {
     dpkv.sm_state = 1;
     f1.dpkv_returned_to_gap_search = 1; 
     }
     break;
  }
    
  if (dpkv.ignition_pulse_teeth >= (DPKV_IGNITION_PULSE_COGS-1))
    TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<FOC1A)|(1<<FOC1B); //конец импульса запуска зажигания 
  
  dpkv.icr_prev = ICR1;
  dpkv.period_prev = dpkv.period_curr * 2;  //двухкратный барьер для селекции синхрометки
  dpkv.cog++; 
  dpkv.ignition_pulse_teeth++; 
}
