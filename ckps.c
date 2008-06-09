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

// (c < p*2.5)&&(c > p*1.5)
#define CKPS_CHECK_FOR_BAD_GAP(c,p) (((c) < (((p) << 1) + ((p) >> 1))) && ((c) > ((p) + ((p) >> 1))))

// p * 2, двухкратный барьер для селекции синхрометки
#define CKPS_GAP_BARRIER(p) ((p) * 2)               
// p * 2.5
//#define CKPS_GAP_BARRIER(p) (((p) << 1) + ((p)>>1))  

#define GetICR() (ICR1)
//#define GetICR() ((ICR1 >> 8)||(ICR1 << 8))


extern unsigned char force_measure_timeout_counter;
extern unsigned char engine_stop_timeout_counter;


typedef struct
{
 unsigned char  ckps_new_engine_cycle_happen:1;      //флаг синхронизации с вращением
 unsigned char  ckps_returned_to_gap_search:1;       //признак того что были отсчитаны все зубья и КА вновь был переведен в режим поиска синхрометки
 unsigned char  ckps_error_flag:1;                   //признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки
 unsigned char  ckps_is_initialized_half_turn_period23:1;
}ckps_flags;

typedef struct
{
  unsigned char sm_state;                   //текущее состояние конечного автомата (КА) 
  unsigned int icr_prev;                    //предыдущее значение регистра захвата
  unsigned int period_curr;                 //последнй измеренный межзубный период
  unsigned int period_prev;                 //предыдущее значение межзубного периода
  unsigned char cog;                        //считает зубья после выреза, начинает считать с 1
  unsigned int measure_start_value;         //запоминает значение регистра захвата для измерения периода полуоборота
  unsigned int current_angle;               //отсчитывает заданный УОЗ при прохождении каждого зуба
  unsigned char ignition_pulse_cogs;        //отсчитывает зубья импульса зажигания 
  unsigned int  half_turn_period;           //хранит последнее измерение времени прохождения n зубьев  
  signed   int  ignition_dwell_angle;       //требуемый УОЗ * ANGLE_MULTIPLAYER
}DPKVSTATE;
 
DPKVSTATE ckps;

//размещаем в свободных регистрах ввода/вывода
__no_init volatile ckps_flags f1@0x22;

//инициализирет структуру данных/состояния ДПКВ и железо на которое он мапится 
void dpkv_init_state(void)
{
  ckps.sm_state = 0;
  ckps.cog = 0;
  ckps.ignition_pulse_cogs = CKPS_IGNITION_PULSE_COGS;
  ckps.half_turn_period = 0xFFFF;                 
  ckps.ignition_dwell_angle = 0;

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
void dpkv_set_dwell_angle(signed int angle)
{
  __disable_interrupt();    
  ckps.ignition_dwell_angle = angle;
  __enable_interrupt();                
}

//Высчитывание мгновенной частоты вращения коленвала по измеренному времени прохождения 30 зубьев шкива.
//Период в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, в одной секунде 1000000 мкс, значит:
unsigned int dpkv_calculate_instant_freq(void)
{
  unsigned int period;
 // unsigned int cog_period;
  __disable_interrupt();
   period = ckps.half_turn_period;           //обеспечиваем атомарный доступ к переменной
  // cog_period = ckps.period_curr;
  __enable_interrupt();                           

  //если самый минимум, значит двигатель остановился 
  if (period!=0xFFFF)  
    return (7500000L)/(period);
  else
    return 0;
}

//устанавливает тип фронта ДПКВ (0 - отрицательный, 1 - положительный)
void dpkv_set_edge_type(unsigned char edge_type)
{
  if (edge_type)
    TCCR1B|= (1<<ICES1);
  else
    TCCR1B&=~(1<<ICES1);
}


unsigned char dpkv_is_error(void)
{
 return f1.ckps_error_flag;
}

void dpkv_reset_error(void)
{
 f1.ckps_error_flag = 0;
}

//эта функция возвращает 1 если был новый цикл зажигания и сразу сбрасывает событие!
unsigned char dpkv_is_cycle_cutover_r()
{
 unsigned char result = f1.ckps_new_engine_cycle_happen;
 f1.ckps_new_engine_cycle_happen = 0;
 return result;
}

#pragma vector=TIMER1_COMPA_vect
__interrupt void timer1_compa_isr(void)
{
 //линия в высоком уровне, теперь настраиваем обе линии на переход в низкий уровень по следующему событию.
 //Начинаем отсчет длительности импульса по зубъям  
  TCCR1A = (1<<COM1A1)|(1<<COM1B1);   
  ckps.ignition_pulse_cogs = 0;
}

#pragma vector=TIMER1_COMPB_vect
__interrupt void timer1_compb_isr(void)
{
 //линия в высоком уровне, теперь настраиваем обе линии на переход в низкий уровень по следующему событию.  
 //Начинаем отсчет длительности импульса по зубъям
  TCCR1A = (1<<COM1A1)|(1<<COM1B1); 
  ckps.ignition_pulse_cogs = 0;
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
     force_measure_timeout_counter = FORCE_MEASURE_TIMEOUT_VALUE;  
     engine_stop_timeout_counter = ENGINE_STOP_TIMEOUT_VALUE;
     if (f1.ckps_returned_to_gap_search)
     {
      if ((ckps.cog != 58)||CKPS_CHECK_FOR_BAD_GAP(ckps.period_curr,ckps.period_prev))
        f1.ckps_error_flag = 1; //ERROR             
      f1.ckps_returned_to_gap_search = 0;
     }

     ckps.cog = 0;
     ckps.sm_state = 2;
     ckps.ignition_pulse_cogs+=2;
    
     //начинаем отсчет угла опережения
     ckps.current_angle = (ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * (CKPS_COGS_BEFORE_TDC - 1);
     }
     break;

   case 2: //--------------реализация УОЗ для 1-4-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     ckps.current_angle-= ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG;

     diff = ckps.current_angle - ckps.ignition_dwell_angle;
     if (diff <= ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2) )
     {
     //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
     OCR1B = GetICR() + ((unsigned long)diff * (ckps.period_curr * 2)) / ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2);  
     
     //сбрасываем флаг прерывания, включаем режим установки линии B в высокий уровень, а A в низкий
     SETBIT(TIFR,OCF1B);
     TCCR1A = (1<<COM1B1)|(1<<COM1B0)|(1<<COM1A1);        
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
     }

     if (ckps.period_curr > 12500) //обороты опустилисть ниже нижнего порога - двигатель остановился
       ckps.sm_state = 0;

     break;

   case 3: //--------------реализация УОЗ для 2-3-----------------------------------------
     //прошел зуб - угол до в.м.т. уменьшился на 6 град.
     ckps.current_angle-= ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG;

     diff = ckps.current_angle - ckps.ignition_dwell_angle;
     if (diff <= ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2) )
     {
     //до запуска зажигания осталось отсчитать меньше 2-x зубов. Необходимо подготовить модуль сравнения
     OCR1A = GetICR() + ((unsigned long)diff * (ckps.period_curr * 2)) / ((ANGLE_MULTIPLAYER * CKPS_DEGREES_PER_COG) * 2);    

     //сбрасываем флаг прерывания, включаем режим установки линии А в высокий уровень, а В в низкий
     SETBIT(TIFR,OCF1A);
     TCCR1A = (1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1);      
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
     }

     if (ckps.period_curr > 12500) //обороты опустилисть ниже нижнего порога - двигатель остановился
       ckps.sm_state = 0;

     break;
  }
   
  if (ckps.ignition_pulse_cogs >= (CKPS_IGNITION_PULSE_COGS-1))
    TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<FOC1A)|(1<<FOC1B); //конец импульса запуска зажигания 
      
  ckps.icr_prev = GetICR();
  ckps.period_prev = ckps.period_curr;  
  ckps.cog++; 
  ckps.ignition_pulse_cogs++; 
}
