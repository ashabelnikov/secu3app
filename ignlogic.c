 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include "ignlogic.h"
#include "funconv.h"
#include "secu3.h"

void advance_angle_state_machine(int16_t* padvance_angle_inhibitor_state, struct ecudata_t* d)
{
 switch(d->engine_mode)
 {
  case EM_START: //режим пуска
   if (d->sens.inst_frq > d->param.smap_abandon)
   {                   
    d->engine_mode = EM_IDLE;    
    idling_regulator_init();    
   }      
   d->curr_angle=start_function(d);               //базовый УОЗ - функция для пуска
   d->airflow = 0;                                //в режиме пуска нет расхода
   *padvance_angle_inhibitor_state = d->curr_angle;//в режиме пуска фильтр отключен
   break;     
              
  case EM_IDLE: //режим холостого хода
   if (d->sens.carb)//педаль газа нажали - в рабочий режим
   {
    d->engine_mode = EM_WORK;
   }             
   work_function(d, 1);                           //обновляем значение расхода воздуха 
   d->curr_angle = idling_function(d);            //базовый УОЗ - функция для ХХ 
   d->curr_angle+=coolant_function(d);            //добавляем к УОЗ температурную коррекцию
   d->curr_angle+=idling_pregulator(d,&idle_period_time_counter);//добавляем регулировку
   break;            
                                             
  case EM_WORK: //рабочий режим 
   if (!d->sens.carb)//педаль газа отпустили - в переходной режим ХХ
   {
    d->engine_mode = EM_IDLE;
    idling_regulator_init();    
   }
   d->curr_angle=work_function(d, 0);           //базовый УОЗ - функция рабочего режима
   d->curr_angle+=coolant_function(d);          //добавляем к УОЗ температурную коррекцию  
   //отнимаем поправку полученную от регулятора по детонации
   d->curr_angle-=d->knock_retard;       
   break;     
       
  default:  //непонятная ситуация - угол в ноль       
   d->curr_angle = 0;
   break;     
 }
}
