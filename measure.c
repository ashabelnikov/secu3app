/****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <inavr.h>
#include "measure.h"
#include "adc.h"

//кол-во значений для усреднения частоты вращения к.в.
#define FRQ_AVERAGING           16                          
#define FRQ4_AVERAGING          4

//размер буферов усреднения по каждому аналоговому датчику
#define MAP_AVERAGING           4   
#define BAT_AVERAGING           4   
#define TMP_AVERAGING           8  

uint16_t freq_circular_buffer[FRQ_AVERAGING];     //буфер усреднения частоты вращения коленвала
uint16_t freq4_circular_buffer[FRQ4_AVERAGING];   //
uint16_t map_circular_buffer[MAP_AVERAGING];      //буфер усреднения абсолютного давления
uint16_t ubat_circular_buffer[BAT_AVERAGING];     //буфер усреднения напряжения бортовой сети
uint16_t temp_circular_buffer[TMP_AVERAGING];     //буфер усреднения температуры охлаждающей жидкости

//обновление буферов усреднения (частота вращения, датчики...)
void meas_update_values_buffers(ecudata* d)
{
 static uint8_t  map_ai  = MAP_AVERAGING-1;
 static uint8_t  bat_ai  = BAT_AVERAGING-1;
 static uint8_t  tmp_ai  = TMP_AVERAGING-1;      
 static uint8_t  frq_ai  = FRQ_AVERAGING-1;
 static uint8_t  frq4_ai = FRQ4_AVERAGING-1;  

 map_circular_buffer[map_ai] = adc_get_map_value();      
 (map_ai==0) ? (map_ai = MAP_AVERAGING - 1): map_ai--;            

 ubat_circular_buffer[bat_ai] = adc_get_ubat_value();      
 (bat_ai==0) ? (bat_ai = BAT_AVERAGING - 1): bat_ai--;            

 temp_circular_buffer[tmp_ai] = adc_get_temp_value();      
 (tmp_ai==0) ? (tmp_ai = TMP_AVERAGING - 1): tmp_ai--;               

 freq_circular_buffer[frq_ai] = d->sens.inst_frq;      
 (frq_ai==0) ? (frq_ai = FRQ_AVERAGING - 1): frq_ai--; 
        
 freq4_circular_buffer[frq4_ai] = d->sens.inst_frq;      
 (frq4_ai==0) ? (frq4_ai = FRQ4_AVERAGING - 1): frq4_ai--;   
  
 d->sens.knock_k = adc_get_knock_value() * 2;
}

//усреднение измеряемых величин используя текущие значения кольцевых буферов усреднения, компенсация 
//погрешностей АЦП, перевод измеренных значений в физические величины.
void meas_average_measured_values(ecudata* d)
{     
 uint8_t i;  uint32_t sum;       
            
 for (sum=0,i = 0; i < MAP_AVERAGING; i++)  //усредняем значение с датчика абсолютного давления
  sum+=map_circular_buffer[i];       
 d->sens.map_raw = adc_compensate((sum/MAP_AVERAGING)*2,d->param.map_adc_factor,d->param.map_adc_correction); 
 d->sens.map = map_adc_to_kpa(d->sens.map_raw, d->param.map_curve_offset, d->param.map_curve_gradient);
          
 for (sum=0,i = 0; i < BAT_AVERAGING; i++)   //усредняем напряжение бортовой сети
  sum+=ubat_circular_buffer[i];      
 d->sens.voltage_raw = adc_compensate((sum/BAT_AVERAGING)*6,d->param.ubat_adc_factor,d->param.ubat_adc_correction);
 d->sens.voltage = ubat_adc_to_v(d->sens.voltage_raw);  
     
 if (d->param.tmp_use) 
 {       
  for (sum=0,i = 0; i < TMP_AVERAGING; i++) //усредняем температуру (ДТОЖ)
   sum+=temp_circular_buffer[i];      
  d->sens.temperat_raw = adc_compensate((5*(sum/TMP_AVERAGING))/3,d->param.temp_adc_factor,d->param.temp_adc_correction); 
  d->sens.temperat = temp_adc_to_c(d->sens.temperat_raw);
 }  
 else                                       //ДТОЖ не используется
  d->sens.temperat = 0;
               
 for (sum=0,i = 0; i < FRQ_AVERAGING; i++)  //усредняем частоту вращения коленвала
  sum+=freq_circular_buffer[i];      
 d->sens.frequen=(sum/FRQ_AVERAGING);    
  
 for (sum=0,i = 0; i < FRQ4_AVERAGING; i++) //усредняем частоту вращения коленвала
  sum+=freq4_circular_buffer[i];      
 d->sens.frequen4=(sum/FRQ4_AVERAGING);           
}

//Вызывать для предварительного измерения перед пуском двигателя. Вызывать только после 
//инициализации АЦП.
void meas_initial_measure(ecudata* d)
{ 
 uint8_t _t, i = 16;
 _t=__save_interrupt();
 __enable_interrupt();
 do
 {
  adc_begin_measure();                                                     
  while(!adc_is_measure_ready()); 
    
  adc_begin_measure_knock();                                                     
  while(!adc_is_measure_ready());    
    
  meas_update_values_buffers(d);
 }while(--i);            
 __restore_interrupt(_t);
 meas_average_measured_values(d);  
}
