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
#include "bitmask.h"
#include "adc.h"
#include "secu3.h"

typedef struct
{
 unsigned int map_abuf[MAP_AVERAGING];           //буфер усреднения абсолютного давления
 unsigned int bat_abuf[BAT_AVERAGING];           //буфер усреднения напряжения бортовой сети
 unsigned int tmp_abuf[TMP_AVERAGING];           //буфер усреднения температуры охлаждающей жидкости

 unsigned char  map_ai;
 unsigned char  bat_ai;
 unsigned char  tmp_ai;      
 unsigned char  sensors_ready;                  //датчики обработаны и значения готовы к считыванию
}adc_state;

adc_state adc;  //переменные состояния АЦП


unsigned int adc_get_map_value(unsigned char index)
{
  return adc.map_abuf[index];
}

unsigned int adc_get_ubat_value(unsigned char index)
{
  return adc.bat_abuf[index];
}

unsigned int adc_get_temp_value(unsigned char index)
{
  return adc.tmp_abuf[index];
}


void adc_begin_measure(void) 
{ 
  //мы не можем запускать новое измерение, если еще не завершилось
  //предыдущее измерение
  if (!adc.sensors_ready)  
    return;

  adc.sensors_ready = 0; 
  ADMUX = ADCI_MAP|ADC_VREF_TYPE; 
  SETBIT(ADCSRA,ADSC);
}  

char adc_is_measure_ready(void)
{
  return adc.sensors_ready; 
}

//инициализация АЦП и его переменных состояния
void adc_init(void)
{
 adc.map_ai = MAP_AVERAGING-1;
 adc.bat_ai = BAT_AVERAGING-1;
 adc.tmp_ai = TMP_AVERAGING-1;      
 
 //инициализация АЦП, параметры: f = 125.000 kHz, 
 //внутренний источник опорного напряжения - 2.56V, прерывание разрешено 
 ADMUX=ADC_VREF_TYPE;
 ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     

 //модуль АЦП готов к новому измерению
 adc.sensors_ready = 1;
}

//прерывание по завершению преобразования АЦП. Измерение значений всех аналоговых датчиков. После запуска
//измерения это прерывание будет вызыватся для каждого входа, до тех пор пока все входы не будут обработаны.
#pragma vector=ADC_vect
__interrupt void ADC_isr(void)
{
 __enable_interrupt(); 

 switch(ADMUX&0x07)
 {
   case ADCI_MAP: //закончено измерение абсолютного давления
      adc.map_abuf[adc.map_ai] = ADC;      

      //обновляем значение индекса буфера усреднения
      (adc.map_ai==0) ? (adc.map_ai = MAP_AVERAGING - 1): adc.map_ai--;            

      ADMUX = ADCI_UBAT|ADC_VREF_TYPE;   
      SETBIT(ADCSRA,ADSC);
      break;

   case ADCI_UBAT://закончено измерение напряжения бортовой сети
      adc.bat_abuf[adc.bat_ai] = ADC;      

      //обновляем значение индекса буфера усреднения
      (adc.bat_ai==0) ? (adc.bat_ai = BAT_AVERAGING - 1): adc.bat_ai--;            

      ADMUX = ADCI_TEMP|ADC_VREF_TYPE;   
      SETBIT(ADCSRA,ADSC);
      break;

   case ADCI_TEMP://закончено измерение температуры охлаждающей жидкости
      adc.tmp_abuf[adc.tmp_ai] = ADC;      

      //обновляем  значение индекса буфера усреднения
      (adc.tmp_ai==0) ? (adc.tmp_ai = TMP_AVERAGING - 1): adc.tmp_ai--;               

      ADMUX = ADCI_MAP|ADC_VREF_TYPE;    
      adc.sensors_ready = 1;                
      break; 
 } 
}


//Компенсирует погрешности АЦП (погрешность смещения и передаточная погрешность)
// adcvalue - значене АЦП для компенсации
// factor = 2^14 * gainfactor, 
// correction = 2^14 * (0.5 - offset * gainfactor),
// 2^16 * realvalue = 2^2 * (adcvalue * factor + correction)
signed int adc_compensate(signed int adcvalue, signed int factor, signed long correction)
{
  return (((((signed long)adcvalue*factor)+correction)<<2)>>16);
}

//SIGNED/UNSIGNED missmatch if result < 0 !!!

unsigned int map_adc_to_kpa(signed int adcvalue)
{
 return adcvalue;
}

unsigned int ubat_adc_to_v(signed int adcvalue)
{
 return adcvalue;
}

signed int temp_adc_to_c(signed int adcvalue)
{
 return adcvalue;
}
