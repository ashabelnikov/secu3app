 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <inavr.h>
#include <ioavr.h>
#include "bitmask.h"
#include "adc.h"
#include "secu3.h"

typedef struct
{
 uint16_t map_value;           //последнее измеренное значение абсолютного давления
 uint16_t ubat_value;          //последнее измеренное значение напряжения бортовой сети
 uint16_t temp_value;          //последнее измеренное значение температуры охлаждающей жидкости
 uint16_t knock_value;         //последнее измеренное значение сигнала детонации

 uint8_t  sensors_ready;       //датчики обработаны и значения готовы к считыванию
 uint8_t  measure_all;         //если 1, то производится измерение всех значений
}adc_state;

adc_state adc;  //переменные состояния АЦП

__monitor
uint16_t adc_get_map_value(void)
{
 return adc.map_value;
}

__monitor
uint16_t adc_get_ubat_value(void)
{
 return adc.ubat_value;
}

__monitor
uint16_t adc_get_temp_value(void)
{
 return adc.temp_value;
}

__monitor
uint16_t adc_get_knock_value(void)
{
 return adc.knock_value;
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

void adc_begin_measure_knock(void) 
{ 
 //мы не можем запускать новое измерение, если еще не завершилось
 //предыдущее измерение
 if (!adc.sensors_ready)  
  return;

 adc.sensors_ready = 0; 
 ADMUX = ADCI_STUB|ADC_VREF_TYPE; 
 SETBIT(ADCSRA,ADSC);
}  

void adc_begin_measure_all(void)
{
 adc.measure_all = 1;
 adc_begin_measure();
}

uint8_t adc_is_measure_ready(void)
{
 return adc.sensors_ready; 
}

//инициализация АЦП и его переменных состояния
void adc_init(void)
{ 
 adc.knock_value = 0;
 adc.measure_all = 0;

 //инициализация АЦП, параметры: f = 125.000 kHz, 
 //внутренний источник опорного напряжения - 2.56V, прерывание разрешено 
 ADMUX=ADC_VREF_TYPE;
 ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     

 //модуль АЦП готов к новому измерению
 adc.sensors_ready = 1;

 //запрещаем компаратор - он нам не нужен
 ACSR=(1<<ACD);
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
   adc.map_value = ADC;      
   ADMUX = ADCI_UBAT|ADC_VREF_TYPE;   
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_UBAT://закончено измерение напряжения бортовой сети
   adc.ubat_value = ADC;      
   ADMUX = ADCI_TEMP|ADC_VREF_TYPE;   
   SETBIT(ADCSRA,ADSC);
   break;

  case ADCI_TEMP://закончено измерение температуры охлаждающей жидкости
   adc.temp_value = ADC;      
   if (0==adc.measure_all)
   {
    ADMUX = ADCI_MAP|ADC_VREF_TYPE;    
    adc.sensors_ready = 1;                
   }
   else
   {
    adc.measure_all = 0;
    ADMUX = ADCI_KNOCK|ADC_VREF_TYPE; 
    SETBIT(ADCSRA,ADSC);
   }     
   break; 
         
  case ADCI_STUB: //это холостое измерение необходимо только для задержки перед измерением сигнала детонации
   ADMUX = ADCI_KNOCK|ADC_VREF_TYPE;
   SETBIT(ADCSRA,ADSC);         
   break; 
            
  case ADCI_KNOCK://закончено измерение сигнала с интегратора канала детонации
   adc.knock_value = ADC;      
   adc.sensors_ready = 1;                
   break; 
 } 
}


//Компенсирует погрешности АЦП (погрешность смещения и передаточная погрешность)
// adcvalue - значене АЦП для компенсации
// factor = 2^14 * gainfactor, 
// correction = 2^14 * (0.5 - offset * gainfactor),
// 2^16 * realvalue = 2^2 * (adcvalue * factor + correction)
int16_t adc_compensate(int16_t adcvalue, int16_t factor, int32_t correction)
{
 return (((((int32_t)adcvalue*factor)+correction)<<2)>>16);
}

//adcvalue - значение напряжения в дискретах АЦП
//offset  = offset_volts / ADC_DISCRETE, где offset_volts - значение в вольтах;
//gradient = 128 * gradient_kpa * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER * ADC_DISCRETE, где gradient_kpa значение в кило-паскалях
uint16_t map_adc_to_kpa(int16_t adcvalue, uint16_t offset, uint16_t gradient)
{
 //АЦП не измеряет отрицательных напряжений, однако отрицательное значение может появится после компенсации погрешностей.
 //Такой ход событий необходимо предотвращать.
 if (adcvalue < 0)
  adcvalue = 0;
   
 //выпажение выглядит так: ((adcvalue + K1) * K2 ) / 128, где K1,K2 - константы.   
 return ( ((uint32_t)(adcvalue + offset)) * ((uint32_t)gradient) ) >> 7; 
}

uint16_t ubat_adc_to_v(int16_t adcvalue)
{
 if (adcvalue < 0)
  adcvalue = 0;
 return adcvalue;
}

int16_t temp_adc_to_c(int16_t adcvalue)
{   
 if (adcvalue < 0)
  adcvalue = 0;
 return (adcvalue - ((int16_t)((TSENS_ZERO_POINT / ADC_DISCRETE)+0.5)) );
}
