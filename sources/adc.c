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
              http://secu-3.org
              email: shabelnikov@secu-3.org
*/

/** \file adc.c
 * Implementation of ADC related functions (API).
 * Functions for read values from ADC, perform conversion to phisical values etc
 * (Функции для работы с АЦП, считывание значений, преобразование в физические величины и т.д.).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "funconv.h"   //simple_interpolation()
#include "magnitude.h"
#include "secu3.h"

/**номер канала используемого для ДАД */
#define ADCI_MAP                2
/**номер канала используемого для напряжения бортовой сети */
#define ADCI_UBAT               1
/**номер канала используемого для ДТОЖ */
#define ADCI_TEMP               0
/**номер канала используемого для канала детонации */
#define ADCI_KNOCK              3
/**заглушка, используется для ADCI_KNOCK чтобы сформировать задержку */
#define ADCI_STUB               4

/**Cтруктура данных состояния АЦП */
typedef struct
{
 volatile uint16_t map_value;   //!< последнее измеренное значение абсолютного давления
 volatile uint16_t ubat_value;  //!< последнее измеренное значение напряжения бортовой сети
 volatile uint16_t temp_value;  //!< последнее измеренное значение температуры охлаждающей жидкости
 volatile uint16_t knock_value; //!< последнее измеренное значение сигнала детонации

 volatile uint8_t sensors_ready;//!< датчики обработаны и значения готовы к считыванию
 uint8_t  measure_all;          //!< если 1, то производится измерение всех значений
}adcstate_t;

/** переменные состояния АЦП */
adcstate_t adc;

uint16_t adc_get_map_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.map_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_ubat_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.ubat_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_temp_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.temp_value;
 _END_ATOMIC_BLOCK();
 return value;
}

uint16_t adc_get_knock_value(void)
{
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = adc.knock_value;
 _END_ATOMIC_BLOCK();
 return value;
}

void adc_begin_measure(uint8_t speed2x)
{
 //мы не можем запускать новое измерение, если еще не завершилось
 //предыдущее измерение
 if (!adc.sensors_ready)
  return;

 adc.sensors_ready = 0;
 ADMUX = ADCI_MAP|ADC_VREF_TYPE;
 if (speed2x)
  CLEARBIT(ADCSRA, ADPS0); //250kHz
 else
  SETBIT(ADCSRA, ADPS0);   //125kHz
 SETBIT(ADCSRA, ADSC);
}

void adc_begin_measure_knock(uint8_t speed2x)
{
 //мы не можем запускать новое измерение, если еще не завершилось
 //предыдущее измерение
 if (!adc.sensors_ready)
  return;

 adc.sensors_ready = 0;
 ADMUX = ADCI_STUB|ADC_VREF_TYPE;
 if (speed2x)
  CLEARBIT(ADCSRA, ADPS0); //250kHz
 else
  SETBIT(ADCSRA, ADPS0);   //125kHz
 SETBIT(ADCSRA, ADSC);
}

void adc_begin_measure_all(void)
{
 adc.measure_all = 1;
 adc_begin_measure(0); //<--normal speed
}

uint8_t adc_is_measure_ready(void)
{
 return adc.sensors_ready;
}

void adc_init(void)
{
 adc.knock_value = 0;
 adc.measure_all = 0;

 //инициализация АЦП, параметры: f = 125.000 kHz,
 //внутренний источник опорного напряжения - 2.56V, прерывание разрешено
 ADMUX=ADC_VREF_TYPE;
 ADCSRA=_BV(ADEN)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);

 //модуль АЦП готов к новому измерению
 adc.sensors_ready = 1;

 //запрещаем компаратор - он нам не нужен
 ACSR=_BV(ACD);
}

/**прерывание по завершению преобразования АЦП. Измерение значений всех аналоговых датчиков. После запуска
 * измерения это прерывание будет вызыватся для каждого входа, до тех пор пока все входы не будут обработаны.
 */
ISR(ADC_vect)
{
 _ENABLE_INTERRUPT();

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

int16_t adc_compensate(int16_t adcvalue, int16_t factor, int32_t correction)
{
 return (((((int32_t)adcvalue*factor)+correction)<<2)>>16);
}

uint16_t map_adc_to_kpa(int16_t adcvalue, int16_t offset, int16_t gradient)
{
 int16_t t;
 //АЦП не измеряет отрицательных напряжений, однако отрицательное значение может появится после компенсации погрешностей.
 //Такой ход событий необходимо предотвращать.
 if (adcvalue < 0)
  adcvalue = 0;

 //выражение выглядит так: ((adcvalue + K1) * K2 ) / 128, где K1,K2 - константы.
 //или
 //выражение выглядит так: ((-5.0 + adcvalue + K1) * K2 ) / 128, где K1,K2 - константы.
 if (gradient > 0)
 {
  t = adcvalue + offset;
  if (t < 0)
   t = 0;
 }
 else
 {
  t = -ROUND(5.0/ADC_DISCRETE) + adcvalue + offset;
  if (t > 0)
   t = 0;
 }
 return ( ((int32_t)t) * gradient ) >> 7;
}

uint16_t ubat_adc_to_v(int16_t adcvalue)
{
 if (adcvalue < 0)
  adcvalue = 0;
 return adcvalue;
}

#ifndef THERMISTOR_CS
//Coolant sensor has linear output. 10mV per C (e.g. LM235)
int16_t temp_adc_to_c(int16_t adcvalue)
{
 if (adcvalue < 0)
  adcvalue = 0;
 return (adcvalue - ((int16_t)((TSENS_ZERO_POINT / ADC_DISCRETE)+0.5)) );
}
#else
//Coolant sensor is thermistor (тип датчика температуры - термистор)
//Note: We assume that voltage on the input of ADC depend on thermistor's resistance linearly.
//Voltage on the input of ADC can be calculated as following:
// U3=U1*Rt*R2/(Rp(Rt+R1+R2)+Rt(R1+R2));
// Rt - thermistor, Rp - pulls up thermistor to voltage U1,
// R1,R2 - voltage divider resistors.

/**Size of lookup table for thermistor */
#define THERMISTOR_LOOKUP_TABLE_SIZE 16

/**Lookup table for converting of thermistor's resistance into temperature 
 * (таблица значений температуры с шагом по напряжению)*/
PGM_DECLARE(int16_t therm_cs_temperature[THERMISTOR_LOOKUP_TABLE_SIZE]) =
{TEMPERATURE_MAGNITUDE(-27.9),TEMPERATURE_MAGNITUDE(-13.6),TEMPERATURE_MAGNITUDE(-3.7),TEMPERATURE_MAGNITUDE(2.4),
TEMPERATURE_MAGNITUDE(8.5),TEMPERATURE_MAGNITUDE(14.1),TEMPERATURE_MAGNITUDE(19.5),TEMPERATURE_MAGNITUDE(24.7),
TEMPERATURE_MAGNITUDE(30.0),TEMPERATURE_MAGNITUDE(35.6),TEMPERATURE_MAGNITUDE(41.4),TEMPERATURE_MAGNITUDE(47.8),
TEMPERATURE_MAGNITUDE(55.8),TEMPERATURE_MAGNITUDE(65.5),TEMPERATURE_MAGNITUDE(78.1),TEMPERATURE_MAGNITUDE(100.0)};

int16_t thermistor_lookup(uint16_t start, uint16_t step, uint16_t adcvalue)
{
 int16_t i, i1;

 if (adcvalue > start)
  adcvalue = start;

 i = ((start - adcvalue) / step);

 if (i >= THERMISTOR_LOOKUP_TABLE_SIZE-1) i = i1 = THERMISTOR_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(adcvalue, PGM_GET_WORD(&therm_cs_temperature[i1]), PGM_GET_WORD(&therm_cs_temperature[i]),
         start - (i1 * step), step)) >> 4;
}

#endif

