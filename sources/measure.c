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

/** \file measure.c
 * Implementation pf processing (averaging, corrections etc) of data comes from ADC and sensors
 * (Реализация обработки (усреднение, корректировки и т.д.) данных поступающих от АЦП и датчиков).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "adc.h"
#include "magnitude.h"
#include "measure.h"
#include "secu3.h"

/**Reads state of gas valve (считывает состояние газового клапана) */
#define GET_GAS_VALVE_STATE(s) (PINC_Bit6)

/**Reads state of throttle gate (only the value, without inversion)
 * считывает состояние дроссельной заслонки (только значение, без инверсии)
 */
#ifndef SECU3T /*SECU-3*/
 #define GET_THROTTLE_GATE_STATE() (PINC_Bit5)
#elif         /*SECU-3T*/
 #define GET_THROTTLE_GATE_STATE() (PINA_Bit7)
#endif

/**Number of values for averaging of RPM for tachometer
 * кол-во значений для усреднения частоты вращения к.в. для оборотов тахометра */
#define FRQ_AVERAGING           16

/**Number of values for avaraging of RPM for starter blocking
 * кол-во значений для усреднения частоты вращения к.в. для блокировки страртера */
#define FRQ4_AVERAGING          4

//размер буферов усреднения по каждому аналоговому датчику
#define MAP_AVERAGING           4                 //!< Number of values for averaging of pressure (MAP)
#define BAT_AVERAGING           4                 //!< Number of values for averaging of board voltage
#define TMP_AVERAGING           8                 //!< Number of values for averaging of coolant temperature

uint16_t freq_circular_buffer[FRQ_AVERAGING];     //!< Ring buffer for RPM averaging for tachometer (буфер усреднения частоты вращения коленвала для тахометра)
uint16_t freq4_circular_buffer[FRQ4_AVERAGING];   //!< Ring buffer for RPM averaging for starter blocking (буфер усреднения частоты вращения коленвала для блокировки стартера)
uint16_t map_circular_buffer[MAP_AVERAGING];      //!< Ring buffer for averaring of MAP sensor (буфер усреднения абсолютного давления)
uint16_t ubat_circular_buffer[BAT_AVERAGING];     //!< Ring buffer for averaring of voltage (буфер усреднения напряжения бортовой сети)
uint16_t temp_circular_buffer[TMP_AVERAGING];     //!< Ring buffer for averaring of coolant temperature (буфер усреднения температуры охлаждающей жидкости)

//обновление буферов усреднения (частота вращения, датчики...)
void meas_update_values_buffers(struct ecudata_t* d, uint8_t rpm_only)
{
 static uint8_t  map_ai  = MAP_AVERAGING-1;
 static uint8_t  bat_ai  = BAT_AVERAGING-1;
 static uint8_t  tmp_ai  = TMP_AVERAGING-1;
 static uint8_t  frq_ai  = FRQ_AVERAGING-1;
 static uint8_t  frq4_ai = FRQ4_AVERAGING-1;

 freq_circular_buffer[frq_ai] = d->sens.inst_frq;
 (frq_ai==0) ? (frq_ai = FRQ_AVERAGING - 1): frq_ai--;

 freq4_circular_buffer[frq4_ai] = d->sens.inst_frq;
 (frq4_ai==0) ? (frq4_ai = FRQ4_AVERAGING - 1): frq4_ai--;

 if (rpm_only)
  return;

 map_circular_buffer[map_ai] = adc_get_map_value();
 (map_ai==0) ? (map_ai = MAP_AVERAGING - 1): map_ai--;

 ubat_circular_buffer[bat_ai] = adc_get_ubat_value();
 (bat_ai==0) ? (bat_ai = BAT_AVERAGING - 1): bat_ai--;

 temp_circular_buffer[tmp_ai] = adc_get_temp_value();
 (tmp_ai==0) ? (tmp_ai = TMP_AVERAGING - 1): tmp_ai--;

 d->sens.knock_k = adc_get_knock_value() * 2;
}

//усреднение измеряемых величин используя текущие значения кольцевых буферов усреднения, компенсация
//погрешностей АЦП, перевод измеренных значений в физические величины.
void meas_average_measured_values(struct ecudata_t* d)
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
#ifndef THERMISTOR_CS
  d->sens.temperat = temp_adc_to_c(d->sens.temperat_raw);
#else
  d->sens.temperat = thermistor_lookup(ROUND(TSENS_V_TMIN/ADC_DISCRETE), ROUND(TSENS_STEP/ADC_DISCRETE), d->sens.temperat_raw);
#endif
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
void meas_initial_measure(struct ecudata_t* d)
{
 uint8_t _t,i = 16;
 _t = _SAVE_INTERRUPT();
 _ENABLE_INTERRUPT();
 do
 {
  adc_begin_measure();
  while(!adc_is_measure_ready());

  meas_update_values_buffers(d, 0); //<-- all
 }while(--i);
 _RESTORE_INTERRUPT(_t);
 meas_average_measured_values(d);
}

void meas_take_discrete_inputs(struct ecudata_t *d)
{
 //--инверсия концевика карбюратора если необходимо, включение/выключение клапана ЭПХХ
 d->sens.carb=d->param.carb_invers^GET_THROTTLE_GATE_STATE(); //результат: 0 - дроссель закрыт, 1 - открыт

 //считываем и сохраняем состояние газового клапана
 d->sens.gas = GET_GAS_VALVE_STATE();

 //переключаем тип топлива в зависимости от состояния газового клапана
#ifndef REALTIME_TABLES
 if (d->sens.gas)
  d->fn_dat = &fw_data.tables[d->param.fn_gas];     //на газе
 else
  d->fn_dat = &fw_data.tables[d->param.fn_gasoline];//на бензине
#else //use tables from RAM
 if (d->sens.gas)
  d->fn_dat = &d->tables_ram[1]; //using gas(на газе)
 else
  d->fn_dat = &d->tables_ram[0]; //using petrol(на бензине)
#endif
}

