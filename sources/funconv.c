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

/** \file funconv.c
 * Implementation of core mathematics and regulation logic.
 * (Реализация основной части математического аппарата и логики регулирования).
 */

#include "port/pgmspace.h"
#include "port/port.h"
#include <stdlib.h>
#include "adc.h"
#include "ckps.h"
#include "funconv.h"
#include "magnitude.h"
#include "secu3.h"
#include "vstimer.h"

//For use with fn_dat pointer, because it can point either to FLASH or RAM
#ifdef REALTIME_TABLES
 #define _GB(x) *(x)
#else
 #define _GB(x) PGM_GET_BYTE(x)
#endif

// Функция билинейной интерполяции (поверхность)
// x, y - значения аргументов интерполируемой функции
// a1,a2,a3,a4 - значение функции в узлах интерполяции (углы четырехугольника)
// x_s,y_s - значения аргументов функции соответствующие началу прямоугольной области
// x_l,y_l - размеры прямоугольной области (по x и y соответственно)
// возвращает интерполированное значение функции * 16
int16_t bilinear_interpolation(int16_t x, int16_t y, int16_t a1, int16_t a2, int16_t a3, int16_t a4,
                               int16_t x_s, int16_t y_s, int16_t x_l, int16_t y_l)
{
 int16_t a23,a14;
 a23 = ((a2 * 16) + (((int32_t)(a3 - a2) * 16) * (x - x_s)) / x_l);
 a14 = (a1 * 16) + (((int32_t)(a4 - a1) * 16) * (x - x_s)) / x_l;
 return (a14 + ((((int32_t)(a23 - a14)) * (y - y_s)) / y_l));
}

// Функция линейной интерполяции
// x - значение аргумента интерполируемой функции
// a1,a2 - значения функции в узлах интерполяции
// x_s - значение аргумента функции в начальной точке
// x_l - длина отрезка между точками
// m - function multiplier
// возвращает интерполированное значение функции * m
int16_t simple_interpolation(int16_t x, int16_t a1, int16_t a2, int16_t x_s, int16_t x_l, uint8_t m)
{
 return ((a1 * m) + (((int32_t)(a2 - a1) * m) * (x - x_s)) / x_l);
}


// Реализует функцию УОЗ от оборотов для холостого хода
// Возвращает значение угла опережения в целом виде * 32. 2 * 16 = 32.
int16_t idling_function(struct ecudata_t* d)
{
 int8_t i;
 int16_t rpm = d->sens.inst_frq;

 //находим узлы интерполяции, вводим ограничение если обороты выходят за пределы
 for(i = 14; i >= 0; i--)
  if (d->sens.inst_frq >= PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[i])) break;

 if (i < 0)  {i = 0; rpm = fw_data.exdata.rpm_grid_points[0];}

 return simple_interpolation(rpm,
             _GB(&d->fn_dat->f_idl[i]), _GB(&d->fn_dat->f_idl[i+1]),
             PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[i]), PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[i]), 16);
}


// Реализует функцию УОЗ от оборотов для пуска двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t start_function(struct ecudata_t* d)
{
 int16_t i, i1, rpm = d->sens.inst_frq;

 if (rpm < 200) rpm = 200; //200 - минимальное значение оборотов

 i = (rpm - 200) / 40;   //40 - шаг по оборотам

 if (i >= 15) i = i1 = 15;
  else i1 = i + 1;

 return simple_interpolation(rpm, _GB(&d->fn_dat->f_str[i]), _GB(&d->fn_dat->f_str[i1]), (i * 40) + 200, 40, 16);
}


// Реализует функцию УОЗ от оборотов(мин-1) и нагрузки(кПа) для рабочего режима двигателя
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t work_function(struct ecudata_t* d, uint8_t i_update_airflow_only)
{
 int16_t  gradient, discharge, rpm = d->sens.inst_frq, l;
 int8_t f, fp1, lp1;

 discharge = (d->param.map_upper_pressure - d->sens.map);
 if (discharge < 0) discharge = 0;

 //map_upper_pressure - верхнее значение давления
 //map_lower_pressure - нижнее значение давления
 gradient = (d->param.map_upper_pressure - d->param.map_lower_pressure) / 16; //делим на количество узлов интерполяции по оси давления
 if (gradient < 1)
  gradient = 1;  //исключаем деление на ноль и отрицательное значение если верхнее давление меньше нижнего
 l = (discharge / gradient);

 if (l >= (F_WRK_POINTS_F - 1))
  lp1 = l = F_WRK_POINTS_F - 1;
 else
  lp1 = l + 1;

 //обновляем переменную расхода воздуха
 d->airflow = 16 - l;

 if (i_update_airflow_only)
  return 0; //выходим если вызвавший указал что мы должны обновить только расход воздуха

 //находим узлы интерполяции, вводим ограничение если обороты выходят за пределы
 for(f = 14; f >= 0; f--)
  if (rpm >= PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[f])) break;

 //рабочая карта работает на rpm_grid_points[0] оборотах и выше
 if (f < 0)  {f = 0; rpm = fw_data.exdata.rpm_grid_points[0];}
  fp1 = f + 1;

 return bilinear_interpolation(rpm, discharge,
        _GB(&d->fn_dat->f_wrk[l][f]),
        _GB(&d->fn_dat->f_wrk[lp1][f]),
        _GB(&d->fn_dat->f_wrk[lp1][fp1]),
        _GB(&d->fn_dat->f_wrk[l][fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[f]),
        (gradient * l),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[f]),
        gradient);
}

//Реализует функцию коррекции УОЗ по температуре(град. Цельсия) охлаждающей жидкости
// Возвращает значение угла опережения в целом виде * 32, 2 * 16 = 32.
int16_t coolant_function(struct ecudata_t* d)
{
 int16_t i, i1, t = d->sens.temperat;

 if (!d->param.tmp_use)
  return 0;   //нет коррекции, если блок неукомплектован ДТОЖ-ом

 //-30 - минимальное значение температуры
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - шаг между узлами интерполяции по температуре
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= 15) i = i1 = 15;
 else i1 = i + 1;

 return simple_interpolation(t, _GB(&d->fn_dat->f_tmp[i]), _GB(&d->fn_dat->f_tmp[i1]),
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 16);
}

//Регулятор холостого хода РХХ
/**Describes state data for idling regulator */
typedef struct
{
 //память регулятора для хранения последнего значения управляющего воздействия (коррекции)
 int16_t output_state;   //!< regulator's memory
}idlregul_state_t;

/**Variable. State data for idling regulator */
idlregul_state_t idl_prstate;

//сброс состояния РХХ
void idling_regulator_init(void)
{
 idl_prstate.output_state = 0;
}

//Пропорциональный регулятор для регулирования оборотов ХХ углом опережения зажигания
// Возвращает значение угла опережения в целом виде * 32.
int16_t idling_pregulator(struct ecudata_t* d, volatile s_timer8_t* io_timer)
{
 int16_t error,factor;
 //зона "подхвата" регулятора при возвращени двигателя из рабочего режима в ХХ
 uint16_t capture_range = 200;

 //если PXX отключен или обороты значительно выше от нормальных холостых оборотов
 // или двигатель не прогрет то выходим  с нулевой корректировкой
 if (!d->param.idl_regul || (d->sens.frequen >(d->param.idling_rpm + capture_range))
    || (d->sens.temperat < d->param.idlreg_turn_on_temp && d->param.tmp_use))
  return 0;

 //вычисляем значение ошибки, ограничиваем ошибку (если нужно), а также, если мы в зоне
 //нечувствительности, то используем расчитанную ранее коррекцию.
 error = d->param.idling_rpm - d->sens.frequen;
 restrict_value_to(&error, -200, 200);
 if (abs(error) <= d->param.MINEFR)
  return idl_prstate.output_state;

 //выбираем необходимый коэффициент регулятора, в зависимости от знака ошибки
 if (error > 0)
  factor = d->param.ifac1;
 else
  factor = d->param.ifac2;

 //изменяем значение коррекции только по таймеру idle_period_time_counter
 if (s_timer_is_action(*io_timer))
 {
  s_timer_set(*io_timer,IDLE_PERIOD_TIME_VALUE);
  idl_prstate.output_state = idl_prstate.output_state + (error * factor) / 4;
 }
 //ограничиваем коррекцию нижним и верхним пределами регулирования
 restrict_value_to(&idl_prstate.output_state, d->param.idlreg_min_angle, d->param.idlreg_max_angle);

 return idl_prstate.output_state;
}

//Нелинейный фильтр ограничивающий скорость изменения УОЗ на переходных режимах двигателя
//new_advance_angle - новое значение УОЗ
//ip_prev_state - значение УОЗ в предыдущем цикле
//intstep_p,intstep_m - значения положительного и отрицательного шагов интегрирования, положительные числа
//Возвращает скорректированный УОЗ
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m)
{
 int16_t difference;
 difference = new_advance_angle - *ip_prev_state;

 if (difference > intstep_p)
 {
  (*ip_prev_state)+= intstep_p;
  return *ip_prev_state;
 }

 if (difference < -intstep_m)
 {
  (*ip_prev_state)-= intstep_m;
  return *ip_prev_state;
 }

 //текущий УОЗ будет предыдущим в следующий раз
 *ip_prev_state = new_advance_angle;
 return *ip_prev_state;
}

//ограничивает указанное значение указанными пределами
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit)
{
 if (*io_value > i_top_limit)
  *io_value = i_top_limit;
 if (*io_value < i_bottom_limit)
  *io_value = i_bottom_limit;
}

// Реализует функцию коэффициента усиления аттенюатора от оборотов
// Возвращает код 0...63 соответсутвующий определенному коэфф. усиления
//(см. HIP9011 datasheet).
uint8_t knock_attenuator_function(struct ecudata_t* d)
{
 int16_t i, i1, rpm = d->sens.inst_frq;

 if (rpm < 200) rpm = 200; //200 - минимальное значение оборотов по оси

 i = (rpm - 200) / 60;   //60 - шаг по оборотам

 if (i >= (KC_ATTENUATOR_LOOKUP_TABLE_SIZE-1))
  i = i1 = (KC_ATTENUATOR_LOOKUP_TABLE_SIZE-1);
 else
  i1 = i + 1;

 return simple_interpolation(rpm, PGM_GET_BYTE(&fw_data.exdata.attenuator_table[i]),
        PGM_GET_BYTE(&fw_data.exdata.attenuator_table[i1]), (i * 60) + 200, 60, 16) >> 4;
}

#ifdef DWELL_CONTROL
uint16_t accumulation_time(struct ecudata_t* d)
{
 int16_t i, i1, voltage = d->sens.voltage;

 if (voltage < VOLTAGE_MAGNITUDE(5.4))
  voltage = VOLTAGE_MAGNITUDE(5.4); //5.4 - минимальное значение напряжения в таблице предусмотренной для 12В бортовой сети

 i = (voltage - VOLTAGE_MAGNITUDE(5.4)) / VOLTAGE_MAGNITUDE(0.4);   //0.4 - шаг по напряжению

 if (i >= COIL_ON_TIME_LOOKUP_TABLE_SIZE-1) i = i1 = COIL_ON_TIME_LOOKUP_TABLE_SIZE-1;
  else i1 = i + 1;

 return simple_interpolation(voltage, PGM_GET_WORD(&fw_data.exdata.coil_on_time[i]), PGM_GET_WORD(&fw_data.exdata.coil_on_time[i1]),
        (i * VOLTAGE_MAGNITUDE(0.4)) + VOLTAGE_MAGNITUDE(5.4), VOLTAGE_MAGNITUDE(0.4), 8) >> 3;
}
#endif

#ifdef THERMISTOR_CS
//Coolant sensor is thermistor (тип датчика температуры - термистор)
//Note: We assume that voltage on the input of ADC depend on thermistor's resistance linearly.
//Voltage on the input of ADC can be calculated as following (divider resistors are used):
// U3=U1*Rt*R2/(Rp(Rt+R1+R2)+Rt(R1+R2));
// Rt - thermistor, Rp - pulls up thermistor to voltage U1,
// R1,R2 - voltage divider resistors.
int16_t thermistor_lookup(uint16_t adcvalue)
{
 int16_t i, i1;

 //Voltage value at the start of axis in ADC discretes (значение напряжения в начале оси в дискретах АЦП)
 uint16_t v_start = PGM_GET_WORD(&fw_data.exdata.cts_vl_begin);
 //Voltage value at the end of axis in ADC discretes (значение напряжения в конце оси в дискретах АЦП)
 uint16_t v_end = PGM_GET_WORD(&fw_data.exdata.cts_vl_end);

 uint16_t v_step = (v_end - v_start) / (THERMISTOR_LOOKUP_TABLE_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= THERMISTOR_LOOKUP_TABLE_SIZE-1) i = i1 = THERMISTOR_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(adcvalue, PGM_GET_WORD(&fw_data.exdata.cts_curve[i]), PGM_GET_WORD(&fw_data.exdata.cts_curve[i1]),
        (i * v_step) + v_start, v_step, 16)) >> 4;
}
#endif

#ifdef SM_CONTROL
uint8_t choke_closing_lookup(struct ecudata_t* d, int16_t* p_prev_temp)
{
 int16_t i, i1, t = d->sens.temperat;

 if (!d->param.tmp_use)
  return 0;   //блок не укомплектован ДТОЖ-ом

 //if difference between current and previous temperature values is less than +/-1.5,
 //then previous value will be used for calculations.
 if (abs(*p_prev_temp - t) < TEMPERATURE_MAGNITUDE(1.5))
  t = *p_prev_temp;
 else
  *p_prev_temp = t; //make it current

 //-5 - минимальное значение температуры
 if (t < TEMPERATURE_MAGNITUDE(-5))
  t = TEMPERATURE_MAGNITUDE(-5);

 //5 - шаг между узлами интерполяции по температуре
 i = (t - TEMPERATURE_MAGNITUDE(-5)) / TEMPERATURE_MAGNITUDE(5);

 if (i >= 15) i = i1 = 15;
 else i1 = i + 1;

 return simple_interpolation(t, PGM_GET_BYTE(&fw_data.exdata.choke_closing[i]), PGM_GET_BYTE(&fw_data.exdata.choke_closing[i1]),
 (i * TEMPERATURE_MAGNITUDE(5)) + TEMPERATURE_MAGNITUDE(-5), TEMPERATURE_MAGNITUDE(5), 16) >> 4;
}

/**Describes state data for idling regulator */
typedef struct
{
 int16_t int_state;   //!< regulator's memory (integrated error)
}chokeregul_state_t;

/**Variable. State data for choke RPM regulator */
chokeregul_state_t choke_regstate;

//reset of choke RPM regulator state
void chokerpm_regulator_init(void)
{
 choke_regstate.int_state = 0;
}

int16_t choke_rpm_regulator(struct ecudata_t* d, int16_t* p_prev_corr)
{
 int16_t error, rpm, t = d->sens.temperat;

 if (0==d->param.choke_rpm[0])
 {
  *p_prev_corr = 0;
  return 0; //regulator is turned off, return zero correction
 }

 //-5 - значение температуры cоответствующее оборотам в первой точке
 //70 - значение температуры соответствующее оборотам во второй точке
 restrict_value_to(&t, TEMPERATURE_MAGNITUDE(-5), TEMPERATURE_MAGNITUDE(70));

 //calculate target RPM value for regulator
 rpm = simple_interpolation(t, d->param.choke_rpm[0], d->param.choke_rpm[1],
 TEMPERATURE_MAGNITUDE(-5), TEMPERATURE_MAGNITUDE(75), 4) >> 2;

 error = rpm - d->sens.frequen;
 if (abs(error) <= 25)   //dead band is +/-25 RPM
  return *p_prev_corr;

 choke_regstate.int_state+= error; //update integrator's state
 restrict_value_to(&choke_regstate.int_state, -8000, 8000); //restrict integrаtor output

 *p_prev_corr = (((int32_t)d->param.choke_rpm_if) * choke_regstate.int_state) >> 14; //additional 4 shift bits to reduce regulator's influence
 restrict_value_to(p_prev_corr, -d->param.sm_steps, d->param.sm_steps); //range must be: +/- d->param.sm_steps

 return *p_prev_corr;
}
#endif

#ifdef AIRTEMP_SENS
//Реализует функцию коррекции УОЗ по температуре воздуха(°C) 
// Возвращает значение угла опережения в целом виде * 32
int16_t airtemp_function(struct ecudata_t* d)
{
 int16_t i, i1, t = d->sens.air_temp;

 if (0)  //todo: check if input remmaped
  return 0;   //do not use correcton if air temperature sensor is turned off

 //-30 - minimum temperature value
 if (t < TEMPERATURE_MAGNITUDE(-30))
  t = TEMPERATURE_MAGNITUDE(-30);

 //10 - step between interpolation points
 i = (t - TEMPERATURE_MAGNITUDE(-30)) / TEMPERATURE_MAGNITUDE(10);

 if (i >= 15) i = i1 = 15;
 else i1 = i + 1;

 return simple_interpolation(t, _GB(&fw_data.exdata.ats_corr[i]), _GB(&fw_data.exdata.ats_corr[i1]),
 (i * TEMPERATURE_MAGNITUDE(10)) + TEMPERATURE_MAGNITUDE(-30), TEMPERATURE_MAGNITUDE(10), 16);
}

int16_t ats_lookup(uint16_t adcvalue)
{
 int16_t i, i1;

 //Voltage value at the start of axis in ADC discretes (значение напряжения в начале оси в дискретах АЦП)
 uint16_t v_start = PGM_GET_WORD(&fw_data.exdata.ats_vl_begin);
 //Voltage value at the end of axis in ADC discretes (значение напряжения в конце оси в дискретах АЦП)
 uint16_t v_end = PGM_GET_WORD(&fw_data.exdata.ats_vl_end);

 uint16_t v_step = (v_end - v_start) / (ATS_LOOKUP_TABLE_SIZE - 1);

 if (adcvalue < v_start)
  adcvalue = v_start;

 i = (adcvalue - v_start) / v_step;

 if (i >= ATS_LOOKUP_TABLE_SIZE-1) i = i1 = ATS_LOOKUP_TABLE_SIZE-1;
 else i1 = i + 1;

 return (simple_interpolation(adcvalue, PGM_GET_WORD(&fw_data.exdata.ats_curve[i]), PGM_GET_WORD(&fw_data.exdata.ats_curve[i1]),
        (i * v_step) + v_start, v_step, 16)) >> 4;
}
#endif //AIRTEMP_SENS
