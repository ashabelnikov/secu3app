/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

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

/** \file adc.h
 * \author Alexey A. Shabelnikov
 * ADC related functions (API).
 * Functions for read values from ADC, perform conversion to phisical values etc
 */

#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

/** ADC discrete in Volts */
#define ADC_DISCRETE            0.0025

/**Slope of curve of the coolant temperature sensor volt/degree */
#define TSENS_SLOPP             0.01

/**Voltage on the output of coolant temperature sensor at 0 degrees */
#define TSENS_ZERO_POINT        2.73

//константа для выбора источника опорного напряжения и коэфф. компенсации опорного напряжения
#ifdef VREF_5V //5V
 #define ADC_VREF_TYPE          0x40    //!< Vref selection constant for 5V
 #define ADC_VREF_FACTOR        1.9531  //!< Vref compensation factor (5.0V/2.56V)
#else         //internal 2.56V
 #define ADC_VREF_TYPE          0xC0    //!< Vref selection constant for 2.56V
 #define ADC_VREF_FACTOR        1.0000  //!< Vref compensation factor (2.56V/2.56V)
#endif

/** Get last measured value of MAP
 * \return value in ADC discretes
 */
uint16_t adc_get_map_value(void);

/** Get last measured value of board voltage
 * \return value in ADC discretes
 */
uint16_t adc_get_ubat_value(void);

/** Get last measured value of CLT
 * \return value in ADC discretes
 */
uint16_t adc_get_temp_value(void);

/** Get last measured value of ADD_I1
 * \return value in ADC discretes
 */
uint16_t adc_get_add_i1_value(void);

/** Get last measured value of ADD_I2
 * \return value in ADC discretes
 */
uint16_t adc_get_add_i2_value(void);

#if !defined(SECU3T) || defined(PA4_INP_IGNTIM)
/** Get last measured value from ADD_I3
 * \return value in the ADC discretes
 */
uint16_t adc_get_add_i3_value(void);
#endif

/** Get latest measured value from throttle gate position sensor
 * \return value in ADC discretes
 */
uint16_t adc_get_carb_value(void);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Get TPSdot value (dv/dt)
 * \return 1-st derivative value of TPS position (V/s), can be negative, voltage in ADC discretes
 */
int16_t adc_get_tpsdot_value(void);
#endif

/** Get last measured value from the knock sensor(s) or ADD_I4 (if TPIC8101 option defined)
 * \return value in ADC discretes
 */
uint16_t adc_get_knock_value(void);

/**запускает измерение значений с датчиков, но только если предыдущее
 * измерение завершено.
 * \param speed2x Double ADC clock (0,1) (Удвоение тактовой частоты АЦП)
 */
void adc_begin_measure(uint8_t speed2x);

#ifndef TPIC8101
/**запускает измерение значения с интегратора канала детонации. Так как после установки
 * сигнала INT/HOLD в 0 выход INTOUT перейдет в полностью корректное состояние только через
 * 20мкс (приблизительно), а запуск измерения может быть произведен сразу, то делаем первое
 * измерение холостым.
 * \param speed2x Double ADC clock (0,1) (Удвоение тактовой частоты АЦП)
 */
void adc_begin_measure_knock(uint8_t speed2x);
#endif

/**проверка готовности АЦП
 *\return возвращает не 0 если измерение готово (АЦП не занято)
 */
uint8_t adc_is_measure_ready(void);

/**инициализация АЦП и его переменных состояния */
void adc_init(void);

/**компенсация погрешностей АЦП или входных цепей (погрешность смещения и передаточная погрешность)
 * \param adcvalue значение АЦП для компенсации
 * \param factor коэффициен масштабирования
 * \param correction смещение
 * \return compensated value (компенсированное значение)
 * \details
 * factor = 2^14 * gainfactor,
 * correction = 2^14 * (0.5 - offset * gainfactor),
 * 2^16 * realvalue = 2^2 * (adcvalue * factor + correction)
 */
int16_t adc_compensate(int16_t adcvalue, uint16_t factor, int32_t correction);

/**переводит значение АЦП в физическую величину - давление
 * \param adcvalue значение в дискретах АЦП
 * \param offset смещение кривой ДАД (Curve offset. Can be negative)
 * \param gradient наклон кривой ДАД (Curve gradient. If < 0, then it means characteristic curve is inverted)
 * \return физическая величина * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER
 * \details
 * offset  = offset_volts / ADC_DISCRETE, где offset_volts - значение в вольтах;
 * gradient = 128 * gradient_kpa * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER * ADC_DISCRETE, где gradient_kpa значение в кило-паскалях
 */
uint16_t map_adc_to_kpa(int16_t adcvalue, int16_t offset, int16_t gradient);

/**переводит значение АЦП в физическую величину - напряжение
 * \param adcvalue значение в дискретах АЦП
 * \return физическая величина * UBAT_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
uint16_t ubat_adc_to_v(int16_t adcvalue);

/**Converts ADC value into phisical magnituge - temperature (given from linear sensor)
 * Переводит значение АЦП в физическую величину - температура, для линейного датчика
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП)
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t temp_adc_to_c(int16_t adcvalue);

/**Converts ADC value of the Throttle Position Sensor to the percentage of throttle opening
 * \param adcvalue значение в дискретах АЦП (Value in ADC discretes)
 * \param offset смещение кривой ДПДЗ (Curve offset. Can be negative)
 * \param gradient наклон кривой ДПДЗ (Curve gradient)
 * \return percentage * 2 (e.g. value of 200 is 100%)
 */
uint8_t tps_adc_to_pc(int16_t adcvalue, int16_t offset, int16_t gradient);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/**Converts ADC value discretes/sec of the TPSdot to the %/sec value
 * \param adcvalue значение в дискретах АЦП (Value in ADC discretes)
 * \param gradient наклон кривой ДПДЗ (Curve gradient)
 * \return percentage/sec
 */
int16_t tpsdot_adc_to_pc(int16_t adcvalue, int16_t gradient);
#endif

/**Measure value of voltage in special mode when interrupts are disabled.
 * Call adc_get_ubat_value() to obtain result of measurement
 */
void adc_measure_voltage(void);

#endif //_ADC_H_
