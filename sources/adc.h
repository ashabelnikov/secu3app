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

/** \file adc.h
 * ADC related functions (API).
 * Functions for read values from ADC, perform conversion to phisical values etc
 * (Функции для работы с АЦП, считывание значений, преобразование в физические величины и т.д.).
 */

#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>

/**одна дискрета АЦП в вольтах */
#define ADC_DISCRETE            0.0025

/**наклон прямой датчика температуры вольт/градус */
#define TSENS_SLOPP             0.01

#ifndef THERMISTOR_CS
/**напряжение на выходе датчика температуры при 0 градусов цельсия */
 #define TSENS_ZERO_POINT        2.73
#else
 /**Voltage which corresponds to minimum temperature */
 #define TSENS_V_TMIN            4.37
 /**Voltage which is step between interpolation nodes in table */
 #define TSENS_STEP              0.27
#endif

/**константа для выбора источника опорного напряжения */
#define ADC_VREF_TYPE           0xC0

/**дискретность физической величины - ДАД */
#define MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER  64

/**дискретность физической величины - напряжения */
#define UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER (1.0/ADC_DISCRETE) //=400

/**дискретность физической величины - ДТОЖ */
#define TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER (TSENS_SLOPP / ADC_DISCRETE) //=4

/** Получение последнего измеренного значения с ДАД
 * \return значение в дискретах АЦП
 */
uint16_t adc_get_map_value(void);

/** Получение последнего измеренного значения напряжения бортовой сети
 * \return значение в дискретах АЦП
 */
uint16_t adc_get_ubat_value(void);

/** Получение последнего измеренного значения с ДТОЖ
 * \return значение в дискретах АЦП
 */
uint16_t adc_get_temp_value(void);

/** Получение последнего измеренного значения сигнала детонации
 * \return значение в дискретах АЦП
 */
uint16_t adc_get_knock_value(void);

/**запускает измерение значений с датчиков, но только если предыдущее
 * измерение завершено.
 */
void adc_begin_measure(uint8_t speed2x);

/**запускает измерение значения с интегратора канала детонации. Так как после установки
 * сигнала INT/HOLD в 0 выход INTOUT перейдет в полностью корректное состояние только через
 * 20мкс (приблизительно), а запуск измерения может быть произведен сразу, то делаем первое
 * измерение холостым.
 */
void adc_begin_measure_knock(uint8_t speed2x);

/**запускает измерение значений с датчиков и сигнала с ДД. Первыми снимаются значения
 * с датчиков, последним сигнал с ДД
 */
void adc_begin_measure_all(void);

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
int16_t adc_compensate(int16_t adcvalue, int16_t factor, int32_t correction);

/**переводит значение АЦП в физическую величину - давление
 * \param adcvalue значение в даскретах АЦП
 * \param offset смещение кривой ДАД (Curve offset. Can be negative)
 * \param gradient наклон кривой ДАД (Curve gradient. If < 0, then it means characteristic curve is inverse)
 * \return физическая величина * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER
 * \details
 * offset  = offset_volts / ADC_DISCRETE, где offset_volts - значение в вольтах;
 * gradient = 128 * gradient_kpa * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER * ADC_DISCRETE, где gradient_kpa значение в кило-паскалях
 */
uint16_t map_adc_to_kpa(int16_t adcvalue, int16_t offset, int16_t gradient);

/**переводит значение АЦП в физическую величину - напряжение
 * \param adcvalue значение в дискретах АЦП
 * \return физическая величина * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER
 */
uint16_t ubat_adc_to_v(int16_t adcvalue);

#ifndef THERMISTOR_CS
/**Converts ADV value into phisical magnituge - temperature (given from linear sensor)
 * Переводит значение АЦП в физическую величину - температура, для линейного датчика
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП)
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER
 */
int16_t temp_adc_to_c(int16_t adcvalue);
#else
/**Converts ADC value into phisical magnitude - temperature (given from thermistor)
 * (переводит значение АЦП в физическую величину - температура для резистивного датчика (термистор))
 * \param start Voltage value at lowest temperature in ADC discretes (значение напряжения при минимальной температуре в дискретах АЦП)
 * \param step Voltage step in ADC discretes (значение шага по напряжению в таблице , в дискретах АЦП)
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП))
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER
 */
int16_t thermistor_lookup(uint16_t start, uint16_t step, uint16_t adcvalue);
#endif

#endif //_ADC_H_
