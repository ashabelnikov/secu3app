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

//��������� ��� ������ ��������� �������� ���������� � �����. ����������� �������� ����������
#ifdef VREF_5V //5V
 #define ADC_VREF_TYPE          0x40    //!< Vref selection constant for 5V
 #define ADC_VREF_FACTOR        1.9531  //!< Vref compensation factor (5.0V/2.56V)
#else         //internal 2.56V
 #define ADC_VREF_TYPE          0xC0    //!< Vref selection constant for 2.56V
 #define ADC_VREF_FACTOR        1.0000  //!< Vref compensation factor (2.56V/2.56V)
#endif

#define ADC_MCP3204_FACTOR      0.488281 //!<2000/4096

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

/** Get MAPdot value (dv/dt)
 * \return 1-st derivative value of the MAP (V/s), can be negative, voltage in ADC discretes
 */
int16_t adc_get_mapdot_value(void);
#endif

/** Get last measured value from the knock sensor(s) or ADD_I4 (if TPIC8101 option defined)
 * \return value in ADC discretes
 */
uint16_t adc_get_knock_value(void);

/**��������� ��������� �������� � ��������, �� ������ ���� ����������
 * ��������� ���������.
 * \param speed2x Double ADC clock (0,1) (�������� �������� ������� ���)
 */
void adc_begin_measure(uint8_t speed2x);

#ifndef TPIC8101
/**��������� ��������� �������� � ����������� ������ ���������. ��� ��� ����� ���������
 * ������� INT/HOLD � 0 ����� INTOUT �������� � ��������� ���������� ��������� ������ �����
 * 20��� (��������������), � ������ ��������� ����� ���� ���������� �����, �� ������ ������
 * ��������� ��������.
 * \param speed2x Double ADC clock (0,1) (�������� �������� ������� ���)
 */
void adc_begin_measure_knock(uint8_t speed2x);
#endif

/**�������� ���������� ���
 *\return ���������� �� 0 ���� ��������� ������ (��� �� ������)
 */
uint8_t adc_is_measure_ready(void);

/**������������� ��� � ��� ���������� ��������� */
void adc_init(void);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Set value of minimum dt (time diffrencial) used by calculation of TPS dot
 * mindt Value of minimum time differencial in 3.2 us units
 */
void adc_set_tpsdot_mindt(uint16_t mindt);

/** Set value of minimum dt (time diffrencial) used by calculation of MAP dot
 * mindt Value of minimum time differencial in 3.2 us units
 */
void adc_set_mapdot_mindt(uint16_t mindt);
#endif

/**����������� ������������ ��� ��� ������� ����� (����������� �������� � ������������ �����������)
 * \param adcvalue �������� ��� ��� �����������
 * \param factor ���������� ���������������
 * \param correction ��������
 * \return compensated value (���������������� ��������)
 * \details
 * factor = 2^14 * gainfactor,
 * correction = 2^14 * (0.5 - offset * gainfactor),
 * 2^16 * realvalue = 2^2 * (adcvalue * factor + correction)
 */
int16_t adc_compensate(int16_t adcvalue, uint16_t factor, int32_t correction);

/**��������� �������� ��� � ���������� �������� - ��������
 * \param adcvalue �������� � ��������� ���
 * \param offset �������� ������ ��� (Curve offset. Can be negative)
 * \param gradient ������ ������ ��� (Curve gradient. If < 0, then it means characteristic curve is inverted)
 * \return ���������� �������� * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER
 * \details
 * offset  = offset_volts / ADC_DISCRETE, ��� offset_volts - �������� � �������;
 * gradient = 128 * gradient_kpa * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER * ADC_DISCRETE, ��� gradient_kpa �������� � ����-��������
 */
uint16_t map_adc_to_kpa(int16_t adcvalue, int16_t offset, int16_t gradient);

/**��������� �������� ��� � ���������� �������� - ����������
 * \param adcvalue �������� � ��������� ���
 * \return ���������� �������� * UBAT_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
uint16_t ubat_adc_to_v(int16_t adcvalue);

/**Converts ADC value into phisical magnituge - temperature (given from linear sensor)
 * ��������� �������� ��� � ���������� �������� - �����������, ��� ��������� �������
 * \param adcvalue Voltage from sensor (���������� � ������� - �������� � ��������� ���)
 * \return ���������� �������� * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t temp_adc_to_c(int16_t adcvalue);

/**Converts ADC value of the Throttle Position Sensor to the percentage of throttle opening
 * \param adcvalue �������� � ��������� ��� (Value in ADC discretes)
 * \param offset �������� ������ ���� (Curve offset. Can be negative)
 * \param gradient ������ ������ ���� (Curve gradient)
 * \return percentage * 2 (e.g. value of 200 is 100%)
 */
uint8_t tps_adc_to_pc(int16_t adcvalue, int16_t offset, int16_t gradient);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/**Converts ADC value discretes/sec of the TPSdot to the %/sec value
 * \param adcvalue Value in ADC discretes
 * \param gradient Curve gradient of TPS
 * \return percentage/sec
 */
int16_t tpsdot_adc_to_pc(int16_t adcvalue, int16_t gradient);

/**Converts ADC value discretes/sec of the MAPdot to the kPa/sec value
 * \param adcvalue Value in ADC discretes
 * \param gradient Curve gradient of MAP
 * \return kPa/sec
 */
int16_t mapdot_adc_to_kpa(int16_t adcvalue, int16_t gradient);
#endif

/**Measure value of voltage in special mode when interrupts are disabled.
 * Call adc_get_ubat_value() to obtain result of measurement
 */
void adc_measure_voltage(void);

#if !defined(SECU3T) && defined(MCP3204)
/** Get last measured value from ADD_I5
 * \return value in the ADC discretes
 */
uint16_t adc_get_add_i5_value(void);

/** Get last measured value from ADD_I6
 * \return value in the ADC discretes
 */
uint16_t adc_get_add_i6_value(void);

/** Get last measured value from ADD_I7
 * \return value in the ADC discretes
 */
uint16_t adc_get_add_i7_value(void);

/** Get last measured value from ADD_I8
 * \return value in the ADC discretes
 */
uint16_t adc_get_add_i8_value(void);
#endif

#endif //_ADC_H_
