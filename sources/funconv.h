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

/** \file funconv.h
 * \author Alexey A. Shabelnikov
 * Core mathematics and regulation logic.
 * (Основная часть математического аппарата и логики регулирования).
 */

#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include <stdint.h>
#include "vstimer.h"

#define IDLE_PERIOD_TIME_VALUE        50    //!< used by idling regulator
#define IDLE_ENTER_TIME_VALUE         150   //!< time for entering closed loop mode

/** f(x) liniar interpolation for function with single argument
 * \param x argument value
 * \param a1 function value at the beginning of interval
 * \param a2 function value at the end of interval
 * \param x_s argument value at the beginning of interval
 * \param x_l length of interval in x
 * \param m function multiplier
 * \return interpolated value of function * m
 */
int16_t simple_interpolation(int16_t x,int16_t a1,int16_t a2,int16_t x_s,int16_t x_l, uint8_t m);

/** f(x,y) liniar interpolation for function with two arguments
 * \param x first argument value
 * \param y second argument value
 * \param a1 function value at the beginning of interval (1 corner)
 * \param a2 function value at the beginning of interval (2 corner)
 * \param a3 function value at the beginning of interval (3 corner)
 * \param a4 function value at the beginning of interval (4 corner)
 * \param x_s first argument value at the beginning of interval
 * \param y_s second argument value at the beginning of interval
 * \param x_l length of interval in x
 * \param y_l length of interval in y
 * \return interpolated value of function * 16
 */
int16_t bilinear_interpolation(int16_t x,int16_t y,int16_t a1,int16_t a2,int16_t a3,int16_t a4,int16_t x_s,int16_t y_s,int16_t x_l,int16_t y_l);

struct ecudata_t;

/** Calculates advance angle from "start" map
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t start_function(struct ecudata_t* d);

/** Calculates advance angle from "idle" map
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t idling_function(struct ecudata_t* d);

/** Calculates advance angle from "work" map
 * \param d pointer to ECU data structure
 * \param i_update_airflow_only
 * \return value of advance angle * 32
 */
int16_t work_function(struct ecudata_t* d, uint8_t i_update_airflow_only);

/** Calculates advance angle correction using coolant temperature
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t coolant_function(struct ecudata_t* d);

/** Knock attenuator look up table function
 * \param d pointer to ECU data structure
 * \return
 */
uint8_t knock_attenuator_function(struct ecudata_t* d);

/**Initialization of idling regulator's data structures */
void idling_regulator_init(void);

/** Idling RPM Regulator function
 * \param d pointer to ECU data structure
 * \param io_timer
 * \return value of advance angle * 32
 */
int16_t idling_pregulator(struct ecudata_t* d, volatile s_timer8_t* io_timer);

/** function for restricting of advance angle alternation speed
 * \param new_advance_angle New value of advance angle (input)
 * \param ip_prev_state state variable for storing value between calls of function
 * \param intstep_p Speed limit for increasing
 * \param intstep_m Speed limit for decreasing
 * \return value of advance angle * 32
 */
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m);

/** Restricts specified value to specified limits
 * \param io_value pointer to value to be restricted. This parameter will also receive result.
 * \param i_bottom_limit bottom limit
 * \param i_top_limit upper limit
 */
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit);

#ifdef DWELL_CONTROL
/** Calculates current accumulation time (dwell control) using current board voltage
 * \param d pointer to ECU data structure
 * \return accumulation time in timer's ticks (1 tick = 4uS, when clock is 16mHz and 1 tick = 3.2uS, when clock is 20mHz)
 */
uint16_t accumulation_time(struct ecudata_t* d);
#endif

#ifdef THERMISTOR_CS
/**Converts ADC value into phisical magnitude - temperature (given from thermistor)
 * (переводит значение АЦП в физическую величину - температура для резистивного датчика (термистор))
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП))
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t thermistor_lookup(uint16_t adcvalue);
#endif

#ifdef SM_CONTROL
/** Obtains choke position (closing %) from coolant temperature using lookup table
 * Получает положение воздушной заслонки (% закрытия) по температре охлаждающей жидкости
 * \param d pointer to ECU data structure
 * \param p_prev_temp pointer to state variable used to store temperature value between calls of
 * this function
 * \return choke closing percentage (value * 2)
 */
uint8_t choke_closing_lookup(struct ecudata_t* d, int16_t* p_prev_temp);

/**Initialization of regulator's data structures*/
void chokerpm_regulator_init(void);

/** RPM regulator function for choke position
 * \param d pointer to ECU data structure
 * \param p_prev_corr pointer to state variable used to store calculated correction between calls of
 * this function
 * \return choke closing correction in SM steps
 */
int16_t choke_rpm_regulator(struct ecudata_t* d, int16_t* p_prev_corr);
#endif

#ifdef AIRTEMP_SENS
/** Calculates advance angle correction using intake air temperature
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t airtemp_function(struct ecudata_t* d);

/**Converts ADC value into phisical magnitude - temperature (given from air temperature sensor)
 * (переводит значение АЦП в физическую величину - температура воздуха)
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП))
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t ats_lookup(uint16_t adcvalue);
#endif

#ifdef FUEL_INJECT
/** Calculates basic injection time using Ideal gas law, VE and AFR lookup tables
 * \param d pointer to ECU data structure
 * \return Base injection time in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_base_pw(struct ecudata_t* d);

/** Calculates injector dead time using lookup table
 * \param d pointer to ECU data structure
 * \return Injector dead time in tics of timer
 */
uint16_t inj_dead_time(struct ecudata_t* d);

/** Calculates pulse width used when engine is stating up
 * \param d pointer to ECU data structure
 * \return Cranking pulse width in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_cranking_pw(struct ecudata_t* d);


/** Calculates afterstart enrichemnt factor using a lookup table
 * \param d pointer to ECU data structure
 * \return Afterstart enrichment * 128
 */
uint8_t inj_aftstr_en(struct ecudata_t* d);

/** Calculates IAC/PWM position vs coolant temperature using a lookup table.
 * This function is used in open-loop idle control algorithm.
 * \param d pointer to ECU data structure
 * \param p_prev_temp pointer to state variable used to store temperature value between calls of
 * this function
 * \param mode 1 - run, 0 - cranking
 * \return position percentage (value * 2)
 */
uint8_t inj_iac_pos_lookup(struct ecudata_t* d, int16_t* p_prev_temp, uint8_t mode);

/** Calculates injection timing from lookup table
 * \param d pointer to ECU data structure
 * \return Injection timing in crank degrees * ANGLE_MULTIPLIER
 */
int16_t inj_timing_lookup(struct ecudata_t* d);

#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates warmup enrichemnt factor using a lookup table
 * \param d pointer to ECU data structure
 * \return Warmup enrichment * 128
 */
uint8_t inj_warmup_en(struct ecudata_t* d);

/** Calculates TPS based acceleration value
 * \param d pointer to ECU data structure
 * \return acceleration factor * 128, value can be negative
 */
int16_t inj_ae_tps_lookup(struct ecudata_t* d);

/** Calculates RPM correction factor for AE
 * \param d pointer to ECU data structure
 * \return factor * 128, positive value
 */
uint8_t inj_ae_rpm_lookup(struct ecudata_t* d);
#endif

#ifdef FUEL_INJECT

/** Calculates CLT correction factor for AE
 * \param d pointer to ECU data structure
 * \return factor * 128, positive value (1.0...2.99)
 */
uint16_t inj_ae_clt_corr(struct ecudata_t* d);

/** Calculates prime pulse width from coolant temperature
 * \param d pointer to ECU data structure
 * \return PW in tics of timer (1 tick = 3.2uS)
 */
uint16_t inj_prime_pw(struct ecudata_t* d);

/** Calculates target idling RPM from coolant temperature
 * \param d pointer to ECU data structure
 * \return RPM value in min-1 units
 */
uint16_t inj_idling_rpm(struct ecudata_t* d);

#endif

#ifdef PA4_INP_IGNTIM
/**Ignition timing correction vs voltage at the PA4 input */
int16_t pa4_function(uint16_t adcvalue);
#endif


#ifdef GD_CONTROL
/**Use VE and AFR tables for gas dosator. Result = VE * AFR * K, where K is stoichiometry constant
 * \return value * 2048
 */
uint16_t gd_ve_afr(struct ecudata_t* d);
#endif


#endif //_FUNCONV_H_
