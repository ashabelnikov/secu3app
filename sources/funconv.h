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

#define IDLE_PERIOD_TIME_VALUE        25    //!< used by idling regulator
#define IDLE_ENTER_TIME_VALUE         150   //!< time for entering closed loop mode

/** Calculates advance angle from "start" map
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t start_function(void);

/** Calculates advance angle from "idle" map
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t idling_function(void);

/** Calculates advance angle from "work" map
 * Uses d ECU data structure
 * \param i_update_airflow_only
 * \return value of advance angle * 32
 */
int16_t work_function(uint8_t i_update_airflow_only);

/** Calculates advance angle correction using coolant temperature
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t coolant_function(void);

/** Knock attenuator look up table function
 * Uses d ECU data structure
 * \return
 */
uint8_t knock_attenuator_function(void);

/**Initialization of idling regulator's data structures */
void idling_regulator_init(void);

/** Idling RPM Regulator function
 * Uses d ECU data structure
 * \param io_timer
 * \return value of advance angle * 32
 */
int16_t idling_pregulator(volatile s_timer8_t* io_timer);

/** function for restricting of advance angle alternation speed
 * \param new_advance_angle New value of advance angle (input)
 * \param ip_prev_state state variable for storing value between calls of function
 * \param intstep_p Speed limit for increasing
 * \param intstep_m Speed limit for decreasing
 * \return value of advance angle * 32
 */
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m);

#ifdef DWELL_CONTROL
/** Calculates current accumulation time (dwell control) using current board voltage
 * Uses d ECU data structure
 * \return accumulation time in timer's ticks (1 tick = 4uS, when clock is 16mHz and 1 tick = 3.2uS, when clock is 20mHz)
 */
uint16_t accumulation_time(void);
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
 * Uses d ECU data structure
 * \param p_prev_temp pointer to state variable used to store temperature value between calls of
 * this function
 * \return choke closing percentage (value * 2)
 */
uint8_t choke_closing_lookup(int16_t* p_prev_temp);

/**Initialization of regulator's data structures*/
void chokerpm_regulator_init(void);

/** RPM regulator function for choke position
 * Uses d ECU data structure
 * \param p_prev_corr pointer to state variable used to store calculated correction between calls of
 * this function
 * \return choke closing correction in SM steps
 */
int16_t choke_rpm_regulator(int16_t* p_prev_corr);
#endif

#ifdef AIRTEMP_SENS
/** Calculates advance angle correction using intake air temperature
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t airtemp_function(void);

/**Converts ADC value into phisical magnitude - temperature (given from air temperature sensor)
 * (переводит значение АЦП в физическую величину - температура воздуха)
 * \param adcvalue Voltage from sensor (напряжение с датчика - значение в дискретах АЦП))
 * \return физическая величина * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t ats_lookup(uint16_t adcvalue);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates pulse width used when engine is stating up
 * Uses d ECU data structure
 * \return Cranking pulse width in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_cranking_pw(void);
#endif

#ifdef FUEL_INJECT
/** Calculates basic injection time using Ideal gas law, VE and AFR lookup tables
 * Uses d ECU data structure
 * \return Base injection time in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_base_pw(void);

/** Calculates injector dead time using lookup table
 * Uses d ECU data structure
 * \return Injector dead time in tics of timer
 */
uint16_t inj_dead_time(void);

/** Calculates IAC/PWM position vs coolant temperature using a lookup table.
 * This function is used in open-loop idle control algorithm.
 * Uses d ECU data structure
 * \param p_prev_temp pointer to state variable used to store temperature value between calls of
 * this function
 * \param mode 1 - run, 0 - cranking
 * \return position percentage (value * 2)
 */
uint8_t inj_iac_pos_lookup(int16_t* p_prev_temp, uint8_t mode);

/** Calculates injection timing from lookup table
 * Uses d ECU data structure
 * \return Injection timing in crank degrees * ANGLE_MULTIPLIER
 */
int16_t inj_timing_lookup(void);

#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates afterstart enrichemnt factor using a lookup table
 * Uses d ECU data structure
 * \return Afterstart enrichment * 128
 */
uint8_t inj_aftstr_en(void);

/** Calculates warmup enrichemnt factor using a lookup table
 * Uses d ECU data structure
 * \return Warmup enrichment * 128
 */
uint8_t inj_warmup_en(void);

/** Calculates TPS based acceleration value
 * Uses d ECU data structure
 * \return acceleration factor * 128, value can be negative
 */
int16_t inj_ae_tps_lookup(void);

/** Calculates RPM correction factor for AE
 * Uses d ECU data structure
 * \return factor * 128, positive value
 */
uint8_t inj_ae_rpm_lookup(void);
#endif

#ifdef FUEL_INJECT

/** Calculates CLT correction factor for AE
 * Uses d ECU data structure
 * \return factor * 128, positive value (1.0...2.99)
 */
uint16_t inj_ae_clt_corr(void);

/** Calculates prime pulse width from coolant temperature
 * Uses d ECU data structure
 * \return PW in tics of timer (1 tick = 3.2uS)
 */
uint16_t inj_prime_pw(void);

/** Calculates target idling RPM from coolant temperature
 * Uses d ECU data structure
 * \return RPM value in min-1 units
 */
uint16_t inj_idling_rpm(void);

/** Calculate idling regulator's rigidity
 * Uses d ECU data structure
 * \param targ_map Idling inlet manifold pressure
 * \param targ_rpm Idling RPM
 * \return value * 128
 */
uint16_t inj_idlreg_rigidity(uint16_t targ_map, uint16_t targ_rpm);

/** Calculates mixture correction coefficient based on the IAC and TPS positions
 * Uses d ECU data structure
 * \raturn value * 8192
 */
uint16_t inj_iacmixtcorr_lookup(void);

#endif

#ifdef PA4_INP_IGNTIM
/**Ignition timing correction vs voltage at the PA4 input */
int16_t pa4_function(uint16_t adcvalue);
#endif


#ifdef GD_CONTROL
/**Use VE and AFR tables for gas dosator. Result = VE * AFR * K, where K is stoichiometry constant
 * Uses d ECU data structure
 * \return value * 2048
 */
uint16_t gd_ve_afr(void);
#endif

#if defined(FUEL_INJECT) /*|| defined(CARB_AFR)*/ || defined(GD_CONTROL)
/** Converts ADC value (voltage) into AFR
 * Uses d ECU data structure
 * \return AFR * 128
 */
int16_t ego_curve_lookup(void);
int16_t ego_curve_min(void);
int16_t ego_curve_max(void);


/** Scales afterstart enrichment depending on the elapsed time (strokes)
 * Uses d ECU data structure
 * \param enrich_counter current value of counter of strokes (decriasing)
 * \return scaled afterstart enrichment factor (value * 128)
 */
uint8_t scale_aftstr_enrich(uint16_t enrich_counter);

#endif

#endif //_FUNCONV_H_
