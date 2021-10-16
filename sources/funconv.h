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
 */

#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include <stdint.h>
#include "vstimer.h"

#define IDLE_PERIOD_TIME_VALUE        25    //!< used by idling regulator

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
 * \return value of advance angle * 32
 */
int16_t work_function(void);

/** Calculates advance angle correction using coolant temperature
 * Uses d ECU data structure
 * \param mode 0 - calculate correction for idling, 1 - calculate correction for work mode
 * \return value of advance angle * 32
 */
int16_t coolant_function(uint8_t mode);

/** Calculates advance angle correction for cranking using coolant temperature
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t crkclt_function(void);

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

#if defined(DWELL_CONTROL) || defined(FUEL_INJECT)
/** Calculates current accumulation time (dwell control) / injector dead time using current board voltage
 * Uses d ECU data structure
 * \param mode Specifies what to calculate - dwell (mode=0) or dead time (mode=1)
 * \return accumulation time in timer's ticks (1 tick = 4uS, when clock is 16mHz and 1 tick = 3.2uS, when clock is 20mHz) /
 *         Injector dead time in tics of timer
 */
uint16_t accumulation_time(uint8_t mode);
#endif

#if defined(THERMISTOR_CS) || defined(AIRTEMP_SENS) || !defined(SECU3T)
/**Converts ADC value into phisical magnitude - temperature (given from thermistor)
 * \param adcvalue Voltage from sensor (in ADC discretes)
 * \param lutab Pointer to related look up table
 * \return physical magnitude * TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER
 */
int16_t thermistor_lookup(uint16_t adcvalue, int16_t _PGM *lutab);
#endif

#if defined(SM_CONTROL) && !defined(FUEL_INJECT)
/**Initialization of regulator's data structures*/
void chokerpm_regulator_init(void);

/** RPM regulator function for choke position
 * Uses d ECU data structure
 * \param p_prev_corr pointer to state variable used to store calculated correction between calls of
 * this function
 * \return choke closing correction in SM steps
 */
int16_t choke_rpm_regulator(int16_t* p_prev_corr);

/** Calculates time which determine delay for switching from cranking to work choke position
 * Uses d ECU data structure
 * \return time in 10ms units
 */
uint16_t choke_cranking_time(void);
#endif

#ifdef AIRTEMP_SENS
/** Calculates advance angle correction using intake air temperature
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t airtemp_function(void);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates pulse width used when engine is stating up
 * Uses d ECU data structure
 * \return Cranking pulse width in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_cranking_pw(void);

/** Updates precalculated values of VE and AFR
 * Uses d ECU data structure
 */
void calc_ve_afr(void);
#endif

#ifdef FUEL_INJECT

/** Calculates basic injection time using Ideal gas law, VE and AFR lookup tables
 * Uses d ECU data structure
 * \return Base injection time in ticks of timer (1 tick = 3.2uS)
 */
uint16_t inj_base_pw(void);

/** Calculates injection timing from lookup table
 * Uses d ECU data structure
 * \return Injection timing in crank degrees * ANGLE_MULTIPLIER
 */
int16_t inj_timing_lookup(void);

/** Gets inj. timing values from constants stored in the parameters
 * \param mode 0 - obtain valie for cranking mode, 1 - obtain value for running mode
 * \return inj. timing value * ANGLE_MULTIPLIER
 */
int16_t param_inj_timing(uint8_t mode);

#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL)
/** Holds state variables used by inj_iac_pos_lookup() */
typedef struct prev_temp_t
{
 int16_t clt;   //!< temperature
 uint8_t i;     //!< index
 uint8_t i1;    //!< index
}prev_temp_t;

/** Initialization of prev_temp_t structure
 * \param p_pt Pointer to prev_temp_t structure to be initialized
 */
void inj_init_prev_clt(prev_temp_t* p_pt);

/** Calculates IAC/PWM position vs coolant temperature using a lookup table.
 * This function is used in open-loop idle control algorithm.
 * Uses d ECU data structure
 * \param p_pt pointer to state variable used to store temperature value between calls of
 * this function
 * \param mode 1 - run, 0 - cranking
 * \return position percentage (value * 2)
 */
uint8_t inj_iac_pos_lookup(prev_temp_t* p_pt, uint8_t mode);

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
 * \param tpsdot TPS dot value (%/s)
 * \return acceleration factor * 128, value can be negative
 */
int16_t inj_ae_tps_lookup(int16_t tpsdot);

/** Calculates RPM correction factor for AE
 * Uses d ECU data structure
 * \return factor * 128, positive value
 */
uint8_t inj_ae_rpm_lookup(void);

/** Calculates CLT correction factor for AE
 * Uses d ECU data structure
 * \return factor * 128, positive value (1.0...2.99)
 */
uint16_t inj_ae_clt_corr(void);

#endif

#ifdef FUEL_INJECT

/** Calculates prime pulse width from coolant temperature
 * Uses d ECU data structure
 * \return PW in tics of timer (1 tick = 3.2uS)
 */
uint16_t inj_prime_pw(void);

/** Calculate idling regulator's rigidity
 * Uses d ECU data structure
 * \param targ_map Idling inlet manifold pressure
 * \param targ_rpm Idling RPM
 * \return value * 128
 */
uint16_t inj_idlreg_rigidity(uint16_t targ_map, uint16_t targ_rpm);

/** Calculates mixture correction coefficient based on the IAC and TPS positions
 * Uses d ECU data structure
 * \return value * 8192
 */
uint16_t inj_iacmixtcorr_lookup(void);

#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL)
/** Calculates target idling RPM from coolant temperature
 * Uses d ECU data structure
 * \return RPM value in min-1 units
 */
uint16_t inj_idling_rpm(void);
#endif

/** Calculates TPS switch point depending on RPM using a look up table
 * Uses d ECU data structure
 * \return TPS value in % * 2
 */
uint16_t tpsswt_function(void);

/** Calculates MAP/TPS load axis allocation depending on RPM using a look up table
 * Uses d ECU data structure
 * \return TPS value in % * 64
 */
uint16_t tpszon_function(void);

#ifdef PA4_INP_IGNTIM
/**Ignition timing correction vs voltage (specified by adcvalue)
 * \param adcvalue Voltage from the selected input
 * \return ignition timing value * ANGLE_MULTIPLIER
 */
int16_t pa4_function(uint16_t adcvalue);
#endif


#ifdef GD_CONTROL
/**Use VE and AFR tables for gas dosator. Result = VE * AFR * K, where K is stoichiometry constant
 * Uses d ECU data structure
 * \return value * 2048
 */
uint16_t gd_ve_afr(void);

/** Calculation of gas dosator position, based on (TPS,RPM)
 * Uses d ECU data structure
 * \return Gas dosator position in % (value * 2)
 */
int16_t gdp_function(void);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
/** Converts ADC value (voltage) into AFR
 * Uses d ECU data structure
 * \return AFR * 128
 */
int16_t ego_curve_lookup(void);
#endif

#if defined(FUEL_INJECT) /*|| defined(CARB_AFR)*/ || defined(GD_CONTROL)
int16_t ego_curve_min(void);
int16_t ego_curve_max(void);


/** Scales afterstart enrichment depending on the elapsed time (strokes)
 * Uses d ECU data structure
 * \param enrich_counter current value of counter of strokes (decriasing)
 * \return scaled afterstart enrichment factor (value * 128)
 */
uint8_t scale_aftstr_enrich(uint16_t enrich_counter);

/** Calculates barometric correction factor
 * Uses d ECU data structure
 * \return factor's value * 4096
 */
int16_t barocorr_lookup(void);

#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)
/** Calculate correction coefficient for PW, coefficient vs MAT.
 * This function can use either corrected MAT value or raw MAt value, based on air flow and CLT
 * Uses d ECU data structure
 * \param rawmat 1 - use raw MAT, 0 - use corrected MAT. If firmware compiled without FUEL_INJECT and GD_CONTROL options,
 * then this parameter is ignored and function will always use raw MAT value
 * \return value * 128
 */
uint8_t inj_airtemp_corr(uint8_t rawmat);
#endif


/** Calculates arguments for some look up tables
  * Uses d ECU data structure
 */
void calc_lookup_args(void);

#if (defined(FUEL_INJECT) || defined(GD_CONTROL)) && !defined(SECU3T)
/** Calculate PW correction from gas temperature using a lookup table
 * Uses d ECU data structure
 * \return coefficient * 128, range 0...1.99
 */
uint8_t inj_gts_pwcorr(void);

/** Calculate PW correction from gas pressure using a lookup table
 * Uses d ECU data structure
 * \return coefficient * 128, range 0...1.99
 */
uint8_t inj_gps_pwcorr(void);
#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)
/** Checks for conditions activating engine blowing mode
 * \return 1 - engine blowing should be active, 0 - not active
 */
uint8_t engine_blowing_cond(void);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates AE value (accel.pump).
 * Uses d ECU data structure
 * \param mode Mode of calculation: 0 - fuel injection, 1 - gas dispenser
 * \param stoich_val Stoichiometric value of current fuel. This parement is ignored for gas dispenser mode
 * \return AE value in PW units
 */
int32_t acc_enrich_calc(uint8_t mode, int16_t stoich_val);

/** Calculates AE value (time based).
 * Uses d ECU data structure
 * \param res 0 - normal operation, 1 - reset internal state machine and do nothing
 * \param stoich_val Stoichiometric value of current fuel. This parement is ignored for gas dispenser mode
 * \return AE value in PW units
 */
int32_t acc_enrich_calc_tb(uint8_t res, int16_t stoich_val);

/**Update AE decay counter. Must be called for each stroke*/
void acc_enrich_decay_counter(void);
#endif

/** Calculates cranking RPM threshold (RPM vs coolant temperature)
 * \return RPM value
 */
uint16_t cranking_thrd_rpm(void);

/** Calculates cranking time (strokes vs coolant temperature)
 * \return number of strokes 0-255
 */
uint16_t cranking_thrd_tmr(void);

/** Calculates start map abandon threshold (RPM vs coolant temperature)
 * \return RPM value
 */
uint16_t smapaban_thrd_rpm(void);

#ifdef _PLATFORM_M1284_
/**Gets knock zone flag from a look up table using current RPM and TPS values
 * \return flag value (0, 1)
 */
uint8_t knock_zone_val(void);
#endif

/** Calculates value of PWM duty from "pwm_dutyx" maps
 * Uses d ECU data structure
 * \param mode 0 - from pwm_duty1 map, 1 - from pwm_duty2 map
 * \return value of duty 0...255
 */
uint16_t pwm_function(uint8_t mode);

#ifdef SPLIT_ANGLE
/** Calculates advance angle from splitting map
 * In map: positive split means the leading plug fires before the trailing plug.
 * Uses d ECU data structure
 * \return value of advance angle * 32
 */
int16_t split_function(void);
#endif

#ifndef SECU3T
/** Calculates PWM duty for gas reducer's heater (duty vs coolant temperature)
 * \return duty value (%) * 2
 */
uint8_t grheat_pwm_duty(void);

/** Calculates gas valve's opening delay vs gas reducer's temperature
 * \return delay in 10ms units (e.g. 100 --> 1 sec)
 */
uint16_t grv_delay(void);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates coefficient vs board voltage.
 * \return value of coefficient * 4096
 */
uint16_t pwmiac_ucoef(void);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Calculates number of after start enrichment strokes vs cooland temperature
 * \param mode 0 - use set 0, 1 - use set 1
 * \return Number of strokes
 */
uint16_t aftstr_strokes(uint8_t mode);
#endif

#if defined(FUEL_INJECT)
/** IAC position correction vs MAT
 * \return value in % * 4
 */
int8_t inj_iac_mat_corr(void);
#endif

#if !defined(SECU3T) && defined(MCP3204)
/**Converts ADC value into phisical magnitude (fuel tank level, exhaust gas temperature or oil pressure)
 * \param adcvalue Voltage from sensor (in ADC discretes)
 * \param lutab Pointer to related look up table (&ftls_curve, &egts_curve or &ops_curve)
 * \return physical magnitude * multiplier (x64/x4/x256)
 */
int16_t exsens_lookup(uint16_t adcvalue, int16_t _PGM *lutab);
#endif

#if !defined(SECU3T)
/**Inj. PW coefficient vs voltage (specified by adcvalue)
 * \param adcvalue Voltage from the selected input
 * \return inj.PW coeff value * 4096
 */
int16_t injpwcoef_function(uint16_t adcvalue);
#endif

/** Calculates MAF flow using a lookup table.
 * \param adcvalue - voltage from the MAF sensor
 * \return Value of flow in g/sec * 64
 */
uint16_t calc_maf_flow(uint16_t adcvalue);

#ifdef FUEL_INJECT
/** Calculates current value of LTFT correction (LTFT(load, rpm))
 * \return Correction coefficient, value * 512
 */
int16_t calc_ltft(void);

/** Finds an index at the RPM grid which current RPM is near to
 * \return index of cell on RPM grid, 255 - if there are no near cells
 */
uint8_t ltft_check_rpm_hit(void);

/** Finds an index at the load grid which current load is near to
 * \return index of cell on load grid, 255 - if there are no near cells
 */
uint8_t ltft_check_load_hit(void);

#endif

#endif //_FUNCONV_H_
