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

/** \file measure.c
 * \author Alexey A. Shabelnikov
 * Implementation of processing (averaging, corrections etc) of data comes from ADC and sensors
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include <stdlib.h>
#include "bitmask.h"
#include "camsens.h"    //cams_enable_cam()
#include "ce_errors.h"
#include "ecudata.h"
#include "spdsens.h"
#include "funconv.h"    //thermistor_lookup(), calc_speed()
#include "injector.h"   //inject_set_config(), inject_set_num_squirts()
#include "ioconfig.h"
#include "magnitude.h"
#include "measure.h"
#include "tables.h"  //for ce_sett_t type
#include "ckps.h"
#ifdef TPIC8101
#include "knock.h"
#endif
#include "ringbuff.h"

//see also ioconfig.c
int16_t iocfg_map_s = 0;
int16_t iocfg_add_i1 = 0;
int16_t iocfg_add_i2 = 0;
#ifndef SECU3T
int16_t iocfg_add_i3 = 0;
#endif
#ifdef TPIC8101
int16_t iocfg_add_i4 = 0;
#endif
#ifdef MCP3204
int16_t iocfg_add_i5 = 0;
int16_t iocfg_add_i6 = 0;
int16_t iocfg_add_i7 = 0;
int16_t iocfg_add_i8 = 0;
#endif

#ifdef VREF_5V //voltage divider is not necessary when ref. voltage is 5V
 /**Special macro for compensating of voltage division (without voltage divider)*/
 #define _RESDIV(v, n, d) (v)
#else //voltage divider is used
 /**Special macro for compensating of voltage division (with voltage divider)*/
 #define _RESDIV(v, n, d) (((n) * (v)) / (d))
#endif

/**Reads state of throttle gate (only the value, without inversion) */
#define GET_THROTTLE_GATE_STATE() (CHECKBIT(PINA, PINA7) > 0)

//Index of each ring buffer
#define FRQ_INPIDX           0                 //!< Index of ring buffer for RPM
#define MAP_INPIDX           1                 //!< Index of ring buffer for MAP
#define BAT_INPIDX           2                 //!< Index of ring buffer for board voltage
#define TMP_INPIDX           3                 //!< Index of ring buffer for CTS
#define TPS_INPIDX           4                 //!< Index of ring buffer for TPS
#define AI1_INPIDX           5                 //!< Index of ring buffer for ADD_I1
#define AI2_INPIDX           6                 //!< Index of ring buffer for ADD_I2
#define SPD_INPIDX           7                 //!< Index of ring buffer for VSS (SPEED_SENSOR option must be included)
#define AI3_INPIDX           8                 //!< Index of ring buffer for ADD_I3 (SECU3T option must be excluded)
#define AI4_INPIDX           9                 //!< Index of ring buffer for ADD_I4 (TPIC8101 option must be included)
#if !defined(SECU3T) && defined(MCP3204)
#define AI5_INPIDX          10                 //!< Index of ring buffer for ADD_I5 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI6_INPIDX          11                 //!< Index of ring buffer for ADD_I6 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI7_INPIDX          12                 //!< Index of ring buffer for ADD_I7 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI8_INPIDX          13                 //!< Index of ring buffer for ADD_I8 (SECU3T option must be excluded AND MCP3204 option must be included)
#endif

/**Ring buffers for all inputs */
ringbuff_t meas[INPUTNUM] = {{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0}};

#ifdef SPEED_SENSOR
/**Stores last value of the VSS pulse counter*/
static uint16_t vss_pulse_count = 0;
#endif

void meas_init_ports(void)
{
 IOCFG_INIT(IOP_GAS_V, 0);    //don't use internal pullup resistor
 IOCFG_INIT(IOP_COND_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_INPUT1, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_INPUT2, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_AUTO_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_MAPSEL0, 0);  //don't use internal pullup resistor
#ifndef SECU3T //SECU-3i
 IOCFG_INIT(IOP_EPAS_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_GPA4_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_GPA5_I, 1);   //use internal pullup resistor
 IOCFG_INIT(IOP_ALTRN_I, 0);  //don't use internal pullup resistor
 IOCFG_INIT(IOP_REFPRS_I, 0); //don't use internal pullup resistor
#endif
 //We don't initialize analog inputs (ADD_I1, ADD_I2, CARB, ADD_I3, ADD_I4) because they are initialised by default
 //and we don't need pullup resistors for them
}

//Update ring buffers
void meas_update_values_buffers_map(ce_sett_t *cesd)
{
 update_buffer(&meas[MAP_INPIDX], adc_get_map_value());

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 if (d.engine_mode != EM_START && !ce_is_error(ECUERROR_MAP_SENSOR_FAIL))
 {
  d.sens.mapdot = adc_compensate(_RESDIV(adc_get_mapdot_value(), 2, 1), d.param.map_adc_factor, 0);
  d.sens.mapdot = mapdot_adc_to_kpa(d.sens.mapdot, d.param.map_curve_gradient);
 }
 else
  d.sens.mapdot = 0; //disable accel.enrichment during cranking or in case of MAP error
#endif
}

//Update ring buffers
void meas_update_values_buffers(ce_sett_t *cesd)
{
 update_buffer(&meas[FRQ_INPIDX], ckps_calculate_instant_freq()); //calculate instant RPM and add it to the buffer

 update_buffer(&meas[BAT_INPIDX], adc_get_ubat_value());

 update_buffer(&meas[TMP_INPIDX], adc_get_temp_value());

 update_buffer(&meas[TPS_INPIDX], adc_get_tps_value());

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 if (d.engine_mode != EM_START && !ce_is_error(ECUERROR_TPS_SENSOR_FAIL))
 {
  d.sens.tpsdot = adc_compensate(_RESDIV(adc_get_tpsdot_value(), 2, 1), d.param.tps_adc_factor, 0);
  d.sens.tpsdot = tpsdot_adc_to_pc(d.sens.tpsdot, d.param.tps_curve_gradient);
 }
 else
  d.sens.tpsdot = 0; //disable accel.enrichment during cranking or in case of TPS error
#endif

 update_buffer(&meas[AI1_INPIDX], adc_get_add_i1_value());

 update_buffer(&meas[AI2_INPIDX], adc_get_add_i2_value());

#if !defined(SECU3T)
 update_buffer(&meas[AI3_INPIDX], adc_get_add_i3_value());
#endif

#if defined(TPIC8101)
 update_buffer(&meas[AI4_INPIDX], adc_get_knock_value());
#endif

#if !defined(SECU3T) && defined(MCP3204)
 update_buffer(&meas[AI5_INPIDX], adc_get_add_i5_value());
 update_buffer(&meas[AI6_INPIDX], adc_get_add_i6_value());
 update_buffer(&meas[AI7_INPIDX], adc_get_add_i7_value());
 update_buffer(&meas[AI8_INPIDX], adc_get_add_i8_value());
#endif

#ifdef SPEED_SENSOR
 update_buffer(&meas[SPD_INPIDX], calc_speed(spdsens_get_period()));
#endif

 if (d.param.knock_use_knock_channel && d.sens.rpm > 200)
 {
#ifdef TPIC8101
  d.sens.knock_raw = adc_compensate(knock_get_adc_value(), ADC_COMP_FACTOR(ADC_VREF_FACTOR), ADC_COMP_CORR(ADC_VREF_FACTOR, 0.0));
#else
#ifdef VREF_5V
  d.sens.knock_raw = adc_compensate(adc_get_knock_value(), ADC_COMP_FACTOR(ADC_VREF_FACTOR), ADC_COMP_CORR(ADC_VREF_FACTOR, 0.0));
#else //internal 2.56V
  d.sens.knock_raw = adc_get_knock_value() * 2;
#endif
#endif
  d.sens.knock_k = ce_is_error(ECUERROR_KSP_CHIP_FAILED) && cesd->ks_v_flg ? cesd->ks_v_em : d.sens.knock_raw;
 }
 else
  d.sens.knock_k = 0; //knock signal value must be zero if knock detection turned off or engine is stopped
}

//Average values in ring buffers, compensate ADC errors and convert raw voltage into physical values
void meas_average_measured_values(ce_sett_t *cesd)
{
 //RPM
 d.sens.rpm = average_buffer(&meas[FRQ_INPIDX]);

 //board voltage
 d.sens.voltage_raw = adc_compensate(average_buffer(&meas[BAT_INPIDX]) * 6, d.param.ubat_adc_factor,d.param.ubat_adc_correction);
 d.sens.voltage = ubat_adc_to_v(ce_is_error(ECUERROR_VOLT_SENSOR_FAIL) && cesd->vbat_v_flg ? cesd->vbat_v_em : d.sens.voltage_raw);

 //TPS
 d.sens.tps_raw = adc_compensate(_RESDIV(average_buffer(&meas[TPS_INPIDX]), 2, 1), d.param.tps_adc_factor, d.param.tps_adc_correction);
 d.sens.tps = tps_adc_to_pc(ce_is_error(ECUERROR_TPS_SENSOR_FAIL) && cesd->tps_v_flg ? cesd->tps_v_em : d.sens.tps_raw, d.param.tps_curve_offset, d.param.tps_curve_gradient);
#if !defined(SECU3T)
 d.sens.tps_dbw = d.sens.tps; //save value that can be above 100% first, this value is used by servo PID.
 if (IOCFG_CHECK(IOP_TPS2))
 {//DBW mode
  if (d.sens.tps > TPS_MAGNITUDE(100))
   d.sens.tps = TPS_MAGNITUDE(100) - (d.sens.tps - TPS_MAGNITUDE(100)); //values > 100% mean that throttle is closing
 }
 else
#endif
 { //direct TPS mode
  if (d.sens.tps > TPS_MAGNITUDE(100))
   d.sens.tps = TPS_MAGNITUDE(100); 
 }

 //CLT
 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
 {
  d.sens.temperat_raw = adc_compensate(_RESDIV(average_buffer(&meas[TMP_INPIDX]), 5, 3),d.param.temp_adc_factor,d.param.temp_adc_correction);
#ifndef THERMISTOR_CS
  d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && cesd->cts_v_flg ? cesd->cts_v_em : d.sens.temperat_raw);
#else
  if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_MAP)) //use linear sensor
   d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && cesd->cts_v_flg ? cesd->cts_v_em : d.sens.temperat_raw);
  else //use lookup table (actual for thermistor sensors)
   d.sens.temperat = thermistor_lookup(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && cesd->cts_v_flg ? cesd->cts_v_em : d.sens.temperat_raw, ram_extabs.cts_curve);
#endif
 }
 else                                       //CTS is not used
  d.sens.temperat = 0;

 //VSS
#ifdef SPEED_SENSOR
 //speed
 d.sens.vss_speed=average_buffer(&meas[SPD_INPIDX]);
 //distance
 uint8_t reset_flag = (vss_pulse_count > 63000);
 //check if we reach 100 000 km, then reset our odometer
 if (d.sens.vss_dist >= VSSDIST_MAG(100000.0))
 {
  d.sens.vss_int_dist = 0; //reset accumulated (integral) value
  reset_flag = 1;          //reset counter
 }
 vss_pulse_count = spdsens_get_pulse_count(reset_flag);
 uint32_t dist = calc_dist(vss_pulse_count);
 d.sens.vss_dist = (d.sens.vss_int_dist + dist) >> (5+3); //calculate distance shown for user
 if (reset_flag)
  d.sens.vss_int_dist+= dist; //accumulate distance
#endif

 //MAP
 int16_t rawval;
 d.sens.map_raw = adc_compensate(_RESDIV(average_buffer(&meas[MAP_INPIDX]), 2, 1), d.param.map_adc_factor, d.param.map_adc_correction);
 iocfg_map_s = ce_is_error(ECUERROR_MAP_SENSOR_FAIL) && cesd->map_v_flg ? cesd->map_v_em : d.sens.map_raw;

 //ADD_I1
 d.sens.add_i1_raw = rawval = adc_compensate(_RESDIV(average_buffer(&meas[AI1_INPIDX]), 2, 1), d.param.ai1_adc_factor, d.param.ai1_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i1 = ce_is_error(ECUERROR_ADD_I1_SENSOR) && cesd->add_i1_v_flg ? cesd->add_i1_v_em : rawval;

 //ADD_I2
 d.sens.add_i2_raw = rawval = adc_compensate(_RESDIV(average_buffer(&meas[AI2_INPIDX]), 2, 1), d.param.ai2_adc_factor, d.param.ai2_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i2 = ce_is_error(ECUERROR_ADD_I2_SENSOR) && cesd->add_i2_v_flg ? cesd->add_i2_v_em : rawval;

 //ADD_I3
#if !defined(SECU3T)
 d.sens.add_i3_raw = rawval = adc_compensate(average_buffer(&meas[AI3_INPIDX]), d.param.ai3_adc_factor, d.param.ai3_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i3 = ce_is_error(ECUERROR_ADD_I3_SENSOR) && cesd->add_i3_v_flg ? cesd->add_i3_v_em : rawval;
#endif

 //ADD_I4
#if defined(TPIC8101)
 d.sens.add_i4_raw = rawval = adc_compensate(average_buffer(&meas[AI4_INPIDX]), d.param.ai4_adc_factor, d.param.ai4_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i4 = ce_is_error(ECUERROR_ADD_I4_SENSOR) && cesd->add_i4_v_flg ? cesd->add_i4_v_em : rawval;
#endif

#if !defined(SECU3T) && defined(MCP3204)
 //ADD_I5
 d.sens.add_i5_raw = rawval = adc_compensate(average_buffer(&meas[AI5_INPIDX]), d.param.ai5_adc_factor, d.param.ai5_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i5 = ce_is_error(ECUERROR_ADD_I5_SENSOR) && cesd->add_i5_v_flg ? cesd->add_i5_v_em : rawval;

 //ADD_I6
 d.sens.add_i6_raw = rawval = adc_compensate(average_buffer(&meas[AI6_INPIDX]), d.param.ai6_adc_factor, d.param.ai6_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i6 = ce_is_error(ECUERROR_ADD_I6_SENSOR) && cesd->add_i6_v_flg ? cesd->add_i6_v_em : rawval;

 //ADD_I7
 d.sens.add_i7_raw = rawval = adc_compensate(average_buffer(&meas[AI7_INPIDX]), d.param.ai7_adc_factor, d.param.ai7_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i7 = ce_is_error(ECUERROR_ADD_I7_SENSOR) && cesd->add_i7_v_flg ? cesd->add_i7_v_em : rawval;

 //ADD_I8
 d.sens.add_i8_raw = rawval =adc_compensate(average_buffer(&meas[AI8_INPIDX]), d.param.ai8_adc_factor, d.param.ai8_adc_correction);
 if (rawval < 0)
  rawval = 0;
 iocfg_add_i8 = ce_is_error(ECUERROR_ADD_I8_SENSOR) && cesd->add_i8_v_flg ? cesd->add_i8_v_em : rawval;
#endif

 //---------------------------------------------------------------
 //We read and save the values of physical inputs. These values are not used in the firmware, but
 //are needed to display voltages regardless of how these inputs are reassigned
 d.sens.add_i1 = IOCFG_GETAS(IOP_ADD_I1);
 d.sens.add_i2 = IOCFG_GETAS(IOP_ADD_I2);

#if !defined(SECU3T)
 d.sens.add_i3 = IOCFG_GETAS(IOP_ADD_I3);
#endif

#if defined(TPIC8101)
 d.sens.add_i4 = IOCFG_GETAS(IOP_ADD_I4);
#endif

#if !defined(SECU3T) && defined(MCP3204)
 d.sens.add_i5 = IOCFG_GETAS(IOP_ADD_I5);
 d.sens.add_i6 = IOCFG_GETAS(IOP_ADD_I6);
 d.sens.add_i7 = IOCFG_GETAS(IOP_ADD_I7);
 d.sens.add_i8 = IOCFG_GETAS(IOP_ADD_I8);
#endif
 //---------------------------------------------------------------

 if (IOCFG_CHECK(IOP_MAP_S))
  d.sens.map = map_adc_to_kpa(IOCFG_GETA(IOP_MAP_S), d.param.map_curve_offset, d.param.map_curve_gradient);
 else
  d.sens.map = 0;

 //MAT sensor
#ifdef AIRTEMP_SENS
 if (IOCFG_CHECK(IOP_AIR_TEMP))
  d.sens.air_temp = thermistor_lookup(IOCFG_GETA(IOP_AIR_TEMP), ram_extabs.ats_curve);
 else
  d.sens.air_temp = 0;
#endif

 //MAP2
 if (IOP_MAP2)
  d.sens.map2 = map_adc_to_kpa(IOCFG_GETA(IOP_MAP2), d.param.map2_curve_offset, d.param.map2_curve_gradient);
 else
  d.sens.map2 = 0; //input is not selected

#ifndef SECU3T //SECU-3i
 if (IOCFG_CHECK(IOP_TMP2))
  d.sens.tmp2 = thermistor_lookup(IOCFG_GETA(IOP_TMP2), ram_extabs.tmp2_curve);
 else
  d.sens.tmp2 = 0; //input is not selected

 //GPS
 if (IOP_GPS)
  d.sens.gps = map_adc_to_kpa(IOCFG_GETA(IOP_GPS), d.param.gps_curve_offset, d.param.gps_curve_gradient);
 else
  d.sens.gps = 0; //input is not selected

#ifdef MCP3204
 if (IOCFG_CHECK(IOP_GRTEMP))
  d.sens.grts = thermistor_lookup(IOCFG_GETA(IOP_GRTEMP), ram_extabs.grts_curve);
 else
  d.sens.grts = 0; //input is not selected
#endif
#endif

#ifndef SECU3T
 if (IOCFG_CHECK(IOP_FTLS_I))
 {
  int16_t add_ix = (((uint32_t)IOCFG_GETA(IOP_FTLS_I)) * ftlscor_ucoef()) >> 12; //apply board voltage correction
  d.sens.ftls = exsens_lookup(add_ix, ram_extabs.ftls_curve);
 }
 else
  d.sens.ftls = 0; //input is not selected
#endif

#ifndef SECU3T
 if (IOCFG_CHECK(IOP_EGTS_I))
  d.sens.egts = exsens_lookup(IOCFG_GETA(IOP_EGTS_I), ram_extabs.egts_curve);
 else
  d.sens.egts = 0; //input is not selected
#endif

#ifndef SECU3T
#ifdef MCP3204
 if (IOCFG_CHECK(IOP_OPS_I))
  d.sens.ops = exsens_lookup(IOCFG_GETA(IOP_OPS_I), ram_extabs.ops_curve);
 else
  d.sens.ops = 0; //input is not selected
#endif
#endif

//select input for lambda sensor
#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 if (IOCFG_CHECK(IOP_LAMBDA))
  d.sens.lambda[0] = IOCFG_GETA(IOP_LAMBDA);
 if (IOCFG_CHECK(IOP_LAMBDA2))
  d.sens.lambda[1] = IOCFG_GETA(IOP_LAMBDA2);

 //calculate mix of two lambda sensors
 if (IOCFG_CHECK(IOP_LAMBDA) && IOCFG_CHECK(IOP_LAMBDA2))
  d.sens.lambda_mx = (d.sens.lambda[0] + d.sens.lambda[1]) / 2;
 else if (!IOCFG_CHECK(IOP_LAMBDA) && IOCFG_CHECK(IOP_LAMBDA2))
  d.sens.lambda_mx = d.sens.lambda[1];
 else
  d.sens.lambda_mx = d.sens.lambda[0];

 if (CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
 { //mix 2 lambda sensors into 1
  d.sens.lambda[0] = d.sens.lambda_mx;
  d.sens.lambda[1] = 0; //discard 2nd sensor because it has already mixed into the 1st one
 }
#endif

 //select an input for MAF sensor
 if (IOCFG_CHECK(IOP_MAF))
  d.sens.maf = calc_maf_flow(IOCFG_GETA(IOP_MAF));
 else
  d.sens.maf = 0; //input is not selected

#ifndef SECU3T
 if (1==PGM_GET_BYTE(&fw_data.exdata.fts_source))
 { //use sensor
#ifdef MCP3204
  if (IOCFG_CHECK(IOP_FTS_I))
   d.sens.fts = exsens_lookup(IOCFG_GETA(IOP_FTS_I), ram_extabs.fts_curve);
  else
   d.sens.fts = 0; //input is not selected
#endif
 }
 else
 {//use CTS+IAT model
  //FTS = IAT + IAT * (CLT - IAT) * 0.01
  int32_t add = (((int32_t)(d.sens.temperat - d.sens.air_temp) * d.sens.air_temp) * 655);
  d.sens.fts = d.sens.air_temp + (add >> 18); //TODO: use SHTDIV32() macro instead of a simple right shift (this will increase accuracy)
 }
#endif

#ifndef SECU3T
 if (IOCFG_CHECK(IOP_FPS))
  d.sens.fps = map_adc_to_kpa(IOCFG_GETA(IOP_FPS), d.param.fps_curve_offset, d.param.fps_curve_gradient);
 else
  d.sens.fps = 0; //input is not selected
#endif

#ifndef SECU3T
 if (IOCFG_CHECK(IOP_APPS1))
 {//APPS1
  d.sens.apps1_raw = IOCFG_GETA(IOP_APPS1);
  d.sens.apps1 = tps_adc_to_pc(d.sens.apps1_raw, d.param.apps1_curve_offset, d.param.apps1_curve_gradient);
  if (d.sens.apps1 > APPS_MAG(100))
   d.sens.apps1 = APPS_MAG(100);
 }
 else
  d.sens.apps1 = 0; //input is not selected
#endif

}

//Call this function for making preliminary measurements before starting engine. Call it only after
//initialization of ADC!
void meas_init(void)
{
 uint8_t _t, i;
 adc_set_map_to_ckp(0); //regular mode
 //set sizes of ring buffers
 for(i = 0; i < INPUTNUM; ++i)
  init_buffer(&meas[i], PGM_GET_BYTE(&fw_data.exdata.inpavnum[i])); 
 //do preliminary measurements
 i = CIRCBUFFMAX;
 _t = _SAVE_INTERRUPT();
 _ENABLE_INTERRUPT();
 do
 {
  _DISABLE_INTERRUPT();
  adc_begin_measure();
  _ENABLE_INTERRUPT();
  while(!adc_is_measure_ready(ADCRDY_SENS));

  meas_update_values_buffers_map(&ram_extabs.cesd);
  meas_update_values_buffers(&ram_extabs.cesd);
 }while(--i);
 _RESTORE_INTERRUPT(_t);
 meas_average_measured_values(&ram_extabs.cesd);
 adc_set_map_to_ckp(PGM_GET_BYTE(&fw_data.exdata.map_samp_mode));
}

#ifdef REALTIME_TABLES
/** Selects set of tables specified by index, sets corresponding flag depending on where tables' set resides: RAM or FLASH
 * Uses d ECU data structure
 * \param set_index Index of tables' set to be selected
 */
static void select_table_set(uint8_t set_index)
{
 if (set_index > (TABLES_NUMBER_PGM-1))
 {
  d.mm_ptr8 = mm_get_byte_ram;
  d.mm_ptr16 = mm_get_word_ram;
  d.mm_ptr12 = mm_get_w12_ram;
 }
 else
 {
  d.fn_dat = &fw_data.tables[set_index];
  d.mm_ptr8 = mm_get_byte_pgm;
  d.mm_ptr16 = mm_get_word_pgm;
  d.mm_ptr12 = mm_get_w12_pgm;
 }
}
#endif

void meas_take_discrete_inputs(void)
{
 //--do inversion of throttle limit switch if necessary
 if (0==d.param.tps_threshold)
  d.sens.carb=d.param.carb_invers^GET_THROTTLE_GATE_STATE(); //result: 0 - throttle is closed, 1 - opened
 else
 {//using a TPS (emulate limit switch)
  d.sens.carb=d.param.carb_invers^(d.sens.tps > d.param.tps_threshold);
 }

 //read state of gas valve input
 //if GAS_V input remapped to other function, then petrol
 d.sens.gas_raw = IOCFG_GET(IOP_GAS_V);
 //use condition result from selected univ.output instead of GAS_V input
 uint8_t gas_v_trig =
#ifdef UNI_OUTPUT
 (d.param.gas_v_uni != 0x0F) ? d.uniout[d.param.gas_v_uni] :
#endif
 d.sens.gas_raw;

 if (d.sens.gas != gas_v_trig)
 {
  d.sens.gas = gas_v_trig;
#ifdef FUEL_INJECT
  if (d.param.inj_config[0] != d.param.inj_config[1] || CHECKBIT(d.param.inj_flags, INJFLG_SECINJROWSWT)) //update settings only if it is necessary!
  {
   //update some parameters which depend on type of fuel
   //TODO: redundant code fragment
   inject_set_num_squirts(d.param.inj_config[d.sens.gas] & 0xF);  //number of squirts
   inject_set_config(d.param.inj_config[d.sens.gas] >> 4, CHECKBIT(d.param.inj_flags, INJFLG_SECINJROWSWT));//type of injection
#if defined(PHASE_SENSOR) && !defined(PHASED_IGNITION)
   cams_enable_cam(
#ifdef FUEL_INJECT
     (d.param.inj_config[d.sens.gas] >> 4) == INJCFG_FULLSEQUENTIAL ||
#endif
     CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF));
#endif
  }
#endif
 }

 //switch set of maps (or fuel type) depending on the state of GAS_V input (gas valve) and additional input (MAPSEL0)
#ifndef REALTIME_TABLES
 uint8_t mapsel0 = IOCFG_GET(IOP_MAPSEL0); //note: if not mapped to real I/O, then stub() will always return 0 (we rely on it)
 if (d.sens.gas) //on gas
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0xF0) != 0xF0)
   mapsel0 = d.uniout[d.param.mapsel_uni >> 4]; //use condition result from selected univ.output instead
#endif
  d.fn_dat = mapsel0 ? &fw_data.tables[1] : &fw_data.tables[d.param.fn_gas];
 }
 else             //on petrol
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0x0F) != 0x0F)
   mapsel0 = d.uniout[d.param.mapsel_uni & 0x0F]; //use condition result from selected univ.output instead
#endif
  d.fn_dat = mapsel0 ? &fw_data.tables[0] : &fw_data.tables[d.param.fn_gasoline];
 }
#else //use tables from RAM
 uint8_t mapsel0 = IOCFG_GET(IOP_MAPSEL0); //note: if not mapped to real I/O, then stub() will always return 0  (we rely on it)
 if (d.sens.gas)
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0xF0) != 0xF0)
   mapsel0 = d.uniout[d.param.mapsel_uni >> 4]; //use condition result from selected univ.output instead
#endif
  select_table_set(mapsel0 ? 1 : d.param.fn_gas);          //on gas
 }
 else
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0x0F) != 0x0F)
   mapsel0 = d.uniout[d.param.mapsel_uni & 0x0F]; //use condition result from selected univ.output instead
#endif
  select_table_set(mapsel0 ? 0 : d.param.fn_gasoline);     //on petrol
 }
#endif

#ifndef SECU3T //SECU-3i
 d.sens.oilpress_ok = !ce_is_error(ECUERROR_OILPRESSURE); //oil pressure sensor

 if (IOCFG_CHECK(IOP_ALTRN_I))
 {
  uint16_t altvolt = IOCFG_GETA(IOP_ALTRN_I); //get state of the input
  if (IOCFG_DTST(altvolt))
  {
   d.sens.generator_ok = IOCFG_DGET(altvolt); //mapped to bare digital input
  }
  else //mapped to an analog input
  {
   d.sens.generator_ok = (altvolt > VOLTAGE_MAGNITUDE(2.5)) && !IOCFG_GETE(IOP_ALTRN_I);
  }
 }
 else //not used (not mapped to physical input)
  d.sens.generator_ok = 1;

 d.sens.epas_i = IOCFG_GET(IOP_EPAS_I);
 d.sens.gpa4_i = IOCFG_GET(IOP_GPA4_I);
 d.sens.refprs_i = IOCFG_GET(IOP_REFPRS_I);
 d.sens.altrn_i = IOCFG_GET(IOP_ALTRN_I);
#endif

 d.sens.auto_i = IOCFG_GET(IOP_AUTO_I);
 d.sens.ign_i = IOCFG_GET(IOP_IGN);
 d.sens.cond_i = IOCFG_GET(IOP_COND_I);
 d.sens.input1 = IOCFG_GET(IOP_INPUT1);
 d.sens.input2 = IOCFG_GET(IOP_INPUT2);
 d.sens.mapsel0 = mapsel0;
}
