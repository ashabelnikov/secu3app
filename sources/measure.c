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
#include "eculogic.h"
#include "spdsens.h"
#include "funconv.h"    //thermistor_lookup()
#include "injector.h"   //inject_set_config(), inject_set_num_squirts()
#include "ioconfig.h"
#include "magnitude.h"
#include "measure.h"
#include "tables.h"  //for ce_sett_t type
#ifdef TPIC8101
#include "knock.h"
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

/**Gets size of corresponding ring buffer*/
#define AVNUM(idx) (PGM_GET_BYTE(&fw_data.exdata.inpavnum[idx]))

//Index of each ring buffer
#define FRQ_INPIDX           0                 //!< Index of ring buffer for RPM
#define MAP_INPIDX           1                 //!< Index of ring buffer for MAP
#define BAT_INPIDX           2                 //!< Index of ring buffer for board voltage
#define TMP_INPIDX           3                 //!< Index of ring buffer for CTS
#define TPS_INPIDX           4                 //!< Index of ring buffer for TPS
#define AI1_INPIDX           5                 //!< Index of ring buffer for ADD_I1
#define AI2_INPIDX           6                 //!< Index of ring buffer for ADD_I2
#define SPD_INPIDX           7                 //!< Index of ring buffer for VSS (SPEED_SENSOR option must be included)
#define AI3_INPIDX           8                 //!< Index of ring buffer for ADD_I3 (SECU3T option must be excluded OR PA4_INP_IGNTIM option must be included)
#define AI4_INPIDX           9                 //!< Index of ring buffer for ADD_I4 (SECU3T option must be excluded AND TPIC8101 option must be included)
#if !defined(SECU3T) && defined(MCP3204)
#define AI5_INPIDX          10                 //!< Index of ring buffer for ADD_I5 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI6_INPIDX          11                 //!< Index of ring buffer for ADD_I6 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI7_INPIDX          12                 //!< Index of ring buffer for ADD_I7 (SECU3T option must be excluded AND MCP3204 option must be included)
#define AI8_INPIDX          13                 //!< Index of ring buffer for ADD_I8 (SECU3T option must be excluded AND MCP3204 option must be included)
#endif

#define CIRCBUFFMAX 8                          //!< Maximum size of ring buffer in items

/**Describes ring buffer for one input*/
typedef struct
{
 uint16_t buff[CIRCBUFFMAX];                    //!< Ring buffer
 uint8_t ai;                                    //!< index in buffer
}meas_input_t;

/**Ring buffers for all inputs */
meas_input_t meas[INPUTNUM] = {{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0}};

static uint16_t update_buffer(uint8_t idx, uint16_t value)
{
 meas[idx].buff[meas[idx].ai] = value;
 (meas[idx].ai==0) ? (meas[idx].ai = AVNUM(idx) - 1): meas[idx].ai--;
 return value;
}

static uint16_t average_buffer(uint8_t idx)
{
 uint8_t i = AVNUM(idx) - 1;  uint32_t sum = 0;
 do
 {
  sum+=meas[idx].buff[i];
 }while(i--);

 //We use shifts instead of division.
 if (AVNUM(idx)==4)
  return sum >> 2;
 if (AVNUM(idx)==8)
  return sum >> 3;
 return sum / AVNUM(idx); //default
}

void meas_init_ports(void)
{
 IOCFG_INIT(IOP_GAS_V, 0);    //don't use internal pullup resistor
#ifndef SECU3T //SECU-3i
 IOCFG_INIT(IOP_IGN, 0);      //don't use internal pullup resistor
 IOCFG_INIT(IOP_COND_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_EPAS_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_OILP_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_GENS_I, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_AUTO_I, 0);   //don't use internal pullup resistor
#endif
 IOCFG_INIT(IOP_INPUT1, 0);   //don't use internal pullup resistor
 IOCFG_INIT(IOP_INPUT2, 0);   //don't use internal pullup resistor
 //We don't initialize analog inputs (ADD_I1, ADD_I2, CARB, ADD_I3, ADD_I4) because they are initialised by default
 //and we don't need pullup resistors for them
}

//Update ring buffers
void meas_update_values_buffers(uint8_t rpm_only, ce_sett_t _PGM *cesd)
{
 int16_t rawval;

 update_buffer(FRQ_INPIDX, d.sens.inst_frq);

 if (rpm_only)
  return;

 rawval = update_buffer(MAP_INPIDX, adc_get_map_value());

#ifdef SEND_INST_VAL
 rawval = ce_is_error(ECUERROR_MAP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->map_v_flg) ? PGM_GET_WORD(&cesd->map_v_em) : adc_compensate(_RESDIV(rawval, 2, 1), d.param.map_adc_factor, d.param.map_adc_correction);
 if (IOCFG_CHECK(IOP_MAP_S))
  d.sens.inst_map = map_adc_to_kpa(rawval, d.param.map_curve_offset, d.param.map_curve_gradient);
 else
  d.sens.inst_map = 0;
#endif

 rawval = update_buffer(BAT_INPIDX, adc_get_ubat_value());
#ifdef SEND_INST_VAL
 d.sens.inst_voltage = ce_is_error(ECUERROR_VOLT_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->vbat_v_flg) ? PGM_GET_WORD(&cesd->vbat_v_em) : adc_compensate(rawval * 6, d.param.ubat_adc_factor, d.param.ubat_adc_correction);
#endif

 update_buffer(TMP_INPIDX, adc_get_temp_value());

 rawval = update_buffer(TPS_INPIDX, adc_get_carb_value());
#ifdef SEND_INST_VAL
 rawval = adc_compensate(_RESDIV(rawval, 2, 1), d.param.tps_adc_factor, d.param.tps_adc_correction);
 d.sens.inst_tps = tps_adc_to_pc(ce_is_error(ECUERROR_TPS_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->tps_v_flg) ? PGM_GET_WORD(&cesd->tps_v_em) : rawval, d.param.tps_curve_offset, d.param.tps_curve_gradient);
 if (d.sens.inst_tps > TPS_MAGNITUDE(100))
  d.sens.inst_tps = TPS_MAGNITUDE(100);
#endif

 rawval = update_buffer(AI1_INPIDX, adc_get_add_i1_value());
#ifdef SEND_INST_VAL
 rawval = adc_compensate(_RESDIV(rawval, 2, 1), d.param.ai1_adc_factor, d.param.ai1_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.inst_add_i1 = ce_is_error(ECUERROR_ADD_I1_SENSOR) && PGM_GET_BYTE(&cesd->add_i1_v_flg) ? PGM_GET_WORD(&cesd->add_i1_v_em) : rawval;
#endif

 update_buffer(AI2_INPIDX, adc_get_add_i2_value());

#if !defined(SECU3T) || defined(PA4_INP_IGNTIM)
 update_buffer(AI3_INPIDX, adc_get_add_i3_value());
#endif

#if !defined(SECU3T) && defined(TPIC8101)
 update_buffer(AI4_INPIDX, adc_get_knock_value());
#endif

#if !defined(SECU3T) && defined(MCP3204)
 update_buffer(AI5_INPIDX, adc_get_add_i5_value());
 update_buffer(AI6_INPIDX, adc_get_add_i6_value());
 update_buffer(AI7_INPIDX, adc_get_add_i7_value());
 update_buffer(AI8_INPIDX, adc_get_add_i8_value());
#endif

 if (d.param.knock_use_knock_channel && d.sens.frequen > 200)
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
  d.sens.knock_k = ce_is_error(ECUERROR_KSP_CHIP_FAILED) && PGM_GET_BYTE(&cesd->ks_v_flg) ? PGM_GET_WORD(&cesd->ks_v_em) : d.sens.knock_raw;
 }
 else
  d.sens.knock_k = 0; //knock signal value must be zero if knock detection turned off or engine is stopped

#ifdef SPEED_SENSOR
 update_buffer(SPD_INPIDX, spdsens_get_period());
 d.sens.distance = spdsens_get_pulse_count();
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 if (d.engine_mode != EM_START && !ce_is_error(ECUERROR_TPS_SENSOR_FAIL))
 {
  d.sens.tpsdot = adc_compensate(_RESDIV(adc_get_tpsdot_value(), 2, 1), d.param.tps_adc_factor, 0);
  d.sens.tpsdot = tpsdot_adc_to_pc(d.sens.tpsdot, d.param.tps_curve_gradient);
 }
 else
  d.sens.tpsdot = 0; //disable accel.enrichment during cranking or in case of TPS error
#endif
}

//Average values in ring buffers, compensate ADC errors and convert raw voltage into physical values
void meas_average_measured_values(ce_sett_t _PGM *cesd)
{
 int16_t rawval;
 d.sens.map_raw = adc_compensate(_RESDIV(average_buffer(MAP_INPIDX), 2, 1), d.param.map_adc_factor, d.param.map_adc_correction);
 if (IOCFG_CHECK(IOP_MAP_S))
  d.sens.map = map_adc_to_kpa(ce_is_error(ECUERROR_MAP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->map_v_flg) ? PGM_GET_WORD(&cesd->map_v_em) : d.sens.map_raw, d.param.map_curve_offset, d.param.map_curve_gradient);
 else
  d.sens.map = 0;

 d.sens.voltage_raw = adc_compensate(average_buffer(BAT_INPIDX) * 6, d.param.ubat_adc_factor,d.param.ubat_adc_correction);
 d.sens.voltage = ubat_adc_to_v(ce_is_error(ECUERROR_VOLT_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->vbat_v_flg) ? PGM_GET_WORD(&cesd->vbat_v_em) : d.sens.voltage_raw);

 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
 {
  d.sens.temperat_raw = adc_compensate(_RESDIV(average_buffer(TMP_INPIDX), 5, 3),d.param.temp_adc_factor,d.param.temp_adc_correction);
#ifndef THERMISTOR_CS
  d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->cts_v_flg) ? PGM_GET_WORD(&cesd->cts_v_em) : d.sens.temperat_raw);
#else
  if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_MAP)) //use linear sensor
   d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->cts_v_flg) ? PGM_GET_WORD(&cesd->cts_v_em) : d.sens.temperat_raw);
  else //use lookup table (actual for thermistor sensors)
   d.sens.temperat = thermistor_lookup(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->cts_v_flg) ? PGM_GET_WORD(&cesd->cts_v_em) : d.sens.temperat_raw, fw_data.exdata.cts_curve);
#endif
 }
 else                                       //CTS is not used
  d.sens.temperat = 0;

 d.sens.frequen=average_buffer(FRQ_INPIDX);

#ifdef SPEED_SENSOR
 d.sens.speed=average_buffer(SPD_INPIDX);
#endif

 d.sens.tps_raw = adc_compensate(_RESDIV(average_buffer(TPS_INPIDX), 2, 1), d.param.tps_adc_factor, d.param.tps_adc_correction);
 d.sens.tps = tps_adc_to_pc(ce_is_error(ECUERROR_TPS_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->tps_v_flg) ? PGM_GET_WORD(&cesd->tps_v_em) : d.sens.tps_raw, d.param.tps_curve_offset, d.param.tps_curve_gradient);
 if (d.sens.tps > TPS_MAGNITUDE(100))
  d.sens.tps = TPS_MAGNITUDE(100);

 d.sens.add_i1_raw = rawval = adc_compensate(_RESDIV(average_buffer(AI1_INPIDX), 2, 1), d.param.ai1_adc_factor, d.param.ai1_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i1 = ce_is_error(ECUERROR_ADD_I1_SENSOR) && PGM_GET_BYTE(&cesd->add_i1_v_flg) ? PGM_GET_WORD(&cesd->add_i1_v_em) : rawval;

 d.sens.add_i2_raw = rawval = adc_compensate(_RESDIV(average_buffer(AI2_INPIDX), 2, 1), d.param.ai2_adc_factor, d.param.ai2_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i2 = ce_is_error(ECUERROR_ADD_I2_SENSOR) && PGM_GET_BYTE(&cesd->add_i2_v_flg) ? PGM_GET_WORD(&cesd->add_i2_v_em) : rawval;

#if !defined(SECU3T) || defined(PA4_INP_IGNTIM)
 d.sens.add_i3_raw = rawval = adc_compensate(average_buffer(AI3_INPIDX), d.param.ai3_adc_factor, d.param.ai3_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i3 = ce_is_error(ECUERROR_ADD_I3_SENSOR) && PGM_GET_BYTE(&cesd->add_i3_v_flg) ? PGM_GET_WORD(&cesd->add_i3_v_em) : rawval;
#endif

#if !defined(SECU3T) && defined(TPIC8101)
 d.sens.add_i4_raw = rawval = adc_compensate(average_buffer(AI4_INPIDX), d.param.ai4_adc_factor, d.param.ai4_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i4 = ce_is_error(ECUERROR_ADD_I4_SENSOR) && PGM_GET_BYTE(&cesd->add_i4_v_flg) ? PGM_GET_WORD(&cesd->add_i4_v_em) : rawval;
#endif

#if !defined(SECU3T) && defined(MCP3204)
 d.sens.add_i5_raw = rawval = adc_compensate(average_buffer(AI5_INPIDX), d.param.ai5_adc_factor, d.param.ai5_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i5 = ce_is_error(ECUERROR_ADD_I5_SENSOR) && PGM_GET_BYTE(&cesd->add_i5_v_flg) ? PGM_GET_WORD(&cesd->add_i5_v_em) : rawval;
 d.sens.add_i6_raw = rawval = adc_compensate(average_buffer(AI6_INPIDX), d.param.ai6_adc_factor, d.param.ai6_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i6 = ce_is_error(ECUERROR_ADD_I6_SENSOR) && PGM_GET_BYTE(&cesd->add_i6_v_flg) ? PGM_GET_WORD(&cesd->add_i6_v_em) : rawval;
 d.sens.add_i7_raw = rawval = adc_compensate(average_buffer(AI7_INPIDX), d.param.ai7_adc_factor, d.param.ai7_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i7 = ce_is_error(ECUERROR_ADD_I7_SENSOR) && PGM_GET_BYTE(&cesd->add_i7_v_flg) ? PGM_GET_WORD(&cesd->add_i7_v_em) : rawval;
 d.sens.add_i8_raw = rawval =adc_compensate(average_buffer(AI8_INPIDX), d.param.ai8_adc_factor, d.param.ai8_adc_correction);
 if (rawval < 0)
  rawval = 0;
 d.sens.add_i8 = ce_is_error(ECUERROR_ADD_I8_SENSOR) && PGM_GET_BYTE(&cesd->add_i8_v_flg) ? PGM_GET_WORD(&cesd->add_i8_v_em) : rawval;
#endif

#ifdef AIRTEMP_SENS
 if (IOCFG_CHECK(IOP_AIR_TEMP))
  d.sens.air_temp = thermistor_lookup(d.sens.add_i2, fw_data.exdata.ats_curve);   //ADD_I2 input selected as MAT sensor
 else
  d.sens.air_temp = 0; //input is not selected
#endif

#ifndef SECU3T //SECU-3i
#ifdef TPIC8101
 if (IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i4 || IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i4i)
  d.sens.map2 = map_adc_to_kpa(d.sens.add_i4, d.param.map2_curve_offset, d.param.map2_curve_gradient); //ADD_I4 input selected as MAP2 sensor
 else
#endif
 {
#ifdef MCP3204
  if (IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i5i)
   d.sens.map2 = map_adc_to_kpa(d.sens.add_i5, d.param.map2_curve_offset, d.param.map2_curve_gradient); //ADD_I5 input selected as MAP2 sensor
  else if (IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i7 || IOCFG_CB(IOP_MAP2) == (fnptr_t)iocfg_g_add_i7i)
   d.sens.map2 = map_adc_to_kpa(d.sens.add_i7, d.param.map2_curve_offset, d.param.map2_curve_gradient); //ADD_I7 input selected as MAP2 sensor
  else
#endif
  d.sens.map2 = 0; //input is not selected
 }
#endif

#ifndef SECU3T //SECU-3i
 if (IOCFG_CHECK(IOP_TMP2))
  d.sens.tmp2 = thermistor_lookup(d.sens.add_i3, fw_data.exdata.tmp2_curve); //ADD_I3 input selected as TMP2 sensor
 else
  d.sens.tmp2 = 0; //input is not selected

#ifdef MCP3204
 if (IOCFG_CHECK(IOP_GRTEMP))
  d.sens.grts = thermistor_lookup(d.sens.add_i6, fw_data.exdata.grts_curve); //ADD_I6 input selected as GRTEMP sensor
 else
  d.sens.grts = 0; //input is not selected
#endif

#ifdef MCP3204
 if (IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i5i)
  d.sens.ftls = exsens_lookup(d.sens.add_i5, fw_data.exdata.ftls_curve); //ADD_I5 input selected as input for fuel tank level sensor
 else if (IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i6 || IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i6i)
  d.sens.ftls = exsens_lookup(d.sens.add_i6, fw_data.exdata.ftls_curve); //ADD_I6 input selected
 else if (IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i7 || IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i7i)
  d.sens.ftls = exsens_lookup(d.sens.add_i7, fw_data.exdata.ftls_curve); //ADD_I7 input selected
 else if (IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i8 || IOCFG_CB(IOP_FTLS_I) == (fnptr_t)iocfg_g_add_i8i)
  d.sens.ftls = exsens_lookup(d.sens.add_i8, fw_data.exdata.ftls_curve); //ADD_I8 input selected as input for fuel tank level sensor
 else
  d.sens.ftls = 0; //input is not selected
#endif

#ifndef SECU3T
 if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i3 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i3i)
  d.sens.egts = exsens_lookup(d.sens.add_i3, fw_data.exdata.egts_curve); //ADD_I3 input selected as input for EGT sensor
#ifdef TPIC8101
 if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i4 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i4i)
  d.sens.egts = exsens_lookup(d.sens.add_i4, fw_data.exdata.egts_curve); //ADD_I4 input selected as input for EGT sensor
#endif
#ifdef MCP3204
 if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i5i)
  d.sens.egts = exsens_lookup(d.sens.add_i5, fw_data.exdata.egts_curve); //ADD_I5 input selected as input for EGT sensor
 else if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i6 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i6i)
  d.sens.egts = exsens_lookup(d.sens.add_i6, fw_data.exdata.egts_curve); //ADD_I6 input selected
 else if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i7 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i7i)
  d.sens.egts = exsens_lookup(d.sens.add_i7, fw_data.exdata.egts_curve); //ADD_I7 input selected
 else if (IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i8 || IOCFG_CB(IOP_EGTS_I) == (fnptr_t)iocfg_g_add_i8i)
  d.sens.egts = exsens_lookup(d.sens.add_i8, fw_data.exdata.egts_curve); //ADD_I8 input selected as input for EGT sensor
#endif
 else
  d.sens.egts = 0; //input is not selected
#endif

#ifdef MCP3204
 if (IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i5i)
  d.sens.ops = exsens_lookup(d.sens.add_i5, fw_data.exdata.ops_curve); //ADD_I5 input selected as input for fuel tank level sensor
 else if (IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i6 || IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i6i)
  d.sens.ops = exsens_lookup(d.sens.add_i6, fw_data.exdata.ops_curve); //ADD_I6 input selected
 else if (IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i7 || IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i7i)
  d.sens.ops = exsens_lookup(d.sens.add_i7, fw_data.exdata.ops_curve); //ADD_I7 input selected
 else if (IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i8 || IOCFG_CB(IOP_OPS_I) == (fnptr_t)iocfg_g_add_i8i)
  d.sens.ops = exsens_lookup(d.sens.add_i8, fw_data.exdata.ops_curve); //ADD_I8 input selected as input for fuel tank level sensor
 else
  d.sens.ops = 0; //input is not selected
#endif

#endif

//select input for lambda sensor
#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
#ifndef SECU3T //SECU-3i
 if (IOCFG_CB(IOP_LAMBDA) == (fnptr_t)iocfg_g_add_i1 || IOCFG_CB(IOP_LAMBDA) == (fnptr_t)iocfg_g_add_i1i)
  d.sens.lambda1 = d.sens.add_i1; //use ADD_I1 /*d.sens.inst_add_i1*/
 else if (IOCFG_CB(IOP_LAMBDA) == (fnptr_t)iocfg_g_add_i3 || IOCFG_CB(IOP_LAMBDA) == (fnptr_t)iocfg_g_add_i3i)
  d.sens.lambda1 = d.sens.add_i3; //use ADD_I3
#else
 d.sens.lambda1 = d.sens.add_i1; //in SECU-3T only ADD_I1 can be used for lambda sensor
#endif
#endif

 //select an input for MAF sensor
 if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_map_s || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_map_si)
  d.sens.maf = calc_maf_flow(ce_is_error(ECUERROR_MAP_SENSOR_FAIL) && PGM_GET_BYTE(&cesd->map_v_flg) ? PGM_GET_WORD(&cesd->map_v_em) : d.sens.map_raw); //MAP input selected as input for MAF sensor
#ifndef SECU3T
#ifdef TPIC8101
 else if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i4 || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i4i)
  d.sens.maf = calc_maf_flow(d.sens.add_i4); //ADD_I4 input selected as input for MAF sensor
#endif
#ifdef MCP3204
 else if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i5i)
  d.sens.maf = calc_maf_flow(d.sens.add_i5); //ADD_I5 input selected as input for MAF sensor
 else if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i6 || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i6i)
  d.sens.maf = calc_maf_flow(d.sens.add_i6); //ADD_I6 input selected as input for MAF sensor
 else if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i7 || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i7i)
  d.sens.maf = calc_maf_flow(d.sens.add_i7); //ADD_I7 input selected as input for MAF sensor
 else if (IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i8 || IOCFG_CB(IOP_MAF) == (fnptr_t)iocfg_g_add_i8i)
  d.sens.maf = calc_maf_flow(d.sens.add_i8); //ADD_I8 input selected as input for MAF sensor
#endif
#endif
 else
  d.sens.maf = 0; //input is not selected

}

//Call this function for making preliminary measurements before starting of engine. Call it only after
//initialization of ADC!
void meas_init(void)
{
 uint8_t _t, i = 16;
 _t = _SAVE_INTERRUPT();
 _ENABLE_INTERRUPT();
 do
 {
  adc_begin_measure(0); //<--normal speed
  while(!adc_is_measure_ready());

  meas_update_values_buffers(0, &fw_data.exdata.cesd); //<-- all
 }while(--i);
 _RESTORE_INTERRUPT(_t);
 meas_average_measured_values(&fw_data.exdata.cesd);
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
 uint8_t gas_v_trig = IOCFG_GET(IOP_GAS_V);
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
   mapsel0 = d.mapsel_uni1; //use condition result from selected univ.output instead
#endif
  d.fn_dat = mapsel0 ? &fw_data.tables[1] : &fw_data.tables[d.param.fn_gas];
 }
 else             //on petrol
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0x0F) != 0x0F)
   mapsel0 = d.mapsel_uni0; //use condition result from selected univ.output instead
#endif
  d.fn_dat = mapsel0 ? &fw_data.tables[0] : &fw_data.tables[d.param.fn_gasoline];
 }
#else //use tables from RAM
 uint8_t mapsel0 = IOCFG_GET(IOP_MAPSEL0); //note: if not mapped to real I/O, then stub() will always return 0  (we rely on it)
 if (d.sens.gas)
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0xF0) != 0xF0)
   mapsel0 = d.mapsel_uni1; //use condition result from selected univ.output instead
#endif
  select_table_set(mapsel0 ? 1 : d.param.fn_gas);          //on gas
 }
 else
 {
#ifdef UNI_OUTPUT
  if ((d.param.mapsel_uni & 0x0F) != 0x0F)
   mapsel0 = d.mapsel_uni0; //use condition result from selected univ.output instead
#endif
  select_table_set(mapsel0 ? 0 : d.param.fn_gasoline);     //on petrol
 }
#endif

#ifndef SECU3T //SECU-3i
 d.sens.oilpress_ok = IOCFG_GET(IOP_OILP_I); //oil pressure sensor
 d.sens.generator_ok = IOCFG_GET(IOP_GENS_I); //generator status

 d.sens.ign_i = IOCFG_GET(IOP_IGN);
 d.sens.cond_i = IOCFG_GET(IOP_COND_I);
 d.sens.epas_i = IOCFG_GET(IOP_EPAS_I);
#endif

 d.sens.input1 = IOCFG_GET(IOP_INPUT1);
 d.sens.input2 = IOCFG_GET(IOP_INPUT2);
}
