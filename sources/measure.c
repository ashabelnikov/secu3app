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
 * Implementation pf processing (averaging, corrections etc) of data comes from ADC and sensors
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

// Size of each ring buffer
#define FRQ_AVERAGING        4                 //!< Number of values for averaging of RPM for tachometer
#define MAP_AVERAGING        4                 //!< Number of values for averaging of pressure (MAP)
#define BAT_AVERAGING        4                 //!< Number of values for averaging of board voltage
#define TMP_AVERAGING        8                 //!< Number of values for averaging of coolant temperature
#define TPS_AVERAGING        4                 //!< Number of values for averaging of throttle position
#define AI1_AVERAGING        4                 //!< Number of values for averaging of ADD_I1
#define AI2_AVERAGING        4                 //!< Number of values for averaging of ADD_I2
#define SPD_AVERAGING        8                 //!< Number of values for averaging of speed sensor periods (SPEED_SENSOR option must be included)
#define AI3_AVERAGING        4                 //!< Number of values for averaging of ADD_I3 (SECU3T option must be excluded OR PA4_INP_IGNTIM option must be included)
#define AI4_AVERAGING        4                 //!< Number of values for averaging of ADD_I4 (SECU3T option must be excluded AND TPIC8101 option must be included)

//Index of each ring buffer
#define FRQ_INPIDX           0                 //!< Index of ring buffer for RPM
#define MAP_INPIDX           1                 //!< Index of ring buffer for MAP
#define BAT_INPIDX           2                 //!< Index of ring buffer for board voltage
#define TMP_INPIDX           3                 //!< Index of ring buffer for CTS
#define TPS_INPIDX           4                 //!< Index of ring buffer for TPS
#define AI1_INPIDX           5                 //!< Index of ring buffer for ADD_I1
#define AI2_INPIDX           6                 //!< Index of ring buffer for ADD_I2
#define SPD_INPIDX           7                 //!< Index of ring buffer for VSS
#define AI3_INPIDX           8                 //!< Index of ring buffer for ADD_I3
#define AI4_INPIDX           9                 //!< Index of ring buffer for ADD_I4

#define CIRCBUFFMAX 8                          //!< Maximum size of ring buffer in items
#define INPUTNUM 10                            //!< number of ring buffers

/**Describes ring buffer for one input*/
typedef struct
{
 uint16_t buff[CIRCBUFFMAX];                    //!< Ring buffer
 uint8_t ai;                                    //!< index in buffer
}meas_input_t;

/**Ring buffers for all inputs */
meas_input_t meas[INPUTNUM] = {{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0},{{0},0}};
/**Number of averages for each input. Values must be a degree of 2 (see average_buffer() for more information)*/
PGM_DECLARE(uint8_t avnum[INPUTNUM]) = {FRQ_AVERAGING, MAP_AVERAGING, BAT_AVERAGING, TMP_AVERAGING, TPS_AVERAGING, AI1_AVERAGING, AI2_AVERAGING, SPD_AVERAGING, AI3_AVERAGING, AI4_AVERAGING};

static uint16_t update_buffer(uint8_t idx, uint16_t value)
{
 meas[idx].buff[meas[idx].ai] = value;
 (meas[idx].ai==0) ? (meas[idx].ai = PGM_GET_BYTE(&avnum[idx]) - 1): meas[idx].ai--;
 return value;
}

static uint16_t average_buffer(uint8_t idx)
{
 uint8_t i = PGM_GET_BYTE(&avnum[idx]) - 1;  uint32_t sum = 0;
 do
 {
  sum+=meas[idx].buff[i];
 }while(i--);

 //We use shifts instead of division.
 uint8_t sht = 4; // sum/16
 if (PGM_GET_BYTE(&avnum[idx])==4)
  sht = 2;
 if (PGM_GET_BYTE(&avnum[idx])==8)
  sht = 3;
 return sum >> sht;
//return sum / PGM_GET_BYTE(&avnum[idx]);
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
#endif
 //We don't initialize analog inputs (ADD_I1, ADD_I2, CARB, ADD_I3, ADD_I4) because they are initialised by default
 //and we don't need pullup resistors for them
}

//Update ring buffers
void meas_update_values_buffers(uint8_t rpm_only, ce_sett_t _PGM *cesd)
{
 uint16_t rawval;

 update_buffer(FRQ_INPIDX, d.sens.inst_frq);

 if (rpm_only)
  return;

 rawval = update_buffer(MAP_INPIDX, adc_get_map_value());

#ifdef SEND_INST_VAL
 rawval = ce_is_error(ECUERROR_MAP_SENSOR_FAIL) ? cesd->map_v_em : adc_compensate(_RESDIV(rawval, 2, 1), d.param.map_adc_factor, d.param.map_adc_correction);
 d.sens.inst_map = map_adc_to_kpa(rawval, d.param.map_curve_offset, d.param.map_curve_gradient);
#endif

 rawval = update_buffer(BAT_INPIDX, adc_get_ubat_value());
#ifdef SEND_INST_VAL
 d.sens.inst_voltage = ce_is_error(ECUERROR_VOLT_SENSOR_FAIL) ? cesd->vbat_v_em : adc_compensate(rawval * 6, d.param.ubat_adc_factor, d.param.ubat_adc_correction);
#endif

 update_buffer(TMP_INPIDX, adc_get_temp_value());

 rawval = update_buffer(TPS_INPIDX, adc_get_carb_value());
#ifdef SEND_INST_VAL
 rawval = adc_compensate(_RESDIV(rawval, 2, 1), d.param.tps_adc_factor, d.param.tps_adc_correction);
 d.sens.inst_tps = tps_adc_to_pc(ce_is_error(ECUERROR_TPS_SENSOR_FAIL) ? cesd->tps_v_em : rawval, d.param.tps_curve_offset, d.param.tps_curve_gradient);
 if (d.sens.inst_tps > TPS_MAGNITUDE(100))
  d.sens.inst_tps = TPS_MAGNITUDE(100);
#endif

 rawval = update_buffer(AI1_INPIDX, adc_get_add_i1_value());
#ifdef SEND_INST_VAL
 d.sens.inst_add_i1 = ce_is_error(ECUERROR_ADD_I1_SENSOR) ? cesd->add_i1_v_em : adc_compensate(_RESDIV(rawval, 2, 1), d.param.ai1_adc_factor, d.param.ai1_adc_correction);
#endif

 update_buffer(AI2_INPIDX, adc_get_add_i2_value());

#if !defined(SECU3T) || defined(PA4_INP_IGNTIM)
 update_buffer(AI3_INPIDX, adc_get_add_i3_value());
#endif

#if !defined(SECU3T) && defined(TPIC8101)
 update_buffer(AI4_INPIDX, adc_get_knock_value());
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
  d.sens.knock_k = ce_is_error(ECUERROR_KSP_CHIP_FAILED) ? cesd->ks_v_em : d.sens.knock_raw;
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
 d.sens.map_raw = adc_compensate(_RESDIV(average_buffer(MAP_INPIDX), 2, 1), d.param.map_adc_factor, d.param.map_adc_correction);
 d.sens.map = map_adc_to_kpa(ce_is_error(ECUERROR_MAP_SENSOR_FAIL) ? cesd->map_v_em : d.sens.map_raw, d.param.map_curve_offset, d.param.map_curve_gradient);

 d.sens.voltage_raw = adc_compensate(average_buffer(BAT_INPIDX) * 6, d.param.ubat_adc_factor,d.param.ubat_adc_correction);
 d.sens.voltage = ubat_adc_to_v(ce_is_error(ECUERROR_VOLT_SENSOR_FAIL) ? cesd->vbat_v_em : d.sens.voltage_raw);

 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
 {
  d.sens.temperat_raw = adc_compensate(_RESDIV(average_buffer(TMP_INPIDX), 5, 3),d.param.temp_adc_factor,d.param.temp_adc_correction);
#ifndef THERMISTOR_CS
  d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) ? cesd->cts_v_em : d.sens.temperat_raw);
#else
  if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_MAP)) //use linear sensor
   d.sens.temperat = temp_adc_to_c(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) ? cesd->cts_v_em : d.sens.temperat_raw);
  else //use lookup table (actual for thermistor sensors)
   d.sens.temperat = thermistor_lookup(ce_is_error(ECUERROR_TEMP_SENSOR_FAIL) ? cesd->cts_v_em : d.sens.temperat_raw, fw_data.exdata.cts_curve);
#endif
 }
 else                                       //CTS is not used
  d.sens.temperat = 0;

 d.sens.frequen=average_buffer(FRQ_INPIDX);

#ifdef SPEED_SENSOR
 d.sens.speed=average_buffer(SPD_INPIDX);
#endif

 d.sens.tps_raw = adc_compensate(_RESDIV(average_buffer(TPS_INPIDX), 2, 1), d.param.tps_adc_factor, d.param.tps_adc_correction);
 d.sens.tps = tps_adc_to_pc(ce_is_error(ECUERROR_TPS_SENSOR_FAIL) ? cesd->tps_v_em : d.sens.tps_raw, d.param.tps_curve_offset, d.param.tps_curve_gradient);
 if (d.sens.tps > TPS_MAGNITUDE(100))
  d.sens.tps = TPS_MAGNITUDE(100);

 d.sens.add_i1_raw = adc_compensate(_RESDIV(average_buffer(AI1_INPIDX), 2, 1), d.param.ai1_adc_factor, d.param.ai1_adc_correction);
 d.sens.add_i1 = ce_is_error(ECUERROR_ADD_I1_SENSOR) ? cesd->add_i1_v_em : d.sens.add_i1_raw;

 d.sens.add_i2_raw = adc_compensate(_RESDIV(average_buffer(AI2_INPIDX), 2, 1), d.param.ai2_adc_factor, d.param.ai2_adc_correction);
 d.sens.add_i2 = ce_is_error(ECUERROR_ADD_I2_SENSOR) ? cesd->add_i2_v_em : d.sens.add_i2_raw;

#if !defined(SECU3T) || defined(PA4_INP_IGNTIM)
 d.sens.add_i3_raw = adc_compensate(average_buffer(AI3_INPIDX), d.param.ai3_adc_factor, d.param.ai3_adc_correction);
 d.sens.add_i3 = ce_is_error(ECUERROR_ADD_I3_SENSOR) ? cesd->add_i3_v_em : d.sens.add_i3_raw;
#endif

#if !defined(SECU3T) && defined(TPIC8101)
 d.sens.add_i4_raw = adc_compensate(average_buffer(AI4_INPIDX), d.param.ai4_adc_factor, d.param.ai4_adc_correction);
 d.sens.add_i4 = ce_is_error(ECUERROR_ADD_I4_SENSOR) ? cesd->add_i4_v_em : d.sens.add_i4_raw;
#endif

#ifdef AIRTEMP_SENS
 if (IOCFG_CHECK(IOP_AIR_TEMP))
  d.sens.air_temp = thermistor_lookup(d.sens.add_i2, fw_data.exdata.ats_curve);   //ADD_I2 input selected as MAT sensor
 else
  d.sens.air_temp = 0; //input is not selected
#endif

#ifndef SECU3T //SECU-3i
#ifdef TPIC8101
 if (IOCFG_CHECK(IOP_MAP2))
  d.sens.map2 = map_adc_to_kpa(d.sens.add_i4, d.param.map2_curve_offset, d.param.map2_curve_gradient); //ADD_I4 input selected as MAP2 sensor
 else
#endif
  d.sens.map2 = 0; //input is not selected
#endif

#ifndef SECU3T //SECU-3i
 if (IOCFG_CHECK(IOP_TMP2))
  d.sens.tmp2 = thermistor_lookup(d.sens.add_i3, fw_data.exdata.tmp2_curve); //ADD_I3 input selected as TMP2 sensor
 else
  d.sens.tmp2 = 0; //input is not selected
#endif

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
  if (d.param.inj_config[0] != d.param.inj_config[1]) //update settings only if it is necessary!
  {
   //update some parameters which depend on type of fuel
   //TODO: redundant code fragment
#ifdef FUEL_INJECT
   inject_set_num_squirts(d.param.inj_config[d.sens.gas] & 0xF);  //number of squirts
   inject_set_config(d.param.inj_config[d.sens.gas] >> 4);        //type of injection
#if defined(PHASE_SENSOR) && !defined(PHASED_IGNITION)
   cams_enable_cam(
#ifdef FUEL_INJECT
     (d.param.inj_config[d.sens.gas] >> 4) == INJCFG_FULLSEQUENTIAL ||
#endif
     CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF));
#endif
#endif
  }
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
#endif
}
