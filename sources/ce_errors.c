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

/** \file ce_errors.c
 * \author Alexey A. Shabelnikov
 * Implementation of controling of CE, errors detection and related functionality.
 */

#include "port/avrio.h"
#include "port/port.h"
#include <string.h>
#include <stddef.h>
#include "adc.h"
#include "bitmask.h"
#include "camsens.h"
#include "ce_errors.h"
#include "ckps.h"
#include "ecudata.h"
#include "eeprom.h"
#include "ioconfig.h"
#include "knock.h"
#include "magnitude.h"
#include "pwrrelay.h"
#include "suspendop.h"
#include "tables.h"  //for ce_sett_t type
#include "vstimer.h"

/**CE state variables structure */
typedef struct
{
 uint32_t ecuerrors;         //!< 32 error codes maximum
 uint32_t merged_errors;     //!< caching errors to preserve resource of the EEPROM
 uint32_t write_errors;      //!< func. eeprom_start_wr_data() launches background process!
 uint16_t bv_tdc;            //!< board voltage debouncong counter for eliminating of false errors during normal transients
 uint8_t  bv_eds;            //!< board voltage error detecting state, used for state machine
 uint8_t  bv_dev;            //!< board voltage deviation flag, if 0, then voltage is below normal, if 1, then voltage is above normal
 uint8_t  turnout_low_priority_errors_counter;
#ifdef DEFERRED_CRC
 uint8_t  en_err_cls;        //!< Enable error's clearing flag
#endif
}ce_state_t;

/**State variables */
ce_state_t ce_state = {0,0,0,0,0,0,0,
#ifdef DEFERRED_CRC
0
#endif
};

/**CE timer. Used for counting of time intervals for CE*/
s_timer16_t ce_control_time_counter = {0,CE_CONTROL_STATE_TIME_VALUE,0};

//operations under errors
void ce_set_error(uint8_t error)
{
 SETBIT32(ce_state.ecuerrors, error);
}

void ce_clear_error(uint8_t error)
{
 CLEARBIT32(ce_state.ecuerrors, error);
}

uint8_t ce_is_error(uint8_t error)
{
 return !!CHECKBIT32(ce_state.ecuerrors, error);
}

/** Internal function. Contains checking logic 
 * Uses d ECU data structure
 * \param cesd Pointer to the CE settings data structure
 */
void check(ce_sett_t _PGM *cesd)
{
 //If error of CKP sensor was, then set corresponding bit of error
 if (ckps_is_error())
 {
  if (pwrrelay_get_state()) //set error only if power is on
  {
   //ignore error in case of stall of an engine
   if (!(((d.st_block) && (d.sens.inst_frq < d.param.starter_off)) || (d.sens.inst_frq < 30)))
    ce_set_error(ECUERROR_CKPS_MALFUNCTION);
  }
  ckps_reset_error();
 }
 else
 {
  ce_clear_error(ECUERROR_CKPS_MALFUNCTION);
 }

#ifdef PHASE_SENSOR
 if (cams_is_error())
 {
  if (pwrrelay_get_state()) //set error only if power is on
  {
   ce_set_error(ECUERROR_CAMS_MALFUNCTION);
  }
  cams_reset_error();
 }
 else
 {
  ce_clear_error(ECUERROR_CAMS_MALFUNCTION);
 }
#endif

 //if error of knock channel was
 if (d.param.knock_use_knock_channel)
 {
  if (knock_is_error())
  {
   ce_set_error(ECUERROR_KSP_CHIP_FAILED);
   knock_reset_error();
  }
  else if ((d.sens.knock_k < PGM_GET_WORD(&cesd->ks_v_min) || d.sens.knock_k > PGM_GET_WORD(&cesd->ks_v_max)) && d.sens.frequen > 1000)
   ce_set_error(ECUERROR_KSP_CHIP_FAILED);
  else
   ce_clear_error(ECUERROR_KSP_CHIP_FAILED);
 }
 else
  ce_clear_error(ECUERROR_KSP_CHIP_FAILED);

 //checking MAP sensor. TODO: implement additional check
 if (((d.sens.map_raw < PGM_GET_WORD(&cesd->map_v_min)) || (d.sens.map_raw > PGM_GET_WORD(&cesd->map_v_max))) && d.sens.carb)
  ce_set_error(ECUERROR_MAP_SENSOR_FAIL);
 else
  ce_clear_error(ECUERROR_MAP_SENSOR_FAIL);

 //checking coolant temperature sensor
 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
 {
  if (d.sens.temperat_raw < PGM_GET_WORD(&cesd->cts_v_min) || d.sens.temperat_raw > PGM_GET_WORD(&cesd->cts_v_max))
   ce_set_error(ECUERROR_TEMP_SENSOR_FAIL);
  else
   ce_clear_error(ECUERROR_TEMP_SENSOR_FAIL);
 }
 else
  ce_clear_error(ECUERROR_TEMP_SENSOR_FAIL);

 //checking the voltage using simple state machine
 if (0==ce_state.bv_eds) //voltage is OK
 {
  if (d.sens.voltage_raw < PGM_GET_WORD(&cesd->vbat_v_min))
  { //below normal
   ce_state.bv_dev = 0, ce_state.bv_eds = 1;
  }
  else if (d.sens.voltage_raw > PGM_GET_WORD(&cesd->vbat_v_max))
  { //above normal
   ce_state.bv_dev = 1, ce_state.bv_eds = 1;
  }
  else
   ce_clear_error(ECUERROR_VOLT_SENSOR_FAIL);

  ce_state.bv_tdc = 800; //init debouncing counter
 }
 else if (1==ce_state.bv_eds) //voltage is not OK
 {
  //use simple debouncing techique to eliminate errors during normal transients (e.g. switching ignition off) 
  if (ce_state.bv_tdc)
  {//state changed? If so, then reset state machine (start again)
   if ((0==ce_state.bv_dev && d.sens.voltage_raw > PGM_GET_WORD(&cesd->vbat_v_min)) ||
       (1==ce_state.bv_dev && d.sens.voltage_raw < PGM_GET_WORD(&cesd->vbat_v_max)))
    ce_state.bv_eds = 0;

   --ce_state.bv_tdc;
  }
  else
  { //debouncing counter is expired
   //error if U > 4 and RPM > 2500
   if (d.sens.voltage_raw > ROUND(4.0 / ADC_DISCRETE) && d.sens.inst_frq > 2500)
    ce_set_error(ECUERROR_VOLT_SENSOR_FAIL);
   else
    ce_clear_error(ECUERROR_VOLT_SENSOR_FAIL);

   ce_state.bv_eds = 0; //reset state machine
  }
 }

 //checking TPS sensor
 if ((d.sens.tps_raw < PGM_GET_WORD(&cesd->tps_v_min)) || (d.sens.tps_raw > PGM_GET_WORD(&cesd->tps_v_max)))
  ce_set_error(ECUERROR_TPS_SENSOR_FAIL);
 else
  ce_clear_error(ECUERROR_TPS_SENSOR_FAIL);

 //checking ADD_I1 sensor
 if ((d.sens.add_i1_raw < PGM_GET_WORD(&cesd->add_i1_v_min)) || (d.sens.add_i1_raw > PGM_GET_WORD(&cesd->add_i1_v_max)))
  ce_set_error(ECUERROR_ADD_I1_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I1_SENSOR);

 //checking ADD_I2 sensor
 if ((d.sens.add_i2_raw < PGM_GET_WORD(&cesd->add_i2_v_min)) || (d.sens.add_i2_raw > PGM_GET_WORD(&cesd->add_i2_v_max)))
  ce_set_error(ECUERROR_ADD_I2_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I2_SENSOR);

#ifndef SECU3T //SECU-3i
 //checking ADD_I3 sensor
 if ((d.sens.add_i3_raw < PGM_GET_WORD(&cesd->add_i3_v_min)) || (d.sens.add_i3_raw > PGM_GET_WORD(&cesd->add_i3_v_max)))
  ce_set_error(ECUERROR_ADD_I3_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I3_SENSOR);
#ifdef TPIC8101
 //checking ADD_I4 sensor
 if ((d.sens.add_i4_raw < PGM_GET_WORD(&cesd->add_i4_v_min)) || (d.sens.add_i4_raw > PGM_GET_WORD(&cesd->add_i4_v_max)))
  ce_set_error(ECUERROR_ADD_I4_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I4_SENSOR);
#endif

#ifdef MCP3204
 //checking ADD_I5 sensor
 if ((d.sens.add_i5_raw < PGM_GET_WORD(&cesd->add_i5_v_min)) || (d.sens.add_i5_raw > PGM_GET_WORD(&cesd->add_i5_v_max)))
  ce_set_error(ECUERROR_ADD_I5_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I5_SENSOR);
 //checking ADD_I6 sensor
 if ((d.sens.add_i6_raw < PGM_GET_WORD(&cesd->add_i6_v_min)) || (d.sens.add_i6_raw > PGM_GET_WORD(&cesd->add_i6_v_max)))
  ce_set_error(ECUERROR_ADD_I6_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I6_SENSOR);
 //checking ADD_I7 sensor
 if ((d.sens.add_i7_raw < PGM_GET_WORD(&cesd->add_i7_v_min)) || (d.sens.add_i7_raw > PGM_GET_WORD(&cesd->add_i7_v_max)))
  ce_set_error(ECUERROR_ADD_I7_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I7_SENSOR);
 //checking ADD_I8 sensor
 if ((d.sens.add_i8_raw < PGM_GET_WORD(&cesd->add_i8_v_min)) || (d.sens.add_i8_raw > PGM_GET_WORD(&cesd->add_i8_v_max)))
  ce_set_error(ECUERROR_ADD_I8_SENSOR);
 else
  ce_clear_error(ECUERROR_ADD_I8_SENSOR);
#endif
#endif

#ifndef SECU3T //SECU-3i
 //perform checking only if IOP_I input is mapped to real input and if set number of strokes elapsed since last engine start
 if (IOCFG_CHECK(IOP_OPS_I) && s_timer_sss() > PGM_GET_WORD(&cesd->oilpress_timer))
 {
  if (d.sens.ops < PGM_GET_WORD(&cesd->oilpress_thrd))
   ce_set_error(ECUERROR_OILPRESSURE);
  else
   ce_clear_error(ECUERROR_OILPRESSURE);
 }
 else
  ce_clear_error(ECUERROR_OILPRESSURE); //can't monitor oil pressure on stall engine
#endif

#ifdef FUEL_INJECT
 //perform checking only if IOP_I input is mapped to real input and if set number of strokes elapsed since last engine start
 if (d.sens.inj_duty > 199)
  ce_set_error(ECUERROR_INJDUTY_LIMIT);
 else
  ce_clear_error(ECUERROR_INJDUTY_LIMIT);
#endif
}

//If any error occurs, the CE is light up for a fixed time. If the problem persists (eg corrupted the program code),
//then the CE will be turned on continuously. At the start of program CE lights up for 0.5 seconds. for indicating
//of the operability.
void ce_check_engine(void)
{
 uint32_t temp_errors;

 check(&fw_data.exdata.cesd);

 //If the timer counted the time, then turn off the CE
 if (s_timer_is_action(&ce_control_time_counter))
 {
  ce_set_state(CE_STATE_OFF);
  d.ce_state = 0; //<--doubling
 }

 //If at least one error is present  - turn on CE and start timer
 if (ce_state.ecuerrors!=0)
 {
  s_timer_set(&ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
  ce_set_state(CE_STATE_ON);
  d.ce_state = 1;  //<--doubling
 }

 temp_errors = (ce_state.merged_errors | (ce_state.ecuerrors & ~_BV32(ECUERROR_SYS_START))); //also exclude saving of ECUERROR_SYS_START flag

 //check for error which is still not in merged_errors
 if (temp_errors!=ce_state.merged_errors)
 {
  //Because at the time of appearing of a new error, EEPROM can be busy (for example, saving options),
  //then it is necessary to run deffered operation, which will be automatically executed as soon as the EEPROM
  //will be released.
  sop_set_operation(SOP_SAVE_CE_MERGED_ERRORS);
 }

 ce_state.merged_errors = temp_errors;

 //copy error's bits into the cache for transferring
 d.ecuerrors_for_transfer|= ce_state.ecuerrors;
}

void ce_save_merged_errors(uint32_t* p_merged_errors)
{
 uint32_t temp_errors;

 if (!p_merged_errors) //overwrite with parameter?
 {
  eeprom_read(&temp_errors, offsetof(eeprom_data_t, errors), sizeof(uint32_t));
  ce_state.write_errors = temp_errors | ce_state.merged_errors;
  if (ce_state.write_errors!=temp_errors)
   eeprom_start_wr_data(0, offsetof(eeprom_data_t, errors), &ce_state.write_errors, sizeof(uint32_t));
 }
 else
 {
  ce_state.merged_errors = *p_merged_errors;
  eeprom_start_wr_data(OPCODE_CE_SAVE_ERRORS, offsetof(eeprom_data_t, errors), (uint8_t*)&ce_state.merged_errors, sizeof(uint32_t));
 }
}

void ce_clear_errors(void)
{
 memset(&ce_state, 0, sizeof(ce_state_t));
 eeprom_write((uint8_t*)&ce_state.write_errors, offsetof(eeprom_data_t, errors), sizeof(uint32_t));
}

void ce_init_ports(void)
{
 IOCFG_INIT(IOP_CE, CE_STATE_ON); //CE is ON (for checking)
}

void ce_stroke_event_notification(void)
{
#ifdef DEFERRED_CRC
 if (ce_state.en_err_cls)
 {
#endif
  //Stop to indicate these errors after starting of engine (after a certain number of strokes)
  if (ce_state.turnout_low_priority_errors_counter == 254)
  {
   ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);
   ce_clear_error(ECUERROR_PROGRAM_CODE_BROKEN);
   ce_clear_error(ECUERROR_EEPROM_TABL_BROKEN);
   ce_clear_error(ECUERROR_EEPROM_LTFT_BROKEN);
  }
  if (ce_state.turnout_low_priority_errors_counter < 255)
   ce_state.turnout_low_priority_errors_counter++;
#ifdef DEFERRED_CRC
 }
#endif
}

#ifdef DEFERRED_CRC
void ce_enable_errors_clearing(void)
{
 ce_state.en_err_cls = 1;
}
#endif
