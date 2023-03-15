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

/** \file params.c
 * \author Alexey A. Shabelnikov
 * Implementation of functionality for work with parameters (save/restore/check)
 */

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/port.h"

#include <string.h>
#include "bitmask.h"
#include "ce_errors.h"
#include "crc16.h"
#include "ecudata.h"
#include "eeprom.h"
#include "ioconfig.h"
#include "jumper.h"
#include "params.h"
#include "starter.h"
#include "suspendop.h"
#include "ventilator.h"
#include "vstimer.h"
#include "wdt.h"
#include "pwrrelay.h"

/**Used for saving parameters (automatic saving). It counts down a timeout. */
s_timer16_t save_param_timeout_counter = {0,0,1}; //already fired!

void save_param_if_need(void)
{
 uint16_t save_param_timeout = PGM_GET_WORD(&fw_data.exdata.save_param_timeout);
 //did parameters change during specified time? Also check if auto saving was enabled.
 if (s_timer_is_action(&save_param_timeout_counter) && save_param_timeout)
 {
  //Are current and saved parameters differ?
  if (memcmp(&eeprom_parameters_cache, &d.param, sizeof(params_t)-PAR_CRC_SIZE))
   sop_set_operation(SOP_SAVE_PARAMETERS);
  s_timer_set(&save_param_timeout_counter, save_param_timeout);
 }
}

void ckps_enable_ignition(uint8_t);
void ckps_init_ports(void);
#ifdef FUEL_INJECT
void inject_init_ports(void);
void inject_set_fuelcut(uint8_t);
#endif

void reset_eeprom_params(void)
{
 uint16_t crc;
 uint16_t i = 5000; //5 seconds max

 ckps_enable_ignition(0);        //turn off ignition
#ifdef FUEL_INJECT
 inject_set_fuelcut(0);          //turn off injection
#endif
 ckps_init_ports();
#ifdef FUEL_INJECT
 inject_init_ports();
#endif
 vent_turnoff();                 //turn off ventilator
 starter_set_blocking_state(1);  //block starter
 IOCFG_INIT(IOP_FL_PUMP, 0);     //turn off fuel pump
 IOCFG_INIT(IOP_IE, 0);          //turn off IE valve solenoid
 IOCFG_INIT(IOP_FE, 0);          //turn off power valve solenoid
 IOCFG_SETF(IOP_FL_PUMP, 0);     //turn off fuel pump
 pwrrelay_init_steppers();

 while(!eeprom_is_idle() && --i)
 {
  wdt_reset_timer();
  _DELAY_US(1000);      //1ms
 }
 _DISABLE_INTERRUPT();

 wdt_reset_timer();
 //1. calculate CRC; 2. write all except 2 bytes of CRC; 3. write CRC
 crc = crc16f((uint8_t _PGM*)&fw_data.def_param, sizeof(params_t)-PAR_CRC_SIZE);
 eeprom_write_P(&fw_data.def_param, EEPROM_PARAM_START, sizeof(params_t)-PAR_CRC_SIZE);
 eeprom_write(&crc, EEPROM_PARAM_START+(sizeof(params_t)-PAR_CRC_SIZE), PAR_CRC_SIZE);

 wdt_reset_timer();
 ce_clear_errors(); //reset saved errors
 wdt_reset_timer();
#ifdef REALTIME_TABLES
 crc = crc16f_b((uint8_t _PGM*)&fw_data.tables[0], sizeof(f_data_t)-sizeof(uint16_t));
 eeprom_write_P(&fw_data.tables[0], EEPROM_REALTIME_TABLES_START, sizeof(f_data_t)-sizeof(uint16_t));
 eeprom_write(&crc, EEPROM_REALTIME_TABLES_START+(sizeof(f_data_t)-sizeof(uint16_t)), sizeof(uint16_t));
#endif
#ifdef FUEL_INJECT
 //Write default values to LTFT map
 memset(&d.inj_ltft[0][0], 0, 256); //set to zero all cells
 crc = crc16((uint8_t*)&d.inj_ltft[0][0], INJ_VE_POINTS_L*INJ_VE_POINTS_F);
 eeprom_write(&d.inj_ltft[0][0], EEPROM_LTFT_TABLES_START, INJ_VE_POINTS_L*INJ_VE_POINTS_F);
 eeprom_write(&crc, EEPROM_LTFT_TABLES_START+(INJ_VE_POINTS_L*INJ_VE_POINTS_F), sizeof(uint16_t));
#endif
 //write 4 bytes of magic number identifying platform
 eeprom_write_P((void _PGM*)(FLASHEND-3), EEPROM_MAGIC_START, 4);
 wdt_reset_device(); //reboot!
}

void load_eeprom_params(void)
{
 if (jumper_get_defeeprom_state())
 {
  if (CHECKBIT(PGM_GET_BYTE(&fw_data.def_param.bt_flags), BTF_USE_RESPAR))
  {
   //User selected to use parameters from a FLASH  only
   MEMCPY_P(&d.param, &fw_data.def_param, sizeof(params_t));
  }
  else
  {
   //User selected to use paramaters from EEPROM
   //Load parameters from EEPROM, and after, check integrity of them
   //Don't take into account bytes of CRC when calculating check sum
   //If check sums don't match, then load default (reserve) parameters from flash
   eeprom_read(&d.param, EEPROM_PARAM_START, sizeof(params_t));

   if (crc16((uint8_t*)&d.param, (sizeof(params_t)-PAR_CRC_SIZE))!=d.param.crc)
   {
    MEMCPY_P(&d.param, &fw_data.def_param, sizeof(params_t));
    ce_set_error(ECUERROR_EEPROM_PARAM_BROKEN);
   }
  }
  //Initialize parameters' cache. In the opposite case unnecessary saving of parameters will occur after start of firmware
  memcpy(&eeprom_parameters_cache, &d.param, sizeof(params_t));
 }
 else
 {//jumper is closed - load default parameters, which will be saved soon. Also, load default data into EEPROM for editable set of tables
  MEMCPY_P(&d.param, &fw_data.def_param, sizeof(params_t));
  ce_clear_errors(); //clear saved CE errors
#ifdef REALTIME_TABLES
  uint16_t crc = crc16f_b((uint8_t _PGM*)&fw_data.tables[0], sizeof(f_data_t)-sizeof(uint16_t));
  eeprom_write_P(&fw_data.tables[0], EEPROM_REALTIME_TABLES_START, sizeof(f_data_t)-sizeof(uint16_t));
  eeprom_write(&crc, EEPROM_REALTIME_TABLES_START+(sizeof(f_data_t)-sizeof(uint16_t)), sizeof(uint16_t));
#endif
#ifdef FUEL_INJECT
 //Write default values to LTFT map
 memset(d.inj_ltft, 0, 256); //set to zero all cells
 crc = crc16((uint8_t*)&d.inj_ltft[0][0], INJ_VE_POINTS_L*INJ_VE_POINTS_F);
 eeprom_write(&d.inj_ltft[0][0], EEPROM_LTFT_TABLES_START, INJ_VE_POINTS_L*INJ_VE_POINTS_F);
 eeprom_write(&crc, EEPROM_LTFT_TABLES_START+(INJ_VE_POINTS_L*INJ_VE_POINTS_F), sizeof(uint16_t));
#endif
  //write 4 bytes of magic number identifying platform
  eeprom_write_P((void _PGM*)(FLASHEND-3), EEPROM_MAGIC_START, 4);
 }
}

#ifdef REALTIME_TABLES
void load_specified_tables_into_ram(uint8_t index)
{
 //load tables depending on index, if index is FLASH, then load from FLASH, if index is EEPROM, then load from EEPROM
 if (index < TABLES_NUMBER_PGM)
  MEMCPY_P(&d.tables_ram, &fw_data.tables[index], sizeof(f_data_t));
 else
 { //load set of tables from EEPROM
  eeprom_read(&d.tables_ram, EEPROM_REALTIME_TABLES_START, sizeof(f_data_t));
  if (crc16_b((uint8_t*)&d.tables_ram, (sizeof(f_data_t)-sizeof(uint16_t)))!=d.tables_ram.checksum)
  {
   ce_set_error(ECUERROR_EEPROM_TABL_BROKEN);
  }
 }

 //notification will be sent about that new set of tables has been loaded
 sop_set_operation(SOP_SEND_NC_TABLSET_LOADED);
}

#endif

#ifdef FUEL_INJECT
void load_ltft_tables_into_ram(void)
{
 uint16_t crc = 0;
 eeprom_read(&d.inj_ltft[0][0], EEPROM_LTFT_TABLES_START, INJ_VE_POINTS_L*INJ_VE_POINTS_F); //read contents of LTFT map
 eeprom_read(&crc, EEPROM_LTFT_TABLES_START+(INJ_VE_POINTS_L*INJ_VE_POINTS_F), sizeof(uint16_t)); //read value of checksum (2 bytes)
 if (crc16_b((uint8_t*)&d.inj_ltft[0][0], INJ_VE_POINTS_L*INJ_VE_POINTS_F)!=crc)
 { //LTFT table is corrupted!
  ce_set_error(ECUERROR_EEPROM_LTFT_BROKEN);
 }
}
#endif

void param_set_save_timer(void)
{
 s_timer_set(&save_param_timeout_counter, PGM_GET_WORD(&fw_data.exdata.save_param_timeout));
}

#ifdef SPEED_SENSOR
void load_odomet_data_into_ram(void)
{
 eeprom_read(&d.sens.vss_int_dist, EEPROM_ODOMETER_START, sizeof(uint32_t));
}
#endif
