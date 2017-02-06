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

/** \file ecudata.c
 * \author Alexey A. Shabelnikov
 * ECU data in RAM (global data structures and state variables)
 */

#include "port/port.h"
#include "ecudata.h"
#include "eculogic.h"   //EM_START

/**ECU data structure. Contains all related data and state information */
struct ecudata_t edat;
/* Cache for buffering parameters used during suspended EEPROM operations */
uint8_t eeprom_parameters_cache[sizeof(params_t) + 1];

#ifdef REALTIME_TABLES
uint8_t mm_get_byte_ram(uint16_t offset)
{
 return *(((uint8_t*)&edat.tables_ram) + offset);
}

uint8_t mm_get_byte_pgm(uint16_t offset)
{
 return PGM_GET_BYTE(((uint8_t _PGM*)edat.fn_dat) + offset);
}

uint16_t mm_get_word_ram(uint16_t offset)
{
 return *((uint16_t*)(((uint8_t*)&edat.tables_ram) + offset));
}

uint16_t mm_get_word_pgm(uint16_t offset)
{
 return PGM_GET_WORD((uint16_t _PGM*)(((uint8_t _PGM*)edat.fn_dat) + offset));
}
#endif


/**Initialization of variables and data structures
 * \param d pointer to ECU data structure
 */
void init_ecu_data(void)
{
 edat.op_comp_code = 0;
 edat.op_actn_code = 0;
 edat.sens.inst_frq = 0;
 edat.corr.curr_angle = 0;
 edat.corr.knock_retard = 0;
 edat.ecuerrors_for_transfer = 0;
 edat.eeprom_parameters_cache = &eeprom_parameters_cache[0];
 edat.engine_mode = EM_START;
 edat.ce_state = 0;
 edat.cool_fan = 0;
 edat.st_block = 0; //starter is not blocked
 edat.sens.tps = edat.sens.tps_raw = 0;
 edat.sens.add_i1 = edat.sens.add_i1_raw = 0;
 edat.sens.add_i2 = edat.sens.add_i2_raw = 0;
 edat.choke_testing = 0;
 edat.choke_pos = 0;
 edat.choke_manpos_d = 0;
 edat.choke_rpm_reg = 0;
 edat.gasdose_testing = 0;    //GD
 edat.gasdose_pos = 0;        //GD
 edat.gasdose_manpos_d = 0;   //GD
 edat.bt_name[0] = 0;
 edat.bt_pass[0] = 0;
 edat.sys_locked = 0; //unlocked
#ifdef FUEL_INJECT
 edat.inj_pw = 0;
 edat.corr.afr = 0;
 edat.corr.inj_timing = 0;
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL) || defined(CARB_AFR)
 edat.corr.lambda = 0;
#endif
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 edat.fc_revlim = 0;
 edat.acceleration = 0;
#endif
}
