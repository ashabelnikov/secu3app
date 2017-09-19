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
#include "bitmask.h"

/**ECU data structure. Contains all related data and state information */
struct ecudata_t d;
/* Cache for buffering parameters used during suspended EEPROM operations */
uint8_t eeprom_parameters_cache[sizeof(params_t) + 1];

#ifdef REALTIME_TABLES
uint8_t mm_get_byte_ram(uint16_t offset)
{
 return *(((uint8_t*)&d.tables_ram) + offset);
}

uint8_t mm_get_byte_pgm(uint16_t offset)
{
 return PGM_GET_BYTE(((uint8_t _PGM*)d.fn_dat) + offset);
}

uint16_t mm_get_word_ram(uint16_t offset)
{
 return *((uint16_t*)(((uint8_t*)&d.tables_ram) + offset));
}

/** Get 12-bit word from SRAM
 * \param offset offset of the array from the beginning of structure
 * \param off offset of cell from the beginning of array
 */
uint16_t mm_get_w12_ram(uint16_t offset, uint8_t off)
{
 uint16_t word = *((uint16_t*)(((uint8_t*)&d.tables_ram) + offset + ((uint16_t)off+(off>>1)) ));
 return ((off & 0x0001) ? word >> 4 : word) & 0x0FFF;
}

uint16_t mm_get_word_pgm(uint16_t offset)
{
 return PGM_GET_WORD((uint16_t _PGM*)(((uint8_t _PGM*)d.fn_dat) + offset));
}
#endif

/** Get 12-bit word from flash
 * \param offset offset of the array from the beginning of structure
 * \param off offset of cell from the beginning of array
 */
uint16_t mm_get_w12_pgm(uint16_t offset, uint8_t off)
{
 uint16_t word = (PGM_GET_WORD((uint16_t _PGM*)(((uint8_t _PGM*)d.fn_dat) + offset + ((uint16_t)off+(off>>1)) )));
 return ((off & 0x0001) ? word >> 4 : word) & 0x0FFF;
}


/**Initialization of variables and data structures
 * \param d pointer to ECU data structure
 */
void init_ecu_data(void)
{
 d.op_comp_code = 0;
 d.op_actn_code = 0;
 d.sens.inst_frq = 0;
 d.corr.curr_angle = 0;
 d.corr.knock_retard = 0;
 d.ecuerrors_for_transfer = 0;
 d.eeprom_parameters_cache = &eeprom_parameters_cache[0];
 d.engine_mode = EM_START;
 d.ce_state = 0;
 d.cool_fan = 0;
 d.st_block = 0; //starter is not blocked
 d.sens.tps = d.sens.tps_raw = 0;
 d.sens.add_i1 = d.sens.add_i1_raw = 0;
 d.sens.add_i2 = d.sens.add_i2_raw = 0;
#ifndef SECU3T //SECU3i
 d.sens.add_i3 = d.sens.add_i3_raw = 0;
#ifdef TPIC8101
 d.sens.add_i4 = d.sens.add_i4_raw = 0;
#endif
#endif
 d.choke_testing = 0;
 d.choke_pos = 0;
 d.choke_manpos_d = 0;
 d.choke_rpm_reg = 0;
 d.gasdose_testing = 0;    //GD
 d.gasdose_pos = 0;        //GD
 d.gasdose_manpos_d = 0;   //GD
 d.bt_name[0] = 0;
 d.bt_pass[0] = 0;
 d.sys_locked = 0; //unlocked
#ifdef FUEL_INJECT
 d.inj_pw = 0;
 d.inj_pw_raw = 0;
 d.inj_dt = 0;
 d.corr.afr = 0;
 d.corr.inj_timing = 0;
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL) || defined(CARB_AFR)
 d.corr.lambda = 0;
#endif
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 d.fc_revlim = 0;
 d.acceleration = 0;
#endif

#ifdef AIRCONDIT
 d.cond_req_rpm = 0;
 d.cond_req_fan = 0;
#endif
}
