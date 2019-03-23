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

//TODO: To reduce memory usage it is possible to use crc or some simple hash algorithms to control state of memory(changes). So this variable becomes unneeded.
/* Cache for buffering parameters used during suspended EEPROM operations */
params_t eeprom_parameters_cache;

/**ECU data structure. Contains all related data and state information */
struct ecudata_t d =
{
 .param = {0},
 .sens = {0},
 .corr = {0},

 .load = 0,

 .ie_valve = 0,
 .fe_valve = 0,
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 .fc_revlim = 0,
#endif
 .cool_fan = 0,
 .st_block = 0,   //starter is not blocked
 .ce_state = 0,
 .airflow = 0,
 .choke_pos = 0,
 .gasdose_pos = 0,  //GD

#ifdef REALTIME_TABLES
 .tables_ram = {{0},{0},{0},
                {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}}, //work
                {0}, //temp
                {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}}, //VE
                {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}}, //AFR
                {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}}, //Timing
                {0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}},
 .mm_ptr8 = 0,
 .mm_ptr16 = 0,
 .mm_ptr12 = 0,
#endif
 .fn_dat = 0,

 .op_comp_code = 0,
 .op_actn_code = 0,
 .ecuerrors_for_transfer = 0,
 .ecuerrors_saved_transfer = 0,
 .use_knock_channel_prev = 0,

 .engine_mode = EM_START,

#ifdef DIAGNOSTICS
 .diag_inp = {0},
 .diag_out = 0,
#endif

 .choke_testing = 0,
 .choke_manpos_d = 0,
 .choke_rpm_reg = 0,

 .gasdose_testing = 0,  //GD
 .gasdose_manpos_d = 0, //GD

 .bt_name = {0,0,0,0,0,0,0,0,0},
 .bt_pass = {0,0,0,0,0,0,0},
 .sys_locked = 0      //unlocked

#ifdef FUEL_INJECT
,.inj_pw = 0,
 .inj_pwns = {0,0},
 .inj_dt = 0,
 .inj_fff = 0,
 .eng_running = 0           //fully stopped
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
,.acceleration = 0
#endif

#ifdef AIRCONDIT
,.cond_req_rpm = 0,
 .cond_req_fan = 0
#endif

#ifdef UNI_OUTPUT
,.mapsel_uni0 = 0,
 .mapsel_uni1 = 0
#endif

 ,.floodclear = 0
 ,.cond_state = 0
};

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
