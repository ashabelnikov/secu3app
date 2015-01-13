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
 * ECU data in RAM (global data structures and state variables)
 */

#include "port/port.h"
#include "ecudata.h"

/**ECU data structure. Contains all related data and state information */
struct ecudata_t edat;

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
 return PGM_GET_WORD(((uint8_t _PGM*)edat.fn_dat) + offset);
}
#endif
