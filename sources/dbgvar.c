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

/** \file uart.c
 * \author Alexey A. Shabelnikov
 * Implementation of service for performing communication via UART.
 * (Реализация поддержки обмена данными через UART).
 */

#ifdef DEBUG_VARIABLES

#include "port/avrio.h"
#include "port/port.h"
#include <stdint.h>
#include "bitmask.h"
#include "dbgvar.h"
#include "ecudata.h"

//User's debug variables. See also dbgvar.h header file.
//Default values are zero.
uint16_t dbg_var1 = 0;   /**User's debug variable 1*/
uint16_t dbg_var2 = 0;   /**User's debug variable 2*/
uint16_t dbg_var3 = 0;   /**User's debug variable 3*/
uint16_t dbg_var4 = 0;   /**User's debug variable 4*/
#endif

