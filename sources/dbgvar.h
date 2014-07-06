/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

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

/** \file dbgvar.h
 * Bebug variables header file. Variables are in the uart.c unit
 */

#ifndef _DBGVAR_H_
#define _DBGVAR_H_

#ifdef DEBUG_VARIABLES
//See also uart.c
extern uint16_t dbg_var1;  //User's debug variable 1
extern uint16_t dbg_var2;  //User's debug variable 2
extern uint16_t dbg_var3;  //User's debug variable 3
extern uint16_t dbg_var4;  //User's debug variable 4
#endif

#endif //_DBGVAR_H_
