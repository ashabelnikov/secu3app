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

/** \file bc_input.h
 * \author Alexey A. Shabelnikov
 * CE errors information output using blink codes
 */

#ifndef _BC_INPUT_H_
#define _BC_INPUT_H_

/**Initialization of used input ports*/
void bc_init_ports(void);

/** Check BC_INPUT and if it is active, then enter blink codes 
 * indication mode. This mode has its own infinite loop.
 * Note that in this mode interrupts must be disabled!
 * Uses d ECU data structure
 */
void bc_indication_mode(void);

#endif //_BC_INPUT_H_
