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

/** \file jumper.h
 * \author Alexey A. Shabelnikov
 * Process jumpers which user can open/close to do some actions
 */

#ifndef _JUMPER_H_
#define _JUMPER_H_

#include <stdint.h>

/** Initialization of used I/O ports */
void jumper_init_ports(void);

/**Get state of "Default EEPROM" jumper which was at the moment of start up
 * \return 0 - closed, 1 - opened.
 */
uint8_t jumper_get_defeeprom_state(void);

#endif //_JUMPER_H_
