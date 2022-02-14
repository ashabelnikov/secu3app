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

/** \file bluetooth.h
 * \author Alexey A. Shabelnikov
 * Bluetooth related functionality and logic (baud rate setting, name, password)
 */

#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#ifdef BLUETOOTH_SUPP

#include <stdint.h>

/**Module initialization
 * \param en_set_baud Enable bluetooth baud rate setting after initialization
 */
void bt_init(uint8_t en_set_baud);

/** Sets bluetooth baud rate. Must be called subsequently until it return non-zero
 * Uses d ECU data structure
 * \param baud ID of baud rate (see uart.h for more information). This is a baud rate we want to set using this function.
 * \return value > 0 if baud rate is set, 0 if baud rate is not set yet
 */
uint8_t bt_set_baud(uint16_t baud);

/** Starts setting of name  and password to bluetooth */
void bt_start_set_namepass(void);

/** Sets bluetooth name and password. Must be called subsequently until it return non-zero
 * Uses d ECU data structure
 * \return value > 0 if name and password are set, 0 if name and password are not set yet
 */
uint8_t bt_set_namepass(void);

#endif //BLUETOOTH_SUPP

#endif //_BLUETOOTH_H_
