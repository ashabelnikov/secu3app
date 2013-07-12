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

/** \file bluetooth.h
 * Bluetooth related functionality and logic (baud rate setting, name, password)
 * (Логика связанная с блютузом (установка скорости, имя, пароль)).
 */

#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <stdint.h>

/**Module initialization */
void bt_init(void);

/** Sets bluetooth baud rate. Must be called subsequently until it return non-zero
 * \param baud Baud rate code (see uart.h for more information)
 * \return value > 0 if baud rate is set, 0 if baud rate is not set yet
 */
uint8_t bt_set_baud(uint16_t baud);

#endif //_BLUETOOTH_H_
