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

/** \file starter.h
 * \author Alexey A. Shabelnikov
 * Working with starter
 * (Работа со стартером).
 */

#ifndef _STARTER_H_
#define _STARTER_H_

/** Initialization of used I/O ports (инициализация используемых портов) */
void starter_init_ports(void);

/** Control of starter (управление стартером)
 * Uses d ECU data structure
 */
void starter_control(void);

/** Blocking/unblocking of starter (блокировка/разблокировка стартера)
 * \param i_state 1 - block, 0 - unblock
 */
void starter_set_blocking_state(uint8_t i_state);

#endif //_STARTER_H_
