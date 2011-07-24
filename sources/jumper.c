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

/** \file jumper.c
 * Implementation of processing of jumpers which user can open/close to do some actions
 * (Реализация обработки перемычек которые пользователь может замыкать/размыкать для выполнения некоторых действий).
 */

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "jumper.h"

/**Retrieves state of jumper from port's pin */
#define GET_DEFEEPROM_JUMPER_STATE() (PINC_Bit2)

void jumper_init_ports(void)
{
 DDRC &= ~(_BV(DDC3)|_BV(DDC2)); //inputs (входы)
 PORTC|= _BV(PC3)|_BV(PC2);
}

uint8_t jumper_get_defeeprom_state(void)
{
 return GET_DEFEEPROM_JUMPER_STATE();
}
