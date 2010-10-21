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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#include <ioavr.h>
#include "jumper.h"

#define GET_DEFEEPROM_JUMPER_STATE() (PINC_Bit2)

void jumper_init_ports(void)
{
 DDRC &= ~((1<<DDC3)|(1<<DDC2)); //входы
 PORTC|= (1<<PC3)|(1<<PC2);
}

uint8_t jumper_get_defeeprom_state(void)
{
 return GET_DEFEEPROM_JUMPER_STATE();
}
