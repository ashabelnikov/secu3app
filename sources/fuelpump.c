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

/** \file fuelpump.c
 * Implementation of controling of electric fuel pump.
 * (Реализация управления электробензонасосом).
 */

#ifdef FUEL_PUMP

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "fuelpump.h"
#include "secu3.h"
#include "vstimer.h"

/** Turn on/turn off fuel pump */
//#define TURN_ON_ELPUMP(s) {PORTB_Bit0 = s;}

void fuelpump_init_ports(void)
{
// PORTB|= _BV(PB0); //valve is turned on (клапан ЭПХХ включен)
// DDRB |= _BV(DDB0);
}

void fuelpump_control(struct ecudata_t* d)
{
 //TODO:
}

#endif //FUEL_PUMP
