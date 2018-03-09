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

/** \file jumper.c
 * \author Alexey A. Shabelnikov
 * Implementation of processing of jumpers which user can open/close to do some actions
 */

#include "port/avrio.h"
#include "port/port.h"
#include "port/intrinsic.h"
#include "bitmask.h"
#include "jumper.h"

/**Retrieves state of jumper from port's pin */
#define GET_DEFEEPROM_JUMPER_STATE() (CHECKBIT(PINC, PINC2) > 0)

/**3 states of jumper read in series 3 times*/
uint8_t de_samples[3];

void jumper_init_ports(void)
{
 uint8_t i = 3;
 DDRC &= ~(_BV(DDC3)|_BV(DDC2)); //inputs
 PORTC|= _BV(PC3)|_BV(PC2);

 _NO_OPERATION();
 _NO_OPERATION();

 //accumulate 3 samples
 while(i--)
  de_samples[i] = GET_DEFEEPROM_JUMPER_STATE();
}

uint8_t jumper_get_defeeprom_state(void)
{
 //Majority gate
 return (de_samples[0] & de_samples[1]) | (de_samples[1] & de_samples[2]) | (de_samples[0] & de_samples[2]);
}
