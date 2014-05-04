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

/** \file injector.c
 * Implementation of fuel injector control
 * (Реализация управления топливной форсункой).
 */

#ifdef FUEL_INJECT

#include "port/port.h"
#include "bitmask.h"
#include "injector.h"

#ifndef _PLATFORM_M644_
 #error "You can not use FUEL_INJECT option without _PLATFORM_M644_"
#endif

/***/
typedef struct
{
 volatile uint16_t inj_time;              //!< Current injection time, used in interrupts
}inj_state_t;

/***/
inj_state_t inj;


void inject_init_state(void)
{
}

void inject_init_ports(void)
{
}

void inject_set_inj_time(uint16_t time)
{
 _BEGIN_ATOMIC_BLOCK();
 inj.inj_time = time;
 _END_ATOMIC_BLOCK();
}

void inject_start_inj(void)
{
}

/***/
ISR(TIMER2_COMPB_VECT)
{
}

//todo: we can try to free TIMER2_COMPA_VECT for second injector by switching cooling fan PWM to TIMER0_COMPB

#endif
