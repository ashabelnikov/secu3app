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

/** \file wdt.c
 * Implementation of watchdog timer API (Реализация модуля сторожевого таймера)
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "wdt.h"

void wdt_start_timer(void)
{
 if (!(WDTCR & _BV(WDE)))
 { //not started yet
  WDTCR = _BV(WDP0) | _BV(WDE);  //timeout = 32ms
 }
}

void wdt_reset_timer(void)
{
 _WATCHDOG_RESET();
}

void wdt_reset_device(void)
{
 wdt_start_timer();
 _DISABLE_INTERRUPT();
 for(;;);
}
