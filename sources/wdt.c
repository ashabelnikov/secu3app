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

/** \file wdt.c
 * Implementation of watchdog timer API (Реализация модуля сторожевого таймера)
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include <stdint.h>
#include "bitmask.h"
#include "wdt.h"

void wdt_start_timer(void)
{
#ifdef _PLATFORM_M644_
 if (!(WDTCSR & _BV(WDE)))
 { //not started yet
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDP0) | _BV(WDE);
 }
#else
 if (!(WDTCR & _BV(WDE)))
 { //not started yet
  WDTCR = _BV(WDP0) | _BV(WDE);  //timeout = 32ms
 }
#endif
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

#ifdef _PLATFORM_M644_
void wdt_turnoff_timer(void) 
{ 
 _BEGIN_ATOMIC_BLOCK();
 _WATCHDOG_RESET();

 //Clear WDRF in MCUSR 
 MCUSR&= ~_BV(WDRF); 

 // Write logical one to WDCE and WDE 
 WDTCSR = _BV(WDCE) | _BV(WDE); 

 // Turn off WDT 
 WDTCSR = 0x00; 

 _END_ATOMIC_BLOCK(); 
}
#endif
 