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

/** \file wdt.h
 * Watchdog timer API (Модуль сторожевого таймера)
 */

#ifndef _WDT_H_
#define _WDT_H_

/** Starts watchdog timer if it is not started yet */
void wdt_start_timer(void);

/** Reset watchdog timer */
void wdt_reset_timer(void);

/** Reset device using watchdog */
void wdt_reset_device(void);

#endif //_WDT_H_
