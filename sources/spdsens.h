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

/** \file spdsens.h
 * \author Alexey A. Shabelnikov
 * Speed sensor support
 * This is only interface to two functions placed in camsens.c
 */

#ifndef _SPDSENS_H_
#define _SPDSENS_H_

#ifdef SPEED_SENSOR

#include <stdint.h>

/** Get current period between VSS speed pulses
 * \return Value in us multiplied by 4 (when clock=16mHz), by 3.2 (when clock=20mHz)
 */
uint16_t spdsens_get_period(void);

/** Get current value of VSS pulse counter (begins to count after switching on ignition)
 * \param reset If this parameter is 1, then counter will be reset after obtaining value from it
 * \return count of VSS pulses (number of pulses since last ignition turn on and before next clearing
 */
uint16_t spdsens_get_pulse_count(uint8_t reset);

#endif //SPEED_SENSOR

#endif //_SPDSENS_H_
