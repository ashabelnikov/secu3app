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

/** \file pwrrelay.h
 * \author Alexey A. Shabelnikov
 * Power management using external relay, allows SECU-3 to be turned on some time
 * after ignition is off. So, for instance electric colling fan can work even when ignition is off
 */

#ifndef _PWRRELAY_H_
#define _PWRRELAY_H_

/** Initialization of used I/O ports */
void pwrrelay_init_ports(void);

/** Control of power relay
 * Uses d ECU data structure
 */
void pwrrelay_control(void);

/** Get System power state. When power management is not available
 * this function will always return 1
 * \return 1 - power is up, 0 - power is down
 */
uint8_t pwrrelay_get_state(void);

/** Init active stepper motors used in the system
 * Uses d ECU data structure
 */
void pwrrelay_init_steppers(void);

/** Set mode of operation
 * \param mode 0 - normal operation (default), 1 - instant reaction on IGN or voltage
 */
void pwrrelay_set_opmode(uint8_t mode);

#endif //_PWRRELAY_H_
