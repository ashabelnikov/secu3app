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

/** \file gasdose.h
 * \author Alexey A. Shabelnikov
 * Gas dose controller (stepper motor).
 */

#ifndef _GASDOSE_H_
#define _GASDOSE_H_

#ifdef GD_CONTROL  //if gas dosator control included

#include <stdint.h>

struct ecudata_t;

/** Initialization of used I/O ports */
void gasdose_init_ports(void);

/** Initialization of the module (state variables etc)*/
void gasdose_init(void);

/** Does control of gas dosator
 * \param d pointer to ECU data structure
 */
void gasdose_control(struct ecudata_t* d);

/** Used in power management
 * \return 1 - gas dose actuator is ready, 0 - not ready
 */
uint8_t gasdose_is_ready(void);

/** Must be called from the main loop to notify about stroke events
 * \param d pointer to ECU data structure
 */
void gasdose_stroke_event_notification(struct ecudata_t* d);

#endif

#endif //_GASDOSE_H_
