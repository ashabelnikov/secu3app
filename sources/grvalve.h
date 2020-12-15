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

/** \file grvalve.h
 * \author Alexey A. Shabelnikov
 * Gas valve control.
 */

#ifndef _GRVALVE_H_
#define _GRVALVE_H_

#ifndef SECU3T

/** Initialization of used I/O ports */
void grvalve_init_ports(void);

/** Performs control of the gas valve
 * Uses d ECU data structure
 */
void grvalve_control(void);

/** Initialization of state variables
 */
void grvalve_init_state(void);

/**Called from main loop when system detects changing of cog number*/
void grvalve_cog_changed_notification(void);

/**called from main loop when system detects engine stop*/
void grvalve_eng_stopped_notification(void);

#endif //SECU-3i

#endif //_GRVALVE_H_
