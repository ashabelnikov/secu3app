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

/** \file etc.h
 * \author Alexey A. Shabelnikov
 * Control electronic throttle
 */

#ifndef _ETC_H_
#define _ETC_H_

#if !defined(SECU3T) && defined(ELEC_THROTTLE)

/**Initialization of I/O ports*/
void etc_init_ports(void);

/**Implements control algorithm
 * Uses d ECU data structure
 */
void etc_control(void);

/** Check if ETC is enabled (used)
 * \return 1 - ETC functionality is enabled, 0 - ETC functionality is disabled
 */
uint8_t etc_is_enabled(void);

/** Gets home position
 * \return home position value * 64
 */
uint16_t etc_get_homepos(void);

#endif //!SECU3T && ELEC_THROTTLE

#endif //_ETC_H_
