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

/** \file fuelcut.h
 * Control of Idle Cut-off valve (Carburetor) or fuel cut (Fuel injection).
 */

#ifndef _IDLECON_H_
#define _IDLECON_H_

struct ecudata_t;

/** Initialization of used I/O ports */
void idlecon_init_ports(void);

/** Does control of valve or fuel cut
 * \param d pointer to ECU data structure
 */
void idlecon_control(struct ecudata_t* d);

#endif //_IDLECON_H_
