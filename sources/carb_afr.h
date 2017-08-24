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

/** \file carb_afr.h
 * \author Alexey A. Shabelnikov
 * Control of AFR on carburetor using electronic actuators driven by PWM
 */

#ifndef _CARB_AFR_H_
#define _CARB_AFR_H_

#ifdef CARB_AFR   //if carburetor AFR control enabled

/**Initialization of I/O ports*/
void carbafr_init_ports(void);

/** Initialization of the module (state variables etc)*/
void carbafr_init(void);

/**Implements control algorithm
 * Uses d ECU data structure
 */
void carbafr_control(void);

#endif //CARB_AFR

#endif //_CARB_AFR_H_
