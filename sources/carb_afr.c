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
 * Implementation of AFR control on carburetor using electronic actuators driven by PWM
 */

#ifdef CARB_AFR

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "carb_afr.h"
#include "ioconfig.h"

#ifdef FUEL_INJECT
 #error "You can't use carburetor AFR control together with fuel injection, please omit FUEL_INJECT option"
#endif

void carbafr_init_ports(void)
{
}

void carbafr_init(void)
{
}

void carbafr_control(struct ecudata_t* d)
{
}

#endif
