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

/** \file aircond.c
 * \author Alexey A. Shabelnikov
 * Implementation of control of air conditioner
 */

#ifdef AIRCONDIT

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "aircond.h"
#include "ioconfig.h"
#include "magnitude.h"

#ifdef SECU3T
 #error "Air conditioner is not supported in the SECU-3T, use SECU-3i (undefine SECU3T)"
#endif

#ifndef FUEL_INJECT
 #error "Air conditioner is not supported without fuel injection, define FUEL_INJECT"
#endif

/**Define state variables */
//typedef struct
//{
//}aircond_t;

/**Instance of state variables */
//aircond_t ac = {0};

void aircond_init_ports(void)
{
 IOCFG_INIT(IOP_COND_O, 0); //conditioner is turned off
}

void aircond_init(void)
{
}

void aircond_control(void)
{
 if (!IOCFG_CHECK(IOP_COND_O))
  return; //COND_O remapped to other function

}

#endif
