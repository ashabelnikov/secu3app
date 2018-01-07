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

/** \file pwrvalve.c
 * \author Alexey A. Shabelnikov
 * Implementation of controlling algorithms for Power Valve (Carburetor)
 */

#ifndef CARB_AFR //power valve functionality isn't needed when carburetor AFR control is used

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "pwrvalve.h"
#include "ioconfig.h"

/** Open/Close FE valve */
#define SET_FE_VALVE_STATE(s) IOCFG_SETF(IOP_FE, s)

void pwrvalve_init_ports(void)
{
 //Output for control of FE valve
 IOCFG_INIT(IOP_FE, 0); //FE valve is off
}

void pwrvalve_control(void)
{
 int16_t discharge;

 discharge = (d.sens.baro_press - d.sens.map);
 if (discharge < 0)
  discharge = 0;
 d.fe_valve = discharge < d.param.fe_on_threshold;
 SET_FE_VALVE_STATE(d.fe_valve);
}

#endif //CARB_AFR
