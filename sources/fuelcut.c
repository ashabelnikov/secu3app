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

/** \file fuelcut.c
 * \author Alexey A. Shabelnikov
 * Implementation of controling of Idle Cut-off valve (Carburetor) of fuel cut (Fuel injection).
 */

#ifndef CARB_AFR
#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "fuelcut.h"
#include "ioconfig.h"
#include "vstimer.h"
#else
#ifdef FUEL_INJECT
 #error "You can not use FUEL_INJECT option together with CARB_AFR"
#endif
#endif

#ifdef FUEL_INJECT

/**State variable for delay counter*/
static uint8_t state = 0;

void fuelcut_init_ports(void)
{
 //empty
}

void fuelcut_control(struct ecudata_t* d)
{
 if (d->sens.inst_frq > d->param.ie_hit)
 {
  //When RPM > hi threshold, then check TPS, CTS and MAP
  if ((!d->sens.carb) && (d->sens.temperat > d->param.fuelcut_cts_thrd) && (d->sens.map < d->param.fuelcut_map_thrd))
  {
   if (0==state)
   {
    s_timer_set(epxx_delay_time_counter, d->param.shutoff_delay);
    state = 1;
   }
   else
   {
    if (s_timer_is_action(epxx_delay_time_counter))
     d->ie_valve = 0;  //Cut fuel
   }
  }
  else
  {
   d->ie_valve = 1;   //normal operation
   state = 0;
  }
 }
 else if (d->sens.inst_frq < d->param.ie_lot || d->sens.carb)
 { //always turn on fuel when RPM < low threshold or throttle is opened
  d->ie_valve = 1;
  state = 0;
 }

 //simple Rev. limitter
 if (d->sens.inst_frq > d->param.revlim_hit)
 {
  d->fc_revlim = 1; //cut fuel
 }
 else if (d->sens.inst_frq < d->param.revlim_lot)
 {
  d->fc_revlim = 0; //restore fuel
 }
}

#else //Carburetor (Idle Cut-off valve control)

#ifndef CARB_AFR //Carb. AFR control supersede idle cut-off functionality

void fuelcut_init_ports(void)
{
 IOCFG_INIT(IOP_IE, 1); //valve is turned on
}

//Implementation of Idle Cut-off valve control. If throttle gate is closed AND frq > [up.threshold] OR
//throttle gate is closed AND frq > [lo.threshold] BUT valve is already closed, THEN turn off
//fuel supply by stopping to apply voltage to valve's coil. ELSE - fuel supply is turned on.
void fuelcut_control(struct ecudata_t* d)
{
 //if throttle gate is opened, then open valve,reload timer and exit from condition
 if (d->sens.carb)
 {
  d->ie_valve = 1;
  s_timer_set(epxx_delay_time_counter, d->param.shutoff_delay);
 }
 //if throttle gate is closed, then state of valve depends on RPM,previous state of valve,timer and type of fuel
 else
  if (d->sens.gas) //gas
   d->ie_valve = ((s_timer_is_action(epxx_delay_time_counter))
   &&(((d->sens.inst_frq > d->param.ie_lot_g)&&(!d->ie_valve))||(d->sens.inst_frq > d->param.ie_hit_g)))?0:1;
  else //petrol
   d->ie_valve = ((s_timer_is_action(epxx_delay_time_counter))
   &&(((d->sens.inst_frq > d->param.ie_lot)&&(!d->ie_valve))||(d->sens.inst_frq > d->param.ie_hit)))?0:1;
 IOCFG_SET(IOP_IE, d->ie_valve);
}

#endif //CARB_AFR

#endif
