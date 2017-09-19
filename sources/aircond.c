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
#include "ce_errors.h"
#include "ecudata.h"
#include "aircond.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"

#ifdef SECU3T
 #error "Air conditioner is not supported in the SECU-3T, use SECU-3i (undefine SECU3T)"
#endif

#ifndef FUEL_INJECT
 #error "Air conditioner is not supported without fuel injection, define FUEL_INJECT"
#endif

#define RPMREQSTEP 2

/**Define state variables */
typedef struct
{
 uint8_t state;
 uint16_t t1;      //!< timer
}aircond_t;

/**Instance of state variables */
aircond_t ac = {0};

void aircond_init_ports(void)
{
 IOCFG_INIT(IOP_COND_O, 0); //conditioner is turned off
}

void aircond_init(void)
{
 ac.state = 0;
}

void aircond_control(void)
{
 if (!IOCFG_CHECK(IOP_COND_O) || !IOCFG_CHECK(IOP_COND_I))
  return; //COND_O or COND_I remapped to other function

 if (!d.st_block)
 { //reset timer if engine is not running
  ac.t1 = s_timer_gtc();
  ac.state = 0;
  d.cond_req_fan = 0, d.cond_req_rpm = 0; //reset cooling fan and RPM requests
  IOCFG_SETF(IOP_COND_O, 0);
 }

 switch(ac.state)
 {
  case 0: //wait 5 seconds after start
   if ((s_timer_gtc() - ac.t1) < SYSTIM_MAGS(5.0))
    break;
   ++ac.state;

  case 1: //wait for turn on request
   d.cond_req_fan = 0; //reset cooling fan request
   IOCFG_SETF(IOP_COND_O, 0); //turned off
   if (IOCFG_GET(IOP_COND_I) && (d.sens.add_i3 < d.param.cond_pvt_on) && !ce_is_error(ECUERROR_ADD_I3_SENSOR) && (d.sens.temperat > TEMPERATURE_MAGNITUDE(75.0)) && (d.sens.tps < TPS_MAGNITUDE(68.0)))
   {
    if (d.sens.frequen < d.param.cond_min_rpm)
    {
     d.cond_req_rpm = d.sens.frequen;
     ++ac.state;   // state = 2
    }
    else
    {
     ac.state = 3; //RPM already ok, skip 2 state
     IOCFG_SETF(IOP_COND_O, 1); //turn on clutch
     ac.t1 = s_timer_gtc();
    }
   }
   break;

  case 2: //smoothly increase RPM if it is less than required
   if (d.cond_req_rpm >= d.param.cond_min_rpm)
   {
    ++ac.state;
    IOCFG_SETF(IOP_COND_O, 1); //turn on clutch
    ac.t1 = s_timer_gtc();
   }

  case 3: //conditioner is turned on, check for turn off conditions
   if (!IOCFG_GET(IOP_COND_I) || (d.sens.add_i3 > d.param.cond_pvt_off) || ce_is_error(ECUERROR_ADD_I3_SENSOR) || (d.sens.tps > TPS_MAGNITUDE(70.0)))
    ac.state = 1;
   if ((s_timer_gtc() - ac.t1) > SYSTIM_MAGS(1.5) && ac.state == 3)
    d.cond_req_fan = 1; //turn on cooling fan after 1.5 seconds
   break;
 }

}

void aircond_stroke_event_notification(void)
{
 if (ac.state == 2)
  d.cond_req_rpm+=RPMREQSTEP;
 else if (ac.state == 1 && d.cond_req_rpm)
  d.cond_req_rpm-=RPMREQSTEP;
}

#endif
