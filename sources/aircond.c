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

#ifndef FUEL_INJECT
 #error "Air conditioner is not supported without fuel injection, define FUEL_INJECT"
#endif

#define RPMREQSTEP 2

#ifndef SECU3T
/** Calculates value of condition used to turn on clutch
 * \return 0 or 1
 */
static uint8_t get_refprs_on_cond(void)
{
 if (IOCFG_CHECK(IOP_REFPRS_I))
 {
  uint16_t refprs = IOCFG_GETA(IOP_REFPRS_I); //get state of the input
  if (IOCFG_DTST(refprs))
   return !IOCFG_DGET(refprs); //mapped to bare digital input
  else //mapped to an analog input
   return (d.param.cond_pvt_on ? (refprs < d.param.cond_pvt_on && !IOCFG_GETE(IOP_REFPRS_I)) : 1); //don't use input if threshold is set to 0
 }
 else //not used (not mapped to physical input)
  return 1;
}

/** Calculates value of condition used to turn off clutch
 * \return 0 or 1
 */
static uint8_t get_refprs_off_cond(void)
{
 if (IOCFG_CHECK(IOP_REFPRS_I))
 {
  uint16_t refprs = IOCFG_GETA(IOP_REFPRS_I); //get state of the input
  if (IOCFG_DTST(refprs))
   return IOCFG_DGET(refprs); //mapped to bare digital input
  else //mapped to an analog input
   return (d.param.cond_pvt_off ? (refprs > d.param.cond_pvt_off || IOCFG_GETE(IOP_REFPRS_I)) : 0); //don't use input if threshold is set to 0
 }
 else //not used (not mapped to physical input)
  return 0;
}
#endif

/**Define state variables */
typedef struct
{
 uint8_t state;
 uint16_t t1;      //!< timer
 uint16_t t2;
}aircond_t;

/**Instance of state variables */
aircond_t ac = {0,0};

void aircond_init_ports(void)
{
#ifndef SECU3T
 IOCFG_INIT(IOP_COND_O, 0); //conditioner is turned off
#endif
}

void aircond_control(void)
{
 if (!IOCFG_CHECK(IOP_COND_I))
  return; //COND_I remapped to other function. Air conditioner control is totally impossible
 //if COND_O mapped to other function, then simple control algorithm will be used. For SECU-3T only simple algorithm can be used.

 if (d.engine_mode==EM_START)
 { //reset timer if engine is not running
  ac.t1 = s_timer_gtc();
  ac.state = 0;
  d.cond_req_fan = 0, d.cond_req_rpm = 0; //reset cooling fan and RPM requests
#ifndef SECU3T
  IOCFG_SETF(IOP_COND_O, 0);
  d.cond_state = 0;
#endif
 }

 switch(ac.state)
 {
  case 0: //wait 5 seconds after start
   if ((s_timer_gtc() - ac.t1) < SYSTIM_MAGS(5.0))
    break;
   ++ac.state;

  case 1: //wait for turn on request
  {
   d.cond_req_fan = 0; //reset cooling fan request
#ifdef SECU3T
   if (IOCFG_GET(IOP_COND_I))
#else
   IOCFG_SETF(IOP_COND_O, 0); //turned off
   d.cond_state = 0;

   if ((!IOCFG_CHECK(IOP_COND_O) && IOCFG_GET(IOP_COND_I)) || (IOCFG_CHECK(IOP_COND_O) && IOCFG_GET(IOP_COND_I) && get_refprs_on_cond()
     && (d.sens.temperat > (int16_t)PGM_GET_WORD(&fw_data.exdata.aircond_clt)) && (d.sens.temperat < ((int16_t)PGM_GET_WORD(&fw_data.exdata.aircond_clt_ovh) - TEMPERATURE_MAGNITUDE(1.0)))
     && (d.sens.tps < PGM_GET_WORD(&fw_data.exdata.aircond_tps))))
#endif
   {
    if (d.sens.rpm < d.param.cond_min_rpm)
    {
     d.cond_req_rpm = d.sens.rpm;
     ++ac.state;   // state = 2
    }
    else
    {
     ac.state = 3; //RPM already ok, skip 2 state
#ifndef SECU3T
     IOCFG_SETF(IOP_COND_O, 1); //turn on clutch
     d.cond_state = 1;
     d.cond_req_rpm = d.param.cond_min_rpm; //specify minimum RPM, see calc_cl_rpm() in funconv.c for more information
#endif
     ac.t1 = s_timer_gtc();
    }
   }
  }
   break;

  case 2: //smoothly increase RPM if it is less than required
   if (d.cond_req_rpm >= d.param.cond_min_rpm)
   {
    ++ac.state;
#ifndef SECU3T
    IOCFG_SETF(IOP_COND_O, 1); //turn on clutch
    d.cond_state = 1;
#endif
    ac.t1 = s_timer_gtc();
   }

  case 3: //conditioner is turned on, check for turn off conditions
  {
   ac.t2 = s_timer_gtc();
#ifdef SECU3T
   if (!IOCFG_GET(IOP_COND_I))
#else
   if ((!IOCFG_CHECK(IOP_COND_O) && !IOCFG_GET(IOP_COND_I)) || (IOCFG_CHECK(IOP_COND_O) && (!IOCFG_GET(IOP_COND_I) || get_refprs_off_cond()
    || (d.sens.temperat >= (int16_t)PGM_GET_WORD(&fw_data.exdata.aircond_clt_ovh))
    || (d.sens.tps > (PGM_GET_WORD(&fw_data.exdata.aircond_tps) + TPS_MAGNITUDE(2.0)))))
)
#endif
    ac.state = 1;
   if ((s_timer_gtc() - ac.t1) > SYSTIM_MAGS(1.5) && ac.state == 3)
    d.cond_req_fan = 1; //turn on cooling fan after 1.5 seconds
  }
   break;
 }

}

void aircond_stroke_event_notification(void)
{
 if (ac.state == 2)                        //smoothly increase RPM
  d.cond_req_rpm+=RPMREQSTEP;
 else if ((s_timer_gtc() - ac.t2) > PGM_GET_WORD(&fw_data.exdata.aircond_idlrpm_delay) && ac.state == 1 && d.cond_req_rpm >= RPMREQSTEP) //smoothly decrease RPM
  d.cond_req_rpm-=RPMREQSTEP;
}

#endif
