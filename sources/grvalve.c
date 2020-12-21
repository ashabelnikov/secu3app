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

/** \file grvalve.c
 * \author Alexey A. Shabelnikov
 * Implementation of gas valve control.
 */

#ifndef SECU3T

#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"
#include "funconv.h"

/**Controls state of GASVAL_O output*/
#define GASVAL_CONTROL(v) IOCFG_SET(IOP_GASVAL_O, (v)); \
  d.gasval_on = (v);

/**Declare state variables structure*/
typedef struct
{
 uint8_t  mode;    //!< state machine
 uint16_t t1;      //!< Used for delay
 uint16_t delay;   //!< sampled value of delay
}grv_state_t;

/**Global instance of state variables */
grv_state_t grvs = {0,0,0};

void grvalve_init_ports(void)
{
 IOCFG_INIT(IOP_GASVAL_O, 0);  //<-- valve is off
}

void grvalve_init_state(void)
{
 grvs.t1 = s_timer_gtc();
 d.gasval_on = 0;
}

void grvalve_control(void)
{
 if (!IOCFG_CHECK(IOP_GASVAL_O))
  return; //output is not assigned to phisical I/O

 if (!d.sens.gas)
 {
  GASVAL_CONTROL(0); //<-- valve is off on petrol
  grvs.t1 = s_timer_gtc();
  grvs.mode = 0;
  return;
 }

 switch(grvs.mode)
 {
  case 0: //sample value of delay
   GASVAL_CONTROL(0); //off
   grvs.delay = grv_delay();
   ++grvs.mode;

  case 1: //count down delay
   if ((s_timer_gtc() - grvs.t1) >= grvs.delay)
   {
    GASVAL_CONTROL(1); //on
    grvs.t1 = s_timer_gtc();
    ++grvs.mode;
   }
   else
   {
    d.gasval_res = 1;
    break;
   }

  case 2: //count down time during which valve is on
   if ((s_timer_gtc() - grvs.t1) >= PGM_GET_WORD(&fw_data.exdata.gasval_ontime))
   {
    GASVAL_CONTROL(0); //off
   }
   d.gasval_res = 0;
   break;

  case 3: //engine is running/cranking
   GASVAL_CONTROL(1);  //<-- valve is on when engine is running
   return;
 }
}

void grvalve_cog_changed_notification(void)
{
 grvs.mode = 3;
}

void grvalve_eng_stopped_notification(void)
{
 if (3==grvs.mode)
 {
  grvs.t1 = s_timer_gtc();
  grvs.mode = 2;
 }
}

#endif //SECU-3i
