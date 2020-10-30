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

/** \file grheat.c
 * \author Alexey A. Shabelnikov
 * Implementation of gas reducer's heating control.
 */

#ifndef SECU3T

#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"
#include "funconv.h"

/***/
#define GRH_DUTY_MULT 10

/***/
#define GRH_PWM_STEPS (GRH_DUTY_MULT*2)

/**Declare state variables structure*/
typedef struct
{
 uint8_t timeout;       //!<
 uint8_t pwm_state;     //!< Needed by finite state machine (current state)
 uint16_t strt_t1;      //!< Used for timing of PWM
 uint16_t strt_t2;      //!< Used for timer
}grh_state_t;

/**Global instance of state variables */
grh_state_t grhs = {0,0,0,0};

void grheat_init_ports(void)
{
 IOCFG_INIT(IOP_GRHEAT, 0);  //<-- heating is off
}

void grheat_control(void)
{
 if (!IOCFG_CHECK(IOP_GRHEAT))
  return; //output is not assigned to phisical I/O

 if ((s_timer_gtc() - grhs.strt_t2) >= PGM_GET_WORD(&fw_data.exdata.grheat_time))
  grhs.timeout = 1;

 if (!d.sens.gas || grhs.timeout)
 {
  IOCFG_SET(IOP_GRHEAT, 0); //off
  grhs.strt_t1 = s_timer_gtc();
  return;
 }


 uint8_t duty = (((uint16_t)grheat_pwm_duty()) * ROUNDU16((1.0/GRH_DUTY_MULT)*2048)) >> 11; //result: 0...20

 //software PWM control
 if (duty == 0)
  IOCFG_SET(IOP_GRHEAT, 0); //off
 else if (duty == GRH_PWM_STEPS)
  IOCFG_SET(IOP_GRHEAT, 1); //on
 else if (0==grhs.pwm_state)
 {
  if ((s_timer_gtc() - grhs.strt_t1) >= (GRH_PWM_STEPS - duty))
  {
   IOCFG_SET(IOP_GRHEAT, 1); //on
   grhs.strt_t1 = s_timer_gtc();
   ++grhs.pwm_state;
  }
 }
 else
 {
  if ((s_timer_gtc() - grhs.strt_t1) >= duty)
  {
   IOCFG_SET(IOP_GRHEAT, 0); //off
   grhs.strt_t1 = s_timer_gtc();
   --grhs.pwm_state;
  }
 }
}

void grheat_cog_changed_notification(void)
{
 grhs.strt_t2 = s_timer_gtc();
 grhs.timeout = 0;
}

#endif //SECU-3i
