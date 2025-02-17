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

/** \file etc.c
 * \author Alexey A. Shabelnikov
 * Implementation of control of electronic throttle
 */

#if !defined(SECU3T) && defined(ELEC_THROTTLE)

#include <stdlib.h>
#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ce_errors.h"
#include "ecudata.h"
#include "etc.h"
#include "funconv.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "mathemat.h"
#include "pwrrelay.h"
#include "ventilator.h"
#include "vstimer.h"
#include "dbgvar.h"

#define HOMESAMPTIME 30   //!< home position sampling delay - 300ms
#define APPSERRTHRD  100  //!< APPS error threshold when the ETC motor will be turned off
#define TPSERRTHRD   100  //!< TPS error threshold when the ETC motor will be turned off

/**Define state variables */
typedef struct
{
 uint8_t state;             //!< state variable used for sampling of home position
 uint16_t home_pos;         //!< throttle position read after turning on of ignition lock
 int16_t  prev_error[2];    //!< for PID's errors
 int16_t  pid_duty;         //!< duty from PID      
 s_timer16_t timer0;        //!< timer
 uint16_t apps_err_counter; //!< APPS error counter
 uint16_t tps_err_counter;  //!< TPS error counter
}etc_t;

/**Instance of state variables */
static etc_t etc = {0, TPS_MAGNITUDE(7.0), {0,0}, 0, {0,HOMESAMPTIME,0}, 0, 0};

void etc_init_ports(void)
{
 IOCFG_INIT(IOP_ETC_PWM1, 1); //1 - because of hardware inverter (74HC04)
 IOCFG_INIT(IOP_ETC_PWM2, 1);
}

/** Tracks TPS and APPS errors
 * \raturn 1 - OK, 0 - error (ETC must be turned off)
 */
static uint8_t track_errors(void)
{
 if (ce_is_error(ECUERROR_TPS_SENSOR_FAIL))
 {
  if (etc.tps_err_counter < TPSERRTHRD)
   ++etc.tps_err_counter;
 }

 if (ce_is_error(ECUERROR_APPS))
 {
  if (etc.apps_err_counter < APPSERRTHRD)
   ++etc.apps_err_counter;
 }
 return (etc.tps_err_counter < TPSERRTHRD) && (etc.apps_err_counter < APPSERRTHRD);
}

/** Samples home position */
static void sample_homepos(void)
{
 if (!pwrrelay_get_state())
 {
  vent_set_duty12(0); //turn off the motor
  s_timer_set(&etc.timer0, HOMESAMPTIME);
  etc.state = 0; //we will sample home position
  etc.pid_duty = 0;
  return; //do nothing!
 }
 else if (0==etc.state)
 {
  if (!s_timer_is_action(&etc.timer0))
   return;
  etc.home_pos = d.sens.tps; //sample home position
  etc.state = 1;
 }
}

void etc_control(void)
{
 if (!etc_is_enabled())
  return; //ETC is not enabled

 sample_homepos();

 if (s_timer_is_action(&etc.timer0))
 {
  s_timer_set(&etc.timer0, d.param.etc_pid_period);
  
  if (!track_errors())
  {
   vent_set_duty12(0); //turn off the motor
   return;
  }

  //calculate TPS position relatime to the home position
  int16_t tpsrh = ((int16_t)d.sens.tps_dbw) - etc.home_pos;
  int16_t duty = etc_spring_preload(tpsrh);   //apply spring preload duty map
  int16_t targ_tps = d.floodclear ? d.sens.apps1 : etc_pedal_to_throttle(); //use look up table(APPS, RPM) as base OR pedal position in the flood clear mode
#ifdef FUEL_INJECT
  targ_tps+= d.etc_idleadd;                   //apply addition from idling regulator
#endif
  //PID
  int16_t error = targ_tps - d.sens.tps_dbw;
  int16_t errdb = etc_acceptable_error(d.sens.tps_dbw);
  if (abs(error) >= errdb)
  {
   int16_t intlim = 100*64; //100%
   restrict_value_to(&error, -intlim, intlim); //limit maximum error
   int16_t derror = error - etc.prev_error[0];
   int16_t dderror = error - (2 * derror) + etc.prev_error[1];
   int32_t pid_add = ((int32_t)derror * d.param.etc_p) + ((int32_t)error * d.param.etc_i) + ((int32_t)dderror * d.param.etc_d);
   int16_t pid_add1 = SHTDIV16(pid_add, 12);
   restrict_value_to(&pid_add1, -16383, 16383);
   etc.pid_duty += pid_add1;
   restrict_value_to(&etc.pid_duty, -16383, 16383);
  }
  etc.prev_error[1] = etc.prev_error[0];
  etc.prev_error[0] = error; //save for further calculation of derror

  duty += etc.pid_duty; //apply PID output to the final duty

  //frictional torque compensation
  if (abs(d.sens.tpsdot) < d.param.etc_frictorq_thrd)
  {
   if (tpsrh < 0)
    duty -= ((int16_t)d.param.etc_frictorq_op) << 2;
   else if (tpsrh > 0)
    duty += ((int16_t)d.param.etc_frictorq_cl) << 2;
  }

  restrict_value_to(&duty, -(((int16_t)d.param.etc_nmax_duty) << 5), ((uint16_t)d.param.etc_pmax_duty) << 5);

  //map 6400 duty discretes to 4095 duty discretes
  int32_t duty32 = ((int32_t)duty)*41933; //41933 = (4095/6400)*65536
  duty = SHTDIV32(duty32, 16);

  vent_set_duty12(duty);
 }
}

uint8_t etc_is_enabled(void)
{
 static uint8_t etc_enabled = 0xFF;
 if (0xFF==etc_enabled)
 {
  //inputs and outputs for ETC were not allocated?
  etc_enabled = IOCFG_CHECK(IOP_TPS2) && IOCFG_CHECK(IOP_APPS1) && IOCFG_CHECK(IOP_APPS2) &&
                IOCFG_CHECK(IOP_ETC_PWM1) && IOCFG_CHECK(IOP_ETC_PWM2);
 }
 return etc_enabled;
}

uint16_t etc_get_homepos(void)
{
 return etc.home_pos;
}

#endif //!SECU3T && ELEC_THROTTLE
