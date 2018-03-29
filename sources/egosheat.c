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

/** \file egosheat.c
 * \author Alexey A. Shabelnikov
 * Implementation of EGO sensor's heating control.
 */

#ifdef EGOS_HEATING

#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "funconv.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"
#include "lambda.h"

/**Declare state variables structure*/
typedef struct
{
 uint8_t state;     //!< Needed by finite state machine (current state)
 uint16_t t1;       //!< Used for timing
}eh_state_t;

/**Global instance of state variables */
eh_state_t eh = {0,0};

void egosheat_init_ports(void)
{
 IOCFG_INIT(IOP_O2SH_O, 0);  //<-- heating is off
}

/** Calculates value of pause using a lookup table. This function doesn't interpolate values
 * \return value of pause in 10ms units (1 second = 100)
 */
static uint16_t get_eh_pause(void)
{
 uint8_t i = COIL_ON_TIME_LOOKUP_TABLE_SIZE;    //last element + 1
 int16_t voltage = VOLTAGE_MAGNITUDE(17.8-0.2); //17.6V
 do
 {
  --i;
  if (d.sens.voltage > voltage)
   break;
  voltage-=VOLTAGE_MAGNITUDE(0.4); //step between function samples
 }while(i > 0);

 return PGM_GET_BYTE(&fw_data.exdata.eh_pause[i]);
}

void egosheat_control(void)
{
 if (!IOCFG_CHECK(IOP_O2SH_O))
  return; //this feature is disabled

 switch(eh.state)
 {
  case 0:
   if (!d.st_block)
    break;
   eh.t1 = s_timer_gtc(); //reset timer
   ++eh.state;            //Go into full-on mode

  case 1:
  {
   uint16_t time = s_timer_gtc() - eh.t1;
   int16_t temperat_thrd = ((int16_t)d.param.eh_temper_thrd)*4;
   uint16_t ht_cold = ((uint16_t)d.param.eh_heating_time[0])*100;
   uint16_t ht_hot = ((uint16_t)d.param.eh_heating_time[1])*100;
   if (((d.sens.temperat < temperat_thrd) && (time < ht_cold)) || ((d.sens.temperat >= temperat_thrd) && (time < ht_hot))
      && !(d.param.inj_lambda_htgdet && lambda_is_activated()))
   {
    IOCFG_SETF(IOP_O2SH_O, 1); //turned on
    break;
   }
   else
   {
    ++eh.state;            //Go into PWM mode
    eh.t1 = s_timer_gtc(); //reset timer
   }
  }
  case 2:
   {
    uint16_t pause = get_eh_pause();
    if (pause)
     IOCFG_SETF(IOP_O2SH_O, 0); //turn off

    if ((s_timer_gtc() - eh.t1) < pause)      //paused heating time
     break;

    ++eh.state;            //Go into active heating mode
    eh.t1 = s_timer_gtc(); //reset timer
   }
  case 3:
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   //always turn off heater when airflow exceeds specified threshold
   if (calc_airflow() > d.param.eh_aflow_thrd) // value / 32
   {
    IOCFG_SETF(IOP_O2SH_O, 0); //turn off
    eh.t1 = s_timer_gtc(); //reset timer
    break;
   }
   else
#endif
    IOCFG_SETF(IOP_O2SH_O, 1); //turn on

   if ((s_timer_gtc() - eh.t1) < d.param.eh_heating_act)   //active heating time
    break;

   --eh.state;            //Go into pause heating mode
   eh.t1 = s_timer_gtc(); //reset timer
 }

 if (!d.st_block)
 {
  IOCFG_SETF(IOP_O2SH_O, 0); //turned off
  eh.state = 0;
 }
}

#endif //EGOS_HEATING
