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

/** \file pwrrelay.c
 * \author Alexey A. Shabelnikov
 * Power management using external relay, allows SECU-3 to be turned on some time
 * after ignition is off. So, for instance electric colling fan can work even ignition is off
 */

#include "port/port.h"
#include "bitmask.h"
#include "choke.h"
#include "ecudata.h"
#include "eeprom.h"
#include "gasdose.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"
#include "smcontrol.h"
#include "gdcontrol.h"
#include "choke.h"
#include "gasdose.h"
#include "wdt.h"

/**Define state variables */
typedef struct
{
 uint8_t state;    //!< state mashine for managing of power states
 uint8_t pwrdown;  //!< power-down flag
}pwrstate_t;

pwrstate_t pwrs = {0,0};   //!< instance of state variables

void pwrrelay_init_ports(void)
{
 IOCFG_INIT(IOP_PWRRELAY, 1); //power relay is turned on (реле включено)
#ifdef SECU3T  //see also initialization in measure.c
 IOCFG_INIT(IOP_IGN, 1);      //init IGN input
#endif
}

void pwrrelay_control(void)
{
 //if this feature is disabled, then do nothing
 if (!IOCFG_CHECK(IOP_PWRRELAY))
  return;

 //apply power management logic
 if (pwrs.pwrdown)
 {//ignition is off

  //We will wait while temperature is high only if temperature sensor is enabled
  //and control of electric cooling fan is used.
  uint8_t temperature_ok = 1;
  if (d.param.tmp_use && IOCFG_CHECK(IOP_ECF))
  {
#ifdef COOLINGFAN_PWM
   if (d.param.vent_pwm) //PWM is available and enabled
    temperature_ok = (d.sens.temperat <= (d.param.vent_on - TEMPERATURE_MAGNITUDE(2.0)));
   else //PWM is available, but disabled
    temperature_ok = (d.sens.temperat <= (d.param.vent_off));
#else
   //PWM is not available
   temperature_ok = (d.sens.temperat <= (d.param.vent_off));
#endif

   //set timeout
   if (0==pwrs.state)
   {
    pwrs.state = 1;
    s_timer16_set(powerdown_timeout_counter, 6000); //60 sec.
   }
  }

  if ((temperature_ok && eeprom_is_idle()
#ifdef SM_CONTROL
      && choke_is_ready()
#endif
#ifdef GD_CONTROL
      && gasdose_is_ready()
#endif
      ) || s_timer16_is_action(powerdown_timeout_counter))
   IOCFG_SETF(IOP_PWRRELAY, 0); //turn off relay
 }
 else
  pwrs.state = 0;

 //if IGN input is not available, then we will check board voltage
 pwrs.pwrdown = IOCFG_CHECK(IOP_IGN) ? (!IOCFG_GET(IOP_IGN)) : (d.sens.voltage < VOLTAGE_MAGNITUDE(4.5));
}

uint8_t pwrrelay_get_state(void)
{
 return (pwrs.pwrdown == 0);
}

void pwrrelay_init_steppers(void)
{
 int cnt = 6000;

 if (!IOCFG_CHECK(IOP_PWRRELAY))
  return; //do nothing if power management is not used

#ifdef SM_CONTROL
 if (IOCFG_CHECK(IOP_SM_STP))
  choke_init_motor();   //send initialization command to choke/IAC motor
#endif
#ifdef GD_CONTROL
 if (IOCFG_CHECK(IOP_GD_STP))
  gasdose_init_motor(); //send initialization command to stepper gas valve motor
#endif

 //wait while at least one of the motors is busy, but no more than several seconds
 while(0
#ifdef SM_CONTROL
 || stpmot_is_busy()
#endif
#ifdef GD_CONTROL
 || gdstpmot_is_busy()
#endif
 )
 {
  wdt_reset_timer();
  _DELAY_US(1000);
  if (--cnt <= 0)
   break;
 }
}
