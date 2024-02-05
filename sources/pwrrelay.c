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
#include "suspendop.h"
#include "wdt.h"

/**Define state variables */
typedef struct
{
 uint8_t state;    //!< state mashine for managing of power states
 uint8_t pwrdown;  //!< power-down flag
 uint8_t opmode;   //!< mode of operation
 uint16_t timer;   //!< powerdown timer
 uint16_t timer1;  //!< powerdown timer1
}pwrstate_t;

pwrstate_t pwrs = {0,0,0,0,0};   //!< instance of state variables

/**Power management timer. Used for power-down timeout*/
s_timer16_t powerdown_timeout_counter = {0,0,1}; //already fired!

void pwrrelay_init_ports(void)
{
 IOCFG_INIT(IOP_PWRRELAY, 1); //power relay is turned on
 IOCFG_INIT(IOP_IGN, 0);      //init IGN input, don't use internal pullup resistor
}

/** Check for engine temperature is ready (Ok).
  * Uses d ECU data structure
  * \return 1 - ready, 0 - not ready (too hot). If CLT sensor is turned off or ECF reassigned to other function,
  * then this function will always return 1 (ready).
  */
static uint8_t clt_is_ready(void)
{
 uint8_t temperature_ok = 1;
 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE) && IOCFG_CHECK(IOP_ECF))
 {
#ifdef COOLINGFAN_PWM
  if (CHECKBIT(d.param.tmp_flags, TMPF_VENT_PWM)) //PWM is available and enabled
   temperature_ok = (d.cool_fan == 0) || (d.sens.temperat <= d.param.vent_off);
  else //PWM is available, but disabled
   temperature_ok = (d.cool_fan == 0);
#else
  //PWM is not available
  temperature_ok = (d.cool_fan == 0);
#endif
 }
 return temperature_ok;
}

void pwrrelay_control(void)
{
 //if this feature is disabled, then do nothing
 if (!IOCFG_CHECK(IOP_PWRRELAY))
  return;

 //apply power management logic
 if (pwrs.pwrdown)
 {//ignition is off

  //set timeout
  if (0==pwrs.state)
  {
   pwrs.state = 1;
   s_timer_set(&powerdown_timeout_counter, 60000); //10 min. max. after ignition turn off
   pwrs.timer = s_timer_gtc();
  }

  //We will wait while temperature is high (only if temperature sensor is enabled
  //and control of electric cooling fan is used), EEPROM is busy, choke/IAC valve or stepper gas valve is busy.

  if ((clt_is_ready() && eeprom_is_idle()
#ifdef SM_CONTROL
      && choke_is_ready()
#endif
#ifdef GD_CONTROL
      && gasdose_is_ready()
#endif
#ifdef FUEL_INJECT
      && !sop_is_operation_active(SOP_SAVE_LTFT)
#endif
#ifdef SPEED_SENSOR
      && !sop_is_operation_active(SOP_SAVE_ODOMET)
#endif
#ifdef FUEL_INJECT
      && !sop_is_operation_active(SOP_SAVE_CONSFUEL)
#endif
#ifdef UNI_OUTPUT
     && (PGM_GET_BYTE(&fw_data.exdata.pwrrelay_uni) == 0x0F || d.uniout[PGM_GET_BYTE(&fw_data.exdata.pwrrelay_uni)])
#endif
      && ((s_timer_gtc() - pwrs.timer) >= PGM_GET_WORD(&fw_data.exdata.pwron_time))
      ) || s_timer_is_action(&powerdown_timeout_counter) || pwrs.opmode)
   IOCFG_SETF(IOP_PWRRELAY, 0); //turn off relay, there is no way back
 }
 else
  pwrs.state = 0;

 //if IGN input is not available, then we will check board voltage
 uint8_t pwrdown = IOCFG_CHECK(IOP_IGN) ? (!IOCFG_GET(IOP_IGN)) : (d.sens.voltage < VOLTAGE_MAGNITUDE(4.5));
 if (!pwrdown)
 { //ignition is ON
  pwrs.timer1 = s_timer_gtc();
  pwrs.pwrdown = 0;
 }
 else
 { //ignition is OFF
  if ((s_timer_gtc() - pwrs.timer1) >= PGM_GET_WORD(&fw_data.exdata.pwron_time1))
  {
#ifdef FUEL_INJECT
   if (0==pwrs.pwrdown)
    sop_set_operation(SOP_SAVE_LTFT);
#endif
#ifdef SPEED_SENSOR
   if (0==pwrs.pwrdown)
    sop_set_operation(SOP_SAVE_ODOMET);
#endif
#ifdef FUEL_INJECT
   if (0==pwrs.pwrdown)
    sop_set_operation(SOP_SAVE_CONSFUEL);
#endif
   pwrs.pwrdown = 1;
  }
 }
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
 if (IOCFG_CHECK(IOP_SM_STP) && !IOCFG_CHECK(IOP_IAC_PWM))
  choke_init_motor();   //send initialization command to choke/IAC motor
#endif
#ifdef GD_CONTROL
 if (IOCFG_CHECK(IOP_GD_STP) && !IOCFG_CHECK(IOP_GD_PWM))
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

void pwrrelay_set_opmode(uint8_t mode)
{
 pwrs.opmode = mode;
}
