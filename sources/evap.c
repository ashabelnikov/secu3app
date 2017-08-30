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

/** \file evap.c
 * \author Alexey A. Shabelnikov
 * Implementation of control of canister purge valve
 */

#ifdef EVAP_CONTROL

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "evap.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "lambda.h"
#include "funconv.h"

#ifdef SECU3T
 #error "Canister purge valve is not supported in the SECU-3T, use SECU-3i (undefine SECU3T)"
#endif

#ifndef FUEL_INJECT
 #error "Canister purge valve is not supported without fuel injection, define FUEL_INJECT"
#endif

#define EVAP_PWM_STEPS 32      //!< software PWM steps (0...31)

//see code in vstimer.c for more information about these variables
uint8_t evap_comp;             //!< Compare register for PWM
volatile uint8_t evap_duty;    //!< PWM duty
uint8_t evap_soft_cnt;         //!< Counter for PWM

/** Set canister purge valve's PWM duty */
#define SET_EVAP_DUTY(v) { \
 evap_duty = (v); \
 }

/**Define state variables */
typedef struct
{
 uint8_t state;               //!< state for VSS condition
}evap_st_t;

/**Instance of state variables */
evap_st_t evap = {0};

void evap_init_ports(void)
{
 IOCFG_INIT(IOP_ECF, 0); //valve is turned off
}

void evap_init(void)
{
 //EVAP is fully closed
 evap_duty = evap_comp = evap_soft_cnt = 0;
 evap.state = 0;
}

void evap_control(void)
{
 if (!IOCFG_CHECK(IOP_EVAP_O))
  return; //EVAP_O remapped to other function

#ifdef SPEED_SENSOR
  if (IOCFG_CHECK(IOP_SPDSENS))
  {
   //TODO: calculate VSS in km/h and use here instead of time constants (bad)
   //thresholds must be 10 and 7 km/h
   if (d.sens.speed < 65450)
    evap.state = 1;
   if (d.sens.speed > 65530)
    evap.state = 0;
  }
  else
   evap.state = 1; //do not use this condition if VSS is not active
#endif

 //Position of valve (PWM duty) depends on the inlet air flow, but also following conditions must be met, otherwise valve will be closed:
 //engine is warm, no idling, no full load, no fuel cut, not rev.limitting, oxygen sensor is warm, speed of vehicle > 10km/h
 if ((d.sens.temperat > TEMPERATURE_MAGNITUDE(75.0)) && (d.sens.tps > TPS_MAGNITUDE(4.0)) && (d.sens.tps < 98.0) && d.ie_valve && !d.fc_revlim && lambda_is_activated()
#ifdef SPEED_SENSOR
   && evap.state
#endif
    )
 {
  uint16_t af = calc_airflow();
  //calculate current duty, based on the current air flow and limits specified by user
  if (af < d.param.evap_afbegin)
   af = d.param.evap_afbegin;
  uint16_t duty = (((uint32_t)(af - d.param.evap_afbegin)) * d.param.evap_afslope) >> 16;
  if (duty > EVAP_PWM_STEPS)
   duty = EVAP_PWM_STEPS;

  SET_EVAP_DUTY(duty); //set evap duty!
 }
 else
  SET_EVAP_DUTY(0); //valve is turned off
}

#endif
