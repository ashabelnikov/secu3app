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
//typedef struct
//{
//}evap_st_t;

/**Instance of state variables */
//evap_st_t evap = {0};

void evap_init_ports(void)
{
 IOCFG_INIT(IOP_ECF, 0); //valve is turned off
}

void evap_init(void)
{
 //EVAP is fully closed
 evap_duty = evap_comp = evap_soft_cnt = 0;
}

void evap_control(void)
{
 SET_EVAP_DUTY(d.param.starter_off - 500);
}

#endif
