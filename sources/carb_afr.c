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

/** \file carb_afr.c
 * \author Alexey A. Shabelnikov
 * Implementation of AFR control on carburetor using electronic actuators driven by PWM
 */

#ifdef CARB_AFR

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "carb_afr.h"
#include "ioconfig.h"
#include "funconv.h"
#include "lambda.h"
#include "magnitude.h"
#include "mathemat.h"
//#include "dbgvar.h"

#ifdef FUEL_INJECT
 #error "You can't use carburetor AFR control together with fuel injection, please omit FUEL_INJECT option"
#endif

#define CAFR_IDL_RPM_THRD 1100 //!< RPM threshold before idling (min-1)
#define CAFR_HLD_RPM_THRD 4000 //!< High load RPM threshold (min-1)
#define CAFR_PWM_STEPS 64      //!< software PWM steps (0...63)

//see code in vstimer.c for more information about these variables
uint8_t cafr_iv_comp;          //!< Compare register for IV channel
volatile uint8_t cafr_iv_duty; //!< IV PWM duty
uint8_t cafr_pv_comp;          //!< Compare register for Pv channel
volatile uint8_t cafr_pv_duty; //!< PV PWM duty
uint8_t cafr_soft_cnt;         //!< Counter

/** Set IV(idle cut off) valve duty */
#define SET_IV_DUTY(v) { \
 cafr_iv_duty = (v); \
 /*dbg_var1 = (v);*/ \
 /*todo: update ie_valve*/ \
 }

/** Set PV(power) valve duty */
#define SET_PV_DUTY(v) { \
 cafr_pv_duty = (v); \
 /*dbg_var2 = (v);*/ \
 /*todo: update fe_valve*/ \
 }

/**Define state variables */
typedef struct
{
 uint16_t state;                //!< control state used for hysteresis
}cafr_st_t;

/**Instance of state variables */
cafr_st_t cas = {0};

void carbafr_init_ports(void)
{
 IOCFG_INIT(IOP_IE, 1); //valve is turned on
 IOCFG_INIT(IOP_FE, 1); //valve is turned on
}

void carbafr_init(void)
{
 //both valves are fully open
 cafr_iv_duty = CAFR_PWM_STEPS-1; //100%
 cafr_pv_duty = CAFR_PWM_STEPS-1;
 cafr_iv_comp = CAFR_PWM_STEPS-1;
 cafr_pv_comp = CAFR_PWM_STEPS-1;
 cafr_soft_cnt = CAFR_PWM_STEPS-1;

 //todo: update ie_valve
 //todo: update fe_valve

 cas.state = 0;
}

/** Get discharge in kPa units relatively to atmospheric pressure
 * Uses d ECU data structure
 * \return discharge value in kPa
 */
static int16_t get_discharge(void)
{
 int16_t discharge = (d.sens.baro_press - d.sens.map);
 return (discharge < 0) ? 0 : discharge;
}

/** Control two actuators depending on current mode
 * Uses d ECU data structure
 * \param mode Mode of operation: 1 - work, 0 - idling
 */
static void control_iv_and_pv(uint8_t mode)
{
 int16_t lambda = lambda_get_mixcor();
 if (mode)
 { //work, control FE, but also control IE if FE=0 and mixture is still rich
   if (lambda > -256)
   { //control AFR using power valve only, normal mode
    int16_t pv_duty = CAFR_PWM_STEPS/2; //basic duty is 50%
    pv_duty = (((uint32_t)pv_duty) * (512 + lambda)) >> 9;
    restrict_value_to(&pv_duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
    SET_PV_DUTY(pv_duty); //control power valve
    SET_IV_DUTY(CAFR_PWM_STEPS-1); //idle cut-off valve is 100% opened
   }
   else
   { //mixture is too rich and power valve is not enough to make it lean, then use idle cut-off valve to additionally lean it
    int16_t iv_duty = CAFR_PWM_STEPS/2; //basic duty is 50%
    iv_duty = (((uint32_t)iv_duty) * (512 + lambda)) >> 8; //div. by 256
    iv_duty+= (CAFR_PWM_STEPS/4);
    restrict_value_to(&iv_duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
    SET_IV_DUTY(iv_duty); //control
    SET_PV_DUTY(CAFR_PWM_STEPS/4); //25%
   }
 }
 else
 { //idling, control IE only, FE = 50%
   int16_t duty = CAFR_PWM_STEPS/2; //basic duty is 50%
   duty = (((uint32_t)duty) * (512 + lambda)) >> 9;
   restrict_value_to(&duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
   SET_IV_DUTY(duty); //control
   SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
 }
}

void carbafr_control(void)
{
#ifdef GD_CONTROL
 //Don't use AFR control when gas doser is activated and fuel type is gas
 if (d.sens.gas && IOCFG_CHECK(IOP_GD_STP))
 {
  IOCFG_INIT(IOP_IE, 0); //valve is turned off
  IOCFG_INIT(IOP_FE, 0); //valve is turned off
  return; //no AFR control
 }
#endif

 if (d.sens.frequen < d.param.inj_lambda_rpm_thrd) //100 min-1
 {
   //both valves are fully opened
   SET_IV_DUTY(CAFR_PWM_STEPS-1); //100%
   SET_PV_DUTY(CAFR_PWM_STEPS-1); //100%
 }
 else
 { //RPM > 100 min-1
  //Use closed loop control when lambda sensor is heated-up and (CTS > 40) and (RPM < 4000) and (discharge > threshold),
  //otherwise use 50% value for both valves
  //
  //(discharge > threshold) means engine is not under full load
  if (lambda_is_activated(2) && (d.sens.temperat > d.param.inj_lambda_temp_thrd) && (d.sens.inst_frq < CAFR_HLD_RPM_THRD) && (get_discharge() > d.param.fe_on_threshold))
  {
   if (d.sens.carb)
   { //throttle is opened
    if (d.sens.inst_frq > CAFR_IDL_RPM_THRD)
    {
     //control FE,--> IE=100%
     control_iv_and_pv(1); //IE can be used to additionally lean mixture
    }
    else
    {
     //control IE, FE = 50%
     control_iv_and_pv(0);
    }
   }
   else
   { //throttle is closed
     switch(cas.state)
     {
      case 0:
       if (d.sens.inst_frq > d.param.ie_hit)
        cas.state = 1;
       else
       {
        //control IE, FE = 50%
        control_iv_and_pv(0);
       }
       break;
      case 1:
       if (d.sens.inst_frq < d.param.ie_lot)
        cas.state = 0;
       else
       {
        d.corr.lambda[0] = d.corr.lambda[1] = 0; //reset lambda correction, because we are in the open-loop mode now
        SET_IV_DUTY(0); //0%
        SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
       }
       break;
     }
   }
  }
  else
  {
   d.corr.lambda[0] = d.corr.lambda[1] = 0; //reset lambda correction, because we are in the open-loop mode now
   SET_IV_DUTY(CAFR_PWM_STEPS/2); //50%
   SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
  }
 }
}

#endif
