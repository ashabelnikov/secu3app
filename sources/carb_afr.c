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

/** \file carb_afr.h
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

#ifdef FUEL_INJECT
 #error "You can't use carburetor AFR control together with fuel injection, please omit FUEL_INJECT option"
#endif

#define CAFR_IDL_RPM_THRD 1100 //!< RPM threshold before idling
#define CAFR_PWM_STEPS 64      //!< software PWM steps

//see code in vstimer.c for more information about these variables
uint8_t cafr_iv_comp;
volatile uint8_t cafr_iv_duty; //IV PWM duty
uint8_t cafr_pv_comp;
volatile uint8_t cafr_pv_duty; //PV PWM duty
uint8_t cafr_soft_cnt;

/***/
#define SET_IV_DUTY(v) cafr_iv_duty = (v)
/***/
#define SET_PV_DUTY(v) cafr_pv_duty = (v)

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

 cas.state = 0;
}

/** Get discharge in kPa units
 * \param d Pointer to ECU data structure
 * \return discharge value in kPa
 */
static int16_t get_discharge(struct ecudata_t* d)
{
 int16_t discharge = (d->param.map_upper_pressure - d->sens.map);
 return (discharge < 0) ? 0 : discharge;
}

/** Control two actuators depending on current mode
 * \param d Pointer to ECU data structure
 * \param mode Mode of operation: 1 - work, 0 - idling
 */
static void control_iv_and_pv(struct ecudata_t* d, uint8_t mode)
{
 if (mode)
 { //work, control FE, but also control IE if FE=0 and mixture is still rich
   if (d->corr.lambda > -256)
   { //control AFR using power valve only, normal mode
    int16_t pv_duty = CAFR_PWM_STEPS/2; //basic duty is 50%
    pv_duty = (((uint32_t)pv_duty) * (512 + d->corr.lambda)) >> 9;
    restrict_value_to(&pv_duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
    SET_PV_DUTY(pv_duty); //control power valve
    SET_IV_DUTY(CAFR_PWM_STEPS-1); //idle cut-off valve is 100% opened
   }
   else
   { //mixture is too rich and power valve is not enough to make it lean, then use idle cut-off valve to additionally lean it
    int16_t iv_duty = CAFR_PWM_STEPS/2; //basic duty is 50%
    duty = (((uint32_t)iv_duty) * (512 + d->corr.lambda)) >> 9;
    restrict_value_to(&iv_duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
    SET_IV_DUTY(iv_duty); //control
    SET_PV_DUTY(CAFR_PWM_STEPS/4); //25%
   }
 }
 else
 { //idling, control IE only, FE = 50%
   int16_t duty = CAFR_PWM_STEPS/2; //basic duty is 50%
   duty = (((uint32_t)duty) * (512 + d->corr.lambda)) >> 9;
   restrict_value_to(&duty, CAFR_PWM_STEPS/4, (CAFR_PWM_STEPS*3)/4);  //Do we need this?
   SET_IV_DUTY(duty); //control
   SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
 }
}

void carbafr_control(struct ecudata_t* d)
{
 if (d->sens.frequen < 100)
 {
   //both valves are fully opened
   SET_IV_DUTY(CAFR_PWM_STEPS-1); //100%
   SET_PV_DUTY(CAFR_PWM_STEPS-1); //100%
   //todo: update ie_valve
   //todo: update fe_valve
 }
 else
 { //RPM > 100 min-1

  //Use closed loop control when lambda sensor is heated-up and (CTS > 40) and (RPM < 4000) and (discharge > threshold),
  //otherwise use 50% value for both valves
  //
  //(discharge > threshold) means engine is not under full load
  if (lambda_is_activated() && (d->sens.temperat > TEMPERATURE_MAGNITUDE(40.0)) && (d->sens.inst_frq < 4000) && (get_discharge(d) > d->param.fe_on_threshold))
  {
   if (d->sens.carb)
   { //throttle is opened
    if (d->sens.inst_frq > CAFR_IDL_RPM_THRD)
    {
     //control FE,--> IE=100%
     control_iv_and_pv(d, 1); //IE can be used to additionally lean mixture
    }
    else
    {
     //control IE, FE = 50%
     control_iv_and_pv(d, 0);
    }
   }
   else
   { //throttle is closed
     switch(cas.state)
     {
      case 0:
       if (d->sens.inst_frq > d->param.ie_hit)  //todo: use idle cut off threshold
        cas.state = 1;
       else
       {
        //control IE, FE = 50%
        control_iv_and_pv(d, 0);
       }
       break;
      case 1:
       if (d->sens.inst_frq < d->param.ie_lot)  //todo: use idle cut off threshold
        cas.state = 0;
       else
       {
        SET_IV_DUTY(0); //0%
        SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
        //todo: update ie_valve
        //todo: update fe_valve
       }
       break;
     }
   }
  }
  else
  {
   SET_IV_DUTY(CAFR_PWM_STEPS/2); //50%
   SET_PV_DUTY(CAFR_PWM_STEPS/2); //50%
   //todo: update ie_valve
   //todo: update fe_valve
  }
 }
}

#endif
