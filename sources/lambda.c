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

/** \file lambda.c
 * \author Alexey A. Shabelnikov
 * Implementation of correction algorithms using an exhaust gas oxygen sensor
 */

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)

#include "port/port.h"
#include <stdlib.h>
#include "ecudata.h"
#include "eculogic.h"
#include "funconv.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "mathemat.h"
#include "vstimer.h"

#define EGO_FC_DELAY 6           //!< 6 strokes

/**Internal state variables*/
typedef struct
{
 uint8_t stroke_counter;         //!< Used to count strokes for correction integration
 uint16_t lambda_t1;             //!< timer
 uint16_t lambda_t2;             //!< timer for ms per step
 uint8_t enabled;                //!< Flag indicates that lambda correction is enabled by timeout
 uint8_t fc_delay;               //!< delay in strokes before lambda correction will be turned on after fuel cut off
 uint8_t gasv_prev;              //!< previous value of GAS_V input
}lambda_state_t;

/**Instance of internal state variables structure*/
static lambda_state_t ego;

void lambda_init_state(void)
{
 ego.stroke_counter = 0;
 ego.enabled = 0;
 ego.fc_delay = 0;
 ego.gasv_prev = 0;
}

void lambda_control(struct ecudata_t* d)
{
 if (d->engine_mode == EM_START)
 {
  ego.lambda_t1 = s_timer_gtc();
  ego.enabled = 0;
  d->corr.lambda = 0;
 }
 else
 {
  if ((s_timer_gtc() - ego.lambda_t1) >= (d->param.inj_lambda_activ_delay*100))
   ego.enabled = 1;
 }
}

/** Process one lambda iteration
 * \param d Pointer to ECU data structure
 */
static void lambda_iteration(struct ecudata_t* d)
{
////////////////////////////////////////////////////////////////////////////////////////
    if (d->param.inj_lambda_senstype==0)
    { //NBO sensor type
     //update EGO correction (with deadband)
     int16_t int_m_thrd = d->param.inj_lambda_swt_point + d->param.inj_lambda_dead_band;
     int16_t int_p_thrd = ((int16_t)d->param.inj_lambda_swt_point) - d->param.inj_lambda_dead_band;
     if (int_p_thrd < 0)
      int_p_thrd = 0;

     if (d->sens.add_i1 > int_m_thrd)
      d->corr.lambda-=d->param.inj_lambda_step_size_m;
     else if (d->sens.add_i1 < int_p_thrd)
      d->corr.lambda+=d->param.inj_lambda_step_size_p;
    }
    else
    { //WBO sensor type (or emulation)
     uint16_t sens_afr = ego_curve_lookup(d);

     int16_t int_m_thrd = d->corr.afr - AFRVAL_MAG(0.05);
     int16_t int_p_thrd = d->corr.afr + AFRVAL_MAG(0.05);
     if (int_m_thrd < 0)
      int_m_thrd = 0;

     if (sens_afr < int_m_thrd)
      d->corr.lambda-=d->param.inj_lambda_step_size_m;
     else if (sens_afr > int_p_thrd)
      d->corr.lambda+=d->param.inj_lambda_step_size_p;
    }
////////////////////////////////////////////////////////////////////////////////////////


#ifdef GD_CONTROL
    //Use special limits when (gas doser is active) AND ((choke control used AND choke not fully opened) OR (choke control isn't used AND engine is not heated))
    if (d->sens.gas && IOCFG_CHECK(IOP_GD_STP) && ((IOCFG_CHECK(IOP_SM_STP) && (d->choke_pos > 0)) || (!IOCFG_CHECK(IOP_SM_STP) && d->sens.temperat <= d->param.idlreg_turn_on_temp)))
     restrict_value_to(&d->corr.lambda, -d->param.gd_lambda_corr_limit_m, d->param.gd_lambda_corr_limit_p);
    else
     restrict_value_to(&d->corr.lambda, -d->param.inj_lambda_corr_limit_m, d->param.inj_lambda_corr_limit_p);
#else
    restrict_value_to(&d->corr.lambda, -d->param.inj_lambda_corr_limit_m, d->param.inj_lambda_corr_limit_p);
#endif
}


void lambda_stroke_event_notification(struct ecudata_t* d)
{
 if (!IOCFG_CHECK(IOP_LAMBDA))
  return; //EGO is not enabled (input was not remapped)

 if (!ego.enabled)
  return; //wait some time before oxygen sensor will be turned on

//do not process EGO correction if it is not needed (gas equipment on the carburetor)
#if !defined(FUEL_INJECT) && !defined(CARB_AFR)
 if (!d->sens.gas || !IOCFG_CHECK(IOP_GD_STP))
 {
  d->corr.lambda = 0;
  return;
 }
#endif

 //used only with fuel injection or gas doser
#if defined(FUEL_INJECT) || defined(GD_CONTROL)

#if !defined(FUEL_INJECT) && defined(GD_CONTROL)
 if ((d->sens.gas && IOCFG_CHECK(IOP_GD_STP))) {
#endif

 //Turn off EGO correction on overrun or rev. limiting
#ifdef GD_CONTROL
 //turn off EGO correction also on idling if gas doser is active
 if (!d->ie_valve || d->fc_revlim || d->acceleration ||  (!d->sens.carb && (d->sens.gas && IOCFG_CHECK(IOP_GD_STP))))
#else
 if (!d->ie_valve || d->fc_revlim || d->acceleration)
#endif
 { //overrun or rev.limiting
  ego.fc_delay = EGO_FC_DELAY;
  d->corr.lambda = 0;
  return;
 }
 else
 {
  if (ego.fc_delay)
  {
   --ego.fc_delay;
   d->corr.lambda = 0;
   return;  //continue count delay
  }
 }

#if !defined(FUEL_INJECT) && defined(GD_CONTROL)
 }
#endif

 //used only by fuel injection and gas doser
 if (d->param.inj_lambda_senstype==0)
 { //NBO sensor type
  int16_t afrerr = abs(d->corr.afr -
#ifdef GD_CONTROL
  ((d->sens.gas && IOCFG_CHECK(IOP_GD_STP)) ? d->param.gd_lambda_stoichval : AFRVAL_MAG(14.7))
#else
  AFRVAL_MAG(14.7) //14.7
#endif
  );

  if (afrerr > AFRVAL_MAG(0.05)) //EGO allowed only when AFR=14.7 for petrol, and 15.6 for LPG
  {
   d->corr.lambda = 0;
   return; //not a stoichiometry AFR
  }
 }
 else
 { //WBO sensor type or emulation
  if ((d->corr.afr < ego_curve_min(d)) || (d->corr.afr > ego_curve_max(d)))
  {
   d->corr.lambda = 0;
   return; //not a stoichiometry AFR
  }
 }

#endif // FUEL_INJECT || GD_CONTROL

 //Reset EGO correction each time fuel type(set of maps) is changed (triggering of level on the GAS_V input)
 if (ego.gasv_prev != d->sens.gas)
 {
  d->corr.lambda = 0;
  ego.gasv_prev = d->sens.gas;
  return; //exit from this iteration
 }

 if (d->sens.inst_frq > d->param.inj_lambda_rpm_thrd)    //RPM > threshold
 {
  if (d->sens.temperat > d->param.inj_lambda_temp_thrd)  //coolant temperature > threshold
  {

   if (d->param.inj_lambda_str_per_stp > 0)
   {//using strokes
    if (ego.stroke_counter)
     ego.stroke_counter--;
    else
    {
     ego.stroke_counter = d->param.inj_lambda_str_per_stp;
     lambda_iteration(d);
    }
   }
   else
   { //using ms
    if ((s_timer_gtc() - ego.lambda_t2) >= (d->param.inj_lambda_ms_per_stp))
    {
     ego.lambda_t2 = s_timer_gtc();
     lambda_iteration(d);
    }
   }
  }
  else
   d->corr.lambda = 0;
 }
 else
  d->corr.lambda = 0;
}

uint8_t lambda_is_activated(void)
{
 return ego.enabled;
}

#endif
