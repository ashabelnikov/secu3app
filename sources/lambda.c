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
#include "bitmask.h"
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
 uint8_t stroke_counter[2];      //!< Used to count strokes for correction integration
 uint16_t lambda_t1;             //!< timer
 uint16_t lambda_t2[2];          //!< timer for ms per step
 uint8_t enabled[2];             //!< Flag indicates that lambda correction is enabled by timeout
 uint8_t fc_delay[2];            //!< delay in strokes before lambda correction will be turned on after fuel cut off
 uint8_t gasv_prev[2];           //!< previous value of GAS_V input
 uint8_t ms_mask[2];             //!< correction mask (used for ms per step)
 uint8_t last_sign[2];           //!< 0 - below, 1 - above
 uint8_t swt_counter[2];         //!< counter of level switch
}lambda_state_t;

/**Instance of internal state variables structure*/
static lambda_state_t ego = {{0},0,{0},{0},{0},{0},{0},{0},{0}};

void lambda_control(void)
{
 if (d.engine_mode == EM_START && d.param.inj_lambda_activ_delay)
 {
  ego.lambda_t1 = s_timer_gtc();
  ego.enabled[0] = ego.enabled[1] = 0;
  d.corr.lambda[0] = d.corr.lambda[1] = 0;
 }
 else
 {
  if ((s_timer_gtc() - ego.lambda_t1) >= (d.param.inj_lambda_activ_delay*100))
  {
   if (CHECKBIT(d.param.inj_lambda_flags, LAMFLG_HTGDET))
   { //deternime oxygen sensor's heating by monitoring voltage.
    int16_t top_thrd = d.param.inj_lambda_swt_point + d.param.inj_lambda_dead_band;
    int16_t bot_thrd = ((int16_t)d.param.inj_lambda_swt_point) - d.param.inj_lambda_dead_band;
    if (bot_thrd < 0)
     bot_thrd = 0;

    if (d.sens.lambda[0] < bot_thrd || d.sens.lambda[0] > top_thrd)
     ego.enabled[0] = 1;
    if (d.sens.lambda[1] < bot_thrd || d.sens.lambda[1] > top_thrd)
     ego.enabled[1] = 1;
   }
   else
   {
    ego.enabled[0] = ego.enabled[1] = 1;
   }
  }
 }
}

/** Process one lambda iteration
 * Uses d ECU data structure
 * \param mask Mask updating for "-" (1) or for "+" (2), 0 - no masking
 * \param inp Input selection: 0 - sensor #1, 1 - sensor #2
 * \return 1,2 - if correction has been updated (- or +), otherwise 0
 */
static uint8_t lambda_iteration(uint8_t mask, uint8_t inp)
{
 uint8_t updated = 0;
////////////////////////////////////////////////////////////////////////////////////////
 if (d.param.inj_lambda_senstype==0)
 { //NBO sensor type
  //update EGO correction (with deadband)
  int16_t int_m_thrd = d.param.inj_lambda_swt_point + d.param.inj_lambda_dead_band;
  int16_t int_p_thrd = ((int16_t)d.param.inj_lambda_swt_point) - d.param.inj_lambda_dead_band;
  if (int_p_thrd < 0)
   int_p_thrd = 0;

  if (d.sens.lambda[inp] /*d.sens.inst_add_i1*/ > int_m_thrd)
  {
   if (1!=mask)
   {
    d.corr.lambda[inp]-=d.param.inj_lambda_step_size_m;
    updated = 1;
#ifdef FUEL_INJECT
    //update switch counter
    if (0 == ego.last_sign[inp])
     ++ego.swt_counter[inp];
    ego.last_sign[inp] = 1;
#endif
   }
  }
  else if (d.sens.lambda[inp] /*d.sens.inst_add_i1*/ < int_p_thrd)
  {
   if (2!=mask)
   {
    d.corr.lambda[inp]+=d.param.inj_lambda_step_size_p;
    updated = 2;
#ifdef FUEL_INJECT
    //update switch counter
    if (1 == ego.last_sign[inp])
     ++ego.swt_counter[inp];
    ego.last_sign[inp] = 0;
#endif
   }
  }
 }
 else
 { //WBO sensor type (or emulation)
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
  int16_t int_m_thrd = d.corr.afr - AFRVAL_MAG(0.05);
  int16_t int_p_thrd = d.corr.afr + AFRVAL_MAG(0.05);
#else //CARB_AFR
  int16_t int_m_thrd = AFRVAL_MAG(14.7) - AFRVAL_MAG(0.05);
  int16_t int_p_thrd = AFRVAL_MAG(14.7) + AFRVAL_MAG(0.05);
#endif
  if (int_m_thrd < 0)
   int_m_thrd = 0;

  if (d.sens.afr[inp] < int_m_thrd)
  {
   if (1!=mask)
   {
    d.corr.lambda[inp]-=d.param.inj_lambda_step_size_m;
    updated = 1;
#ifdef FUEL_INJECT
    //update switch counter
    if (1 == ego.last_sign[inp])
     ++ego.swt_counter[inp];
    ego.last_sign[inp] = 0;
#endif
   }
  }
  else if (d.sens.afr[inp] > int_p_thrd)
  {
   if (2!=mask)
   {
    d.corr.lambda[inp]+=d.param.inj_lambda_step_size_p;
    updated = 2;
#ifdef FUEL_INJECT
    //update switch counter
    if (0 == ego.last_sign[inp])
     ++ego.swt_counter[inp];
    ego.last_sign[inp] = 1;
#endif
   }
  }
 }
////////////////////////////////////////////////////////////////////////////////////////

#ifdef GD_CONTROL
 //Use special limits when (gas doser is active) AND ((choke control used AND choke not fully opened) OR (choke control isn't used AND engine is not heated))
 if (d.sens.gas && IOCFG_CHECK(IOP_GD_STP) && ((IOCFG_CHECK(IOP_SM_STP) && (d.choke_pos > 0)) || (!IOCFG_CHECK(IOP_SM_STP) && d.sens.temperat <= d.param.idlreg_turn_on_temp)))
  restrict_value_to(&d.corr.lambda[inp], -d.param.gd_lambda_corr_limit_m, d.param.gd_lambda_corr_limit_p);
 else
  restrict_value_to(&d.corr.lambda[inp], -d.param.inj_lambda_corr_limit_m, d.param.inj_lambda_corr_limit_p);
#else
 restrict_value_to(&d.corr.lambda[inp], -d.param.inj_lambda_corr_limit_m, d.param.inj_lambda_corr_limit_p);
#endif
 return updated;
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
int16_t lambda_get_stoichval(void)
{
 return (d.sens.gas ? d.param.gd_lambda_stoichval : AFRVAL_MAG(14.7));
}
#endif

void lambda_stroke_event_notification(void)
{
 uint8_t chnum = CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN) ? 1 : 2; //skip processing 2nd sensor if mixing is selected
 for(uint8_t i = 0; i < chnum; ++i)
 {
  if (!IOCFG_CHECK(i ? IOP_LAMBDA2 : IOP_LAMBDA))
   continue; //EGO is not enabled (input was not remapped)

  if (!ego.enabled[i])
   continue; //wait some time before oxygen sensor will be turned on

  //do not process EGO correction if it is not needed (gas equipment on the carburetor)
#if !defined(FUEL_INJECT) && !defined(CARB_AFR)
  if (!d.sens.gas || !IOCFG_CHECK(IOP_GD_STP))
  {
   d.corr.lambda[i] = 0;
   continue;
  }
#endif

 //used only with fuel injection or gas doser
#if defined(FUEL_INJECT) || defined(GD_CONTROL)

#if !defined(FUEL_INJECT) && defined(GD_CONTROL)
  if ((d.sens.gas && IOCFG_CHECK(IOP_GD_STP))) {
#endif

   //Turn off EGO correction on overrun or rev. limiting or on idling (if enabled)
   if (!d.ie_valve || d.fc_revlim || d.acceleration || (!d.sens.carb && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_IDLCORR)))
   { //overrun or rev.limiting
    ego.fc_delay[i] = EGO_FC_DELAY;
    d.corr.lambda[i] = 0;
    continue;
   }
   else
   {
    if (ego.fc_delay[i])
    {
      --ego.fc_delay[i];
     d.corr.lambda[i] = 0;
     continue;  //continue count delay
    }
   }

#if !defined(FUEL_INJECT) && defined(GD_CONTROL)
  }
#endif

  //used only by fuel injection and gas doser
  if (d.param.inj_lambda_senstype==0)
  { //NBO sensor type
   int16_t afrerr = abs(d.corr.afr - lambda_get_stoichval());

   if (afrerr > AFRVAL_MAG(0.05)) //EGO allowed only when AFR=14.7 for petrol, and 15.6 for LPG
   {
    d.corr.lambda[i] = 0;
    continue; //not a stoichiometry AFR
   }
  }
  else
  { //WBO sensor type or emulation
   if ((d.corr.afr < ego_curve_min()) || (d.corr.afr > ego_curve_max()))
   {
    d.corr.lambda[i] = 0;
    continue; //out of range
   }
  }

  //Reset EGO correction if work point is out of set zone
  if (!lambda_zone_val())
  {
   d.corr.lambda[i] = 0;
   continue; //not allowed for this (RPM,load)
  }

#endif // FUEL_INJECT || GD_CONTROL

  //Reset EGO correction each time fuel type(set of maps) is changed (triggering of level on the GAS_V input)
  if (ego.gasv_prev[i] != d.sens.gas)
  {
   d.corr.lambda[i] = 0;
   ego.gasv_prev[i] = d.sens.gas;
   continue; //exit from this iteration
  }

  if ((d.sens.inst_frq > d.param.inj_lambda_rpm_thrd) && (d.sens.temperat > d.param.inj_lambda_temp_thrd))    //RPM > threshold && coolant temperature > threshold
  {
   if (d.param.inj_lambda_str_per_stp > 0)
   {//using strokes
    if (ego.stroke_counter[i])
     ego.stroke_counter[i]--;
    else
    {
     ego.stroke_counter[i] = d.param.inj_lambda_str_per_stp;
     lambda_iteration(0, i);
    }
   }
   else
   { //using ms
    uint8_t updated = lambda_iteration(ego.ms_mask[i], i);
    if (updated)
    {
     ego.lambda_t2[i] = s_timer_gtc();
     ego.ms_mask[i] = updated;
    }
    else
    {
     if ((s_timer_gtc() - ego.lambda_t2[i]) >= (d.param.inj_lambda_ms_per_stp))
      ego.ms_mask[i] = 0;
    }
   }
  }
  else
   d.corr.lambda[i] = 0;
 }
}

uint8_t lambda_is_activated(uint8_t inp)
{
 if (inp < 2)
 {
  return ego.enabled[inp];
 }
 else
 {
  if (IOCFG_CHECK(IOP_LAMBDA) && IOCFG_CHECK(IOP_LAMBDA2))
   return ego.enabled[0] && ego.enabled[1];
  else if (IOCFG_CHECK(IOP_LAMBDA2))
   return ego.enabled[1];
  else
   return ego.enabled[0];
 }
}

void lambda_eng_stopped_notification(void)
{
 d.corr.lambda[0] = 0;
 d.corr.lambda[1] = 0;
}

#ifdef FUEL_INJECT
void lambda_reset_swt_counter(uint8_t inp)
{
 ego.swt_counter[inp] = 0;
}

uint8_t lambda_get_swt_counter(uint8_t inp)
{
 return ego.swt_counter[inp];
}
#endif

#if defined(CARB_AFR) || defined(GD_CONTROL)
int16_t lambda_get_mixcor(void)
{
 if (CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
  return d.corr.lambda[0]; //already mixed at sensor level
 if (IOCFG_CHECK(IOP_LAMBDA) && IOCFG_CHECK(IOP_LAMBDA2))
  return (d.corr.lambda[0] + d.corr.lambda[1]) / 2;
 else if (IOCFG_CHECK(IOP_LAMBDA2))
  return d.corr.lambda[1];
 else
  return d.corr.lambda[0];
}
#endif

#endif
