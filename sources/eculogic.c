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

/** \file eculogic.c
 * \author Alexey A. Shabelnikov
 * Implementation of logic for calculation and regulation of ignition timing and fuel injection
 */

#include "port/port.h"
#include <stdlib.h>
#include "bitmask.h"
#include "ckps.h"
#include "ecudata.h"
#include "eculogic.h"
#include "funconv.h"
#include "injector.h"
#include "magnitude.h"
#include "vstimer.h"
#include "ioconfig.h"
#include "lambda.h"
#include "mathemat.h"

/**Reserved value used to indicate that value is not used in corresponding mode*/
#define AAV_NOTUSED 0x7FFF

typedef struct
{
#ifdef FUEL_INJECT
 uint16_t aftstr_enrich_counter; //!< Stroke counter used in afterstart enrichment implementation
 uint16_t prime_delay_tmr;       //!< Timer variable used for prime pulse delay
 uint8_t  prime_ready;           //!< Indicates that prime pulse was fired or skipped if cranking was started before
 uint8_t  cog_changed;           //!< Flag which indicates there was crankshaft revolution after last power on
 uint8_t  ae_decay_counter;      //!< AE decay counter
 uint16_t aef_decay;             //!< AE factor value at the start of decay
 uint8_t  aef_started;           //!< flag, indicates that decay will be started
#endif
 int16_t  calc_adv_ang;          //!< calculated advance angle
 int16_t  advance_angle_inhibitor_state; //!<
}logic_state_t;

/**Instance of internal state variables structure*/
static logic_state_t lgs = {
#ifdef FUEL_INJECT
 0,0,0,0,0,0,0,
#endif
 0,0
};

void ignlogic_init(void)
{
#ifdef FUEL_INJECT
 lgs.prime_delay_tmr = s_timer_gtc();
#endif
 d.sens.baro_press = PRESSURE_MAGNITUDE(101.3); //set default value to prevent wrong conditions when barometric pressure will not be sampled for some reasons
}

#ifdef FUEL_INJECT

#ifndef SECU3T
/** Applies gas temperature and pressure corrections (coefficients) to the inj. PW
 * \param pw PW to be corrected
 * \return Corrected PW
 */
uint32_t pw_gascorr(uint32_t pw)
{
 pw = (pw * inj_gts_pwcorr()) >> 7;              //apply gas temperature correction
 pw = (pw * inj_gps_pwcorr()) >> 7;              //apply gas pressure correction
 return pw;
}
#endif

/**Limit value of injection PW
 * \return limited value (16 bit)
 */
static uint16_t lim_inj_pw(uint32_t *value)
{
 return (*value) > 65535 ? 65535 : (*value);
}

/**"Normal conditions" constant for calculating of NC pulse width, value of this constant = 1397*/
#define PWNC_CONST ROUND((100.0*MAP_PHYSICAL_MAGNITUDE_MULTIPLIER*256) / (293.15*TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER))

/** Calculates AE value.
 * Uses d ECU data structure
 * \return AE value in PW units
 */
static int32_t calc_acc_enrich(void)
{
 //calculate normal conditions PW, MAP=100kPa, IAT=20°C, AFR=14.7 (petrol) or d.param.gd_lambda_stoichval (gas).
 //For AFR=14.7 and inj_sd_igl_const=86207 we should get result near to 2000.48
 int32_t pwnc = (((((uint32_t)PWNC_CONST) * nr_1x_afr(lambda_get_stoichval() << 3)) >> 12) * d.param.inj_sd_igl_const[d.sens.gas]) >> 15;
 int16_t aef = inj_ae_tps_lookup();               //calculate basic AE factor value

 if (abs(d.sens.tpsdot) < d.param.inj_ae_tpsdot_thrd)
 {
  if (lgs.aef_started)
  {
   lgs.ae_decay_counter = d.param.inj_ae_decay_time; //init counter
   lgs.aef_decay = aef;
   lgs.aef_started = 0;
  }
  //stop decay if gas pedal fully released
  if (!d.sens.carb)
   lgs.ae_decay_counter = 0;
  d.acceleration = (lgs.ae_decay_counter > 0);
  //apply decay factor
  aef = (((int32_t)lgs.aef_decay) * lgs.ae_decay_counter) / d.param.inj_ae_decay_time; //TODO: replace division by multiplication with 1 / inj_ae_decay_time constant
 }
 else
 {
  lgs.aef_started = 1;
  d.acceleration = 1;
 }

 aef = ((int32_t)aef * inj_ae_clt_corr()) >> 7;   //apply CLT correction factor to AE factor
 aef = ((int32_t)aef * inj_ae_rpm_lookup()) >> 7; //apply RPM correction factor to AE factor
 return (pwnc * aef) >> 7;                        //apply AE factor to the normal conditions PW
}
#endif

#ifdef PA4_INP_IGNTIM
/** Calculates manual ignition timing correction value depending on the selected input (PA4 for SECU-3T or ADD_I3|ADD_I4 for SECU-3i)
 * \return value * ANGLE_MULTIPLIER
 */
int16_t manual_igntim(void)
{
#ifdef SECU3T
 return pa4_function(d.sens.add_i3); //PA4 only, can't be remapped
#else //SECU-3i
 if (IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i3 || IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i3i)
  return pa4_function(d.sens.add_i3);
#ifdef TPIC8101
 else if (IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i4 || IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i4i)
  return pa4_function(d.sens.add_i4);
#endif
 else
  return 0; //not mapped to real I/O
#endif
}
#endif


/** Perform fuel calculations used on idling and work
 */
#ifdef FUEL_INJECT
static void fuel_calc(void)
{
 uint32_t pw = inj_base_pw();

 if (CHECKBIT(d.param.inj_flags, INJFLG_USEAIRDEN))
  pw = (pw * inj_airtemp_corr(0)) >> 7;           //apply air density correction (if enabled)

 pw = (pw * inj_warmup_en()) >> 7;              //apply warmup enrichemnt factor
 if (lgs.aftstr_enrich_counter)
  pw= (pw * (128 + scale_aftstr_enrich(lgs.aftstr_enrich_counter))) >> 7; //apply scaled afterstart enrichment factor
 pw= (pw * (512 + d.corr.lambda)) >> 9;         //apply lambda correction additive factor (signed)
 pw= (pw * inj_iacmixtcorr_lookup()) >> 13;     //apply mixture correction vs IAC
 pw+= calc_acc_enrich();                        //add acceleration enrichment
 if (((int32_t)pw) < 0)
  pw = 0;
 if (d.param.barocorr_type)
  pw = (pw * barocorr_lookup()) >> 12;           //apply barometric correction

#ifndef SECU3T
 if (CHECKBIT(d.param.inj_flags, INJFLG_USEADDCORRS))
  pw = pw_gascorr(pw);                              //apply gas corrections
#endif

 d.inj_pw_raw = lim_inj_pw(&pw);
 d.inj_dt = accumulation_time(1);                //apply dead time
 pw+= d.inj_dt;
 if (d.ie_valve && !d.fc_revlim && d.eng_running)
  d.inj_pw = lim_inj_pw(&pw);
 else d.inj_pw = 0;
}
#endif

/** Measures barometric pressure and stores result into d.baro_press variable.
 *  If barocorrection is turned off, then baro_press variable receives value = 101.3kPa
 */
void sample_baro_pressure(void)
{
 if (d.param.barocorr_type == 0) //disabled
  d.sens.baro_press = PRESSURE_MAGNITUDE(101.3); //default, use atmospheric pressure measured from sea level
 else if (d.param.barocorr_type < 3)  //static or dynamic corr. using primary MAP
  d.sens.baro_press = d.sens.map;
#ifndef SECU3T
 else //additional MAP, dynamic corr. (SECU-3i only)
  d.sens.baro_press = d.sens.map2;
#endif

 restrict_value_to((int16_t*)&d.sens.baro_press, PRESSURE_MAGNITUDE(70.0), PRESSURE_MAGNITUDE(120.0));
}

/**
*/
uint8_t get_use_injtim_map_flag(void)
{
 return (d.sens.gas ? CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP_G) : CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP));
}

void ignlogic_system_state_machine(void)
{
 int16_t angle = 0;

 //Sample atmospheric pressure each loop if dynamic barocorrection selected (either from MAP1 or MAP2)
 if (d.param.barocorr_type > 1)
  sample_baro_pressure();

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 calc_ve_afr();
#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)
 engine_blowing_cond(); //check for entering flood clear mode
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 if (IOCFG_CHECK(IOP_LAMBDA))
 {
  if (d.param.inj_lambda_senstype==0 || !lambda_is_activated()) //NBO or not activated
   d.sens.afr = 0;
  else //WBO or emulation
   d.sens.afr = ego_curve_lookup();
 }
#endif

 switch(d.engine_mode)
 {
  case EM_START: //cranking mode
#ifdef FUEL_INJECT
   if (d.param.inj_prime_delay)
   {
    //fire prime pulse before cranking
    if (!lgs.prime_ready && ((s_timer_gtc() - lgs.prime_delay_tmr) >= ((uint16_t)d.param.inj_prime_delay*10)))
    {
     if (!lgs.cog_changed && d.param.inj_prime_cold) //skip prime pulse if cranking has started or if it is disabled (=0)
      inject_open_inj(inj_prime_pw());               //start prime pulse
     lgs.prime_ready = 1;
    }
   }
   else //inj_prime_delay == 0
   {
    if (!lgs.prime_ready && lgs.cog_changed)
    {
     if (d.param.inj_prime_cold)                    //output prime pulse only if cranking has started and if it is not disabled (=0)
      inject_open_inj(inj_prime_pw());              //start prime pulse
     lgs.prime_ready = 1;
    }
   }

   d.corr.inj_timing = d.param.inj_timing_crk[d.sens.gas];

#endif
   if (d.sens.inst_frq > d.param.smap_abandon)
   {
    d.engine_mode = EM_IDLE;
    idling_regulator_init();
#ifdef FUEL_INJECT
    lgs.aftstr_enrich_counter = d.param.inj_aftstr_strokes << 1; //init engine strokes counter
#endif
   }
   angle = d.corr.strt_aalt = start_function();     //basic ignition timing - cranking map
   d.corr.temp_aalt = crkclt_function();            //CLT corr. on cranking
   angle+=d.corr.temp_aalt;
   d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
   d.airflow = 0;                                   //no "air flow" on cranking

#ifdef FUEL_INJECT
   { //PW = CRANKING + DEADTIME
   uint32_t pw = inj_cranking_pw();
   if (d.param.barocorr_type)
    pw = (pw * barocorr_lookup()) >> 12;             //apply barometric correction
#ifndef SECU3T
   if (CHECKBIT(d.param.inj_flags, INJFLG_USEADDCORRS))
    pw = pw_gascorr(pw);                              //apply gas corrections
#endif
   d.inj_pw_raw = lim_inj_pw(&pw);
   d.inj_dt = accumulation_time(1);                  //apply dead time
   pw+= d.inj_dt;
   if (d.eng_running)
    d.inj_pw = lim_inj_pw(&pw);
   else
    d.inj_pw = 0;
   d.acceleration = 0; //no acceleration
   }
#endif

#ifdef PA4_INP_IGNTIM
   d.corr.pa4_aac = 0;
#endif

   break;

  case EM_IDLE: //idling mode
   if (d.sens.carb)//gas pedal depressed - go into work mode
   {
    d.engine_mode = EM_WORK;
   }
   angle = d.corr.idle_aalt = idling_function(); //basic ignition timing - idling map
   d.corr.temp_aalt = coolant_function();      //add CLT correction to ignition timing
   angle+=d.corr.temp_aalt;
#ifdef AIRTEMP_SENS
   d.corr.airt_aalt = airtemp_function();      //add air temperature correction
   angle+=d.corr.airt_aalt;
#else
   d.corr.airt_aalt = 0;
#endif
#ifdef PA4_INP_IGNTIM
   d.corr.pa4_aac = manual_igntim();
   angle+=d.corr.pa4_aac;
#endif
   d.corr.idlreg_aac = idling_pregulator(&idle_period_time_counter);//add correction from idling regulator
   angle+=d.corr.idlreg_aac;
   d.corr.strt_aalt = d.corr.work_aalt = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = get_use_injtim_map_flag() ? inj_timing_lookup() : d.param.inj_timing[d.sens.gas];
#endif
   break;

  default:
  case EM_WORK: //work mode (load)
   if (!d.sens.carb)//gas pedal released - go in to idling mode
   {
    d.engine_mode = EM_IDLE;
    idling_regulator_init();
   }

#if defined(SM_CONTROL) && !defined(FUEL_INJECT)
   //use idling map if choke RPM regulator is active
   if (d.choke_rpm_reg)
   {
    angle = d.corr.idle_aalt = idling_function();//basic ignition timing - idling map
    d.corr.work_aalt = AAV_NOTUSED;
   }
   else
   {
    angle = d.corr.work_aalt = work_function();//basic ignition timing - work map
    d.corr.idle_aalt = AAV_NOTUSED;
   }
#else
   angle = d.corr.work_aalt = work_function();//basic ignition timing - work map
   d.corr.idle_aalt = AAV_NOTUSED;
#endif

   d.corr.temp_aalt = coolant_function();   //add CLT correction to ignition timing
   angle+=d.corr.temp_aalt;
#ifdef AIRTEMP_SENS
   d.corr.airt_aalt = airtemp_function();   //add air temperature correction;
   angle+=d.corr.airt_aalt;
#else
   d.corr.airt_aalt = 0;
#endif
#ifdef PA4_INP_IGNTIM
   d.corr.pa4_aac = manual_igntim();
   angle+=d.corr.pa4_aac;
#endif
   //substract correction obtained from detonation regulator
   angle-=d.corr.knock_retard;
   d.corr.strt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = get_use_injtim_map_flag() ? inj_timing_lookup() : d.param.inj_timing[d.sens.gas];
#endif
   break;
 }

 //Add octane correction (constant value specified by user) and remember it in the octan_aac variable
 angle+=d.param.angle_corr;
 d.corr.octan_aac = d.param.angle_corr;
 //Limit ignition timing using set limits
 restrict_value_to(&angle, d.param.min_angle, d.param.max_angle);
 //If zero ignition timing mode is set, then 0
 if (d.param.zero_adv_ang)
  angle = 0;

 lgs.calc_adv_ang = angle; //save calculated advance angle
}

void ignlogic_stroke_event_notification(void)
{
 //Limit fast transients of ignition timing (filtering). So, it can not change more than specified value during 1 engine stroke
 //Filtering is turned off on cranking
 if (EM_START == d.engine_mode)
 {
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
  int16_t strt_map_angle = start_function();
  ckps_set_shutter_spark(0==strt_map_angle);
  d.corr.curr_angle = lgs.advance_angle_inhibitor_state = (0==strt_map_angle ? 0 : lgs.calc_adv_ang);
#else
  d.corr.curr_angle = lgs.advance_angle_inhibitor_state = lgs.calc_adv_ang;
#endif
 }
 else
 {
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
  ckps_set_shutter_spark(d.sens.frequen < 200 && 0==start_function());
#endif
  d.corr.curr_angle = advance_angle_inhibitor(lgs.calc_adv_ang, &lgs.advance_angle_inhibitor_state, d.param.angle_inc_speed, d.param.angle_dec_speed);
 }

#ifdef FUEL_INJECT
 //update afterstart enrichemnt counter
 if (lgs.aftstr_enrich_counter)
  --lgs.aftstr_enrich_counter;
 //update AE decay counter
 if (lgs.ae_decay_counter)
  --lgs.ae_decay_counter;
#endif
}

void ignlogic_cog_changed_notification(void)
{
#ifdef FUEL_INJECT
 lgs.cog_changed = 1;
 d.eng_running = 1; //running
#endif
}

void ignlogic_eng_stopped_notification(void)
{
 d.engine_mode = EM_START; //cranking
 d.corr.curr_angle = lgs.calc_adv_ang;
#ifdef FUEL_INJECT
 d.eng_running = 0; //stopped
#endif

 //Sample atmospheric pressure which will be used for barometric correction.
 sample_baro_pressure();
}
