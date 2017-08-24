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

/**Reserved value used to indicate that value is not used in corresponding mode*/
#define AAV_NOTUSED 0x7FFF

#ifdef FUEL_INJECT
typedef struct
{
 uint16_t aftstr_enrich_counter; //!< Stroke counter used in afterstart enrichment implementation
 uint16_t prime_delay_tmr;       //!< Timer variable used for prime pulse delay
 uint8_t  prime_ready;           //!< Indicates that prime pulse was fired or skipped if cranking was started before
 uint8_t  cog_changed;           //!< Flag which indicates there was crankshaft revolution after last power on
}logic_state_t;

/**Instance of internal state variables structure*/
static logic_state_t lgs;
#endif

void ignlogic_init(void)
{
#ifdef FUEL_INJECT
 lgs.aftstr_enrich_counter = 0;
 lgs.prime_delay_tmr = s_timer_gtc();
 lgs.prime_ready = 0;
 lgs.cog_changed = 0;
#endif
}

#ifdef FUEL_INJECT

/**Limit value of injection PW
 * \return limited value (16 bit)
 */
static uint16_t lim_inj_pw(uint32_t *value)
{
 return (*value) > 65535 ? 65535 : (*value);
}

/** Calculates AE value.
 * Uses d ECU data structure
 * \return AE value in PW units
 */
static int32_t calc_acc_enrich(void)
{
 //calculate normal conditions PW, MAP=100kPa, IAT=20°C, AFR=14.7 (petrol)
 int32_t pwnc = (ROUND((100.0*MAP_PHYSICAL_MAGNITUDE_MULTIPLIER*256) / (293.15*14.7*TEMP_PHYSICAL_MAGNITUDE_MULTIPLIER)) * d.param.inj_sd_igl_const) >> 12;
 int16_t aef = inj_ae_tps_lookup();               //calculate basic AE factor value

 if (abs(d.sens.tpsdot) < d.param.inj_ae_tpsdot_thrd) {
  d.acceleration = 0;
  return 0;                                       //no acceleration or deceleration
 }
 d.acceleration = 1;

 aef = ((int32_t)aef * inj_ae_clt_corr()) >> 7;   //apply CLT correction factor to AE factor
 aef = ((int32_t)aef * inj_ae_rpm_lookup()) >> 7; //apply RPM correction factor to AE factor
 return (pwnc * aef) >> 7;                        //apply AE factor to the normal conditions PW
}
#endif

/** Perform fuel calculations used on idling and work
 */
#ifdef FUEL_INJECT
static void fuel_calc(void)
{
   uint32_t pw = inj_base_pw();
   pw = (pw * inj_warmup_en()) >> 7;              //apply warmup enrichemnt factor
   if (lgs.aftstr_enrich_counter)
    pw= (pw * (128 + scale_aftstr_enrich(lgs.aftstr_enrich_counter))) >> 7; //apply scaled afterstart enrichment factor
   pw= (pw * (512 + d.corr.lambda)) >> 9;         //apply lambda correction additive factor (signed)
   pw= (pw * inj_iacmixtcorr_lookup()) >> 13;     //apply mixture correction vs IAC
   pw+= calc_acc_enrich();                        //add acceleration enrichment
   if (((int32_t)pw) < 0)
    pw = 0;
   d.inj_pw_raw = lim_inj_pw(&pw);
   d.inj_dt = inj_dead_time();
   pw+= d.inj_dt;
   if (d.ie_valve && !d.fc_revlim)
    d.inj_pw = lim_inj_pw(&pw);
   else d.inj_pw = 0;
}
#endif

int16_t ignlogic_system_state_machine(void)
{
 int16_t angle;
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

   d.corr.inj_timing = d.param.inj_timing_crk;

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
   d.corr.idle_aalt = d.corr.work_aalt = d.corr.temp_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
   d.airflow = 0;                                   //no "air flow" on cranking

#ifdef FUEL_INJECT
   { //PW = CRANKING + DEADTIME
   uint32_t pw = inj_cranking_pw();
   d.inj_pw_raw = lim_inj_pw(&pw);
   d.inj_dt = inj_dead_time();
   pw+= d.inj_dt;
   d.inj_pw = lim_inj_pw(&pw);
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
   work_function(1);                           //update air flow value
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
   d.corr.pa4_aac = pa4_function(d.sens.add_i3);
   angle+=d.corr.pa4_aac;
#endif
   d.corr.idlreg_aac = idling_pregulator(&idle_period_time_counter);//add correction from idling regulator
   angle+=d.corr.idlreg_aac;
   d.corr.strt_aalt = d.corr.work_aalt = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP) ? inj_timing_lookup() : d.param.inj_timing;
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
   //air flow will be always 1 if choke RPM regulator is active
   if (d.choke_rpm_reg)
   {
    work_function(1);                      //update air flow value
    angle = d.corr.idle_aalt = idling_function();//basic ignition timing - idling map
    d.corr.work_aalt = AAV_NOTUSED;
   }
   else
   {
    angle = d.corr.work_aalt = work_function(0);//basic ignition timing - work map
    d.corr.idle_aalt = AAV_NOTUSED;
   }
#else
   angle = d.corr.work_aalt = work_function(0);//basic ignition timing - work map
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
   d.corr.pa4_aac = pa4_function(d.sens.add_i3);
   angle+=d.corr.pa4_aac;
#endif
   //substract correction obtained from detonation regulator
   angle-=d.corr.knock_retard;
   d.corr.strt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP) ? inj_timing_lookup() : d.param.inj_timing;
#endif
   break;
 }
 return angle; //return calculated advance angle
}

void ignlogic_stroke_event_notification(void)
{
#ifdef FUEL_INJECT
 //update afterstart enrichemnt counter
 if (lgs.aftstr_enrich_counter)
  --lgs.aftstr_enrich_counter;
#endif
}

void ignlogic_cog_changed_notification(void)
{
#ifdef FUEL_INJECT
 lgs.cog_changed = 1;
#endif
}
