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
 uint8_t  sfc_transient_e;       //!< Counter for soft transient from normal injection to the fuel cut mode
 uint8_t  sfc_transient_l;       //!< Counter for soft transient from the fuel cut mode to normal injection
 uint16_t sfc_pw_e;              //!<
 uint16_t sfc_pw_l;              //!<
#endif
 int16_t  calc_adv_ang;          //!< calculated advance angle
 int16_t  advance_angle_inhibitor_state; //!<
}logic_state_t;

/**Instance of internal state variables structure*/
static logic_state_t lgs = {
#ifdef FUEL_INJECT
 0,0,0,0,0,0,0,0,
#endif
 0,0
};

void eculogic_init(void)
{
#ifdef FUEL_INJECT
 lgs.prime_delay_tmr = s_timer_gtc();
#endif
 d.sens.baro_press = PRESSURE_MAGNITUDE(101.3); //set default value to prevent wrong conditions when barometric pressure will not be sampled for some reasons
}

#if defined(FUEL_INJECT) && !defined(SECU3T)
/** Applies gas temperature and pressure corrections (coefficients) to the inj. PW
 * \param pw PW to be corrected, will also receive result
 */
void pw_gascorr(int32_t* pw)
{
 (*pw) = ((*pw) * inj_gts_pwcorr()) >> 7;              //apply gas temperature correction
 (*pw) = ((*pw) * inj_gps_pwcorr()) >> 7;              //apply gas pressure correction
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
#ifdef MCP3204
 else if (IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i5 || IOCFG_CB(IOP_IGNTIM) == (fnptr_t)iocfg_g_add_i5i)
  return pa4_function(d.sens.add_i5);
#endif
 else
  return 0; //not mapped to real I/O
#endif
}
#endif

#if defined(IFR_VS_MAP_CORR) && defined(FUEL_INJECT)
/** Injector's flow rate correction vs manifold absolute pressure
 *  coefficient = 1.0 / sqrt(frap - map / 300.0)
 * \return correction factor * 256
 */
static uint16_t ifr_vs_map_corr(void)
{
 //uint16_t frap = ROUNDU16(500.0 * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER); //absolute pressure in the fuel rail
 return ui32_sqrt((3UL * 100UL * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER * 65535UL) / (PGM_GET_WORD(&fw_data.exdata.frap) - d.sens.map)); //65535 = 256 ^ 2
}
#endif

#ifdef FUEL_INJECT
/** Resets transient smoothing for the forced idle fuel cut */
static void reset_smooth_fuelcut(void)
{
 lgs.sfc_transient_e = PGM_GET_BYTE(&fw_data.exdata.fi_enter_strokes);
 lgs.sfc_transient_l = PGM_GET_BYTE(&fw_data.exdata.fi_leave_strokes);
 lgs.sfc_pw_e = lgs.sfc_pw_l = 0;
}

/** Implements transient smoothing for the forced idle fuel cut.
 * \param pw Calculated injection PW
 * \return Injection PW transformed according to the current state of fuel cut
 */
static uint16_t apply_smooth_fuelcut(uint16_t pw)
{
 if (d.ie_valve)
 { //leave fuel cut mode
  if (PGM_GET_BYTE(&fw_data.exdata.fi_leave_strokes) > 0)
   pw = simple_interpolation(lgs.sfc_transient_l, lgs.sfc_pw_e, pw, 0, PGM_GET_BYTE(&fw_data.exdata.fi_leave_strokes), 1);
  lgs.sfc_transient_e = 0;
  lgs.sfc_pw_l = pw;
 }
 else
 { //enter fuel cut mode
  if (PGM_GET_BYTE(&fw_data.exdata.fi_enter_strokes) > 0)
   pw = simple_interpolation(lgs.sfc_transient_e, lgs.sfc_pw_l, 0, 0, PGM_GET_BYTE(&fw_data.exdata.fi_enter_strokes), 1);
  else
   pw = 0;
  lgs.sfc_transient_l = 0;
  lgs.sfc_pw_e = pw;
 }
 return pw;
}

/** Finalizes specified inj. PW value (adds injector lag and precalculates normal and shrinked values)
 * \param pw Pointer to the variable which contains value of PW
 * \return PW value ready to be used to drive injectors
 */
static uint16_t finalize_inj_time(int32_t* pw)
{
 int32_t pw_s = *pw;
 uint16_t pwns[2];

 d.inj_dt = (int16_t)accumulation_time(1);      //calculate dead time (injector lag), value is signed

 uint16_t inj_min_pw = ((uint16_t)(d.param.inj_min_pw[d.sens.gas])) * 8;
 uint16_t inj_max_pw = PGM_GET_WORD(&fw_data.exdata.inj_max_pw);

 //add inj. lag and restrict result
 (*pw)+= d.inj_dt;
 if ((*pw) < inj_min_pw)
  pwns[0] = inj_min_pw;
 else if ((*pw) > inj_max_pw)
  pwns[0] = inj_max_pw;
 else
  pwns[0] = *pw;

 //Precalculate shrinked injection time depending on cylinder number for emergency semi-sequential/simultaneous mode
 if (!(d.param.ckps_engine_cyl & 1))
  pw_s>>= 1; //2 times (even cylinder number engines)
 else if (d.param.ckps_engine_cyl == 5)
  pw_s = (pw_s * 13107) >> 16;  //divide by 5 (13107 = (1/5)*65536)
 else if (d.param.ckps_engine_cyl == 3)
  pw_s = (pw_s * 21845) >> 16;  //divide by 3 (21845 = (1/3)*65536)

 //add inj. lag and restrict result
 pw_s+= d.inj_dt;
 if (pw_s < inj_min_pw)
  pwns[1] = inj_min_pw;
 else if (pw_s > inj_max_pw)
  pwns[1] = inj_max_pw;
 else
  pwns[1] = pw_s;

 //store precalculated values (see inject_set_fullsequential() for more information)
 _BEGIN_ATOMIC_BLOCK();
 d.inj_pwns[0] = pwns[0];
 d.inj_pwns[1] = pwns[1];
 _END_ATOMIC_BLOCK();

 return (inject_is_shrinked() ? pwns[1] : pwns[0]);
}

/** Perform fuel calculations used on idling and work
 */
static void fuel_calc(void)
{
#ifdef GD_CONTROL
 if (!(d.sens.gas && IOCFG_CHECK(IOP_GD_STP)))
 {
#endif
 int32_t pw = inj_base_pw();

#ifdef IFR_VS_MAP_CORR
 if (PGM_GET_WORD(&fw_data.exdata.frap))
  pw = (pw * ifr_vs_map_corr()) >> 8;            //apply injector's flow rate vs manifold pressure correction
#endif

 if (CHECKBIT(d.param.inj_flags, INJFLG_USEAIRDEN))
  pw = (pw * inj_airtemp_corr(0)) >> 7;           //apply air density correction (if enabled)

 pw = (pw * inj_warmup_en()) >> 7;              //apply warmup enrichemnt factor
 if (lgs.aftstr_enrich_counter)
  pw= (pw * (128 + scale_aftstr_enrich(lgs.aftstr_enrich_counter))) >> 7; //apply scaled afterstart enrichment factor
 pw= (pw * (512 + d.corr.lambda)) >> 9;         //apply lambda correction additive factor (signed)
 pw= (pw * inj_iacmixtcorr_lookup()) >> 13;     //apply mixture correction vs IAC
 if (d.param.barocorr_type)
  pw = (pw * barocorr_lookup()) >> 12;           //apply barometric correction

#ifndef SECU3T
 if (CHECKBIT(d.param.inj_flags, INJFLG_USEADDCORRS))
  pw_gascorr(&pw);                              //apply gas corrections
#endif
 pw+= acc_enrich_calc(0, lambda_get_stoichval());//add acceleration enrichment

 d.inj_pw = finalize_inj_time(&pw);
 d.inj_pw = apply_smooth_fuelcut(d.inj_pw);
 if (!(d.inj_pw > INJPW_MAG(0.1) && !d.fc_revlim && d.eng_running))
  d.inj_pw = 0;
#ifdef GD_CONTROL
}
else
 d.inj_pw = 0;
#endif
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

/** Gets state of the "Use inj. timing map" check
 * \return 0 - using simple constant, 1 - using a 3D map
*/
uint8_t get_use_injtim_map_flag(void)
{
 return (d.sens.gas ? CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP_G) : CHECKBIT(d.param.inj_flags, INJFLG_USETIMINGMAP));
}

void eculogic_system_state_machine(void)
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
   reset_smooth_fuelcut();
#ifndef SECU3T
   if (d.gasval_on)
    lgs.prime_delay_tmr = s_timer_gtc();
#endif
   if (d.param.inj_prime_delay)
   {
    //fire prime pulse before cranking
    if (!lgs.prime_ready && ((s_timer_gtc() - lgs.prime_delay_tmr) >= ((uint16_t)d.param.inj_prime_delay*10)))
    {
     if (!lgs.cog_changed && d.param.inj_prime_cold) //skip prime pulse if cranking has started or if it is disabled (=0)
#ifdef GD_CONTROL
      if (!(d.sens.gas && IOCFG_CHECK(IOP_GD_STP)))
#endif
      inject_open_inj(inj_prime_pw());               //start prime pulse
     lgs.prime_ready = 1;
    }
   }
   else //inj_prime_delay == 0
   {
    if (!lgs.prime_ready && lgs.cog_changed)
    {
     if (d.param.inj_prime_cold)                    //output prime pulse only if cranking has started and if it is not disabled (=0)
#ifdef GD_CONTROL
      if (!(d.sens.gas && IOCFG_CHECK(IOP_GD_STP)))
#endif
      inject_open_inj(inj_prime_pw());              //start prime pulse
     lgs.prime_ready = 1;
    }
   }

   d.corr.inj_timing = d.param.inj_timing_crk[d.sens.gas];

#endif
   if (d.sens.inst_frq > (d.param.smap_abandon ? d.param.smap_abandon : smapaban_thrd_rpm()))
   {
    d.engine_mode = EM_IDLE;
    idling_regulator_init();
#ifdef FUEL_INJECT
    lgs.aftstr_enrich_counter = aftstr_strokes(d.sens.gas); //init engine strokes counter
#endif
   }
   angle = d.corr.strt_aalt = start_function();     //basic ignition timing - cranking map
   d.corr.temp_aalt = crkclt_function();            //CLT corr. on cranking
   angle+=d.corr.temp_aalt;
   d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
   d.airflow = 0;                                   //no "air flow" on cranking

#ifdef SPLIT_ANGLE
   d.corr.split_angle = 0;                          //no splitting during cranking
#endif

#ifdef FUEL_INJECT
#ifdef GD_CONTROL
   if (!(d.sens.gas && IOCFG_CHECK(IOP_GD_STP)))
#endif
   { //PW = CRANKING + DEADTIME
   int32_t pw = inj_cranking_pw();
   if (d.param.barocorr_type)
    pw = (pw * barocorr_lookup()) >> 12;             //apply barometric correction
#ifndef SECU3T
   if (CHECKBIT(d.param.inj_flags, INJFLG_USEADDCORRS))
    pw_gascorr(&pw);                                 //apply gas corrections
#endif

   d.inj_pw = finalize_inj_time(&pw);
   if (!(d.eng_running))
    d.inj_pw = 0;

   d.acceleration = 0; //no acceleration
   }
#ifdef GD_CONTROL
   else
    d.inj_pw = 0;
#endif
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
   if (PGM_GET_BYTE(&fw_data.exdata.igntim_wrkmap))
   { //it is selected always to use working mode's map
    angle = d.corr.work_aalt = work_function();//basic ignition timing - work map
    d.corr.idle_aalt = AAV_NOTUSED;
   }
   else
   { //use idle map (default)
    angle = d.corr.idle_aalt = idling_function(); //basic ignition timing - idling map
    d.corr.work_aalt = AAV_NOTUSED;
   }
   d.corr.temp_aalt = coolant_function(0);     //add CLT correction to ignition timing
   angle+=d.corr.temp_aalt;
#ifdef AIRTEMP_SENS
   d.corr.airt_aalt = airtemp_function();      //add air temperature correction
   angle+=d.corr.airt_aalt;
#else
   d.corr.airt_aalt = 0;
#endif
#ifdef PA4_INP_IGNTIM
   if (PGM_GET_BYTE(&fw_data.exdata.manigntim_idl))
   {
    d.corr.pa4_aac = manual_igntim();
    angle+=d.corr.pa4_aac;
   }
#endif
   d.corr.idlreg_aac = idling_pregulator(&idle_period_time_counter);//add correction from idling regulator
   angle+=d.corr.idlreg_aac;
   d.corr.strt_aalt = AAV_NOTUSED;

#ifdef SPLIT_ANGLE
   d.corr.split_angle = split_function();        //calculate and store split angle value
#endif

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

   d.corr.temp_aalt = coolant_function(1);  //add CLT correction to ignition timing
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

#ifdef SPLIT_ANGLE
   d.corr.split_angle = split_function();        //calculate and store split angle value
#endif

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

 //if input is active, then use specified value of ignition timing
 if (IOCFG_CHECK(IOP_AUTO_I))
 {
  if (IOCFG_GET(IOP_AUTO_I))
   angle = (int16_t)PGM_GET_WORD(&fw_data.exdata.shift_igntim);
 }

 lgs.calc_adv_ang = angle; //save calculated advance angle
}

void eculogic_stroke_event_notification(void)
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

#ifdef SPLIT_ANGLE
  //if d.corr.split_angle is positive, then leading (1st) spark plug fired before the trailing one (2nd)
  //if d.corr.split_angle is negative, then trailing (2nd) spark plug fires before the leading one (1st)
  d.corr.curr_angle1 = d.corr.curr_angle - d.corr.split_angle;
  //Limit ignition timing for trailing plug using set limits
  restrict_value_to(&d.corr.curr_angle1, d.param.min_angle, d.param.max_angle);
#endif

#ifdef FUEL_INJECT
 //update afterstart enrichemnt counter
 if (lgs.aftstr_enrich_counter)
  --lgs.aftstr_enrich_counter;

 if (!d.sens.gas)
  d.aftstr_enr = (0 != lgs.aftstr_enrich_counter);

 //update AE decay counter
 acc_enrich_decay_counter();

 //update counters for smoothing of entering/leaving from forced idle mode
 if (lgs.sfc_transient_e < PGM_GET_BYTE(&fw_data.exdata.fi_enter_strokes))
  lgs.sfc_transient_e++;
 if (lgs.sfc_transient_l < PGM_GET_BYTE(&fw_data.exdata.fi_leave_strokes))
  lgs.sfc_transient_l++;
#endif
}

void eculogic_cog_changed_notification(void)
{
#ifdef FUEL_INJECT
 lgs.cog_changed = 1;
 d.eng_running = 1; //running
#endif
}

void eculogic_eng_stopped_notification(void)
{
 d.engine_mode = EM_START; //cranking
 d.corr.curr_angle = lgs.calc_adv_ang;
#ifdef FUEL_INJECT
 d.eng_running = 0; //stopped
#endif

 d.aftstr_enr = 0;

 //Sample atmospheric pressure which will be used for barometric correction.
 sample_baro_pressure();
}
