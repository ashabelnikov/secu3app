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
#include "ltft.h"

/**Module's local state variables*/
typedef struct
{
#ifdef FUEL_INJECT
 uint16_t aftstr_enrich_counter; //!< Stroke counter used in afterstart enrichment implementation
 uint16_t aftstr_enrich_counter0;//!< flat part counter
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
 0,0,0,0,0,0,0,0,0,
#endif
 0,0
};

/**Used by idling regulator's controlling algorithm*/
s_timer16_t idle_period_time_counter = {0,0,1}; //already fired!

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
/** Calculates manual ignition timing correction value depending on the selected input (ADD_I4 for SECU-3T or ADD_I3/4/5 for SECU-3i)
 * \return value * ANGLE_MULTIPLIER
 */
static int16_t manual_igntim(void)
{
#ifdef SECU3T
#ifdef TPIC8101
 if (IOCFG_CHECK(IOP_IGNTIM))
  return pa4_function(IOCFG_GETA(IOP_IGNTIM));
 else
#endif
  return 0; //not mapped to real I/O
#else //SECU3i
 if (IOCFG_CHECK(IOP_IGNTIM))
  return pa4_function(IOCFG_GETA(IOP_IGNTIM));
 else
  return 0; //not mapped to real I/O
#endif
}
#endif

#if !defined(SECU3T) && defined(FUEL_INJECT)
/** Calculates manual correction value of injection PW depending on the selected input (ADD_I3/4/5/6/7/8 for SECU-3i)
 * \return value * 4096
 */
static int16_t manual_injpw(void)
{
 if (IOCFG_CHECK(IOP_INJPWC_I))
  return injpwcoef_function(IOCFG_GETA(IOP_INJPWC_I));
 else
  return 4096; //not mapped to real I/O, return 1.000
}
#endif

#if defined(IFR_VS_MAP_CORR) && defined(FUEL_INJECT)
/** Injector's flow rate correction vs manifold absolute pressure
 *  coefficient = 1.0 / sqrt(((frgp + pbaro) - map) / ifr_gp) = sqrt(ifr_gp / ((frgp + pbaro) - map))
 * \return correction factor * 256
 */
static uint16_t ifr_vs_map_corr(void)
{
 uint16_t frap = d.sens.baro_press;
#ifndef SECU3T
 if (PGM_GET_BYTE(&fw_data.exdata.ifrvmc_use_fps) && !IOCFG_GETE(IOP_FPS))
  frap+= d.sens.fps; //use fuel pressure sensor
 else
#endif
  frap+= PGM_GET_WORD(&fw_data.exdata.frgp); //use simple constant

 if (frap <= d.sens.map)
  frap = d.sens.map + 1; //prevent div. by zero
 return ui32_sqrt((((uint32_t)PGM_GET_WORD(&fw_data.exdata.ifr_gp)) * 65536UL) / (frap - d.sens.map)); //65536 = 256^2, sqrt(256^2) = 256
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
  if (PGM_GET_BYTE(&fw_data.exdata.fi_leave_strokes) > 0 && (d.sens.tps < PGM_GET_WORD(&fw_data.exdata.sfc_tps_thrd)))
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
 * \param pw1 Pointer to the variable which contains value of PW for bank1
 * \param pw2 Pointer to the variable which contains value of PW for bank2
 * \return PW value ready to be used. Note that values of PW used to drive injectors are stored to inj_pwns array
 */
static uint16_t finalize_inj_time(int32_t* pw1, int32_t* pw2)
{
 uint16_t pwns[2];
 d.inj_dt = (int16_t)accumulation_time(1);      //calculate dead time (injector lag), value is signed
 uint16_t inj_min_pw = ((uint16_t)(d.param.inj_min_pw[d.sens.gas])) * 8;
 uint16_t inj_max_pw = d.param.inj_max_pw[d.sens.gas];

 if (!CHECKBIT(d.param.strt_flags, STRTF_LIMCRANPW) && d.engine_mode == EM_START)
 { //remove max PW limitation on cranking if limitation was disabled by user
  inj_max_pw = 65535;
 }

#ifdef XTAU_CORR
 if ((1==d.param.wallwet_model && 0==d.sens.gas) || (2==d.param.wallwet_model && 1==d.sens.gas) || 3==d.param.wallwet_model)
  calc_xtau(pw1, pw2); //apply x-tau corrections
#endif

 //apply correction of non-linearity at small PWs
 if (PGM_GET_BYTE(&fw_data.exdata.use_injnonlin_corr))
 {
  if (*pw1 < inj_nonlin_binsmax())
   *pw1 = inj_nonlin_lookup(*pw1);
  if (*pw2 < inj_nonlin_binsmax())
   *pw2 = inj_nonlin_lookup(*pw2);
 }

 uint8_t i;
 for (i = 0; i < INJ_CHANNELS_MAX; ++i)
 {
  uint16_t m; int16_t a;
  inj_cylmultadd(i, &m, &a);

  //Always use 1st lambda channel if sensors' mixing is on
  uint8_t chsel = CHECKBIT(d.param.lambda_selch, i) && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN);

  //Precalculate normal injection time
  int32_t tpw = ((chsel ? *pw2 : *pw1) * m) >> 8;
  tpw+= a;
  //add inj. lag and restrict result
  tpw+= d.inj_dt;
  if (tpw < inj_min_pw)
   pwns[0] = inj_min_pw;
  else if (tpw > inj_max_pw)
   pwns[0] = inj_max_pw;
  else
   pwns[0] = tpw;
  pwns[0] = apply_smooth_fuelcut(pwns[0]);

  //Precalculate shrinked injection time depending on cylinder number for emergency semi-sequential/simultaneous mode
  tpw = ((chsel ? *pw2 : *pw1) * m) >> 8;
  tpw+= a;
  if (!(d.param.ckps_engine_cyl & 1))
   tpw>>= 1; //2 times (even cylinder number engines)
  else if (d.param.ckps_engine_cyl == 5)
   tpw = (tpw * 13107) >> 16;  //divide by 5 (13107 = (1/5)*65536)
  else if (d.param.ckps_engine_cyl == 3)
   tpw = (tpw * 21845) >> 16;  //divide by 3 (21845 = (1/3)*65536)
  //add inj. lag and restrict result
  tpw+= d.inj_dt;
  if (tpw < inj_min_pw)
   pwns[1] = inj_min_pw;
  else if (tpw > inj_max_pw)
   pwns[1] = inj_max_pw;
  else
   pwns[1] = tpw;
  pwns[1] = apply_smooth_fuelcut(pwns[1]);

  //store precalculated values (see inject_set_fullsequential() for more information)
  _BEGIN_ATOMIC_BLOCK();
  d.inj_pwns[0][i] = pwns[0];
  d.inj_pwns[1][i] = pwns[1];
  _END_ATOMIC_BLOCK();
 }

 //calculate value for user based on the avarage result
 uint32_t inj_pw = 0;
 uint8_t s = inject_is_shrinked();
 uint8_t rowadd = 0;
 if (CHECKBIT(d.param.inj_flags, INJFLG_SECINJROWSWT) && d.sens.gas)
  rowadd = INJ_CHANNELS_MAX / 2;
 for (i = 0; i < d.param.ckps_engine_cyl; ++i)
  inj_pw+=d.inj_pwns[s][i + rowadd];
 return inj_pw / d.param.ckps_engine_cyl;
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
 if (PGM_GET_WORD(&fw_data.exdata.frgp))
  pw = (pw * ifr_vs_map_corr()) >> 8;            //apply injector's flow rate vs manifold pressure correction
#endif

 if (CHECKBIT(d.param.inj_flags, INJFLG_USEAIRDEN))
  pw = (pw * inj_airtemp_corr(0)) >> 7;           //apply air density correction (if enabled)

 pw = (pw * inj_warmup_en()) >> 7;              //apply warmup enrichemnt factor
 if (lgs.aftstr_enrich_counter)
  pw= (pw * (128 + scale_aftstr_enrich(lgs.aftstr_enrich_counter))) >> 7; //apply scaled afterstart enrichment factor

 pw= (pw * inj_iacmixtcorr_lookup()) >> 13;     //apply mixture correction vs IAC
 if (d.param.barocorr_type)
  pw = (pw * barocorr_lookup()) >> 12;           //apply barometric correction

#ifndef SECU3T
 if (CHECKBIT(d.param.inj_flags, INJFLG_USEADDCORRS))
  pw_gascorr(&pw);                              //apply gas corrections
#endif

#ifndef SECU3T
 if (IOCFG_CHECK(IOP_INJPWC_I))
 {
  if (PGM_GET_BYTE(&fw_data.exdata.maninjpw_idl) || !(d.engine_mode==EM_IDLE))
   pw = (pw * manual_injpw()) >> 12;             //apply manual inj.PW coefficient
 }
#endif

#if !defined(SECU3T) && defined(FUEL_INJECT)
 uint8_t fdc_use = PGM_GET_BYTE(&fw_data.exdata.fueldens_corr_use);
 if ((fdc_use==d.sens.gas) || (fdc_use==2))
  pw = (pw * fueldens_corr()) >> 14;            //apply fuel density correction
#endif

 int32_t pw2 = pw;

 if (0xFF!=d.param.lambda_selch)
  pw = (pw * (512 + d.corr.lambda[0])) >> 9;      //apply lambda correction additive factor (signed) from sensor #1
 if (0x00!=d.param.lambda_selch && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
  pw2 = (pw2 * (512 + d.corr.lambda[1])) >> 9;    //apply lambda correction additive factor (signed) from sensor #2

 if (ltft_is_active())
 {
  if (0xFF!=d.param.lambda_selch)
   pw = (pw * calc_ltft(0)) >> 9;            //apply LTFT correction #1
  if (0x00!=d.param.lambda_selch && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
   pw2 = (pw2 * calc_ltft(1)) >> 9;          //apply LTFT correction #2
 }

 if (0==d.param.inj_ae_type)
 {
  int32_t ae_add = acc_enrich_calc(0, lambda_get_stoichval());//add acceleration enrichment (accel. pump)
  pw+=  ae_add;
  pw2+= ae_add;
 }
 else
 {
  int32_t ae_add = acc_enrich_calc_tb(0, lambda_get_stoichval());//add acceleration enrichment (time based)
  pw+=  ae_add;
  pw2+= ae_add;
 }

 d.inj_pw = finalize_inj_time(&pw, &pw2);
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
 else //additional MAP, dynamic corr.
  d.sens.baro_press = d.sens.map2;

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
 inj_corrected_mat();
#endif

#if defined(FUEL_INJECT) || defined(SM_CONTROL) || defined(GD_CONTROL)
 engine_blowing_cond(); //check for entering flood clear mode
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 if (IOCFG_CHECK(IOP_LAMBDA) || CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
 {
  if (d.param.inj_lambda_senstype==0 || !lambda_is_activated(0)) //NBO or not activated
   d.sens.afr[0] = 0;
  else //WBO or emulation
   d.sens.afr[0] = ego_curve_lookup(0);
 }
 else
  d.sens.afr[0] = 0;

 if (IOCFG_CHECK(IOP_LAMBDA2) && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN))
 {
  if (d.param.inj_lambda_senstype==0 || !lambda_is_activated(1)) //NBO or not activated
   d.sens.afr[1] = 0;
  else //WBO or emulation
   d.sens.afr[1] = ego_curve_lookup(1);
 }
 else
   d.sens.afr[1] = 0;
#endif

 switch(d.engine_mode)
 {
  case EM_START: //cranking mode
#ifdef FUEL_INJECT
   reset_smooth_fuelcut();   //reset fuel cut state variables
   acc_enrich_calc_tb(1, 0); //reset state variables of the AE algo
#ifndef SECU3T
   if (d.gasval_res)
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
      inject_open_inj(inj_prime_pw(), PGM_GET_BYTE(&fw_data.exdata.inj_prime_times)); //start prime pulse
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
      inject_open_inj(inj_prime_pw(), PGM_GET_BYTE(&fw_data.exdata.inj_prime_times));//start prime pulse
     lgs.prime_ready = 1;
    }
   }

   d.corr.inj_timing = param_inj_timing(0);

#endif

   //Change engine mode to idling if:
   //RPM is above threshold and engine start is allowed in the flood clear mode. If engine start in the flood clear mode is not allowed
   //but driver has released gas pedal, then allow start of engine.
   if (d.sens.rpm > (d.param.smap_abandon ? d.param.smap_abandon : smapaban_thrd_rpm()) && (CHECKBIT(d.param.strt_flags, STRTF_FLDCLRSTR) || 0==d.floodclear))
   {
    d.engine_mode = EM_IDLE;
    idling_regulator_init();
#ifdef FUEL_INJECT
    lgs.aftstr_enrich_counter = aftstr_strokes(d.sens.gas); //init engine strokes counter
    lgs.aftstr_enrich_counter0 = PGM_GET_WORD(&fw_data.exdata.aftstr_flat_strokes);
    if (CHECKBIT(d.param.inj_flags, INJFLG_FSAFTERSTART))
     ckps_enable_fullsequential(); //enable switching into a full sequential mode
#endif
   }
   angle = d.corr.strt_aalt = start_function();     //basic ignition timing - cranking map
   d.corr.temp_aalt = crkclt_function();            //CLT corr. on cranking
   angle+=d.corr.temp_aalt;
   d.corr.knkret_aac = d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
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

   int32_t pw2 = pw;

   d.inj_pw = finalize_inj_time(&pw, &pw2);
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
   d.corr.pa4_aac = AAV_NOTUSED;                      //manual ignition timing correction is not used during cranking
#endif

   break;

  case EM_IDLE: //idling mode
   if (d.sens.carb)//gas pedal depressed - go into work mode
   {
    d.engine_mode = EM_WORK;
   }
   if (CHECKBIT(d.param.igntim_flags, IGNTF_WRKMAP))
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
   if (CHECKBIT(d.param.igntim_flags, IGNTF_MANIDL))
   {
    d.corr.pa4_aac = manual_igntim();
    angle+=d.corr.pa4_aac;
   }
   else
    d.corr.pa4_aac = AAV_NOTUSED;
#endif
   d.corr.idlreg_aac = idling_pregulator(&idle_period_time_counter);//add correction from idling regulator
   angle+=d.corr.idlreg_aac;
   d.corr.knkret_aac = d.corr.strt_aalt = AAV_NOTUSED; //knock control is not used during idling

#ifdef SPLIT_ANGLE
   d.corr.split_angle = split_function();        //calculate and store split angle value
#endif

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = get_use_injtim_map_flag() ? inj_timing_lookup() : param_inj_timing(1);
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
   d.corr.knkret_aac = d.corr.knock_retard;
   d.corr.strt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;

#ifdef SPLIT_ANGLE
   d.corr.split_angle = split_function();        //calculate and store split angle value
#endif

#ifdef FUEL_INJECT
   {//PW = (BASE * WARMUP * AFTSTR_ENRICH) + LAMBDA_CORR + ACCEL_ENRICH + DEADTIME
    fuel_calc();
   }

   d.corr.inj_timing = get_use_injtim_map_flag() ? inj_timing_lookup() : param_inj_timing(1);
#endif
   break;
 }

 //Add octane correction (constant value specified by user) and remember it in the octan_aac variable
 angle+=d.param.angle_corr;
 d.corr.octan_aac = d.param.angle_corr;

 //if input is active, then use specified value of ignition timing
 if (IOCFG_CHECK(IOP_AUTO_I))
 {
  if (IOCFG_GET(IOP_AUTO_I))
  {
   angle = d.param.shift_igntim;
   d.corr.octan_aac = AAV_NOTUSED;
#ifdef PA4_INP_IGNTIM
   d.corr.pa4_aac = AAV_NOTUSED;
#endif
   d.corr.knkret_aac = d.corr.temp_aalt = d.corr.strt_aalt = d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
  }
 }

 //Limit ignition timing using set limits
 restrict_value_to(&angle, d.param.min_angle, d.param.max_angle);

 //If zero ignition timing mode is set, then 0
 if (d.param.zero_adv_ang)
 {
  angle = 0;
  d.corr.octan_aac = AAV_NOTUSED;
#ifdef PA4_INP_IGNTIM
   d.corr.pa4_aac = AAV_NOTUSED;
#endif
  d.corr.knkret_aac = d.corr.temp_aalt = d.corr.strt_aalt = d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
 }
 else if (CHECKBIT(d.param.igntim_flags, IGNTF_MANZERO))
 {
  angle = d.param.angle_corr;
#ifdef PA4_INP_IGNTIM
  d.corr.pa4_aac = AAV_NOTUSED;
#endif
  d.corr.knkret_aac = d.corr.temp_aalt = d.corr.strt_aalt = d.corr.idle_aalt = d.corr.work_aalt = d.corr.airt_aalt = d.corr.idlreg_aac = AAV_NOTUSED;
 }

 lgs.calc_adv_ang = angle; //save calculated advance angle

#ifdef FUEL_INJECT
 d.sens.inj_duty = inject_calc_duty();
#endif
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
  ckps_set_shutter_spark(d.sens.rpm < 200 && 0==start_function());
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
 if (0==lgs.aftstr_enrich_counter0)
 {
  if (lgs.aftstr_enrich_counter)
   --lgs.aftstr_enrich_counter;
 }
 else
  --lgs.aftstr_enrich_counter0;

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
 if (!CHECKBIT(d.param.inj_flags, INJFLG_FSAFTERSTART))
  ckps_enable_fullsequential(); //system will switch into a full sequential mode on cranking
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 d.aftstr_enr = 0;
#endif

 //Sample atmospheric pressure which will be used for barometric correction.
 sample_baro_pressure();
}
