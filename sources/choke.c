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

/** \file choke.c
 * \author Alexey A. Shabelnikov
 * Implementation of carburetor choke control.
 */

// SM_CONTROL - control carburetor's choke
// FUEL_INJECT - control IAC using PWM
// SM_CONTROL & FUEL_INJECT - control IAC using stepper or PWM
#if defined(SM_CONTROL) || defined(FUEL_INJECT)

#include "port/port.h"
#include <stdlib.h>
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "funconv.h"
#include "magnitude.h"
#include "mathemat.h"
#include "smcontrol.h"
#include "pwrrelay.h"
#include "ventilator.h"
#include "etc.h"

#if defined(FUEL_INJECT) && !defined(AIRTEMP_SENS)
 #error "You can not use FUEL_INJECT option without AIRTEMP_SENS"
#endif

/**Direction used to set choke to the initial position */
#define INIT_POS_DIR SM_DIR_CW

#ifdef FUEL_INJECT

#ifdef SM_CONTROL

//See flags variable in choke_st_t
#define CF_POWERDOWN    0  //!< powerdown flag (used if power management is enabled)
#define CF_MAN_CNTR     1  //!< manual control mode flag
#define CF_SMDIR_CHG    2  //!< flag, indicates that stepper motor direction has changed during motion
#endif
#define CF_CL_LOOP      3  //!< IAC closed loop flag
#define CF_HOT_ENG      4  //!<
#define CF_INITFRQ      5  //!<  indicates that we are ready to set normal frequency after initialization and first movement
#define CF_REACH_TR     6  //!<

#else // Carburetor's choke stuff
//#define USE_RPMREG_TURNON_DELAY 1  //undefine this constant if you don't need delay

/**During this time system can't exit from RPM regulation mode*/
#define RPMREG_ENEX_TIME (10*100)

#ifdef USE_RPMREG_TURNON_DELAY
#define RPMREG_ENTO_TIME (3*100)
#endif

//See flags variable in choke_st_t
#define CF_POWERDOWN    0  //!< powerdown flag (used if power management is enabled)
#define CF_MAN_CNTR     1  //!< manual control mode flag
#define CF_RPMREG_ENEX  2  //!< flag which indicates that it is allowed to exit from RPM regulation mode
#define CF_SMDIR_CHG    3  //!< flag, indicates that stepper motor direction has changed during motion
#ifdef USE_RPMREG_TURNON_DELAY
#define CF_PRMREG_ENTO  4  //!< indicates that system is entered to RPM regulation mode
#endif
#define CF_INITFRQ      5  //!<  indicates that we are ready to set normal frequency after initialization and first movement

#endif //FUEL_INJECT

/**Define state variables*/
typedef struct
{
 uint8_t   state;          //!< state machine state
 uint16_t  smpos;          //!< current position of stepper motor in steps
 uint8_t   cur_dir;        //!< current value of SM direction (SM_DIR_CW or SM_DIR_CCW)
 int16_t   smpos_prev;     //!< start value of stepper motor position (before each motion)
 uint8_t   strt_mode;      //!< state machine state used for starting mode
 uint16_t  strt_t1;        //!< used for time calculations by calc_sm_position()
 uint8_t   flags;          //!< state flags (see CF_ definitions)
 uint16_t  rpmreg_t1;      //!< used to call RPM regulator function
 prev_temp_t prev_temp;    //!< used for inj_iac_pos_lookup()

#ifndef FUEL_INJECT
 int16_t   rpmreg_prev;    //!< previous value of RPM regulator
 uint16_t  rpmval_prev;    //!< used to store RPM value to detect exit from RPM regulation mode
 uint16_t  strt_t2;        //!< used for time calculations by calc_sm_position()
#endif

#ifdef FUEL_INJECT
 int16_t   prev_rpm_error[2];//!< previous value of closed-loop RPM error
 int16_t   iac_pos;        //!< IAC pos between calls of the closed loop regulator
 int16_t   iac_add;        //!< Smoothly increased value
 uint8_t   epas_offadded;  //!<
#endif

}choke_st_t;

/**Instance of state variables */
choke_st_t chks = {0,0,0,0,0,0,0,0,{0,0,0}
#ifndef FUEL_INJECT
                   ,0,0,0
#endif
#ifdef FUEL_INJECT
                   ,{0,0},0,0,0
#endif
                  };

void choke_init_ports(void)
{
#ifdef SM_CONTROL
 stpmot_init_ports();
#endif
#ifdef FUEL_INJECT
 IOCFG_INIT(IOP_IAC_PWM, 1); //1 - because of hardware invertor
#endif
}

/** Calculates choke position (%*2) from step value
 * \param value Position of choke in stepper motor steps
 * \param steps total number of steps
 * \return choke position in %*2
 */
uint8_t calc_percent_pos(uint16_t value, uint16_t steps)
{
 return (((uint32_t)value) * 200) / steps;
}

#ifndef FUEL_INJECT

/** Returns 1 if RPM regulator allowed 
 * Uses d ECU data structure
 * \return  1 - allowed, 0 - not allowed
 */
static uint8_t is_rpmreg_allowed(void)
{
 return !(d.sens.gas && CHECKBIT(d.param.choke_flags, CKF_OFFRPMREGONGAS));
}

/** Used by calc_sm_position() function
 * Uses d ECU data structure
 * \param regval Value of correction from RPM regulator to be added to result
 * \param mode 0 - use cranking lookup table, 1 - use work lookup table, 2 - interpolation between cranking and work positions (see time_since_crnk parameter)
 * \param time_since_crnk - time for interpolation mode in 10ms units
 * \return choke position in steps
 */
static int16_t choke_pos_final(int16_t regval, uint8_t mode, uint16_t time_since_crnk)
{
 if (CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE) && !d.floodclear)
 {
  uint16_t pos;
  if (mode < 2)
   pos = inj_iac_pos_lookup(&chks.prev_temp, mode); //cranking or work
  else
  { //use interpolation
   int16_t crnk_ppos = inj_iac_pos_lookup(&chks.prev_temp, 0); //crank pos
   int16_t run_ppos = inj_iac_pos_lookup(&chks.prev_temp, 1);  //run pos
   pos = simple_interpolation(time_since_crnk, crnk_ppos, run_ppos, 0, d.param.inj_cranktorun_time, 128) >> 7;
  }
  pos = (((uint32_t)pos) * inj_airtemp_corr(1)) >> 7;  //use raw MAT as argument to lookup table
  return ((((int32_t)d.param.sm_steps) * pos) / 200) + regval;
 }
 else
  return 0; //fully opened
}

/** Calculates choke position (for carburetor)
 * Work flow: Start-->Wait some time-->Cranking to work pos. transition-->RPM regul.-->Ready
 * Uses d ECU data structure
 * \return Correction value in SM steps
 */
int16_t calc_sm_position(void)
{
 int16_t rpm_corr = 0;
 uint16_t time_since_crnk = d.param.inj_cranktorun_time;

 switch(chks.strt_mode)
 {
  case 0:  //starting
   if (d.engine_mode!=EM_START)
   {
    chks.strt_t1 = s_timer_gtc();
    ++chks.strt_mode; // = 1
    //set choke RPM regulation flag (will be activated after delay)
    d.choke_rpm_reg = CHECKBIT(d.param.choke_flags, CKF_USECLRPMREG) && is_rpmreg_allowed();
   }
   break; //use startup correction
  case 1:
   if ((s_timer_gtc() - chks.strt_t1) >= choke_cranking_time())
   {
    ++chks.strt_mode; // = 2
    chks.strt_t1 = s_timer_gtc();     //set timer to prevent RPM regulation exiting during set period of time
   }
   break; //use startup correction

  case 2: //wait specified crank-to-run time and interpolate between crank and run positions
   {
    time_since_crnk = (s_timer_gtc() - chks.strt_t1); //update time value
    if ((time_since_crnk >= d.param.inj_cranktorun_time) || (d.sens.rpm <= inj_idling_rpm()))
    {
     ++chks.strt_mode;                 //transition has finished, we will immediately fall into mode 3, use run value
     chks.rpmreg_prev = 0;             //we will enter RPM regulation mode with zero correction
     chks.rpmval_prev = d.sens.rpm;
     chks.strt_t2 = s_timer_gtc();     //set timer to prevent RPM regulation exiting during set period of time
     chks.rpmreg_t1 = s_timer_gtc();
     chokerpm_regulator_init();
     CLEARBIT(chks.flags, CF_RPMREG_ENEX);
#ifdef USE_RPMREG_TURNON_DELAY
     CLEARBIT(chks.flags, CF_PRMREG_ENTO);
#endif
    }
    else
     return choke_pos_final(0, 2, time_since_crnk); //use interpolated value
   }

  case 3:
   time_since_crnk = (s_timer_gtc() - chks.strt_t1);  //update time value
   if (time_since_crnk >= d.param.inj_cranktorun_time)
    ++chks.strt_mode;

  case 4:
  {
   uint16_t tmr = s_timer_gtc();
   if ((tmr - chks.rpmreg_t1) >= PGM_GET_BYTE(&fw_data.exdata.iacreg_period))
   {
    chks.rpmreg_t1 = tmr;  //reset timer
    if ((tmr - chks.strt_t2) >= RPMREG_ENEX_TIME) //do we ready to enable RPM regulation mode exiting?
     SETBIT(chks.flags, CF_RPMREG_ENEX);
#ifdef USE_RPMREG_TURNON_DELAY
    if ((tmr - chks.strt_t2) >=  RPMREG_ENTO_TIME)
     SETBIT(chks.flags, CF_PRMREG_ENTO);
    if (CHECKBIT(chks.flags, CF_PRMREG_ENTO))
#endif
    rpm_corr = choke_rpm_regulator(&chks.rpmreg_prev);
    //detect fast throttle opening only if RPM > 1000
    if (d.sens.temperat >= (d.param.idlreg_turn_on_temp /*+ 1*/) ||
       (CHECKBIT(chks.flags, CF_RPMREG_ENEX) && (d.sens.rpm > 1000) && (((int16_t)d.sens.rpm - (int16_t)chks.rpmval_prev) > 180)))
    {
     chks.strt_mode = 5;    //exit from closed loop mode
     rpm_corr = 0;
     d.choke_rpm_reg = 0;
    }
    else
     chks.rpmval_prev = d.sens.rpm;
   }
   else
    rpm_corr = chks.rpmreg_prev;
  }

  if (!is_rpmreg_allowed()) //Is RPM regulator not allowed?
  {
   d.choke_rpm_reg = 0;     //always don't use regulator when fuel type is gas
   rpm_corr = 0;            //regulator's correction is zero
  }

  case 5:
   return choke_pos_final(rpm_corr, (time_since_crnk >= d.param.inj_cranktorun_time) ? 1 : 2, time_since_crnk); //work position + correction from RPM regulator
 }

 return choke_pos_final(0, 0, 0); //cranking position only
}
#endif

#ifdef SM_CONTROL
/** Set choke to initial position. Because we have no position feedback, we
 * must use number of steps more than stepper actually has.
 * Uses d ECU data structure
 * \param dir Direction (see description on stpmot_dir() function)
 */
static void initial_pos(uint8_t dir)
{
 stpmot_freq(CHECKBIT(d.param.choke_flags, CKF_MAXFREQINIT) ? 0 : d.param.sm_freq);
 CLEARBIT(chks.flags, CF_INITFRQ);
 stpmot_dir(dir);                                             //set direction
 if (0==d.sens.carb && CHECKBIT(d.param.choke_flags, CKF_USETHROTTLEPOS))
  stpmot_run(d.param.sm_steps >> 2);                         //run using number of steps = 25%
 else
  stpmot_run(d.param.sm_steps + (d.param.sm_steps >> 5));   //run using number of steps + 3%
}

/** Stepper motor control for normal working mode
 * Uses d ECU data structure
 * \param pos current calculated (target) position of stepper motor
 */
void sm_motion_control(int16_t pos)
{
 restrict_value_to(&pos, 0, d.param.sm_steps);
 if (CHECKBIT(chks.flags, CF_SMDIR_CHG))                      //direction has changed
 {
  if (!stpmot_is_busy())
  {
   chks.smpos = chks.smpos_prev + ((chks.cur_dir == SM_DIR_CW) ? -stpmot_stpcnt() : stpmot_stpcnt());
   CLEARBIT(chks.flags, CF_SMDIR_CHG);
  }
 }
 if (!CHECKBIT(chks.flags, CF_SMDIR_CHG))                     //normal operation
 {
  if (!stpmot_is_busy())
  {
   if (!CHECKBIT(chks.flags, CF_INITFRQ))                     //set normal frequency after first complete movement
    stpmot_freq(d.param.sm_freq);
   SETBIT(chks.flags, CF_INITFRQ);

   int16_t diff = pos - chks.smpos;
   if (diff != 0)
   {
    chks.cur_dir = diff < 0 ? SM_DIR_CW : SM_DIR_CCW;
    stpmot_dir(chks.cur_dir);
    _DELAY_US(2);                                             //ensure dir to step minimum delay
    stpmot_run(abs(diff));                                    //start stepper motor
    chks.smpos_prev = chks.smpos;                             //remember position when SM started motion
    chks.smpos = pos;                                         //this is a target position
   }
  }
  else //busy
  {
   //Check if curent target direction is not match new target direction. If it is not match, then
   //stop stepper motor and go to the direction changing.
   if (((chks.smpos - chks.smpos_prev) & 0x8000) != ((pos - chks.smpos_prev) & 0x8000))
   {
    stpmot_run(0);                                            //stop stepper motor
    SETBIT(chks.flags, CF_SMDIR_CHG);
   }
  }
 }
}
#endif //SM_CONTROL

#ifdef FUEL_INJECT

/**Calculates 1-st transition threshold*/
static uint16_t calc_rpm_thrd1(uint16_t rpm)
{
 return (((uint32_t)rpm) * (((uint16_t)d.param.idl_coef_thrd1) + 128)) >> 7;
}

/**calculates 2-nd transition threshold*/
static uint16_t calc_rpm_thrd2(uint16_t rpm)
{
 return (((uint32_t)rpm) * (((uint16_t)d.param.idl_coef_thrd2) + 128)) >> 7;
}

/** Checks is closed loop activated
 * \return 1 - CL is activated, 0 - CL is not activated
 */
static uint8_t is_cl_activated(void)
{
 return CHECKBIT(d.param.idl_flags, IRF_USE_INJREG) && (!d.sens.gas || CHECKBIT(d.param.idl_flags, IRF_USE_CLONGAS));
}

/**Calculates base position of IAC valve
 * \param run_pos 1 - use run position look up table, 0 - use cranking position look up table
 * \param use_mul 1 - multiply position by factor, 0 - don't multiply
 * \return position in % * 32
 */
int16_t get_base_position(uint8_t run_pos, uint8_t use_mul)
{
 int16_t pos = inj_iac_pos_lookup(&chks.prev_temp, run_pos) << 4; //x16
 if (use_mul)
  pos = (((int32_t)pos) * PGM_GET_WORD(&fw_data.exdata.iac_wrkadd_coeff)) >> 8;
 pos += (((int16_t)inj_iac_mat_corr()) << 3); //x4x8=x32
 return pos;
}

/** Checks if set thresholds are passed, so the system is ready to start CL
 * \return 1 - ready to start CL mode (set thresholds are passed), 0 - not ready
 */
static uint8_t is_ready_for_cl(void)
{
 if (d.sens.rpm > (((uint32_t)calc_cl_rpm() * PGM_GET_WORD(&fw_data.exdata.iac_clen_coeff)) >> 8))
  SETBIT(chks.flags, CF_REACH_TR);
 return (CHECKBIT(chks.flags, CF_REACH_TR) && (d.sens.rpm <= (((uint32_t)calc_cl_rpm() * PGM_GET_WORD(&fw_data.exdata.iac_clon_coeff)) >> 8)) && is_cl_activated());
}

/** PID core
 * \param rigidity Rigidity function
 * \param error Error for I-component
 * \param derror Error for P-component
 * \param dderror Error for D-component
 */
void do_pid(uint16_t rigidity, int16_t error, int16_t derror, int16_t dderror)
{
 uint16_t idl_reg_i = (error < 0) ? d.param.idl_reg_i[0] : d.param.idl_reg_i[1];
 uint16_t idl_reg_p = (derror < 0) ? d.param.idl_reg_p[0] : d.param.idl_reg_p[1];
 int32_t add = ((int32_t)rigidity * (((int32_t)derror * idl_reg_p) + ((int32_t)error * idl_reg_i) + ((int32_t)dderror * d.param.idl_reg_d)));
 int16_t cl_add = SHTDIV16(add, 8+7-2);
 restrict_value_to(&cl_add, -16383, 16383); //prevent orerflow
 chks.iac_pos += cl_add;
 restrict_value_to(&chks.iac_pos, -16383, 16383); //prevent overflow
}

/** Closed loop mode - call PID, enter and exit */
void do_closed_loop(void)
{
 //calculate target RPM and transition RPM thresholds
 int16_t rpm = calc_cl_rpm();
 int16_t rpmt = PGM_GET_BYTE(&fw_data.exdata.tmrpmtc_mode) ? inj_idling_rpm() : rpm;
 uint16_t rpm_thrd1 = calc_rpm_thrd1(rpmt), rpm_thrd2 = calc_rpm_thrd2(rpmt);
 int16_t error = rpm - d.sens.rpm, intlim = d.param.idl_intrpm_lim * 10;
 restrict_value_to(&error, -intlim, intlim); //limit maximum error (for P and I)

 if (!CHECKBIT(chks.flags, CF_CL_LOOP) && (d.engine_mode == EM_IDLE && d.sens.rpm < rpm_thrd1))
 {
  SETBIT(chks.flags, CF_CL_LOOP);   //enter closed loop, position of valve will be determined only by regulator
  chks.prev_rpm_error[0] = error;   //reset previous error
  chks.prev_rpm_error[1] = error;   //reset previous error
 }
 if (CHECKBIT(chks.flags, CF_CL_LOOP) && (d.engine_mode != EM_IDLE || d.sens.rpm > rpm_thrd2))
  CLEARBIT(chks.flags, CF_CL_LOOP); //exit closed loop, position of valve will be determined by maps

 if (CHECKBIT(chks.flags, CF_CL_LOOP))
 { //closed loop mode is active
  uint16_t rigidity = inj_idlreg_rigidity(d.param.idl_map_value, rpm);  //regulator's rigidity

  if (abs(error) > d.param.iac_reg_db)
  {
   d.iac_in_deadband = 0;
   int16_t derror = error - chks.prev_rpm_error[0];
   int16_t dderror = error - (2 * derror) + chks.prev_rpm_error[1];

   if (PGM_GET_BYTE(&fw_data.exdata.cold_eng_int))
   { //use special algorithm
    if (CHECKBIT(chks.flags, CF_HOT_ENG) || (d.sens.temperat >= (int16_t)PGM_GET_WORD(&fw_data.exdata.iacreg_turn_on_temp)) || (d.sens.rpm >= rpm))
    { //hot engine or RPM above or equal target idling RPM
     do_pid(rigidity, error, derror, dderror); 
     SETBIT(chks.flags, CF_HOT_ENG);
    }
    else
    { //cold engine, use only intergral part of requlator
     if ((error > 0) && (derror > 0)) //works only if errors are positive
      do_pid(rigidity, error, 0, 0); 
    }
   }
   else
   { //use regular alrorithm
    do_pid(rigidity, error, derror, dderror); 
   }
  }
  else
  {
   d.iac_in_deadband = 1;
  }

  chks.prev_rpm_error[1] = chks.prev_rpm_error[0];
  chks.prev_rpm_error[0] = error; //save for further calculation of derror
 }
 else
 { //closed loop is not active
  chks.iac_pos = get_base_position(1, 0); //use run pos as base, don't multiply
  if (d.engine_mode == EM_IDLE)
  {
   //if thrass_algo=0, then use old algorithm, if thrass_algo=1, then use new algorithm
   if  (d.sens.rpm > rpm_thrd2 && 0==PGM_GET_BYTE(&fw_data.exdata.thrass_algo))
   {
    //use throttle assist map or just a simple constant
    chks.iac_add = ((uint16_t)(CHECKBIT(d.param.idl_flags, IRF_USE_THRASSMAP) ? inj_iac_thrass() : d.param.idl_to_run_add)) << 4; //x16
   }
   else
   { //RPM between thrd1 and thrd2
    chks.iac_add-=PGM_GET_BYTE(&fw_data.exdata.idltorun_stp_en); //enter
    if (chks.iac_add < 0)
     chks.iac_add = 0;
   }
   chks.iac_pos+=chks.iac_add; //x16, work position + addition
  }
  else
  {
   if  (d.sens.rpm > rpm_thrd2)
   {
    chks.iac_add+=PGM_GET_BYTE(&fw_data.exdata.idltorun_stp_le); //leave
    //use throttle assist map or just a simple constant
    uint16_t max_add = ((uint16_t)(CHECKBIT(d.param.idl_flags, IRF_USE_THRASSMAP) ? inj_iac_thrass() : d.param.idl_to_run_add)) << 4; //x16
    if (chks.iac_add > max_add)
     chks.iac_add = max_add;
    chks.iac_pos+=chks.iac_add; //x16, work position + addition
   }
  }
 }
}

#ifndef SECU3T
/**Displace IAC position when EPAS turns on (one time displacement)*/
void epas_displace_cl(void)
{
 if (IOCFG_CHECK(IOP_EPAS_I))
 {
  if (!IOCFG_GET(IOP_EPAS_I))
  {
   if (!chks.epas_offadded)
   {
    chks.iac_pos+=((uint16_t)PGM_GET_BYTE(&fw_data.exdata.epas_iacoff)) << 4;
    chks.epas_offadded = 1;
   }
  }
  else
  {
   chks.epas_offadded = 0; //allow displacement again
  }
 }
}
#endif

/** Calculate stepper motor position for normal mode (fuel injection)
 * Uses d ECU data structure
 * \param pwm 1 - PWM IAC, 0 - SM IAC
 * \return stepper motor position in steps
 */
int16_t calc_sm_position(uint8_t pwm)
{
 int16_t iac_pos_o = 0;
 switch(chks.strt_mode)
 {
  case 0:  //cranking mode
   chks.iac_pos = get_base_position(0, 0); //use ckank pos, don't multiply

   CLEARBIT(chks.flags, CF_CL_LOOP); //closed loop is not active
   if (d.engine_mode!=EM_START)
   {
    chks.iac_add = 0;
    CLEARBIT(chks.flags, CF_HOT_ENG);
    chks.strt_t1 = s_timer_gtc();
    ++chks.strt_mode; //next state = 1
    chks.rpmreg_t1 = s_timer_gtc();
    CLEARBIT(chks.flags, CF_REACH_TR);
   }
   break;

  case 1: //wait specified crank-to-run time and interpolate between crank and run positions
   {
    uint16_t time_since_crnk = (s_timer_gtc() - chks.strt_t1);
    if (time_since_crnk >= inj_cranktorun_time())
    {
     chks.strt_mode = 2; //transition has finished, we will immediately fall into mode 2, use run value
     chks.iac_pos = get_base_position(1, 1); //use run pos, multiply
     chks.strt_t1 = s_timer_gtc();
    }
    else
    {
     int16_t crnk_ppos = get_base_position(0, 0); //use ckank pos, don't multiply
     int16_t run_ppos = get_base_position(1, 1); //use run pos, multiply
     chks.iac_pos = simple_interpolation(time_since_crnk, crnk_ppos, run_ppos, 0, inj_cranktorun_time(), 4) >> 2; //result will be x32

     if (is_ready_for_cl())
     {
      chks.strt_mode = 3; //allow closed loop before finishing crank to run transition (abort transition and start closed loop immediately)
      goto clic_imm;
     }
     else
      break;              //use interpolated value (still in mode 1)
    }
   }

  case 2: //run mode with addiditon to position
    if ((s_timer_gtc() - chks.strt_t1) >= PGM_GET_WORD(&fw_data.exdata.iac_wrkadd_time) || is_ready_for_cl())
    {
     chks.strt_mode = 3;  //we immediately fall into mode 3
     chks.iac_pos = get_base_position(1, 0); //use run pos, don't multiply
    }
    else
    {
     chks.iac_pos = get_base_position(1, 1); //use run pos, multiply
     break;
    }

  case 3: //run mode
clic_imm:
   if (is_cl_activated()) //use closed loop on gas fuel only if it is enabled by corresponding flag
   { //closed loop mode
    uint16_t tmr = s_timer_gtc();
    if ((tmr - chks.rpmreg_t1) < PGM_GET_BYTE(&fw_data.exdata.iacreg_period))
     goto skip_pid; //not time to call regulator, exit
    chks.rpmreg_t1 = tmr;  //reset timer

    do_closed_loop();

    //Displace IAC position when cooling fan turns on (one time only)
    if (d.vent_req_on)
    {
     chks.iac_pos+=((uint16_t)PGM_GET_BYTE(&fw_data.exdata.vent_iacoff)) << 4;
     d.vent_req_on = 0;
    }

    //Displace IAC position when EPAS turns on (one time displacement).
    //Displacement will take place only if EPAS_I is not reassigned to other function and EPAS_I = 0
    #ifndef SECU3T
    epas_displace_cl();
    #endif

skip_pid:
    {
    uint16_t idl_iacminpos = d.param.idl_iacminpos;
    #ifdef AIRCONDIT
    #ifdef SECU3T
    if (IOCFG_GET(IOP_COND_I))
    #else
    if (d.cond_state)
    #endif
     idl_iacminpos+=PGM_GET_BYTE(&fw_data.exdata.iac_cond_add);
    #endif

    //Restrict IAC position using specified limits
    iac_pos_o = chks.iac_pos;
    restrict_value_to(&iac_pos_o, (idl_iacminpos) << 4, ((uint16_t)d.param.idl_iacmaxpos) << 4);
    }
    goto already_restricted;
   }
   else
   { //open loop mode
    CLEARBIT(chks.flags, CF_CL_LOOP);
    chks.iac_pos = get_base_position(1, 0); //use run pos, don't multiply

    //Displace IAC position when cooling fan turns on
    if (d.vent_req_on)
     chks.iac_pos+=((uint16_t)PGM_GET_BYTE(&fw_data.exdata.vent_iacoff)) << 4;
    //Displace IAC position when EPAS turns on
    #ifndef SECU3T
    if (IOCFG_CHECK(IOP_EPAS_I) && !IOCFG_GET(IOP_EPAS_I))
     chks.iac_pos+=((uint16_t)PGM_GET_BYTE(&fw_data.exdata.epas_iacoff)) << 4;
    #endif
   }
   break;
 }

 //Restrict IAC position
 iac_pos_o = chks.iac_pos;
 restrict_value_to(&iac_pos_o, 0, 3200);

already_restricted:
 d.iac_closed_loop = !!CHECKBIT(chks.flags, CF_CL_LOOP);

 if (d.floodclear)
  return 0; //0% (use d.param.sm_steps for 100%)
 else
 {
  if (1==pwm) //PWM IAC
   return ((((int32_t)4095) * iac_pos_o) / 3200); //convert percentage position to PWM duty
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
  else if (2==pwm) //ETC
   return ((((int32_t)d.param.etc_idleadd_max) * iac_pos_o) / 3200); //convert percentage position to PWM duty
#endif
  else
   return ((((int32_t)d.param.sm_steps) * iac_pos_o) / 3200); //convert percentage position to SM steps
 }
}
#endif

void choke_control(void)
{
#ifdef FUEL_INJECT
 if (!CHECKBIT(chks.flags, CF_CL_LOOP))
  d.corr.rigid_arg = 255;                                    //default value (means not used), actual value will be set in the calc_sm_position() function
 if (IOCFG_CHECK(IOP_IAC_PWM))
 { //use PWM IAC
  uint16_t  pos = calc_sm_position(1);                       //calculate PWM duty
  if (pos > 4095) pos = 4095;
  d.choke_pos = calc_percent_pos(pos, 4095);                 //update position value
  pos = (((uint32_t)pos) * pwmiac_ucoef()) >> 12;
  if (pos > 4095) pos = 4095;
  vent_set_duty12(pos);
  chks.state = 5; //set for proper operation of choke_is_ready()
  return;
 }
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 else if (etc_is_enabled())
 {
  uint16_t pos = calc_sm_position(2);                        //calculate position addition for ETC
  if (d.param.etc_idleadd_max > 0)                           //prevent division by zero
   d.choke_pos = calc_percent_pos(pos, d.param.etc_idleadd_max);//update position value
  else
   d.choke_pos = 0;
  d.etc_idleadd = pos;
  chks.state = 5; //set for proper operation of choke_is_ready()
  return;
 }
#endif
#endif

 if (!IOCFG_CHECK(IOP_SM_STP))
  return;                                                     //stepper motor control is not enabled: do nothing

#ifdef SM_CONTROL
 switch(chks.state)
 {
  case 0:                                                     //Initialization of choke position
   if (!IOCFG_CHECK(IOP_PWRRELAY))                            //Skip initialization if power management is enabled
    initial_pos(INIT_POS_DIR);
   chks.state = 2;
   inj_init_prev_clt(&chks.prev_temp);
   break;

  case 1:                                                     //system is being preparing to power-down
   initial_pos(INIT_POS_DIR);
   chks.state = 2;
   break;

  case 2:                                                     //wait while choke is being initialized
   if (!stpmot_is_busy())                                     //ready?
   {
    if (CHECKBIT(chks.flags, CF_POWERDOWN))
     chks.state = 3;                                          //ready to power-down
    else
     chks.state = 5;                                          //normal working
    chks.smpos = 0;                                           //initial position (fully opened)
    d.choke_pos = calc_percent_pos(chks.smpos, d.param.sm_steps);//update position value
    CLEARBIT(chks.flags, CF_SMDIR_CHG);
   }
   break;

  case 3:                                                     //power-down
   if (pwrrelay_get_state())
   {
    CLEARBIT(chks.flags, CF_POWERDOWN);
    chks.state = 5;
   }
   break;

  case 5:                                                     //normal working mode
   if (d.choke_testing)
   {
    initial_pos(INIT_POS_DIR);
    chks.state = 6;                                           //start testing
   }
   else
   {
    int16_t pos;
    if (!CHECKBIT(chks.flags, CF_MAN_CNTR))
    {
#ifdef FUEL_INJECT
     pos = calc_sm_position(0);                               //calculate stepper motor position
#else
     pos = calc_sm_position();                                //calculate stepper motor position
#endif
     if (d.choke_manpos_d)
      SETBIT(chks.flags, CF_MAN_CNTR); //enter manual mode
    }
    else
    { //manual control
     pos = chks.smpos + d.choke_manpos_d;
     d.choke_manpos_d = 0;
    }

    sm_motion_control(pos);                                //SM command execution
   }
   d.choke_pos = calc_percent_pos(chks.smpos, d.param.sm_steps);//update position value
   goto check_pwr;

  //     Testing modes
  case 6:                                                     //initialization of choke
   if (!stpmot_is_busy())                                     //ready?
   {
    d.choke_pos = 0;//update position value
    stpmot_freq(d.param.sm_freq);
    stpmot_dir(SM_DIR_CCW);
    stpmot_run(d.param.sm_steps);
    chks.state = 7;
   }
   goto check_tst;

  case 7:
   if (!stpmot_is_busy())                                     //ready?
   {
    d.choke_pos = 200;//update position value
    stpmot_freq(d.param.sm_freq);
    stpmot_dir(SM_DIR_CW);
    stpmot_run(d.param.sm_steps);
    chks.state = 6;
   }
   goto check_tst;

  default:
  check_tst:
   if (!d.choke_testing)
    chks.state = 1;                                           //exit choke testing mode
  check_pwr:
   if (!pwrrelay_get_state())
   {                                                          //power-down
    SETBIT(chks.flags, CF_POWERDOWN);
    chks.state = 1;
   }
   break;
 }
#endif //SM_CONTROL
}

void choke_eng_stopped_notification(void)
{
 chks.strt_mode = 0; //engine is stopped, so, go into the cranking mode again
}

#ifdef SM_CONTROL
uint8_t choke_is_ready(void)
{
 return (chks.state == 5 || chks.state == 3) || !IOCFG_CHECK(IOP_SM_STP);
}

void choke_init_motor(void)
{
 initial_pos(INIT_POS_DIR);
}
#endif

#endif //SM_CONTROL || FUEL_INJECT
