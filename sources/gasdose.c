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

/** \file gasdose.c
 * \author Alexey A. Shabelnikov
 * Implementation of gas dose controller (stepper motor).
 */

#ifdef GD_CONTROL

#include "port/port.h"
#include "port/pgmspace.h"
#include <stdlib.h>
#include "bitmask.h"
#include "ecudata.h"
#include "funconv.h"
#include "gdcontrol.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "mathemat.h"
#include "pwrrelay.h"
#include "ventilator.h"
#include "eculogic.h"

/**Direction used to set stepper motor to the initial position */
#define INIT_POS_DIR SM_DIR_CW

//See flags variable in gasdose_st_t
#define CF_POWERDOWN    0  //!< powerdown flag (used if power management is enabled)
#define CF_MAN_CNTR     1  //!< manual control mode flag
#define CF_SMDIR_CHG    3  //!< flag, indicates that stepper motor direction has changed during motion

/**Define state variables*/
typedef struct
{
 uint8_t   state;          //!< state machine state
 uint16_t  smpos;          //!< current position of stepper motor in steps
 uint8_t   cur_dir;        //!< current value of SM direction (SM_DIR_CW or SM_DIR_CCW)
 int16_t   smpos_prev;     //!< start value of stepper motor position (before each motion)
 uint8_t   flags;          //!< state flags (see CF_ definitions)
 uint8_t   acc_strokes;    //!< strokes counter for acceleration enrichment
 uint16_t  aftstr_enrich_counter; //!< Stroke counter used in afterstart enrichment
}gasdose_st_t;

/**Instance of state variables */
static gasdose_st_t gds = {0,0,0,0,0,0,0};

/** Calculates AE value for gas doser
 * Uses d ECU data structure
 * \return AE value in %
 */
static int16_t calc_gd_acc_enrich(void)
{
 int32_t gdnc = GD_MAGNITUDE(100.0);               //normal conditions %
 int16_t aef = inj_ae_tps_lookup();                //calculate basic AE factor value

//------------------------------
 int16_t int_m_thrd = d.param.inj_lambda_swt_point + d.param.inj_lambda_dead_band;
 int16_t int_p_thrd = ((int16_t)d.param.inj_lambda_swt_point) - d.param.inj_lambda_dead_band;
 if (int_p_thrd < 0)
  int_p_thrd = 0;

 if (((d.sens.tpsdot > d.param.inj_ae_tpsdot_thrd) && (d.sens.add_i1 < int_m_thrd)) ||
     ((d.sens.tpsdot < (-d.param.inj_ae_tpsdot_thrd)) && (d.sens.add_i1 > int_p_thrd)))
 {
  d.acceleration  = 1;
  gds.acc_strokes = 5; //init acceleration strokes counter
 }

 if (((d.sens.tpsdot < d.param.inj_ae_tpsdot_thrd) && ((d.sens.add_i1 > int_m_thrd) || (gds.acc_strokes == 0))) ||
     ((d.sens.tpsdot > (-d.param.inj_ae_tpsdot_thrd)) && ((d.sens.add_i1 < int_p_thrd) || (gds.acc_strokes == 0))))
 {
  d.acceleration = 0;
 }

 if (!d.acceleration)
  return 0; //no acceleration enrichment
//------------------------------

/*
 if (abs(d.sens.tpsdot) < d.param.inj_ae_tpsdot_thrd) {
  d.acceleration = 0;
  return 0;                                        //no acceleration or deceleration
 }
 d.acceleration = 1;
*/


 //For now we don't use CLT correction factor
/*
 aef = ((int32_t)aef * inj_ae_clt_corr()) >> 7;    //apply CLT correction factor to AE factor
*/
 aef = ((int32_t)aef * inj_ae_rpm_lookup()) >> 7;  //apply RPM correction factor to AE factor

 return (gdnc * aef) >> 7;                         //apply AE factor to the normal conditions
}

//=============================================================================================================================


void gasdose_init_ports(void)
{
 gdstpmot_init_ports();
}

/** Calculates actuator position (%*2) from step value
 * \param value Position in stepper motor steps
 * \param steps total number of steps
 * \return actuator position in %*2
 */
static uint8_t calc_percent_pos(uint16_t value, uint16_t steps)
{
 return (((uint32_t)value) * GD_MAGNITUDE(100.0)) / steps;
}


/** Set stepper motor to initial position. Because we have no position feedback, we
 * must use number of steps more than stepper actually has.
 * Uses d ECU data structure
 * \param dir Direction (see description on gdstpmot_dir() function)
 */
static void initial_pos(uint8_t dir)
{
 gdstpmot_freq(d.param.gd_maxfreqinit ? 0 : d.param.gd_freq);
 gdstpmot_dir(dir);                                           //set direction
 gdstpmot_run(d.param.gd_steps + (d.param.gd_steps >> 4));  //run using number of steps + 6%
}

//TODO: redundant to simular function in choke.c, remove this redundant copy in the future
/** Stepper motor control for normal working mode
 * Uses d ECU data structure
 * \param pos current calculated (target) position of stepper motor
 */
static void sm_motion_control(int16_t pos)
{
 int16_t diff;
 restrict_value_to(&pos, 0, d.param.gd_steps);
 if (CHECKBIT(gds.flags, CF_SMDIR_CHG))                       //direction has changed
 {
  if (!gdstpmot_is_busy())
  {
   gds.smpos = gds.smpos_prev + ((gds.cur_dir == SM_DIR_CW) ? -gdstpmot_stpcnt() : gdstpmot_stpcnt());
   CLEARBIT(gds.flags, CF_SMDIR_CHG);
  }
 }
 if (!CHECKBIT(gds.flags, CF_SMDIR_CHG))                      //normal operation
 {
  diff = pos - gds.smpos;
  if (!gdstpmot_is_busy())
  {
   if (diff != 0)
   {
    gds.cur_dir = diff < 0 ? SM_DIR_CW : SM_DIR_CCW;
    gdstpmot_dir(gds.cur_dir);
    gdstpmot_run(abs(diff));                                  //start stepper motor
    gds.smpos_prev = gds.smpos;                               //remember position when SM started motion
    gds.smpos = pos;                                          //this is a target position
   }
  }
  else //busy
  {
   //Check if curent target direction is not match new target direction. If it is not match, then
   //stop stepper motor and go to the direction changing.
   if (((gds.smpos - gds.smpos_prev) & 0x8000) != ((pos - gds.smpos_prev) & 0x8000))
   {
    gdstpmot_run(0);                                          //stop stepper motor
    SETBIT(gds.flags, CF_SMDIR_CHG);
   }
  }
 }
}

/** Calculate stepper motor position for normal mode
 * Uses d ECU data structure
 * \param pwm 0 - stepper valve, 1 - PWM valve
 * \return stepper motor position in steps or PWM duty (depending on pwm input parameter)
 */
static int16_t calc_sm_position(uint8_t pwm)
{
 int16_t corr, pos;

 if (d.engine_mode==EM_START)
 {
  gds.aftstr_enrich_counter = d.param.inj_aftstr_strokes << 1; //init engine strokes counter

  uint16_t pw = inj_cranking_pw();    //use Cranking PW injection map to define GD position on cranking
  if (pw > 3125)
   pw = 3125; //10ms max.

  //convert from 3.2uS timer ticks to %, 3125(10ms) corresponds to 100%
  pos = (((int32_t)pw) * 1049) >> 14; //1049 = (GD_MAGNITUDE(100.0) / 3125) * 16384

  goto cranking_pos;
 }

 pos = gdp_function(); //basic position, value in %

 //apply correction from VE and AFR maps
 pos = (((int32_t)pos) * gd_ve_afr()) >> 11;

 //apply warmup enrichemnt factor
 pos = (((int32_t)pos) * inj_warmup_en()) >> 7;

 //apply correction from IAT sensor, use approximation instead of division
 //pos = (((int32_t)pos) * TEMPERATURE_MAGNITUDE(273.15)) / (d.sens.air_temp + TEMPERATURE_MAGNITUDE(273.15));
 //int16_t corr = (d.sens.air_temp < TEMPERATURE_MAGNITUDE(20)) ? TEMPERATURE_MAGNITUDE(4116) - (d.sens.air_temp * 17) : TEMPERATURE_MAGNITUDE(3990) - (d.sens.air_temp * 10); //my
 corr = (d.sens.air_temp < TEMPERATURE_MAGNITUDE(30)) ? TEMPERATURE_MAGNITUDE(4110) - (d.sens.air_temp * 15) : TEMPERATURE_MAGNITUDE(3970) - (d.sens.air_temp * 10);   //alvikagal
 pos = ((int32_t)pos * (corr)) >> 14;

 if (d.sens.gas) //gas valve will function when petrol is used, but in very limited mode
 {
  //pos = (((int32_t)pos) * (512 + d.corr.lambda)) >> 9; //apply EGO correction
  pos = pos + ((GD_MAGNITUDE(100.0) * d.corr.lambda) >> 9); //proposed by alvikagal
  if (pos > GD_MAGNITUDE(100.0))
   pos = GD_MAGNITUDE(100.0);

  pos+= calc_gd_acc_enrich();    //apply acceleration enrichment

  if (d.engine_mode == EM_START)
   gds.aftstr_enrich_counter = d.param.inj_aftstr_strokes << 1; //init engine strokes counter
  else if (gds.aftstr_enrich_counter)
   pos= ((int32_t)pos * (128 + scale_aftstr_enrich(gds.aftstr_enrich_counter))) >> 7; //apply scaled afterstart enrichment factor

  pos = pos - (d.ie_valve ? 0 : d.param.gd_fc_closing); //apply fuel cut flag

  pos = (d.fc_revlim ? 0 : pos); //apply rev.limit flag
 }

cranking_pos:

 if (pwm)
  return ((((int32_t)256) * pos) / 200); //convert percentage position to PWM duty
 else
  return ((((int32_t)d.param.gd_steps) * pos) / GD_MAGNITUDE(100.0)); //finally, convert from % to SM steps
}

void gasdose_control(void)
{
 if (IOCFG_CHECK(IOP_GD_PWM))
 { //use PWM valve
  uint16_t  pos = calc_sm_position(1);                       //calculate PWM duty
  if (pos > 255) pos = 255;
  d.gasdose_pos = calc_percent_pos(pos, 256);                //update position value
  vent_set_duty8(pos);
  return;
 }

 if (!IOCFG_CHECK(IOP_GD_STP))
  return; //gas dose control was not enabled (outputs were not remapped)

 switch(gds.state)
 {
  case 0:                                                     //Initialization of stepper motor position
   if (!IOCFG_CHECK(IOP_PWRRELAY))                            //Skip initialization if power management is enabled
    initial_pos(INIT_POS_DIR);
   gds.state = 2;
   break;

  case 1:                                                     //system is being preparing to power-down
   initial_pos(INIT_POS_DIR);
   gds.state = 2;
   break;

  case 2:                                                     //wait while stepper motor is being initialized
   if (!gdstpmot_is_busy())                                   //ready?
   {
    if (CHECKBIT(gds.flags, CF_POWERDOWN))
     gds.state = 3;                                           //ready to power-down
    else
     gds.state = 5;                                           //normal working
    gds.smpos = 0;                                            //initial position (fully opened)
    CLEARBIT(gds.flags, CF_SMDIR_CHG);
   }
   break;

  case 3:                                                     //power-down
   if (pwrrelay_get_state())
   {
    CLEARBIT(gds.flags, CF_POWERDOWN);
    gds.state = 5;
   }
   break;

  case 5:                                                     //normal working mode
   if (d.gasdose_testing)
   {
    initial_pos(INIT_POS_DIR);
    gds.state = 6;                                            //start testing
   }
   else
   {
    int16_t pos;
    if (!CHECKBIT(gds.flags, CF_MAN_CNTR))
    {
     pos = calc_sm_position(0);                               //calculate stepper motor position
     if (d.gasdose_manpos_d)
      SETBIT(gds.flags, CF_MAN_CNTR);                         //enter manual mode
    }
    else
    { //manual control
     pos = gds.smpos + d.gasdose_manpos_d;
     d.gasdose_manpos_d = 0;
    }

    gdstpmot_freq(d.param.gd_freq);
    sm_motion_control(pos);                                   //SM command execution
   }
   d.gasdose_pos = calc_percent_pos(gds.smpos, d.param.gd_steps);//update position value
   goto check_pwr;

  //     Testing modes
  case 6:                                                     //initialization of stepper motor
   if (!gdstpmot_is_busy())                                   //ready?
   {
    d.gasdose_pos = GD_MAGNITUDE(0);                          //update position value
    gdstpmot_freq(d.param.gd_freq);
    gdstpmot_dir(SM_DIR_CCW);
    gdstpmot_run(d.param.gd_steps);
    gds.state = 7;
   }
   goto check_tst;

  case 7:
   if (!gdstpmot_is_busy())                                   //ready?
   {
    d.gasdose_pos = GD_MAGNITUDE(100.0);                     //update position value
    gdstpmot_freq(d.param.gd_freq);
    gdstpmot_dir(SM_DIR_CW);
    gdstpmot_run(d.param.gd_steps);
    gds.state = 6;
   }
   goto check_tst;

  default:
  check_tst:
   if (!d.gasdose_testing)
    gds.state = 1;                                            //exit stepper motor testing mode
  check_pwr:
   if (!pwrrelay_get_state())
   {                                                          //power-down
    SETBIT(gds.flags, CF_POWERDOWN);
    gds.state = 1;
   }
   break;
 }
}

uint8_t gasdose_is_ready(void)
{
 return (gds.state == 5 || gds.state == 3) || !IOCFG_CHECK(IOP_GD_STP);
}

void gasdose_stroke_event_notification(void)
{
 if (gds.acc_strokes)
  --gds.acc_strokes;

 //update afterstart enrichemnt counter
 if (gds.aftstr_enrich_counter)
  --gds.aftstr_enrich_counter;
}

void gasdose_init_motor(void)
{
 initial_pos(INIT_POS_DIR);
}

#endif //GD_CONTROL
