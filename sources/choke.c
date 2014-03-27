/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

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
 * Implementation of carburetor choke control.
 * (Реализация управления воздушной заслонкой карбюратора).
 */

#ifdef SM_CONTROL

#include "port/port.h"
#include <stdlib.h>
#include "adc.h"
#include "ioconfig.h"
#include "funconv.h"
#include "magnitude.h"
#include "secu3.h"
#include "smcontrol.h"
#include "pwrrelay.h"

#define USE_THROTTLE_POS 1        //undefine this constant if you don't need to use throttle limit switch in choke initialization
#define USE_RPMREG_TURNON_DELAY 1  //undefine this constant if you don't need delay

/**Direction used to set choke to the initial position */
#define INIT_POS_DIR SM_DIR_CW

/**RPM regulator call period, 50ms*/
#define RPMREG_CORR_TIME 10

/**During this time system can't exit from RPM regulation mode*/
#define RPMREG_ENEX_TIME (10*100)

#ifdef USE_RPMREG_TURNON_DELAY
#define RPMREG_ENTO_TIME (3*100)
#endif

/**Define state variables*/
typedef struct
{
 uint8_t   state;          //!< state machine state
 uint8_t   pwdn;           //!< powerdown flag (used if power management is enabled)
 uint16_t  smpos;          //!< current position of stepper motor in steps
 int16_t   prev_temp;      //!< used for choke_closing_lookup()
 uint8_t   manual;         //!< manual control mode

 uint8_t   strt_mode;      //!< state machine state used for starting mode
 uint16_t  strt_t1;        //!< used for time calculations by calc_startup_corr()

 int16_t   rpmreg_prev;    //!< previous value of RPM regulator
 uint16_t  rpmreg_t1;      //!< used to call RPM regulator function
 uint16_t  rpmval_prev;    //!< used to store RPM value to detect exit from RPM regulation mode
 uint8_t   rpmreg_enex;    //!< flag which indicates that it is allowed to exit from RPM regulation mode
 uint8_t   smdirchg;       //!< flag, indicates that stepper motor direction has changed during motion
 uint8_t   cur_dir;        //!< current value of SM direction (SM_DIR_CW or SM_DIR_CCW)
 int16_t   smpos_prev;     //!< start value of stepper motor position (before each motion)
#ifdef USE_RPMREG_TURNON_DELAY
 uint8_t   rpmreg_ento;
#endif
}choke_st_t;

/**Instance of state variables */
choke_st_t chks = {0};

void choke_init_ports(void)
{
 stpmot_init_ports();
}

void choke_init(void)
{
 stpmot_init();
 chks.state = 0;
 chks.pwdn = 0;
 chks.strt_mode = 0;
 chks.manual = 0;
 chks.rpmreg_prev = 0;
 chks.rpmreg_enex = 0;
}

/** Calculates choke position correction at startup mode and from RPM regulator
 * Work flow: Start-->Wait 3 sec.-->RPM regul.-->Ready
 * \param d pointer to ECU data structure
 */
int16_t calc_startup_corr(struct ecudata_t* d)
{
 int16_t rpm_corr = 0;
 switch(chks.strt_mode)
 {
  case 0:  //starting
   if (d->st_block)
   {
    chks.strt_t1 = s_timer_gtc();
    chks.strt_mode = 1;
   }
   break; //use startup correction
  case 1:
   if ((s_timer_gtc() - chks.strt_t1) >= d->param.choke_corr_time)
   {
    chks.strt_mode = 2;
    chks.rpmreg_prev = 0; //we will enter RPM regulation mode with zero correction
    chks.rpmval_prev = d->sens.frequen;
    chks.strt_t1 = s_timer_gtc();     //set timer to prevent RPM regulation exiting during set period of time
    chks.rpmreg_t1 = s_timer_gtc();
    chokerpm_regulator_init();
    chks.rpmreg_enex = 0;
#ifdef USE_RPMREG_TURNON_DELAY
    chks.rpmreg_ento = 0;
#endif
    //set choke RPM regulation flag
    d->choke_rpm_reg = (0!=d->param.choke_rpm[0]);
   }
   break; //use startup correction
  case 2:
  {
   uint16_t tmr = s_timer_gtc();
   if ((tmr - chks.rpmreg_t1) >= RPMREG_CORR_TIME)
   {
    chks.rpmreg_t1 = tmr;  //reset timer
    if ((tmr - chks.strt_t1) >= RPMREG_ENEX_TIME) //do we ready to enable RPM regulation mode exiting?
     chks.rpmreg_enex = 1;
#ifdef USE_RPMREG_TURNON_DELAY
    if ((tmr - chks.strt_t1) >=  RPMREG_ENTO_TIME)
     chks.rpmreg_ento = 1;
    if (chks.rpmreg_ento)
#endif
    rpm_corr = choke_rpm_regulator(d, &chks.rpmreg_prev);
    //detect fast throttle opening only if RPM > 1000
    if (d->sens.temperat > (d->param.idlreg_turn_on_temp + 1) ||
       (chks.rpmreg_enex && (d->sens.frequen > 1000) && (((int16_t)d->sens.frequen - (int16_t)chks.rpmval_prev) > 180)))
    {
     chks.strt_mode = 3; //exit
     rpm_corr = 0;
     d->choke_rpm_reg = 0;
    }
    else
     chks.rpmval_prev = d->sens.frequen;
   }
   else
    rpm_corr = chks.rpmreg_prev;
  }
  case 3:
   if (!d->st_block)
    chks.strt_mode = 0; //engine is stopped, so use correction again
   return rpm_corr;  //correction from RPM regulator only
 }

 if (d->sens.temperat > d->param.choke_corr_temp)
  return 0; //Do not use correction if coolant temperature > threshold
 else if (d->sens.temperat < TEMPERATURE_MAGNITUDE(0))
  return d->param.sm_steps; //if temperature  < 0, then choke must be fully closed
 else
  return (((int32_t)d->param.sm_steps) * d->param.choke_startup_corr) / 200;
}

/** Set choke to initial position. Because we have no position feedback, we
 * must use number of steps more than stepper actually has.
 * \param d pointer to ECU data structure
 * \param dir Direction (see description on stpmot_dir() function)
 */
static void initial_pos(struct ecudata_t* d, uint8_t dir)
{
 stpmot_dir(dir);                                             //set direction
#ifdef USE_THROTTLE_POS
 if (0==d->sens.carb)
  stpmot_run(d->param.sm_steps >> 2);                         //run using number of steps = 25%
 else
#endif
  stpmot_run(d->param.sm_steps + (d->param.sm_steps >> 5));   //run using number of steps + 3%
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

/** Stepper motor control for normal working mode
 * \param d pointer to ECU data structure
 * \param pos current calculated (target) position of stepper motor
 */
void sm_motion_control(struct ecudata_t* d, int16_t pos)
{
 int16_t diff;
 restrict_value_to(&pos, 0, d->param.sm_steps);
 if (1==chks.smdirchg)                                        //direction has changed
 {
  if (!stpmot_is_busy())
  {
   chks.smpos = chks.smpos_prev + ((chks.cur_dir == SM_DIR_CW) ? -stpmot_stpcnt() : stpmot_stpcnt());
   chks.smdirchg = 0;
  }
 }
 if (0==chks.smdirchg)                                        //normal operation
 {
  diff = pos - chks.smpos;
  if (!stpmot_is_busy())
  {
   if (diff != 0)
   {
    chks.cur_dir = diff < 0 ? SM_DIR_CW : SM_DIR_CCW;
    stpmot_dir(chks.cur_dir);
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
    chks.smdirchg = 1;
   }
  }
 }
}

/** Calculate stepper motor position for normal mode
 * \param d pointer to ECU data structure
 * \return stepper motor position in steps
 */
int16_t calc_sm_position(struct ecudata_t* d)
{
 return ((((int32_t)d->param.sm_steps) * choke_closing_lookup(d, &chks.prev_temp)) / 200) + calc_startup_corr(d);
}

void choke_control(struct ecudata_t* d)
{
 switch(chks.state)
 {
  case 0:                                                     //Initialization of choke position
   if (!IOCFG_CHECK(IOP_PWRRELAY))                            //Skip initialization if power management is enabled
    initial_pos(d, INIT_POS_DIR);
   chks.state = 2;
   chks.prev_temp = d->sens.temperat;
   break;

  case 1:                                                     //system is being preparing to power-down
   initial_pos(d, INIT_POS_DIR);
   chks.state = 2;
   break;

  case 2:                                                     //wait while choke is being initialized
   if (!stpmot_is_busy())                                     //ready?
   {
    if (chks.pwdn)
     chks.state = 3;                                          //ready to power-down
    else
     chks.state = 5;                                          //normal working
    chks.smpos = 0;                                           //initial position (fully opened)
    chks.smdirchg = 0;
   }
   break;

  case 3:                                                     //power-down
   if (pwrrelay_get_state())
   {
    chks.pwdn = 0;
    chks.state = 5;
   }
   break;

  case 5:                                                     //normal working mode
   if (d->choke_testing)
   {
    initial_pos(d, INIT_POS_DIR);
    chks.state = 6;                                           //start testing
   }
   else
   {
    int16_t pos;
    if (!chks.manual)
    {
     pos = calc_sm_position(d);                               //calculate stepper motor position
     if (d->choke_manpos_d)
      chks.manual = 1; //enter manual mode
    }
    else
    { //manual control
     pos = chks.smpos + d->choke_manpos_d;
     d->choke_manpos_d = 0;
    }

    sm_motion_control(d, pos);                                //SM command execution
   }
   d->choke_pos = calc_percent_pos(chks.smpos, d->param.sm_steps);//update position value
   goto check_pwr;

  //     Testing modes
  case 6:                                                     //initialization of choke
   if (!stpmot_is_busy())                                     //ready?
   {
    d->choke_pos = 0;//update position value
    stpmot_dir(SM_DIR_CCW);
    stpmot_run(d->param.sm_steps);
    chks.state = 7;
   }
   goto check_tst;

  case 7:
   if (!stpmot_is_busy())                                     //ready?
   {
    d->choke_pos = 200;//update position value
    stpmot_dir(SM_DIR_CW);
    stpmot_run(d->param.sm_steps);
    chks.state = 6;
   }
   goto check_tst;

  default:
  check_tst:
   if (!d->choke_testing)
    chks.state = 1;                                           //exit choke testing mode
  check_pwr:
   if (!pwrrelay_get_state())
   {                                                          //power-down
    chks.pwdn = 1;
    chks.state = 1;
   }
   break;
 }
}

uint8_t choke_is_ready(void)
{
 return (chks.state == 5 || chks.state == 3);
}

#endif //SM_CONTROL
