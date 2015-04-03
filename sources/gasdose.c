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
#include "pwrrelay.h"

//=============================================================================================================================

#define GASDOSE_POS_RPM_SIZE 16   //!< RPM axis size
#define GASDOSE_POS_TPS_SIZE 16   //!< TPS axis size
#define _GD(v) GD_MAGNITUDE(v)    //!< For encoding of gas dose actuator position value

/** Gas dose actuator position vs (TPS,RPM)
 */
PGM_DECLARE(uint8_t gasdose_pos[GASDOSE_POS_TPS_SIZE][GASDOSE_POS_RPM_SIZE]) =
{//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //100%
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}  //0%
};

/**TPS % between two interpolation points, additionally multiplied by 16 */
#define TPS_AXIS_STEP TPS_MAGNITUDE((100.0*16)/(GASDOSE_POS_TPS_SIZE-1))

/** Calculation îf gas dosator position, based on (TPS,RPM)
 * \param d Pointer to ECU data structure
 * \return Gas dosator position in % (value * 2)
 */
int16_t gdp_function(struct ecudata_t* d)
{
 int16_t rpm = d->sens.inst_frq, tps = d->sens.tps * 16;
 int8_t t = (tps / TPS_AXIS_STEP), f, tp1, fp1;

 if (t >= (GASDOSE_POS_TPS_SIZE - 1))
  tp1 = t = GASDOSE_POS_TPS_SIZE - 1;
 else
  tp1 = t + 1;

 //find interpolation points, then restrict RPM if it fall outside set range
 for(f = GASDOSE_POS_RPM_SIZE-2; f >= 0; f--)
  if (rpm >= PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[f])) break;

 //Gas dose map works from rpm_grid_points[0] and upper
 if (f < 0)  {f = 0; rpm = PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[0]);}
 if (rpm > PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[GASDOSE_POS_RPM_SIZE-1])) rpm = PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[GASDOSE_POS_RPM_SIZE-1]);
 fp1 = f + 1;

 return bilinear_interpolation(rpm, tps,  //note that tps is additionally multiplied by 16
        PGM_GET_BYTE(gasdose_pos[t][f]),
        PGM_GET_BYTE(gasdose_pos[tp1][f]),
        PGM_GET_BYTE(gasdose_pos[tp1][fp1]),
        PGM_GET_BYTE(gasdose_pos[t][fp1]),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_points[f]),
        (TPS_AXIS_STEP*t),
        PGM_GET_WORD(&fw_data.exdata.rpm_grid_sizes[f]),
        TPS_AXIS_STEP) >> 4;
}

//=============================================================================================================================


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
}gasdose_st_t;

/**Instance of state variables */
static gasdose_st_t gds = {0};

void gasdose_init_ports(void)
{
 gdstpmot_init_ports();
}

void gasdose_init(void)
{
 gds.state = 0;
 gdstpmot_init();
 CLEARBIT(gds.flags, CF_POWERDOWN);
 CLEARBIT(gds.flags, CF_MAN_CNTR);
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
 * \param d pointer to ECU data structure
 * \param dir Direction (see description on gdstpmot_dir() function)
 */
static void initial_pos(struct ecudata_t* d, uint8_t dir)
{
 gdstpmot_dir(dir);                                             //set direction
 gdstpmot_run(d->param.sm_steps + (d->param.sm_steps >> 5));    //run using number of steps + 3%
}

//TODO: redundant to simular function in choke.c, remove this redundant copy in the future
/** Stepper motor control for normal working mode
 * \param d pointer to ECU data structure
 * \param pos current calculated (target) position of stepper motor
 */
static void sm_motion_control(struct ecudata_t* d, int16_t pos)
{
 int16_t diff;
 restrict_value_to(&pos, 0, d->param.sm_steps);
 if (CHECKBIT(gds.flags, CF_SMDIR_CHG))                      //direction has changed
 {
  if (!gdstpmot_is_busy())
  {
   gds.smpos = gds.smpos_prev + ((gds.cur_dir == SM_DIR_CW) ? -gdstpmot_stpcnt() : gdstpmot_stpcnt());
   CLEARBIT(gds.flags, CF_SMDIR_CHG);
  }
 }
 if (!CHECKBIT(gds.flags, CF_SMDIR_CHG))                     //normal operation
 {
  diff = pos - gds.smpos;
  if (!gdstpmot_is_busy())
  {
   if (diff != 0)
   {
    gds.cur_dir = diff < 0 ? SM_DIR_CW : SM_DIR_CCW;
    gdstpmot_dir(gds.cur_dir);
    gdstpmot_run(abs(diff));                                    //start stepper motor
    gds.smpos_prev = gds.smpos;                             //remember position when SM started motion
    gds.smpos = pos;                                         //this is a target position
   }
  }
  else //busy
  {
   //Check if curent target direction is not match new target direction. If it is not match, then
   //stop stepper motor and go to the direction changing.
   if (((gds.smpos - gds.smpos_prev) & 0x8000) != ((pos - gds.smpos_prev) & 0x8000))
   {
    gdstpmot_run(0);                                            //stop stepper motor
    SETBIT(gds.flags, CF_SMDIR_CHG);
   }
  }
 }
}

/** Calculate stepper motor position for normal mode
 * \param d pointer to ECU data structure
 * \return stepper motor position in steps
 */
static int16_t calc_sm_position(struct ecudata_t* d)
{
 return ((((int32_t)d->param.gd_steps) * gdp_function(d)) / GD_MAGNITUDE(100.0));
}

void gasdose_control(struct ecudata_t* d)
{
 switch(gds.state)
 {
  case 0:                                                     //Initialization of stepper motor position
   if (!IOCFG_CHECK(IOP_PWRRELAY))                            //Skip initialization if power management is enabled
    initial_pos(d, INIT_POS_DIR);
   gds.state = 2;
   break;

  case 1:                                                     //system is being preparing to power-down
   initial_pos(d, INIT_POS_DIR);
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
   if (d->gasdose_testing)
   {
    initial_pos(d, INIT_POS_DIR);
    gds.state = 6;                                            //start testing
   }
   else
   {
    int16_t pos;
    if (!CHECKBIT(gds.flags, CF_MAN_CNTR))
    {
     pos = calc_sm_position(d);                               //calculate stepper motor position
     if (d->gasdose_manpos_d)
      SETBIT(gds.flags, CF_MAN_CNTR);                         //enter manual mode
    }
    else
    { //manual control
     pos = gds.smpos + d->gasdose_manpos_d;
     d->gasdose_manpos_d = 0;
    }

    sm_motion_control(d, pos);                                //SM command execution
   }
   d->gasdose_pos = calc_percent_pos(gds.smpos, d->param.sm_steps);//update position value
   goto check_pwr;

  //     Testing modes
  case 6:                                                     //initialization of stepper motor
   if (!gdstpmot_is_busy())                                   //ready?
   {
    d->gasdose_pos = GD_MAGNITUDE(0);                         //update position value
    gdstpmot_dir(SM_DIR_CCW);
    gdstpmot_run(d->param.sm_steps);
    gds.state = 7;
   }
   goto check_tst;

  case 7:
   if (!gdstpmot_is_busy())                                   //ready?
   {
    d->gasdose_pos = GD_MAGNITUDE(100.0);                     //update position value
    gdstpmot_dir(SM_DIR_CW);
    gdstpmot_run(d->param.sm_steps);
    gds.state = 6;
   }
   goto check_tst;

  default:
  check_tst:
   if (!d->gasdose_testing)
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
 return (gds.state == 5 || gds.state == 3);
}

#endif //GD_CONTROL
