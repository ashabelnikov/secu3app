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
#include "ioconfig.h"
#include "funconv.h"
#include "secu3.h"
#include "smcontrol.h"
#include "pwrrelay.h"

/**Direction used to set choke to the initial position */
#define INIT_POS_DIR SM_DIR_CW

/**Startup choke closing correction time, 3 sec. */
#define STARTUP_CORR_TIME (3*100)

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
}

int16_t calc_startup_corr(struct ecudata_t* d)
{
 switch(chks.strt_mode)
 {
  case 0:  //starting
   if (d->st_block)
   {
    chks.strt_t1 = s_timer_gtc();
    chks.strt_mode = 1;
   }
   break; //use correction
  case 1:
   if ((s_timer_gtc() - chks.strt_t1) >= STARTUP_CORR_TIME)
    chks.strt_mode = 2;
   break; //use correction
  case 2:
   if (!d->st_block)
    chks.strt_mode = 0; //engine is stopped, so use correction again
   return 0; //do not use correction
 }

 return (((int32_t)d->param.sm_steps) * d->param.choke_startup_corr) / 200;
}

/** Set choke to initial position. Because we have no position feedback, we
 * must use number of steps more than stepper actually has.
 * \param dir Direction (see description on stpmot_dir() function)
 * \param steps Total number of steps of stepper motor
 */
static void initial_pos(uint8_t dir, uint16_t steps)
{
 stpmot_dir(dir);                     //set direction
 stpmot_run(steps + (steps >> 5));    //run using number of steps + 3%
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

void choke_control(struct ecudata_t* d)
{
 switch(chks.state)
 {
  case 0:                                                     //Initialization of choke position
   if (!IOCFG_CHECK(IOP_PWRRELAY))                            //Skip initialization if power management is enabled
    initial_pos(INIT_POS_DIR, d->param.sm_steps);
   chks.state = 2;
   chks.prev_temp = d->sens.temperat;
   break;

  case 1:                                                     //system is being preparing to power-down
   initial_pos(INIT_POS_DIR, d->param.sm_steps);
   chks.state = 2;
   break;

  case 2:                                                     //wait while choke is being initialized
   if (!stpmot_is_busy())                                     //ready?
   {
    if (chks.pwdn)
     chks.state = 3;                                          //ready to power-down
    else
     chks.state = 5;                                          //normal working
    chks.smpos = 0;
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
    initial_pos(INIT_POS_DIR, d->param.sm_steps);
    chks.state = 6;                                           //start testing
   }
   else
   {
    int16_t pos, diff;
    if (!chks.manual)
    {
     int16_t tmp_pos = (((int32_t)d->param.sm_steps) * choke_closing_lookup(d, &chks.prev_temp)) / 200;
     int16_t rpm_cor = 0; //todo
     pos = tmp_pos + rpm_cor + calc_startup_corr(d);
     if (d->choke_manpos_d)
      chks.manual = 1; //enter manual mode
    }
    else
    { //manual control
     pos = chks.smpos + d->choke_manpos_d;
     d->choke_manpos_d = 0;
    }

    restrict_value_to(&pos, 0, d->param.sm_steps);
    diff = pos - chks.smpos;
    if (!stpmot_is_busy() && diff != 0)
    {
     stpmot_dir(diff < 0 ? SM_DIR_CW : SM_DIR_CCW);
     stpmot_run(abs(diff));                                    //start stepper motor
     chks.smpos += diff;
    }
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
