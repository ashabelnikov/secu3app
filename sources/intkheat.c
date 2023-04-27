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

/** \file intkheat.c
 * \author Alexey A. Shabelnikov
 * Implementation of intake manifold heating control.
 */

#ifdef INTK_HEATING

#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"

/**Declare state variables structure*/
typedef struct
{
 uint8_t state;     //!< Needed by finite state machine (current state)
 uint16_t strt_t1;  //!< Used for timing
 uint8_t cog_changed;
}ih_state_t;

/**Global instance of state variables */
ih_state_t ih = {0,0,0};

void intkheat_init_ports(void)
{
 IOCFG_INIT(IOP_INTK_HEAT, 0);  //<-- heating is off
}

void intkheat_control(void)
{
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE))
 {
  IOCFG_INIT(IOP_INTK_HEAT, 0);
  return;
 }

 switch(ih.state)
 {
  case 0: //turn on heating and start timer
   IOCFG_SETF(IOP_INTK_HEAT, (d.sens.temperat < (int16_t)PGM_GET_WORD(&fw_data.exdata.heating_t_off)));   // control heating
   ih.strt_t1 = s_timer_gtc();
   ih.state = 1;
   break;

  case 1: //wait 10 minutes and turn off heating or it will be turned off immediatelly if crankshaft begin to revolve
   if (((s_timer_gtc() - ih.strt_t1) >= ((uint16_t)PGM_GET_BYTE(&fw_data.exdata.heating_time)*600)) || ih.cog_changed)
   {
    IOCFG_SETF(IOP_INTK_HEAT, 0);                                                          // turn off heating
    ih.state = 2;
   }
   break;

  case 2: //control heating if engine is running, otherwise turn it off
   if (d.engine_mode!=EM_START)
   { //engine is running
    IOCFG_SETF(IOP_INTK_HEAT, (d.sens.temperat < (int16_t)PGM_GET_WORD(&fw_data.exdata.heating_t_off)));  // control heating
   }
   else
    IOCFG_SETF(IOP_INTK_HEAT, 0);
   break;
 }
}

void intkheat_cog_changed_notification(void)
{
 ih.cog_changed = 1;
}

#endif //INTK_HEATING
