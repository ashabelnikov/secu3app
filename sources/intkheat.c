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
 * (Реализация управления подогревом впускного коллектора).
 */

#ifdef INTK_HEATING

#include "port/port.h"
#include "ckps.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "vstimer.h"

/**Input manifold heating time - 10 minutes */
#define HEATING_TIME (10U*60U*100U)

/**Heating off temperature */
#define HEATING_T_OFF  65.0

/**Declare state variables structure*/
typedef struct
{
 uint8_t state;     //!< Needed by finite state machine (current state)
 uint16_t strt_t1;  //!< Used for timing
}ih_state_t;

/**Global instance of state variables */
ih_state_t ih;

void intkheat_init_ports(void)
{
 IOCFG_INIT(IOP_INTK_HEAT, 0);  //<-- heating is off
}

void intkheat_init(void)
{
 ih.state = 0;
}

void intkheat_control(struct ecudata_t *d)
{
 if (!d->param.tmp_use)
 {
  IOCFG_INIT(IOP_INTK_HEAT, 0);
  return;
 }

 switch(ih.state)
 {
  case 0: //turn on heating and start timer
   IOCFG_SET(IOP_INTK_HEAT, (d->sens.temperat < TEMPERATURE_MAGNITUDE(HEATING_T_OFF)));   // control heating
   ih.strt_t1 = s_timer_gtc();
   ih.state = 1;
   break;

  case 1: //wait 10 minutes and turn off heating or it will be turned off immediatelly if crankshaft begin to revolve
   if (((s_timer_gtc() - ih.strt_t1) >= HEATING_TIME) || ckps_is_cog_changed())
   {
    IOCFG_SET(IOP_INTK_HEAT, 0);                                                          // turn off heating
    ih.state = 2;
   }
   break;

  case 2: //control heating if engine is running, otherwise turn it off
   if (d->st_block)
   { //engine is running
    IOCFG_SET(IOP_INTK_HEAT, (d->sens.temperat < TEMPERATURE_MAGNITUDE(HEATING_T_OFF)));  // control heating
   }
   else
    IOCFG_SET(IOP_INTK_HEAT, 0);
   break;
 }
}

#endif //INTK_HEATING
