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

/** \file fuelpump.c
 * Implementation of controling of electric fuel pump.
 * (Реализация управления электробензонасосом).
 */

#ifdef FUEL_PUMP

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "fuelpump.h"
#include "ioconfig.h"
#include "secu3.h"
#include "vstimer.h"

/**Turn off timeout used at startup, 5 seconds*/
#define FP_TURNOFF_TIMEOUT_STRT 500

/**Turn off timeout used when engine stops, 3 seconds */
#define FP_TURNOFF_TIMEOUT_STOP 300

/** Turn on/turn off fuel pump */
#define TURN_ON_ELPUMP(s) IOCFG_SET(IOP_FL_PUMP, s)

typedef struct
{
 uint8_t state;   //!< State for finite-state machine
}fp_state_t;

/**Global instance of state variables */
fp_state_t fpstate = {0};

void fuelpump_init_ports(void)
{
 IOCFG_INIT(IOP_FL_PUMP, 1); //fuel pump is on
}

void fuelpump_init(void)
{
 TURN_ON_ELPUMP(1); //turn on
 s_timer16_set(fuel_pump_time_counter, FP_TURNOFF_TIMEOUT_STRT);
 fpstate.state = 0;
}

void fuelpump_control(struct ecudata_t* d)
{
 switch(fpstate.state)
 {
  case 0: //pump is turned on
   //Turn off pump if timer is expired or gas valve is turned on
   if (d->sens.gas || s_timer16_is_action(fuel_pump_time_counter))
   {
    TURN_ON_ELPUMP(0); //turn off
    fpstate.state = 1;
   }

   //reset timer periodically if engine is still running
   if (d->sens.frequen4 > 0)
    s_timer16_set(fuel_pump_time_counter, FP_TURNOFF_TIMEOUT_STOP);

   break;

  case 1: //pump is turned off
   //Do not turn on pump if gas valve is turned on
   if (!d->sens.gas && d->sens.frequen4 > 0)
   {
    TURN_ON_ELPUMP(1); //turn on
    s_timer16_set(fuel_pump_time_counter, FP_TURNOFF_TIMEOUT_STOP);
    fpstate.state = 0;
   }

   break;
 }
}

#endif //FUEL_PUMP
