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

/** \file starter.c
 * \author Alexey A. Shabelnikov
 * Implementation of working with starter (blocking)
 */

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ce_errors.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "starter.h"
#include "vstimer.h"

void starter_set_blocking_state(uint8_t i_state)
{
 IOCFG_SETF(IOP_ST_BLOCK, !i_state);
}

void starter_init_ports(void)
{
 IOCFG_INIT(IOP_ST_BLOCK, 0); //starter is blocked
}

void starter_control(void)
{
 if (d.sys_locked)
 { //system locked by immobilizer
  starter_set_blocking_state(1), d.st_block = 1;
  return;
 }

 //control of starter's blocking (starter is blocked after reaching the specified RPM, but will not turn back!)
 if (d.sens.frequen > d.param.starter_off)
  starter_set_blocking_state(1), d.st_block = 1;

 if (d.sens.frequen < 30)
  starter_set_blocking_state(0), d.st_block = 0; //unblock starter
}
