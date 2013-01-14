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

/** \file smcontrol.c
 * Implementation of stepper motor control
 * (Реализация управления шаговым двигателем).
 */

#ifdef SM_CONTROL

#include "port/intrinsic.h"
#include "port/port.h"
#include "ioconfig.h"
#include "smcontrol.h"
#include "tables.h"

volatile uint16_t sm_steps = 0;
volatile uint8_t sm_latch = 0;
uint16_t sm_steps_b = 0;
uint8_t sm_pulse_state = 0;

void stpmot_init_ports(void)
{
 IOCFG_INIT(IOP_SM_DIR, SM_DIR_CW);
 IOCFG_INIT(IOP_SM_STP, 0); //high level at the output
}

void stpmot_init(void)
{
 //todo
}

void stpmot_dir(uint8_t dir)
{
 //Speaking about L297, CW/~CCW input synchronized internally therefore 
 //direction can be changed at any time
 IOCFG_SET(IOP_SM_DIR, dir);
}

void stpmot_run(uint16_t steps)
{
 _DISABLE_INTERRUPT();
  sm_steps = steps;
  sm_latch = 1;
 _ENABLE_INTERRUPT();
}

#endif
