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

/** \file gdcontrol.c
 * \author Alexey A. Shabelnikov
 * Implementation of stepper motor control
 * (Реализация управления шаговым двигателем).
 */

#ifdef GD_CONTROL

#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "ioconfig.h"
#include "gdcontrol.h"
#include "tables.h"

volatile uint16_t gdsm_steps = 0;
volatile uint8_t gdsm_latch = 0;
uint16_t gdsm_steps_b = 0;
uint8_t gdsm_pulse_state = 0;
volatile uint16_t gdsm_steps_cnt = 0;


void gdstpmot_init_ports(void)
{
 IOCFG_INIT(IOP_GD_DIR, SM_DIR_CW);
 IOCFG_INIT(IOP_GD_STP, 0); //high level at the output
}

void gdstpmot_dir(uint8_t dir)
{
 //Speaking about L297, CW/~CCW input synchronized internally therefore
 //direction can be changed at any time
 IOCFG_SETF(IOP_GD_DIR, dir);
}

void gdstpmot_run(uint16_t steps)
{
 if (steps)
 {
  _DISABLE_INTERRUPT();
  gdsm_steps_cnt = 0;
  _ENABLE_INTERRUPT();
 }
 _DISABLE_INTERRUPT();
  gdsm_steps = steps;
  gdsm_latch = 1;
 _ENABLE_INTERRUPT();
}

uint8_t gdstpmot_is_busy(void)
{
 uint16_t current;
 uint8_t latching;
 _DISABLE_INTERRUPT();
 current = gdsm_steps_b;
 latching = gdsm_latch;
 _ENABLE_INTERRUPT();
 return (current > 0 || latching); //busy?
}

uint16_t gdstpmot_stpcnt(void)
{
 uint16_t count;
 _DISABLE_INTERRUPT();
 count = gdsm_steps_cnt;
 _ENABLE_INTERRUPT();
 return count;
}

#endif
