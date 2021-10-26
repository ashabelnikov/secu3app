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

/** \file smcontrol.c
 * \author Alexey A. Shabelnikov
 * Implementation of stepper motor control
 */

#ifdef SM_CONTROL

#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "ioconfig.h"
#include "smcontrol.h"
#include "tables.h"

volatile uint16_t sm_steps = 0;    //!< Used to transfer steps from main loop to interrupt
volatile uint8_t sm_latch = 0;     //!< Used to help transfer steps from main loop to interrupt (trigger)
volatile uint16_t sm_steps_b = 0;  //!< Counter of SM steps used in interrupt
uint8_t sm_pulse_state = 0;        //!< State of step's pulse, used in interrupt
volatile uint16_t sm_steps_cnt = 0;//!< Number of actually run steps
volatile uint8_t sm_freq = 0;      //!< Used to transfer frequency divider's value from main loop to interrupt

void stpmot_init_ports(void)
{
 IOCFG_INIT(IOP_SM_DIR, SM_DIR_CW);
 IOCFG_INIT(IOP_SM_STP, 0); //high level at the output
}

void stpmot_dir(uint8_t dir)
{
 //Speaking about L297, CW/~CCW input synchronized internally therefore
 //direction can be changed at any time
 IOCFG_SETF(IOP_SM_DIR, dir);
}

void stpmot_run(uint16_t steps)
{
 if (steps)
 {
  _DISABLE_INTERRUPT();
  sm_steps_cnt = 0;
  _ENABLE_INTERRUPT();
 }
 _DISABLE_INTERRUPT();
 sm_steps = steps;
 sm_latch = 1;
 _ENABLE_INTERRUPT();
}

uint8_t stpmot_is_busy(void)
{
 uint16_t current;
 uint8_t latching;
 _DISABLE_INTERRUPT();
 current = sm_steps_b;
 latching = sm_latch;
 _ENABLE_INTERRUPT();
 return (current > 0 || latching); //busy?
}

uint16_t stpmot_stpcnt(void)
{
 uint16_t count;
 _DISABLE_INTERRUPT();
 count = sm_steps_cnt;
 _ENABLE_INTERRUPT();
 return count;
}

void stpmot_freq(uint8_t freq)
{
 sm_freq = freq;
}

#endif
