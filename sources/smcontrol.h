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

/** \file smcontrol.h
 * \author Alexey A. Shabelnikov
 * Stepper motor control
 */

#ifndef _SMCONTROL_H_
#define _SMCONTROL_H_

#ifdef SM_CONTROL

#include <stdint.h>

/** Initialization of used I/O ports */
void stpmot_init_ports(void);

/** ID of the clockwise direction */
#define SM_DIR_CW   0

/** ID of the counterclockwise direction */
#define SM_DIR_CCW  1

/** Set stepper motor direction
 * \param dir Direction (0 - backward, 1 - forward)
 */
void stpmot_dir(uint8_t dir);

/** Run stepper motor using specified number of steps
 * \param steps Number of steps to run. Use 0 if you want to stop
 * the stepper motor.
 */
void stpmot_run(uint16_t steps);

/**Check if stepper motor is busy (busy means running at the moment)
 * \return 1 - stepper motor is busy, 0 - stepper motor is idle
 */
uint8_t stpmot_is_busy(void);

/** This function returns number of actually processed steps
 * \return number of actually processed steps
 */
uint16_t stpmot_stpcnt(void);

/** Set frequency of STEP pulses
 * \param freq Frequency code: 0, 1, 2, 3
 */
void stpmot_freq(uint8_t freq);

#endif

#endif //_SMCONTROL_H_
