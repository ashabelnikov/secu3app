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

/** \file gdcontrol.h
 * \author Alexey A. Shabelnikov
 * Stepper motor control for gas dose control
 * (Управление шаговым двигателем для дозатора газа).
 */

#ifndef _GDCONTROL_H_
#define _GDCONTROL_H_

#ifdef GD_CONTROL

#include <stdint.h>

/** Initialization of used I/O ports (инициализация используемых портов) */
void gdstpmot_init_ports(void);

/** ID of the clockwise direction (направление по часовой стрелке) */
#ifndef SM_DIR_CW
#define SM_DIR_CW   0
#endif

/** ID of the counterclockwise direction (направление против часовой стрелки) */
#ifndef SM_DIR_CCW
#define SM_DIR_CCW  1
#endif

/** Set stepper motor direction
 * \param dir Direction (0 - backward, 1 - forward)
 */
void gdstpmot_dir(uint8_t dir);

/** Run stepper motor using specified number of steps
 * \param steps Number of steps to run. Use 0 if you want to stop
 * the stepper motor.
 */
void gdstpmot_run(uint16_t steps);

/**Check if stepper motor is busy (busy means running at the moment)
 * \return 1 - stepper motor is busy, 0 - stepper motor is idle
 */
uint8_t gdstpmot_is_busy(void);

/** This function returns number of actually processed steps
 * \return number of actually processed steps
 */
uint16_t gdstpmot_stpcnt(void);

#endif

#endif //_GDCONTROL_H_
