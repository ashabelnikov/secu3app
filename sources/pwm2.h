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

/** \file pwm2.h
 * \author Alexey A. Shabelnikov
 * Additional 2 versatile PWM channels.
 */

#ifndef _PWM2_H_
#define _PWM2_H_

/**Initialization of used I/O ports*/
void pwm2_init_ports(void);

/**Initialization of internal state variables and hardware */
void pwm2_init_state(void);

/**Control of PWM channels.Must be called from the main loop
 * Uses d ECU data structure
 */
void pwm2_control(void);

/**Set PWM frequency for specified channel
 * \param ch Number of channel to be configured (0 or 1)
 * \param period value of PWM period (period = 1/f * 524288); 524288 = 2^19
 */
void pwm2_set_pwmfrq(uint8_t ch, uint16_t period);

#endif //_PWM2_H_
