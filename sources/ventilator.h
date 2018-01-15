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

/** \file ventilator.h
 * \author Alexey A. Shabelnikov
 * Cooling fan's control related functions.
 */

#ifndef _VENTILATOR_H_
#define _VENTILATOR_H_

/**Initialization of used I/O ports*/
void vent_init_ports(void);

/**Control of cooling fan.
 * Uses d ECU data structure
 */
void vent_control(void);

/**Initialization of internal state */
void vent_init_state(void);

/**Turn off cooling fan (used inside bc_input unit). This function also turns off
 * IAC_PWM and GD_PWM (if one of them mapped to ECF)
 * Uses d ECU data structure
 */
void vent_turnoff(void);

/**Set PWM frequency
 * \param period value of PWM period (period = 1/f * 524288)
 */
void vent_set_pwmfrq(uint16_t period);

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
/** Use by IAC when ECF is remapped to IAC_PWM
 * \param duty 8-bit PWM duty value (0...255)
 */
void vent_set_duty8(uint8_t duty);
#endif

/**Called from main loop when system detects changing of cog number*/
void vent_cog_changed_notification(void);

#endif //_VENTILATOR_H_
