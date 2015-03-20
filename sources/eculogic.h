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

/** \file eculogic.h
 * \author Alexey A. Shabelnikov
 * Logic determining calculation and regulation of anvance angle
 * (Логика определяющая вычисление и регулирование угла опережения).
 */

#ifndef _IGNLOGIC_H_
#define _IGNLOGIC_H_

#include <stdint.h>

struct ecudata_t;

/** Start mode of engine (used by state machine) */
#define EM_START 0

/** Idle mode of engine (used by state machine) */
#define EM_IDLE  1

/** Work mode of engine (used by state machine) */
#define EM_WORK  2

/**Initialization of state variables */
void ignlogic_init(void);

/**Implements state machine of engine's modes (конечный автомат режимов двигателя)
 * \param d pointer to ECU data structure
 * \return advance angle
 */
int16_t ignlogic_system_state_machine(struct ecudata_t* d);

/** Must be called from main loop to notify about stroke events
 * \param d pointer to ECU data structure
 */
void ignlogic_stroke_event_notification(struct ecudata_t* d);

#endif //_IGNLOGIC_H_
