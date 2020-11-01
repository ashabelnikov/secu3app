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
 */

#ifndef _ECULOGIC_H_
#define _ECULOGIC_H_

#include <stdint.h>

/** Start mode of engine (used by state machine) */
#define EM_START 0

/** Idle mode of engine (used by state machine) */
#define EM_IDLE  1

/** Work mode of engine (used by state machine) */
#define EM_WORK  2

/**Initialization of state variables */
void eculogic_init(void);

/**Implements state machine of engine's modes
 * Uses d ECU data structure
 */
void eculogic_system_state_machine(void);

/** Must be called from main loop to notify about stroke events
 * Uses d ECU data structure
 */
void eculogic_stroke_event_notification(void);

/**Called from main loop when system detects changing of cog number*/
void eculogic_cog_changed_notification(void);

/**called from main loop when system detects engine stop*/
void eculogic_eng_stopped_notification(void);

#endif //_ECULOGIC_H_
