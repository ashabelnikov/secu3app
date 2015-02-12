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

/** \file camsens.h
 * Processing of camshaft position sensor.
 * (Обработка датчика фаз).
 */

#ifndef _CAMSENS_H_
#define _CAMSENS_H_

#include <stdint.h>

/**Initialization of used input ports*/
void cams_init_ports(void);

/** Initialization of state variables */
void cams_init_state_variables(void);

/** Initialization of cam module (Hardware & variables) */
void cams_init_state(void);

/** Must be called from main loop to perform some operations */
void cams_control(void);

/**Checks for event(VR input) and automatically resets the flag
 * \return 1 - event was pending, otherwise - 0 */
uint8_t cams_vr_is_event_r(void);

/** Set edge type for VR input (Установка фронта для ДНО)
 * \param edge_type 0 - falling (спадающий), 1 - rising (нарастающий)
 */
void cams_vr_set_edge_type(uint8_t edge_type);

#ifdef PHASE_SENSOR
/** Sets threshold value (number of teeth between pulses) for errors checking
 * \param threshold number of teeth between pulses */
void cams_set_error_threshold(uint16_t threshold);

/** Edge detection (if non-interrupt driven) and checking logic. Must be executed on each tooth. */
void cams_detect_edge(void);

/** Check is cam sensor ready
 * \return value > 0 if cam sensor is ready */
uint8_t cams_is_ready(void);

/** Checks for errors
 * \return value > 0 if error occured before or between calls of this fuction, otherwise 0 */
uint8_t cams_is_error(void);

/** Reset internal error flag */
void cams_reset_error(void);

/**Checks for event(Hall input) and automatically resets the flag
 * \return 1 - event was pending, otherwise - 0 */
uint8_t cams_is_event_r(void);

#endif //PHASE_SENSOR

#endif //_CAMSENS_H_
