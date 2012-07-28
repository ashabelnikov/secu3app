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

/** \file camsens.c
 * Implementation of camshaft position sensor's processing.
 * (Реализация обработки датчика фаз).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "camsens.h"

//Functionality added when either PHASE_SENSOR or SECU3T is defined
#if defined(PHASE_SENSOR) || defined(SECU3T)

#ifndef SECU3T /*SECU-3*/
 /** Get logic level from cam sensor output */
 #define GET_CAMSTATE() (PINC_Bit4)
#endif

/** Defines state variables */
typedef struct
{
 volatile uint8_t cam_ok;             //!< indicates presence of cam sensor (works properly)
 uint8_t cam_error;                   //!< error flag, indicates error
#ifndef SECU3T /*SECU-3*/
 uint8_t prev_level;                  //!< previos logic level of sensor's output
#endif
 uint8_t err_threshold;               //!< error threshold in teeth
 volatile uint8_t err_counter;        //!< teeth counter
 volatile uint8_t event;              //!< flag which indicates Hall cam sensor's event
#ifdef SECU3T
 volatile uint8_t vr_event;           //!< flag which indicates VR cam sensor's event
#endif
}camstate_t;

/** Global instance of cam sensor state variables */
camstate_t camstate;

void cams_init_state_variables(void)
{
#ifndef SECU3T /*SECU-3*/
 camstate.prev_level = GET_CAMSTATE();
#endif
 camstate.cam_ok = 0; //not Ok
 camstate.err_threshold = 60 * 2;
 camstate.err_counter = 0;
 camstate.event = 0;
#ifdef SECU3T
 camstate.vr_event = 0;
#endif
}

void cams_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 cams_init_state_variables();
 camstate.cam_error = 0; //no errors

#ifdef SECU3T /*SECU-3T*/
 //interrupt by rising edge
 MCUCR|= _BV(ISC11) | _BV(ISC10);
 MCUCR|= _BV(ISC01) | _BV(ISC00);
 GICR|=_BV(INT1) | _BV(INT0);
#endif

 _END_ATOMIC_BLOCK();
}

#ifdef SECU3T /*SECU-3T*/
uint8_t cams_vr_is_event_r(void)
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = camstate.vr_event;
 camstate.vr_event = 0; //reset event flag
 _END_ATOMIC_BLOCK();
 return result;
}

/**Interrupt from CAM sensor (VR). Marked as REF_S on the schematics */
ISR(INT0_vect)
{
 camstate.vr_event = 1; //set event flag
}
#endif //SECU3T
#endif //defined(PHASE_SENSOR) || defined(SECU3T)


//Functionality added to compilation only when PHASE_SENSOR defined
#ifdef PHASE_SENSOR
void cams_set_error_threshold(uint8_t threshold)
{
 camstate.err_threshold = threshold;
}

void cams_detect_edge(void)
{
#ifndef SECU3T /*SECU-3*/
 uint8_t level = GET_CAMSTATE();
 if (camstate.prev_level != level)
 {
  if (0 != level) //interesting edge from cam sensor
  {
   camstate.cam_ok = 1;
   camstate.err_counter = 0;
   camstate.prev_level = level;
   camstate.event = 1;
   return; //detected
  }
 }
#endif

 if (++camstate.err_counter > camstate.err_threshold)
 {
  camstate.cam_ok = 0;
  camstate.cam_error = 1;
  camstate.err_counter = 0;
 }

#ifndef SECU3T /*SECU-3*/
 camstate.prev_level = level;
#endif
}

uint8_t cams_is_ready(void)
{
 return camstate.cam_ok;
}

uint8_t cams_is_error(void)
{
 return camstate.cam_error;
}

void cams_reset_error(void)
{
 camstate.cam_error = 0;
}

uint8_t cams_is_event_r(void)
{
 uint8_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = camstate.event;
 camstate.event = 0; //reset event flag
 _END_ATOMIC_BLOCK();
 return result;
}

#ifdef SECU3T /*SECU-3T*/
/**Interrupt from CAM sensor (Hall)*/
ISR(INT1_vect)
{
 camstate.cam_ok = 1;
 camstate.err_counter = 0;
 camstate.event = 1; //set event flag
}
#endif //SECU3T
#endif //PHASE_SENSOR
