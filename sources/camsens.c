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

#ifdef PHASE_SENSOR

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
 CamCallback edg_callback;            //!< Callback function to call on cam edge
 CamCallback err_callback;            //!< Callback function to call on error (cam is missing)
}camstate_t;

/** Global instance of cam sensor state variables */
camstate_t camstate;

void cams_init_state_variables(void)
{
 camstate.prev_level = GET_CAMSTATE();
 camstate.cam_ok = 0; //not Ok
 camstate.err_threshold = 60 * 2;
 camstate.err_counter = 0;
}

void cams_init_state(void)
{
 _BEGIN_ATOMIC_BLOCK();
 cams_init_state_variables();
 camstate.edg_callback = 0;
 camstate.err_callback = 0;
 camstate.cam_error = 0; //no errors

#ifdef SECU3T /*SECU-3T*/
 //interrupt by rising edge
 MCUCR|= _BV(ISC11) | _BV(ISC10);
 GICR|=_BV(INT1);
#endif

 _END_ATOMIC_BLOCK();
}

void cams_set_callbacks(CamCallback p_edg_callback, CamCallback p_err_callback)
{
 camstate.edg_callback = p_edg_callback;
 camstate.err_callback = p_err_callback;
}

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
   if (camstate.edg_callback)
    camstate.edg_callback();
   return; //detected
  }
 }
#endif

 if (++camstate.err_counter > camstate.err_threshold)
 {
  camstate.cam_ok = 0;
  camstate.cam_error = 1;
  camstate.err_counter = 0;
  if (camstate.err_callback)
   camstate.err_callback();
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

#ifdef SECU3T /*SECU-3T*/
/**Interrupt from CAM sensor */
ISR(INT1_vect)
{
 camstate.cam_ok = 1;
 camstate.err_counter = 0;
 if (camstate.edg_callback)
  camstate.edg_callback();
}
#endif

#endif //PHASE_SENSOR
