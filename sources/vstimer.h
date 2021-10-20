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

/** \file vstimer.h
 * \author Alexey A. Shabelnikov
 * Virtual system timers
 */

#ifndef _VSTIMER_H_
#define _VSTIMER_H_

#include "port/interrupt.h"
#include "port/intrinsic.h"
#include <stdint.h>

/**Context data for timers*/
typedef struct
{
 uint16_t start;   //!< Start value of the 10ms system counter
 uint16_t time;    //!< Time for timer in 10ms units
 uint8_t  action;  //!< Action flag (becomes 1 when timer's time has come)
}s_timer16_t;

/**Initialization of state of specified timer. One tick = 10ms. Set specified timer for specified period
 * \param ctx Context data structure (timer's object)
 * \param time Time in 10ms units
 */
void s_timer_set(s_timer16_t* ctx, uint16_t time);

/**Checks whenever specified timer is fired
 * \param ctx Context data structure (timer's object)
 * \return 1 - timer has fired, 0 - timer is not fired
 */
uint8_t s_timer_is_action(s_timer16_t* ctx);

/**Get value of the system 10ms counter
 * \return Current value of the system's counter
 */
uint16_t s_timer_gtc(void);

/**Initialization of system timers */
void s_timer_init(void);

#ifdef DIAGNOSTICS
/**Enter diagnostics compatible mode*/
void s_timer_enter_diag(void);
#endif

/**Get number of strokes elapsed since last start of engine*/
uint16_t s_timer_sss(void);

/**Updates stroke related logic. Must be called from the main loop*/
void s_timer_stroke_event_notification(void);

/**Called from the main loop when engine is stopped*/
void s_timer_eng_stopped_notification(void);

#endif //_VSTIMER_H_
