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

/** \file vstimer.c
 * Implementation of virtual system timers
 * (Реализация виртуальных системных таймеров).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "ioconfig.h" //for SM_CONTROL
#include "secu3.h"
#include "vstimer.h"

/**Reload value for timer 2, for 2ms */
#define TIMER2_RELOAD_VALUE  6

/**Addition value for compare register */
#define COMPADD 5

/**Reload count for system timer's divider, to obtain 10 ms from 2 ms */
#define DIVIDER_RELOAD 4

//TODO: Do refactoring of timers! Implement callback mechanism.

/**for division, to achieve 10ms, because timer overflovs each 2 ms */
uint8_t divider = DIVIDER_RELOAD;

volatile uint16_t sys_counter = 0; //!< system tick counter, 1 tick = 10ms

#ifdef SM_CONTROL
//See smcontrol.c
extern volatile uint16_t sm_steps;
extern volatile uint8_t sm_latch;
extern uint16_t sm_steps_b;
extern uint8_t sm_pulse_state;
#endif


void s_timer_set8(s_timer8_t* timer, uint8_t value)
{
 timer->start_val = _AB(sys_counter, 0);
 timer->timeout = value;
}

void s_timer_set16(s_timer16_t* timer, uint16_t value)
{
 _BEGIN_ATOMIC_BLOCK();
 timer->start_val = sys_counter;
 _END_ATOMIC_BLOCK();
 timer->timeout = value;
}

uint8_t s_timer_gtc8(void)
{
 return _AB(sys_counter, 0);
}

uint16_t s_timer_gtc16(void)
{
 uint16_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = sys_counter;
 _END_ATOMIC_BLOCK();
 return result;
}

uint8_t s_timer_isexp8(const s_timer8_t* timer)
{
 return (s_timer_gtc8() - (timer->start_val) >= timer->timeout);
}

uint8_t s_timer_isexp16(const s_timer16_t* timer)
{
 return (s_timer_gtc16() - (timer->start_val) >= timer->timeout);
}

void s_timer_init(void)
{
 TCCR2|= _BV(CS22)|_BV(CS20);  //clock = 125kHz (tick = 8us)
 TCNT2 = 0;
 TIMSK|= _BV(TOIE2);           //enable T/C 2 overflow interrupt
}

/**Interrupt routine which called when T/C 2 overflovs - used for counting time intervals in system
 *(for generic usage). Called each 2ms. System tick is 10ms, and so we divide frequency by 5
 */
ISR(TIMER2_OVF_vect)
{
 TCNT2 = TIMER2_RELOAD_VALUE;

#ifdef COOLINGFAN_PWM
 //for PWM's generation (for cooling fan). We need to reinitialize OCR2 because it looses correct value
 //(I guess) after TCNT2 write. Compare interrupt shifted in time from overflow interrupt by COMPADD
 //value
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
#endif

 _ENABLE_INTERRUPT();

#ifdef SM_CONTROL
 if (!sm_pulse_state && sm_latch) {
  sm_steps_b = sm_steps;
  sm_latch = 0;
 }

 if (sm_steps_b)
 {
  if (!sm_pulse_state) {
   IOCFG_SET(IOP_SM_STP, 1); //falling edge
   sm_pulse_state = 1;
  }
  else {//The step occurs on the rising edge of ~CLOCK signal
   IOCFG_SET(IOP_SM_STP, 0); //rising edge
   sm_pulse_state = 0;
   --sm_steps_b;
  }
 }
#endif

 if (divider > 0)
  --divider;
 else
 {//each 10 ms
  divider = DIVIDER_RELOAD;
  ++sys_counter;
 }
}
