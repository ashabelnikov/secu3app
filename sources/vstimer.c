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


/**Addition value for compare register */
#define COMPADD              5

#ifdef _PLATFORM_M644_
#define TIMER2_RELOAD_VALUE  0
#define DIVIDER_RELOAD       5
#else
/**Reload value for timer 2 */
#define TIMER2_RELOAD_VALUE  6               //for 2 ms

/**Reload count for system timer's divider, to obtain 10 ms from 2 ms */
#define DIVIDER_RELOAD       4
#endif

//TODO: Do refactoring of timers! Implement callback mechanism.

volatile s_timer8_t  send_packet_interval_counter = 0;    //!< used for sending of packets
volatile s_timer8_t  force_measure_timeout_counter = 0;   //!< used by measuring process when engine is stopped
volatile s_timer8_t  ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE; //!< used for counting of time intervals for CE
volatile s_timer8_t  engine_rotation_timeout_counter = 0; //!< used to determine that engine was stopped
volatile s_timer8_t  epxx_delay_time_counter = 0;         //!< used by idle economizer's controlling algorithm
volatile s_timer8_t  idle_period_time_counter = 0;        //!< used by idling regulator's controlling algorithm
volatile s_timer16_t save_param_timeout_counter = 0;      //!< used for saving of parameters (automatic saving)
#ifdef FUEL_PUMP
volatile s_timer16_t fuel_pump_time_counter = 0;          //!< used for fuel pump
#endif
volatile s_timer16_t powerdown_timeout_counter = 0;       //!< used for power-down timeout 

/**for division, to achieve 10ms, because timer overflovs each 2 ms */
uint8_t divider = DIVIDER_RELOAD;

volatile uint16_t sys_counter = 0;                        //!< system tick counter, 1 tick = 10ms

#ifdef SM_CONTROL
//See smcontrol.c
extern volatile uint16_t sm_steps;
extern volatile uint8_t sm_latch;
extern uint16_t sm_steps_b;
extern uint8_t sm_pulse_state;
#endif

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
#ifdef _PLATFORM_M644_
 OCR2A = TIMER2_RELOAD_VALUE + COMPADD;
#else
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
#endif
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
  s_timer_update(force_measure_timeout_counter);
  s_timer_update(save_param_timeout_counter);
  s_timer_update(send_packet_interval_counter);
  s_timer_update(ce_control_time_counter);
  s_timer_update(engine_rotation_timeout_counter);
  s_timer_update(epxx_delay_time_counter);
  s_timer_update(idle_period_time_counter);
#ifdef FUEL_PUMP
  s_timer_update(fuel_pump_time_counter);
#endif
  s_timer_update(powerdown_timeout_counter);
  ++sys_counter;
 }
}

void s_timer_init(void)
{
#ifdef _PLATFORM_M644_
 TCCR2B|= _BV(CS22)|_BV(CS20); //clock = 156.25kHz (tick = 6.4us)
 TCNT2 = 0;
 TIMSK2|= _BV(TOIE2);
#else
 TCCR2|= _BV(CS22)|_BV(CS20);  //clock = 125kHz (tick = 8us)
 TCNT2 = 0;
 TIMSK|= _BV(TOIE2);           //enable T/C 2 overflow interrupt
#endif
}
