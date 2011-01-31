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

#include <ioavr.h>
#include <inavr.h>
#include "vstimer.h"
#include "secu3.h"

#define TIMER2_RELOAD_VALUE  6               //for 2 ms
#define COMPADD 5
#define DIVIDER_RELOAD 4

//TODO: Do refactoring of timers! Implement callback mechanism.
volatile s_timer8_t  send_packet_interval_counter = 0;
volatile s_timer8_t  force_measure_timeout_counter = 0;
volatile s_timer8_t  ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE;
volatile s_timer8_t  engine_rotation_timeout_counter = 0;
volatile s_timer8_t  epxx_delay_time_counter = 0;
volatile s_timer8_t  idle_period_time_counter = 0;
volatile s_timer16_t save_param_timeout_counter = 0;

//for division, to achieve 10ms, because timer overflovs each 2 ms
uint8_t divider = DIVIDER_RELOAD;

//Interrupt routine which called when T/C 2 overflovs - used for counting time intervals in system 
//(for generic usage). Called each 2ms. System tick is 10ms, and so we divide frequency by 5
#pragma vector=TIMER2_OVF_vect
__interrupt void timer2_ovf_isr(void)
{
 TCNT2 = TIMER2_RELOAD_VALUE;
 
#ifdef VENTILATOR_PWM 
 //for PWM reneration (ventilator). We need to reinitialize OCR2 because it looses correct value 
 //(I guess) after TCNT2 write. Compare interrupt shifted in time from overflow interrupt by COMPADD
 //value 
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
#endif

 __enable_interrupt();     

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
 }
}

void s_timer_init(void)
{
 TCCR2|= (1<<CS22)|(1<<CS20);  //clock = 125kHz (tick = 8us) 
 TCNT2 = 0;
 TIMSK|= (1<<TOIE2);           //enable T/C 2 overflov interrupt                          
}
