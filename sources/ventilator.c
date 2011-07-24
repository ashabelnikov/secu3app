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

/** \file ventilator.c
 * Implementation of cooling fan's control related functions.
 * (Реализация функций для управления электрическим вентилятором).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/port.h"
#include "bitmask.h"
#include "secu3.h"
#include "ventilator.h"

/**Turns on/off cooling fan*/
#define SET_COOLINGFAN_STATE(s) {PORTB_Bit1 = s;}

/**Warning must be the same as another definition in vstimer.h!*/
#define TIMER2_RELOAD_VALUE  6

/**will be added to TIMER2_RELOAD_VALUE at the initialization */
#define COMPADD 5

/**number of PWM discretes */
#define PWM_STEPS 25

volatile uint8_t pwm_state; //!< For state machine. 0 - passive, 1 - active
volatile uint8_t pwm_duty;  //!< current duty value

void vent_init_ports(void)
{
 //configure used I/O ports
 PORTB&= ~_BV(PB1);
 DDRB |= _BV(DDB1);
}

void vent_init_state(void)
{
 pwm_state = 0;  //begin from active level
 pwm_duty = 0;   // 0%
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
}

/**Sets duty value
 * \param duty value to be set
 */
void vent_set_duty(uint8_t duty)
{
 //TODO: Maybe we need double buffering?
 pwm_duty = duty;

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  TIMSK&=~_BV(OCIE2);
  SET_COOLINGFAN_STATE(0);
 }
 else if (duty == PWM_STEPS)
 {
  TIMSK&=~_BV(OCIE2);
  SET_COOLINGFAN_STATE(1);
 }
 else
  TIMSK|=_BV(OCIE2);
}

/**T/C 2 Compare interrupt for renerating of PWM (cooling fan control)*/
ISR(TIMER2_COMP_vect)
{
 if (0 == pwm_state)
 { //start active part
  SET_COOLINGFAN_STATE(1);
  OCR2+= pwm_duty;
  ++pwm_state;
 }
 else
 { //start passive part
  SET_COOLINGFAN_STATE(0);
  OCR2+= (PWM_STEPS - pwm_duty);
  --pwm_state;
 }
}

//Control of electric cooling fan (engine cooling), only in case if coolant temperature
//sensor is present in system
void vent_control(struct ecudata_t *d)
{
 if (!d->param.tmp_use)
  return;

#ifndef COOLINGFAN_PWM
 if (d->sens.temperat >= d->param.vent_on)
  SET_COOLINGFAN_STATE(1);
 if (d->sens.temperat <= d->param.vent_off)
  SET_COOLINGFAN_STATE(0);
#else //PWM mode
 if (!d->param.vent_pwm)
 {
  if (d->sens.temperat >= d->param.vent_on)
   vent_set_duty(PWM_STEPS);
  if (d->sens.temperat <= d->param.vent_off)
   vent_set_duty(0);
 }
 else
 {
  //note: We skip 1 and 24 values of duty
  int16_t dd = d->param.vent_on - d->sens.temperat;
  if (dd < 2)
   dd = 0;         //restrict to max.
  if (dd > (PWM_STEPS-2))
   dd = PWM_STEPS; //restrict to min.
  //TODO: implement kick on turn on
  vent_set_duty(PWM_STEPS - dd);
 }
#endif
}
