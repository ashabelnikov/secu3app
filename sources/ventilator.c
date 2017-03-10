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

/** \file ventilator.c
 * \author Alexey A. Shabelnikov
 * Implementation of cooling fan's control related functions.
 * (Реализация функций для управления электрическим вентилятором).
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "ventilator.h"

/**Turns on/off cooling fan
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef COOLINGFAN_PWM
#ifdef REV9_BOARD
 #define COOLINGFAN_TURNON()  CLEARBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() SETBIT(PORTD, PD7)
#else //REV6
 #define COOLINGFAN_TURNON()  SETBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() CLEARBIT(PORTD, PD7)
#endif
#endif //COOLINGFAN_PWM

/**number of PWM discretes for 5kHz with 20mHz quartz */
#define PWM_STEPS 31

volatile uint8_t pwm_state;     //!< For state machine. 0 - passive, 1 - active
volatile uint16_t pwm_steps;    //!< number of timer ticks per PWM period
volatile uint16_t pwm_duty_1;   //!< current duty value (+)
volatile uint16_t pwm_duty_2;   //!< current duty value (-)
volatile uint8_t tmr2a_h;       //!< used for extending OCR2A to 16 bit

void vent_init_ports(void)
{
#ifdef COOLINGFAN_PWM
 IOCFG_INIT(IOP_ECF, 1); //coolong fan is turned Off
#endif
}

void vent_init_state(void)
{
 pwm_state = 0;  //begin from active level
 pwm_steps = 31;
 pwm_duty_1 = 0;
 pwm_duty_2 = 0;
 tmr2a_h = 0;
}

#ifdef COOLINGFAN_PWM
/**Sets duty value
 * \param duty value to be set
 */
void vent_set_duty(uint8_t duty)
{
 //TODO: Maybe we need double buffering?
 uint16_t duty_1 = ((uint32_t)duty * pwm_steps * 16) >> 12;

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  _DISABLE_INTERRUPT();
  TIMSK2&=~_BV(OCIE2A);
  _ENABLE_INTERRUPT();
  COOLINGFAN_TURNOFF();
 }
 else if (duty == 255)
 {
  _DISABLE_INTERRUPT();
  TIMSK2&=~_BV(OCIE2A);
  _ENABLE_INTERRUPT();
  COOLINGFAN_TURNON();
 }
 else
 {
  _DISABLE_INTERRUPT();
  TIMSK2|=_BV(OCIE2A);
  pwm_duty_1 = duty_1;
  if (0==_AB(pwm_duty_1, 0))                        //avoid strange bug which appears when OCR2A is set to the same value as TCNT2
   (_AB(pwm_duty_1, 0))++;
  pwm_duty_2 = pwm_steps - pwm_duty_1;
  if (0==_AB(pwm_duty_2, 0))                        //avoid strange bug which appears when OCR2A is set to the same value as TCNT2
   (_AB(pwm_duty_2, 0))++;
  _ENABLE_INTERRUPT();
 }
}

/**T/C 2 Compare interrupt for generating of PWM (cooling fan control)*/
ISR(TIMER2_COMPA_vect)
{
 if (tmr2a_h)
 {
  --tmr2a_h;
 }
 else
 {
  if (0 == pwm_state)
  { //start active part
   COOLINGFAN_TURNON();
   OCR2A = TCNT2 + _AB(pwm_duty_1, 0);
   tmr2a_h = _AB(pwm_duty_1, 1);
   ++pwm_state;
  }
  else
  { //start passive part
   COOLINGFAN_TURNOFF();
   OCR2A = TCNT2 + _AB(pwm_duty_2, 0);
   tmr2a_h = _AB(pwm_duty_2, 1);
   --pwm_state;
  }
 }
}
#endif

//Control of electric cooling fan (engine cooling), only in case if coolant temperature
//sensor is present in system
void vent_control(struct ecudata_t *d)
{
 //exit if coolant temperature sensor is disabled or there is no I/O assigned to
 //electric cooling fan
 if (!d->param.tmp_use || !IOCFG_CHECK(IOP_ECF))
  return;

#ifndef COOLINGFAN_PWM //control cooling fan by using relay only
 if (d->sens.temperat >= d->param.vent_on)
  IOCFG_SET(IOP_ECF, 1), d->cool_fan = 1; //turn on
 if (d->sens.temperat <= d->param.vent_off)
  IOCFG_SET(IOP_ECF, 0), d->cool_fan = 0; //turn off
#else //control cooling fan either by using relay or PWM
 if (!d->param.vent_pwm)
 { //relay
  //We don't need interrupts for relay control
  _DISABLE_INTERRUPT();
  TIMSK2&=~_BV(OCIE2A);
  _ENABLE_INTERRUPT();

  if (d->sens.temperat >= d->param.vent_on)
   IOCFG_SET(IOP_ECF, 1), d->cool_fan = 1; //turn on
  if (d->sens.temperat <= d->param.vent_off)
   IOCFG_SET(IOP_ECF, 0), d->cool_fan = 0; //turn off
 }
 else
 {
  uint16_t d_val;
  //note: We skip 1 and 24 values of duty
  int16_t dd = d->param.vent_on - d->sens.temperat;
  if (dd < 2)
   dd = 0;         //restrict to max.
  if (dd > (PWM_STEPS-2))
  {
   dd = PWM_STEPS; //restrict to min.
   d->cool_fan = 0; //turned off
  }
  else
   d->cool_fan = 1; //turned on

  d_val = ((uint16_t)(PWM_STEPS - dd) * 256) / PWM_STEPS;
  if (d_val > 255) d_val = 255;
  //TODO: implement kick on turn on
  vent_set_duty(d_val);
 }
#endif
}

void vent_turnoff(struct ecudata_t *d)
{
#ifndef COOLINGFAN_PWM
 IOCFG_SET(IOP_ECF, 0);
#else
 if (!d->param.vent_pwm && !IOCFG_CHECK(IOP_IAC_PWM) && !IOCFG_CHECK(IOP_GD_PWM))
  IOCFG_SET(IOP_ECF, 0);
 else
  COOLINGFAN_TURNOFF();
#endif
}

void vent_set_pwmfrq(uint16_t period)
{
 //period = 1/f * 524288
 //39062 = 156250/4
 pwm_steps = ((uint32_t)(39062 * period)) >> 17;
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
void vent_set_duty8(uint8_t duty)
{
#ifdef COOLINGFAN_PWM
 vent_set_duty(duty);
#endif
}
#endif
