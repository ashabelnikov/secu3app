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
#ifdef SECU3T /*SECU-3T*/

#ifdef REV9_BOARD
 #define COOLINGFAN_TURNON()  CLEARBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() SETBIT(PORTD, PD7)
#else //REV6
 #define COOLINGFAN_TURNON()  SETBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() CLEARBIT(PORTD, PD7)
#endif

#else         /*SECU-3*/
 #define COOLINGFAN_TURNON()  SETBIT(PORTB, PB1)
 #define COOLINGFAN_TURNOFF() CLEARBIT(PORTB, PB1)
#endif
#endif //COOLINGFAN_PWM

#ifdef _PLATFORM_M644_
 /** Timer reload value for 20mHz quartz
  * Warning must be the same as another definition in vstimer.h!*/
 #define TIMER2_RELOAD_VALUE  0
 /**number of PWM discretes for 5kHz with 20mHz quartz */
 #define PWM_STEPS 31
#else
 #define TIMER2_RELOAD_VALUE  6    //!< Timer reload value for 16mHz quartz
 #define PWM_STEPS 25              //!< Number of PWM discretes for 16mHz quartz
#endif

/**will be added to TIMER2_RELOAD_VALUE at the initialization */
#define COMPADD 5

volatile uint8_t pwm_state;     //!< For state machine. 0 - passive, 1 - active
#ifdef _PLATFORM_M644_
volatile uint16_t pwm_steps;    //!< number of timer ticks per PWM period
volatile uint16_t pwm_duty_1;   //!< current duty value (+)
volatile uint16_t pwm_duty_2;   //!< current duty value (-)
volatile uint8_t tmr2a_h;       //!< used for extending OCR2A to 16 bit
#else
volatile uint8_t pwm_duty;      //!< current duty value
#endif

void vent_init_ports(void)
{
#if defined(COOLINGFAN_PWM) && defined(SECU3T)
 IOCFG_INIT(IOP_ECF, 1); //coolong fan is turned Off
#else //relay only or SECU-3
 IOCFG_INIT(IOP_ECF, 0); //coolong fan is turned Off
#endif
}

void vent_init_state(void)
{
 pwm_state = 0;  //begin from active level
#ifdef _PLATFORM_M644_
 pwm_steps = 31;
 pwm_duty_1 = 0;
 pwm_duty_2 = 0;
 tmr2a_h = 0;
 OCR2A = TIMER2_RELOAD_VALUE + COMPADD;
#else
 pwm_duty = 0;   // 0%
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
#endif
}

#ifdef COOLINGFAN_PWM
/**Sets duty value
 * \param duty value to be set
 */
void vent_set_duty(uint8_t duty)
{
#ifdef _PLATFORM_M644_
 uint16_t duty_1 = ((uint32_t)duty * pwm_steps * 16) >> 12;
#else
 //TODO: Maybe we need double buffering?
 pwm_duty = duty;
#endif

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  _DISABLE_INTERRUPT();
#ifdef _PLATFORM_M644_
  TIMSK2&=~_BV(OCIE2A);
#else
  TIMSK&=~_BV(OCIE2);
#endif
  _ENABLE_INTERRUPT();
  COOLINGFAN_TURNOFF();
 }
#ifdef _PLATFORM_M644_
 else if (duty == 255)
#else
 else if (duty == PWM_STEPS)
#endif
 {
  _DISABLE_INTERRUPT();
#ifdef _PLATFORM_M644_
  TIMSK2&=~_BV(OCIE2A);
#else
  TIMSK&=~_BV(OCIE2);
#endif
  _ENABLE_INTERRUPT();
  COOLINGFAN_TURNON();
 }
 else
 {
  _DISABLE_INTERRUPT();
#ifdef _PLATFORM_M644_
  TIMSK2|=_BV(OCIE2A);
  pwm_duty_1 = duty_1;
  if (0==_AB(pwm_duty_1, 0))                        //avoid strange bug which appears when OCR2A is set to the same value as TCNT2
   (_AB(pwm_duty_1, 0))++;
  pwm_duty_2 = pwm_steps - pwm_duty_1;
  if (0==_AB(pwm_duty_2, 0))                        //avoid strange bug which appears when OCR2A is set to the same value as TCNT2
   (_AB(pwm_duty_2, 0))++;
#else
  TIMSK|=_BV(OCIE2);
#endif
  _ENABLE_INTERRUPT();
 }
}

/**T/C 2 Compare interrupt for generating of PWM (cooling fan control)*/
#ifdef _PLATFORM_M644_
ISR(TIMER2_COMPA_vect)
#else
ISR(TIMER2_COMP_vect)
#endif
{
#ifdef _PLATFORM_M644_
 if (tmr2a_h)
 {
  --tmr2a_h;
 }
 else
 {
#endif
  if (0 == pwm_state)
  { //start active part
   COOLINGFAN_TURNON();
#ifdef _PLATFORM_M644_
   OCR2A = TCNT2 + _AB(pwm_duty_1, 0);
   tmr2a_h = _AB(pwm_duty_1, 1);
#else
   OCR2+= pwm_duty;
#endif
   ++pwm_state;
  }
  else
  { //start passive part
   COOLINGFAN_TURNOFF();
#ifdef _PLATFORM_M644_
   OCR2A = TCNT2 + _AB(pwm_duty_2, 0);
   tmr2a_h = _AB(pwm_duty_2, 1);
#else
   OCR2+= (PWM_STEPS - pwm_duty);
#endif
   --pwm_state;
  }
#ifdef _PLATFORM_M644_
 }
#endif
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
#ifdef _PLATFORM_M644_
  TIMSK2&=~_BV(OCIE2A);
#else
  TIMSK&=~_BV(OCIE2);
#endif
  _ENABLE_INTERRUPT();

  if (d->sens.temperat >= d->param.vent_on)
   IOCFG_SET(IOP_ECF, 1), d->cool_fan = 1; //turn on
  if (d->sens.temperat <= d->param.vent_off)
   IOCFG_SET(IOP_ECF, 0), d->cool_fan = 0; //turn off
 }
 else
 {
#ifdef _PLATFORM_M644_
  uint16_t d_val;
#endif
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

#ifdef _PLATFORM_M644_
  d_val = ((uint16_t)(PWM_STEPS - dd) * 256) / PWM_STEPS;
  if (d_val > 255) d_val = 255;
  //TODO: implement kick on turn on
  vent_set_duty(d_val);
#else
  vent_set_duty(PWM_STEPS - dd);
#endif
 }
#endif
}

void vent_turnoff(struct ecudata_t *d)
{
#ifndef COOLINGFAN_PWM
 IOCFG_SET(IOP_ECF, 0);
#else
 if (!d->param.vent_pwm && !IOCFG_CHECK(IOP_IAC_PWM))
  IOCFG_SET(IOP_ECF, 0);
 else
  COOLINGFAN_TURNOFF();
#endif
}

#ifdef _PLATFORM_M644_
void vent_set_pwmfrq(uint16_t period)
{
 //period = 1/f * 524288
 //39062 = 156250/4
 pwm_steps = ((uint32_t)(39062 * period)) >> 17;
}

#ifdef FUEL_INJECT
void vent_set_duty8(uint8_t duty)
{
#ifdef COOLINGFAN_PWM
 vent_set_duty(duty);
#endif
}
#endif
#endif
