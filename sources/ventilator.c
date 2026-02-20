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
 */

#include <stdlib.h>
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "ioconfig.h"
#include "ventilator.h"
#include "vstimer.h"
#include "magnitude.h"

/**Turns on/off cooling fan
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef COOLINGFAN_PWM
#if defined(REV9_BOARD) || !defined(SECU3T)
 #define COOLINGFAN_TURNON()  CLEARBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() SETBIT(PORTD, PD7)
#else //REV6
 #define COOLINGFAN_TURNON()  SETBIT(PORTD, PD7)
 #define COOLINGFAN_TURNOFF() CLEARBIT(PORTD, PD7)
#endif
//ETC (SECU-3i only)
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 #define ETC_PWM2_TURNON() CLEARBIT(PORTC, PC5)
 #define ETC_PWM2_TURNOFF() SETBIT(PORTC, PC5)
#endif
#endif //COOLINGFAN_PWM

volatile uint8_t pwm_state;     //!< For state machine. 0 - passive, 1 - active
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
volatile uint8_t pwm_mode = 0;
#endif
volatile uint16_t pwm_steps;    //!< number of timer ticks per PWM period
volatile uint16_t pwm_duty_1;   //!< current duty value (+)
volatile uint16_t pwm_duty_2;   //!< current duty value (-)
volatile uint8_t tmr2a_h;       //!< used for extending OCR2A to 16 bit
uint16_t vent_tmr;
uint8_t vent_tmrexp;
uint16_t vent_tmr1;             //!< used for delay
uint8_t vent_delst;
#ifdef AIRCONDIT
uint8_t acss_state = 0;         //!< State machine used for soft start (air conditioner)
uint16_t acss_tmr;
uint8_t acss_duty;
#endif

#if defined(COOLINGFAN_PWM) && !defined(SECU3T)
//see vstimer.c for more information
uint8_t vent_comp = 0;
volatile uint8_t vent_duty = 0; //!< By default PWM is disabled
uint8_t vent_soft_cnt = 0;
#endif

void vent_init_ports(void)
{
#if defined(COOLINGFAN_PWM)
 IOCFG_INIT(IOP_ECF, 1); //cooling fan is turned Off
#else //relay only
 IOCFG_INIT(IOP_ECF, 0); //cooling fan is turned Off
#endif
}

void vent_init_state(void)
{
 pwm_state = 0;  //begin from active level
 pwm_steps = PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps);
 pwm_duty_1 = 0;
 pwm_duty_2 = 0;
 tmr2a_h = 0;
 vent_tmr = s_timer_gtc();
 vent_tmrexp = 0;
 vent_tmr1 = 0;
 vent_delst = 0;
#ifdef AIRCONDIT
 acss_state = 0;
 acss_tmr = s_timer_gtc();
 acss_duty = 0;
#endif
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 pwm_mode = 0;
#endif
}

#ifdef COOLINGFAN_PWM
/**Sets duty value
 * \param duty value to be set
 */
void vent_set_duty(int16_t duty, uint8_t bits)
{
 //TODO: Maybe we need double buffering?
 uint16_t duty_1 = ((uint32_t)abs(duty) * pwm_steps) >> bits;

#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 if (IOCFG_CHECK(IOP_ETC_PWM2))
 {//signed
  //We don't need interrupts if duty is 0, 100% or -100%
  if (duty == 0)
  {
   _DISABLE_INTERRUPT();
   TIMSK2&=~_BV(OCIE2A);
   _ENABLE_INTERRUPT();
   COOLINGFAN_TURNOFF();
   ETC_PWM2_TURNOFF();
  }
  else if (duty >= ((int16_t)_BV16(bits)-1)) //100%
  {
   _DISABLE_INTERRUPT();
   TIMSK2&=~_BV(OCIE2A);
   _ENABLE_INTERRUPT();
   COOLINGFAN_TURNON();
   ETC_PWM2_TURNOFF();
  }
  else if (duty <= -((int16_t)_BV16(bits)-1)) //-100%
  {
   _DISABLE_INTERRUPT();
   TIMSK2&=~_BV(OCIE2A);
   _ENABLE_INTERRUPT();
   COOLINGFAN_TURNOFF();
   ETC_PWM2_TURNON();
  }
  else
  {
   if (duty < 0)
   {
    pwm_mode = 1;
    COOLINGFAN_TURNOFF();
   }
   else
   {
    ETC_PWM2_TURNOFF();
    pwm_mode = 0;
   }
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
 else
#endif
 {
  //We don't need interrupts if duty is 0 or 100%
  if (duty == 0)
  {
   _DISABLE_INTERRUPT();
   TIMSK2&=~_BV(OCIE2A);
   _ENABLE_INTERRUPT();
   COOLINGFAN_TURNOFF();
  }
  else if (duty >= _BV16(bits)-1)
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
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
  if (pwm_mode == 1)
  {
   if (0 == pwm_state)
   { //start active part
    ETC_PWM2_TURNON();
    OCR2A = TCNT2 + _AB(pwm_duty_1, 0);
    tmr2a_h = _AB(pwm_duty_1, 1);
    ++pwm_state;
   }
   else
   { //start passive part
    ETC_PWM2_TURNOFF();
    OCR2A = TCNT2 + _AB(pwm_duty_2, 0);
    tmr2a_h = _AB(pwm_duty_2, 1);
    --pwm_state;
   }
  }
  else
#endif
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
}
#endif

//Control of electric cooling fan (engine cooling), only in case if coolant temperature
//sensor is present in system
void vent_control(void)
{
 //exit if coolant temperature sensor is disabled or there is no I/O assigned to
 //electric cooling fan
 if (!CHECKBIT(d.param.tmp_flags, TMPF_CLT_USE) || !IOCFG_CHECK(IOP_ECF))
  return;

 //set flag if timer is expired
 if ((0 != d.param.vent_tmr) && ((s_timer_gtc() - vent_tmr) > d.param.vent_tmr))
  vent_tmrexp = 1;

#ifndef COOLINGFAN_PWM //control cooling fan by using relay only
 if (!vent_tmrexp && (d.sens.temperat >= d.param.vent_on
#ifdef AIRCONDIT
     || d.cond_req_fan  //always fully turn on cooling fan if request from air conditioner exists
#endif
    ))
 {
  if (0==vent_delst)
  { //delay
   vent_tmr1 = s_timer_gtc();
   vent_delst++;
   d.vent_req_on = 1;
  }
  else if (1==vent_delst)
  { //delay has been expired
   if ((s_timer_gtc() - vent_tmr1) > PGM_GET_BYTE(&fw_data.exdata.vent_delay))
    IOCFG_SETF(IOP_ECF, 1), d.cool_fan = 1; //turn on
  }
 }
 else if (vent_tmrexp || d.sens.temperat <= d.param.vent_off)
 {
  IOCFG_SETF(IOP_ECF, 0), d.cool_fan = 0; //turn off
  vent_delst = 0;
  d.vent_req_on = 0;
 }
#else //control cooling fan either by using relay or PWM
 if (!CHECKBIT(d.param.tmp_flags, TMPF_VENT_PWM))
 { //relay
  //We don't need interrupts for relay control
  _DISABLE_INTERRUPT();
  TIMSK2&=~_BV(OCIE2A);
  _ENABLE_INTERRUPT();
#ifndef SECU3T
  vent_duty = 0; //disable software PWM
#endif

  if (!vent_tmrexp && (d.sens.temperat >= d.param.vent_on
#ifdef AIRCONDIT
     || d.cond_req_fan  //always fully turn on cooling fan if request from air conditioner exists
#endif
     ))
  {
   if (0==vent_delst)
   { //delay
    vent_tmr1 = s_timer_gtc();
    vent_delst++;
    d.vent_req_on = 1;
   }
   else if (1==vent_delst)
   { //delay has been expired
    if ((s_timer_gtc() - vent_tmr1) > PGM_GET_BYTE(&fw_data.exdata.vent_delay))
     IOCFG_SETF(IOP_ECF, 1), d.cool_fan = 1; //turn on
   }
  }
  else if (vent_tmrexp || d.sens.temperat <= d.param.vent_off)
  {
   IOCFG_SETF(IOP_ECF, 0), d.cool_fan = 0; //turn off
   vent_delst = 0;
   d.vent_req_on = 0;
  }
  d.vent_duty = 0;
 }
 else
 { //PWM
  uint16_t d_val;
  int16_t dd = d.param.vent_on - d.sens.temperat;

  //use hysteresis when a cooling fan is going to fully turn off
  if (d.cool_fan && dd > (PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps)-PGM_GET_BYTE(&fw_data.exdata.vent_minband)))
   dd = (d.param.vent_on - (int16_t)PGM_GET_BYTE(&fw_data.exdata.vent_pwm_turnoff_hyst)) - d.sens.temperat;

#ifdef AIRCONDIT
  //smoothly turn on cooling fan if request from air conditioner exists
   if (0==d.cond_req_fan)
   {
    acss_state = 0;
    acss_duty = 0;
   }
   else
   {
    if (0==acss_state)
    { //start soft starting
     acss_tmr = s_timer_gtc();
     acss_duty = PGM_GET_BYTE(&fw_data.exdata.vent_minband);
     acss_state++;  //=1
    }
    else if (1==acss_state)
    { //transition
     uint16_t period = SYSTIM_MAGS(3.0) / (PGM_GET_BYTE(&fw_data.exdata.vent_maxband) - PGM_GET_BYTE(&fw_data.exdata.vent_minband));
     if ((s_timer_gtc() - acss_tmr) > period)
     {
      acss_tmr = s_timer_gtc();
      acss_duty++;  //=2
     }
     if (acss_duty >= PGM_GET_BYTE(&fw_data.exdata.vent_maxband))
      acss_state++; //=3
    }
   }
  int16_t aa = PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps)-acss_duty;
  if (aa < dd) //select duty from air conditioner only if it greater than duty from temperature
   dd = aa;
#endif

  if (dd < 0)
   dd = 0;         //restrict to max.
  if (vent_tmrexp || dd > (PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps)-PGM_GET_BYTE(&fw_data.exdata.vent_minband)))
  {
   dd = PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps); //restrict to min.
   d.cool_fan = 0; //turned off
  }
  else
   d.cool_fan = 1; //turned on

  //save duty in % for later display (1 discrete = 0.5%)
  d.vent_duty = ((uint16_t)(PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps) - dd) * 200) / PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps);

#ifndef SECU3T
  if (IOCFG_CMP(IOP_ADD_O2, IOP_ECF) || IOCFG_CMP(IOP_O2SH_O, IOP_ECF) || IOCFG_CMP(IOP_PWRRELAY, IOP_ECF))
  { //low frequency software PWM
   vent_duty = dd;
   if (vent_duty == 0)
    IOCFG_SETF(IOP_ECF, 0); //turn on
  }
  else
  { //high frequency PWM
   vent_duty = 0; //disable software PWM
   d_val = ((uint16_t)(PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps) - dd) * 256) / PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps);
   if (d_val > 255) d_val = 255;
   vent_set_duty(d_val, 8);
  }
#else //SECU-3T
  d_val = ((uint16_t)(PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps) - dd) * 256) / PGM_GET_BYTE(&fw_data.exdata.vent_pwmsteps);
  if (d_val > 255) d_val = 255;
  vent_set_duty(d_val, 8);
#endif

 }
#endif
}

void vent_turnoff(void)
{
#ifndef COOLINGFAN_PWM
 IOCFG_SETF(IOP_ECF, 0);
#else
 if (!CHECKBIT(d.param.tmp_flags, TMPF_VENT_PWM))
  IOCFG_SETF(IOP_ECF, 0);
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 if (IOCFG_CHECK(IOP_ETC_PWM1) && IOCFG_CHECK(IOP_ETC_PWM2))
 { //ETC mode
  COOLINGFAN_TURNOFF(); //ETC_PWM1
  ETC_PWM2_TURNOFF();   //ETC_PWM2
 }
 else
#endif
 { //others
  if (CHECKBIT(d.param.tmp_flags, TMPF_VENT_PWM) || (IOCFG_CHECK(IOP_IAC_PWM) || IOCFG_CHECK(IOP_GD_PWM)))
   COOLINGFAN_TURNOFF();
 }
#endif
}

void vent_set_pwmfrq(uint16_t period)
{
 //period = 1/f * 524288
 //39062 = 156250/4
 pwm_steps = ((uint32_t)(39062 * period)) >> 17;
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL) || (!defined(SECU3T) && defined(ELEC_THROTTLE))
void vent_set_duty12(int16_t duty)
{
#ifdef COOLINGFAN_PWM
 vent_set_duty(duty, 12);
#endif
}
#endif

void vent_cog_changed_notification(void)
{
 vent_tmr = s_timer_gtc();
 vent_tmrexp = 0;
}
