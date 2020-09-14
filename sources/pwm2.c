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

/** \file pwm2.c
 * \author Alexey A. Shabelnikov
 * Implementation of additional 2 versatile PWM channels.
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "funconv.h"
#include "ioconfig.h"
#include "pwm2.h"
#include "vstimer.h"
#include "ioconfig.h"

/**Turn on PWM channel
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef SECU3T
 #define PWMx_TURNON(ch) switch(pwm2.iomode[ch]) \
 { \
  case 0: /*ADD_O1*/ \
   CLEARBIT(PORTC, PC5); \
   break; \
  case 1: /*ADD_O2*/ \
   CLEARBIT(PORTA, PA4); \
   break; \
  case 2: /*BL*/ \
   SETBIT(PORTC, PC3); \
   break; \
  case 3: /*DE*/ \
   SETBIT(PORTC, PC2); \
   break; \
 }
#else //SECU-3i
 #define PWMx_TURNON(ch) switch(pwm2.iomode[ch]) \
 { \
  case 0: /*IGN_O5*/ \
   CLEARBIT(PORTC, PC5); \
   break; \
  case 1: /*INJ_O5*/ \
   SETBIT(PORTB, PB0); \
   break; \
  case 2: /*BL*/ \
   SETBIT(PORTC, PC3); \
   break; \
  case 3: /*DE*/ \
   SETBIT(PORTC, PC2); \
   break; \
 }
#endif

/**Turn off PWM channel
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef SECU3T
 #define PWMx_TURNOFF(ch) switch(pwm2.iomode[ch]) \
 { \
  case 0: /*ADD_O1*/ \
   SETBIT(PORTC, PC5); \
   break; \
  case 1: /*ADD_O2*/ \
   SETBIT(PORTA, PA4); \
   break; \
  case 2: /*BL*/ \
   CLEARBIT(PORTC, PC3); \
   break; \
  case 3: /*DE*/ \
   CLEARBIT(PORTC, PC2); \
   break; \
 }
#else //SECU-3i
 #define PWMx_TURNOFF(ch) switch(pwm2.iomode[ch]) \
 { \
  case 0: /*IGN_O5*/ \
   SETBIT(PORTC, PC5); \
   break; \
  case 1: /*INJ_O5*/ \
   CLEARBIT(PORTB, PB0); \
   break; \
  case 2: /*BL*/ \
   CLEARBIT(PORTC, PC3); \
   break; \
  case 3: /*DE*/ \
   CLEARBIT(PORTC, PC2); \
   break; \
 }
#endif

/**Describes interal state veriables */
typedef struct
{
 uint16_t pwm_steps[2];            //!< number of timer ticks per PWM period
 volatile uint8_t iomode[2];       //!< stores I/O configuration used in interrupts
 volatile uint8_t pwm_state[2];    //!< For state machine. 0 - passive, 1 - active
 volatile uint16_t pwm_duty_1[2];  //!< current duty value (+)
 volatile uint16_t pwm_duty_2[2];  //!< current duty value (-)
}pwm2_t;

/**Instance of internal state variables */
pwm2_t pwm2 = {{0,0},{255,255},{0,0},{0,0},{0,0}};

void pwm2_init_ports(void)
{
 IOCFG_INIT(IOP_PWM1, 0); //PWM1 channel is turned Off
 IOCFG_INIT(IOP_PWM2, 0); //PWM2 channel is turned Off
}

void pwm2_init_state(void)
{
 TCCR3B = _BV(CS31) | _BV(CS30); //start timer, clock  = 312.5 kHz

 //PWM1:
#ifdef SECU3T
 if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_add_o1 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_add_o1i)
  pwm2.iomode[0] = 0; //ADD_O1
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_add_o2 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_add_o2i)
  pwm2.iomode[0] = 1; //ADD_O2
#else //SECU-3i
 if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out5 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out5i)
  pwm2.iomode[0] = 0; //IGN_O5
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_inj_out5 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_inj_out5i)
  pwm2.iomode[0] = 1; //INJ_O5
#endif
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_bli)
  pwm2.iomode[0] = 2; //BL
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_dei)
  pwm2.iomode[0] = 3; //DE
 else
 { //No I/O assigned, disable interrupt
  CLEARBIT(TIMSK3, OCIE3A);
 }

 //PWM2:
#ifdef SECU3T
 if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_add_o1 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_add_o1i)
  pwm2.iomode[1] = 0; //ADD_O1
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_add_o2 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_add_o2i)
  pwm2.iomode[1] = 1; //ADD_O2
#else //SECU-3i
 if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out5 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out5i)
  pwm2.iomode[1] = 0; //IGN_O5
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_inj_out5 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_inj_out5i)
  pwm2.iomode[1] = 1; //INJ_O5
#endif
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_bli)
  pwm2.iomode[1] = 2; //BL
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_dei)
  pwm2.iomode[1] = 3; //DE
 else
 { //No I/O assigned, disable interrupt
  CLEARBIT(TIMSK3, OCIE3B);
 }
}

/**T/C 3 Compare interrupt for generating of PWM (channel 0)*/
ISR(TIMER3_COMPA_vect)
{
 if (0 == pwm2.pwm_state[0])
 { //start active part
  PWMx_TURNON(0);
  OCR3A = TCNT3 + pwm2.pwm_duty_1[0];
  ++pwm2.pwm_state[0];
 }
 else
 { //start passive part
  PWMx_TURNOFF(0);
  OCR3A = TCNT3 + pwm2.pwm_duty_2[0];
  --pwm2.pwm_state[0];
 }
}

/**T/C 3 Compare interrupt for generating of PWM (channel 1)*/
ISR(TIMER3_COMPB_vect)
{
 if (0 == pwm2.pwm_state[1])
 { //start active part
  PWMx_TURNON(1);
  OCR3B = TCNT3 + pwm2.pwm_duty_1[1];
  ++pwm2.pwm_state[1];
 }
 else
 { //start passive part
  PWMx_TURNOFF(1);
  OCR3B = TCNT3 + pwm2.pwm_duty_2[1];
  --pwm2.pwm_state[1];
 }
}

/**Set PWM duty for specified channel
 * \param ch Number of channel to be configured (0 or 1)
 * \param duty 8-bit PWM duty value (0...255)
 */
static void pwm2_set_duty8(uint8_t ch, uint8_t duty)
{
 uint16_t duty_1 = ((uint32_t)duty * pwm2.pwm_steps[ch] * 16) >> 12;
 uint16_t duty_2 = pwm2.pwm_steps[ch] - duty_1;

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  if (0==ch)
  {
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3A);  //disable interrupt for ch0
   _ENABLE_INTERRUPT();
  }
  else
  { //1
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3B); //disable interrupt for ch1
   _ENABLE_INTERRUPT();
  }
  PWMx_TURNOFF(ch);      //fully OFF
 }
 else if (duty == 255)
 {
  if (0==ch)
  {
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3A); //disable interrupt for ch0
   _ENABLE_INTERRUPT();
  }
  else
  { //1
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3B); //disable interrupt for ch1
   _ENABLE_INTERRUPT();
  }
  PWMx_TURNON(ch);       //fully ON
 }
 else
 {
  if (0==ch)
  {
   _DISABLE_INTERRUPT();
   SETBIT(TIMSK3, OCIE3A);
   pwm2.pwm_duty_1[0] = duty_1;
   pwm2.pwm_duty_2[0] = duty_2;
   _ENABLE_INTERRUPT();
  }
  else
  { //1
   _DISABLE_INTERRUPT();
   SETBIT(TIMSK3, OCIE3B);
   pwm2.pwm_duty_1[1] = duty_1;
   pwm2.pwm_duty_2[1] = duty_2;
   _ENABLE_INTERRUPT();
  }
 }
}

void pwm2_control(void)
{
 if (pwm2.iomode[0] != 255)
 { //ch 0
  pwm2_set_duty8(0, pwm_function(0));
 }
 if (pwm2.iomode[1] != 255)
 { //ch 1
  pwm2_set_duty8(1, pwm_function(1));
 }
}

void pwm2_set_pwmfrq(uint8_t ch, uint16_t period)
{
 //period must be <= 53000 (~10Hz minimum frequency)
 //period = 1/f * 524288; 524288 = 2^19
 //78125 = 312500/4; 4 = 2^2
 pwm2.pwm_steps[ch] = ((uint32_t)(78125UL * period)) >> (19-2);
}
