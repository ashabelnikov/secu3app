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

#ifndef SECU3T
/**See ioconfig.c for more information*/
extern uint8_t spi_PORTB;
#endif

/**Turn on PWM channel (fast version)
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef SECU3T
 #define PWMx_TURNON_F(ch) switch(pwm2.iomode[ch]) \
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
 #define PWMx_TURNON_F(ch) switch(pwm2.iomode[ch]) \
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
  case 6: /*IGN_O3*/ \
   CLEARBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_O4*/ \
   CLEARBIT(PORTC, PC1); \
   break; \
  case 13: /*TACH_O*/ \
   SETBIT(PORTC, PC4); \
   break; \
 }
#endif

#ifdef DIAGNOSTICS
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
  /*following cases are used only by diagnistics: */ \
  case 4: /*IGN_OUT1*/ \
   CLEARBIT(PORTD, PD4); \
   break; \
  case 5: /*IGN_OUT2*/ \
   CLEARBIT(PORTD, PD5); \
   break; \
  case 6: /*IGN_OUT3*/ \
   CLEARBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_OUT4*/ \
   CLEARBIT(PORTC, PC1); \
   break; \
  case 8: /*ECF*/ \
   CLEARBIT(PORTD, PD7); \
   break; \
  case 9: /*CE*/ \
   CLEARBIT(PORTB, PB2); \
   break; \
  case 10: /*ST_BLOCK*/ \
   CLEARBIT(PORTB, PB1); \
   break; \
  case 11: /*IE - high side output*/ \
   SETBIT(PORTB, PB0); \
   break; \
  case 12: /*FE - high side output*/ \
   SETBIT(PORTC, PC7); \
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
  /*following cases are used only by diagnistics: */ \
  case 4: /*IGN_O1*/ \
   CLEARBIT(PORTD, PD4); \
   break; \
  case 5: /*IGN_O2*/ \
   CLEARBIT(PORTD, PD5); \
   break; \
  case 6: /*IGN_O3*/ \
   CLEARBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_O4*/ \
   CLEARBIT(PORTC, PC1); \
   break; \
  case 8: /*INJ_O1*/ \
   SETBIT(PORTB, PB1); \
   break; \
  case 9: /*INJ_O2*/ \
   SETBIT(PORTC, PC6); \
   break; \
  case 10: /*INJ_O3*/ \
   SETBIT(PORTB, PB2); \
   break; \
  case 11: /*INJ_O4*/ \
   SETBIT(PORTC, PC7); \
   break; \
  case 12: /*ECF*/ \
   SETBIT(PORTD, PD7); \
   break; \
  case 13: /*TACH_O*/ \
   SETBIT(PORTC, PC4); \
   break; \
  case 14: /*STBL_O*/ \
   SETBIT(spi_PORTB, 1); \
   break; \
  case 15: /*CEL_O*/ \
   SETBIT(spi_PORTB, 5); \
   break; \
  case 16: /*FPMP_O*/ \
   SETBIT(spi_PORTB, 3); \
   break; \
  case 17: /*PWRR_O*/ \
   SETBIT(spi_PORTB, 2); \
   break; \
  case 18: /*EVAP_O*/ \
   SETBIT(spi_PORTB, 6); \
   break; \
  case 19: /*O2SH_O*/ \
   SETBIT(spi_PORTB, 7); \
   break; \
  case 20: /*COND_O*/ \
   SETBIT(spi_PORTB, 4); \
   break; \
  case 21: /*ADD_O2*/ \
   SETBIT(spi_PORTB, 0); \
   break; \
 }
#endif

#endif //DIAGNOSTICS

/**Turn off PWM channel (fast version)
 * This is redundant definitions (see ioconfig.c), but it is opportunity to
 * speed up corresponding ISR
 */
#ifdef SECU3T
 #define PWMx_TURNOFF_F(ch) switch(pwm2.iomode[ch]) \
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
 #define PWMx_TURNOFF_F(ch) switch(pwm2.iomode[ch]) \
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
  case 6: /*IGN_O3*/ \
   SETBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_O4*/ \
   SETBIT(PORTC, PC1); \
   break; \
  case 13: /*TACH_O*/ \
   CLEARBIT(PORTC, PC4); \
   break; \
 }
#endif

#ifdef DIAGNOSTICS
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
  /*following cases are used only by diagnistics: */ \
  case 4: /*IGN_OUT1*/ \
   SETBIT(PORTD, PD4); \
   break; \
  case 5: /*IGN_OUT2*/ \
   SETBIT(PORTD, PD5); \
   break; \
  case 6: /*IGN_OUT3*/ \
   SETBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_OUT4*/ \
   SETBIT(PORTC, PC1); \
   break; \
  case 8: /*ECF*/ \
   SETBIT(PORTD, PD7); \
   break; \
  case 9: /*CE*/ \
   SETBIT(PORTB, PB2); \
   break; \
  case 10: /*ST_BLOCK*/ \
   SETBIT(PORTB, PB1); \
   break; \
  case 11: /*IE - high side output*/ \
   CLEARBIT(PORTB, PB0); \
   break; \
  case 12: /*FE - high side output*/ \
   CLEARBIT(PORTC, PC7); \
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
  /*following cases are used only by diagnistics: */ \
  case 4: /*IGN_O1*/ \
   SETBIT(PORTD, PD4); \
   break; \
  case 5: /*IGN_O2*/ \
   SETBIT(PORTD, PD5); \
   break; \
  case 6: /*IGN_O3*/ \
   SETBIT(PORTC, PC0); \
   break; \
  case 7: /*IGN_O4*/ \
   SETBIT(PORTC, PC1); \
   break; \
  case 8: /*INJ_O1*/ \
   CLEARBIT(PORTB, PB1); \
   break; \
  case 9: /*INJ_O2*/ \
   CLEARBIT(PORTC, PC6); \
   break; \
  case 10: /*INJ_O3*/ \
   CLEARBIT(PORTB, PB2); \
   break; \
  case 11: /*INJ_O4*/ \
   CLEARBIT(PORTC, PC7); \
   break; \
  case 12: /*ECF*/ \
   CLEARBIT(PORTD, PD7); \
   break; \
  case 13: /*TACH_O*/ \
   CLEARBIT(PORTC, PC4); \
   break; \
  case 14: /*STBL_O*/ \
   CLEARBIT(spi_PORTB, 1); \
   break; \
  case 15: /*CEL_O*/ \
   CLEARBIT(spi_PORTB, 5); \
   break; \
  case 16: /*FPMP_O*/ \
   CLEARBIT(spi_PORTB, 3); \
   break; \
  case 17: /*PWRR_O*/ \
   CLEARBIT(spi_PORTB, 2); \
   break; \
  case 18: /*EVAP_O*/ \
   CLEARBIT(spi_PORTB, 6); \
   break; \
  case 19: /*O2SH_O*/ \
   CLEARBIT(spi_PORTB, 7); \
   break; \
  case 20: /*COND_O*/ \
   CLEARBIT(spi_PORTB, 4); \
   break; \
  case 21: /*ADD_O2*/ \
   CLEARBIT(spi_PORTB, 0); \
   break; \
 }
#endif

#endif //DIAGNOSTICS

/**Describes interal state veriables */
typedef struct
{
 uint16_t pwm_steps[2];            //!< number of timer ticks per PWM period
 volatile uint8_t iomode[2];       //!< stores I/O configuration used in interrupts
 volatile uint8_t pwm_state[2];    //!< For state machine. 0 - passive, 1 - active
 volatile uint16_t pwm_duty_1[2];  //!< current duty value (+)
 volatile uint16_t pwm_duty_2[2];  //!< current duty value (-)
#ifdef FUEL_INJECT
 uint8_t  compa_mode;              //!< mode of functioning of COMPA channel: 0 - for PWM1; 1 - for generation of fuel consumption signal
#endif
 uint8_t  compb_mode;              //!< mode of functioning of COMPB channel: 0 - for PWM2; 1 - for generation of fuel consumption signal; 2 - for generation of pulses for tachometer
}pwm2_t;

/**Instance of internal state variables */
pwm2_t pwm2 = {{0,0},{255,255},{0,0},{0,0},{0,0},
#ifdef FUEL_INJECT
0,
#endif
0
};

void pwm2_init_ports(void)
{
 IOCFG_INIT(IOP_PWM1, 0); //PWM1 channel is turned Off
 IOCFG_INIT(IOP_PWM2, 0); //PWM2 channel is turned Off
#ifdef FUEL_INJECT
 IOCFG_INIT(IOP_FL_CONS, 0); // channel is turned Off
#endif
 IOCFG_INIT(IOP_VTACHOM, 0);
}

void pwm2_init_state(void)
{
 //note: it is also started in the ckps.c module (when option SPLIT_ANGLE is active)
 TCCR3B = _BV(CS31) | _BV(CS30); //start timer, clock  = 312.5 kHz

 //in firmware with SPLIT_ANGLE COMPA channel of timer 3 is busy by CKPS algorithm and can be used here for PWM1, FL_CONS and VTACHOM
#ifndef SPLIT_ANGLE
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
 if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out3 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out3i)
  pwm2.iomode[0] = 6; //IGN_O3
 if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out4 || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_ign_out4i)
  pwm2.iomode[0] = 7; //IGN_O4
 if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_tach_o || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_tach_oi)
  pwm2.iomode[0] = 13; //TACH_O
#endif
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_bli)
  pwm2.iomode[0] = 2; //BL
 else if (IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_PWM1)==(fnptr_t)iocfg_s_dei)
  pwm2.iomode[0] = 3; //DE
#ifdef FUEL_INJECT
 //FL_CONS:
#ifdef SECU3T
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o1 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o1i)
  {pwm2.iomode[0] = 0; pwm2.compa_mode = 1;} //ADD_O1
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o2 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o2i)
  {pwm2.iomode[0] = 1; pwm2.compa_mode = 1;} //ADD_O2
#else //SECU-3i
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out5 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out5i)
  {pwm2.iomode[0] = 0; pwm2.compa_mode = 1;} //IGN_O5
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_inj_out5 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_inj_out5i)
  {pwm2.iomode[0] = 1; pwm2.compa_mode = 1;} //INJ_O5
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out3 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out3i)
  {pwm2.iomode[0] = 6; pwm2.compa_mode = 1;} //IGN_O3
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out4 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out4i)
  {pwm2.iomode[0] = 7; pwm2.compa_mode = 1;} //IGN_O4
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_tach_o || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_tach_oi)
  {pwm2.iomode[0] = 13; pwm2.compa_mode = 1;} //TACH_O
#endif
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_bli)
  {pwm2.iomode[0] = 2; pwm2.compa_mode = 1;} //BL
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_dei)
  {pwm2.iomode[0] = 3; pwm2.compa_mode = 1;} //DE
#endif
 else
 { //No I/O assigned, disable interrupt
  CLEARBIT(TIMSK3, OCIE3A);
 }
#endif

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
 if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out3 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out3i)
  pwm2.iomode[1] = 6; //IGN_O3
 if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out4 || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_ign_out4i)
  pwm2.iomode[1] = 7; //IGN_O4
 if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_tach_o || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_tach_oi)
  pwm2.iomode[1] = 13; //TACH_O
#endif
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_bli)
  pwm2.iomode[1] = 2; //BL
 else if (IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_PWM2)==(fnptr_t)iocfg_s_dei)
  pwm2.iomode[1] = 3; //DE
#if defined(FUEL_INJECT) && defined(SPLIT_ANGLE)
 //FL_CONS:
#ifdef SECU3T
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o1 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o1i)
  {pwm2.iomode[1] = 0; pwm2.compb_mode = 1;} //ADD_O1
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o2 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_add_o2i)
  {pwm2.iomode[1] = 1; pwm2.compb_mode = 1;} //ADD_O2
#else //SECU-3i
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out5 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out5i)
  {pwm2.iomode[1] = 0; pwm2.compb_mode = 1;} //IGN_O5
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_inj_out5 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_inj_out5i)
  {pwm2.iomode[1] = 1; pwm2.compb_mode = 1;} //INJ_O5
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out3 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out3i)
  {pwm2.iomode[1] = 6; pwm2.compb_mode = 1;} //IGN_O3
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out4 || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_ign_out4i)
  {pwm2.iomode[1] = 7; pwm2.compb_mode = 1;} //IGN_O4
 if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_tach_o || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_tach_oi)
  {pwm2.iomode[1] = 13; pwm2.compb_mode = 1;} //TACH_O
#endif
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_bli)
  {pwm2.iomode[1] = 2; pwm2.compb_mode = 1;} //BL
 else if (IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_FL_CONS)==(fnptr_t)iocfg_s_dei)
  {pwm2.iomode[1] = 3; pwm2.compb_mode = 1;} //DE
#endif
 //VTACHOM:
#ifdef SECU3T
 else if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_add_o1 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_add_o1i)
  {pwm2.iomode[1] = 0; pwm2.compb_mode = 2;} //ADD_O1
 else if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_add_o2 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_add_o2i)
  {pwm2.iomode[1] = 1; pwm2.compb_mode = 2;} //ADD_O2
#else //SECU-3i
 if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out5 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out5i)
  {pwm2.iomode[1] = 0; pwm2.compb_mode = 2;} //IGN_O5
 else if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_inj_out5 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_inj_out5i)
  {pwm2.iomode[1] = 1; pwm2.compb_mode = 2;} //INJ_O5
 if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out3 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out3i)
  {pwm2.iomode[1] = 6; pwm2.compb_mode = 2;} //IGN_O3
 if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out4 || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_ign_out4i)
  {pwm2.iomode[1] = 7; pwm2.compb_mode = 2;} //IGN_O4
 if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_tach_o || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_tach_oi)
  {pwm2.iomode[1] = 13; pwm2.compb_mode = 2;} //TACH_O
#endif
 else if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_bl || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_bli)
  {pwm2.iomode[1] = 2; pwm2.compb_mode = 2;} //BL
 else if (IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_de || IOCFG_CB(IOP_VTACHOM)==(fnptr_t)iocfg_s_dei)
  {pwm2.iomode[1] = 3; pwm2.compb_mode = 2;} //DE
 else
 { //No I/O assigned, disable interrupt
  CLEARBIT(TIMSK3, OCIE3B);
 }
}

#ifndef SPLIT_ANGLE
//When SPLIT_ANGLE is active this ISR is used in ckps.c for ign. splitting
/**T/C 3 Compare interrupt for generating of PWM (channel 0)*/
ISR(TIMER3_COMPA_vect)
{
 if (0 == pwm2.pwm_state[0])
 { //start active part
  PWMx_TURNON_F(0); //<--use fast version
  OCR3A = TCNT3 + pwm2.pwm_duty_1[0];
  ++pwm2.pwm_state[0];
 }
 else
 { //start passive part
  PWMx_TURNOFF_F(0); //<--use fast version
  OCR3A = TCNT3 + pwm2.pwm_duty_2[0];
  --pwm2.pwm_state[0];
 }
}
#endif

/**T/C 3 Compare interrupt for generating of PWM (channel 1)*/
ISR(TIMER3_COMPB_vect)
{
 if (0 == pwm2.pwm_state[1])
 { //start active part
#ifdef DIAGNOSTICS
  PWMx_TURNON(1);
#else
  PWMx_TURNON_F(1); //<--use fast version
#endif
  OCR3B = TCNT3 + pwm2.pwm_duty_1[1];
  ++pwm2.pwm_state[1];
 }
 else
 { //start passive part
#ifdef DIAGNOSTICS
  PWMx_TURNOFF(1);
#else
  PWMx_TURNOFF_F(1); //<--use fast version
#endif
  OCR3B = TCNT3 + pwm2.pwm_duty_2[1];
  --pwm2.pwm_state[1];
 }
}

/**Set PWM duty for specified channel
 * \param ch Number of channel to be configured (0 or 1)
 * \param duty 8-bit PWM duty value (0...255)
 */
#ifdef DIAGNOSTICS
void pwm2_set_duty8(uint8_t ch, uint8_t duty)
#else
static void pwm2_set_duty8(uint8_t ch, uint8_t duty)
#endif
{
 uint16_t duty_1 = ((uint32_t)duty * pwm2.pwm_steps[ch] * 16) >> 12;
 uint16_t duty_2 = pwm2.pwm_steps[ch] - duty_1;

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  if (0==ch)
  {
#ifndef SPLIT_ANGLE
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3A);  //disable interrupt for ch0
   _ENABLE_INTERRUPT();
#endif
  }
  else
  { //1
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3B); //disable interrupt for ch1
   _ENABLE_INTERRUPT();
  }
#ifdef DIAGNOSTICS
  PWMx_TURNOFF(ch);      //fully OFF
#else
  PWMx_TURNOFF_F(ch);    //fully OFF, fast version
#endif
 }
 else if (duty == 255)
 {
  if (0==ch)
  {
#ifndef SPLIT_ANGLE
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3A); //disable interrupt for ch0
   _ENABLE_INTERRUPT();
#endif
  }
  else
  { //1
   _DISABLE_INTERRUPT();
   TIMSK3&=~_BV(OCIE3B); //disable interrupt for ch1
   _ENABLE_INTERRUPT();
  }
#ifdef DIAGNOSTICS
  PWMx_TURNON(ch);       //fully ON
#else
  PWMx_TURNON_F(ch);     //fully ON, fast version
#endif
 }
 else
 {
  if (0==ch)
  {
#ifndef SPLIT_ANGLE
   _DISABLE_INTERRUPT();
   SETBIT(TIMSK3, OCIE3A);
   pwm2.pwm_duty_1[0] = duty_1;
   pwm2.pwm_duty_2[0] = duty_2;
   _ENABLE_INTERRUPT();
#endif
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

#ifdef FUEL_INJECT
/**Set fuel consumption signal's frequency and duty for COMPA channel
 * \param ch Number of channel to be configured (0 or 1)
 */
static void pwm2_set_fl_cons(uint8_t ch)
{
 //see inject_calc_fuel_flow() in injector.c for more information
 uint16_t duty_1 = ((uint32_t)((256 * 312500) / 2)) / d.inj_fff;

 if (0==ch) //use channel A
 {
  if (d.inj_fff < 614)     //614 = 2.4 * 256 (2.4Hz is the minimum frequency)
  { //no fuel flow
   _DISABLE_INTERRUPT();
    TIMSK3&=~_BV(OCIE3A);  //disable interrupt for COMPA
   _ENABLE_INTERRUPT();
    PWMx_TURNOFF_F(0);     //fully OFF
  }
  else
  { //produce fuel consumption signal
   _DISABLE_INTERRUPT();
    SETBIT(TIMSK3, OCIE3A);
    pwm2.pwm_duty_1[0] = duty_1;
    pwm2.pwm_duty_2[0] = duty_1;
   _ENABLE_INTERRUPT();
  }
 }
 else //use channel B
 {
  if (d.inj_fff < 614)     //614 = 2.4 * 256 (2.4Hz is the minimum frequency)
  { //no fuel flow
   _DISABLE_INTERRUPT();
    TIMSK3&=~_BV(OCIE3B);  //disable interrupt for COMPA
   _ENABLE_INTERRUPT();
    PWMx_TURNOFF_F(1);     //fully OFF
  }
  else
  { //produce fuel consumption signal
   _DISABLE_INTERRUPT();
    SETBIT(TIMSK3, OCIE3B);
    pwm2.pwm_duty_1[1] = duty_1;
    pwm2.pwm_duty_2[1] = duty_1;
   _ENABLE_INTERRUPT();
  }
 }
}
#endif

/**Set VTACHOM signal frequency depending on RPM and settings
 */
static void pwm2_set_vtachom(void)
{
 uint16_t rpm = d.sens.aver_rpm;
 if (0==rpm)
 { //engine stopped
  _DISABLE_INTERRUPT();
  CLEARBIT(TIMSK3, OCIE3B);
  _ENABLE_INTERRUPT();
  PWMx_TURNOFF_F(1);     //fully OFF
  return;
 }
 if (rpm < 150)
  rpm = 150; //prevent overflow in the following calculations

 //9375000 = (312500*60)/2
 uint32_t duty = ((((uint32_t)9375000) / rpm) * PGM_GET_WORD(&fw_data.exdata.vtachom_mult)) >> 13;
 if (duty > 65535)
  duty = 65535; //prevent possible overflow
 uint16_t duty_1 = duty;
 uint16_t duty_2 = duty;

 _DISABLE_INTERRUPT();
 SETBIT(TIMSK3, OCIE3B);
 pwm2.pwm_duty_1[1] = duty_1;
 pwm2.pwm_duty_2[1] = duty_2;
 _ENABLE_INTERRUPT();
}

void pwm2_control(void)
{
#ifndef SPLIT_ANGLE
 if (pwm2.iomode[0] != 255)
 { //ch 0
#ifdef FUEL_INJECT
  if (0==pwm2.compa_mode)
   pwm2_set_duty8(0, pwm_function(0)); //generate PWM
  else //=1
   pwm2_set_fl_cons(0);                //generate fuel consumption signal (flequency with 50% duty)
#else
  pwm2_set_duty8(0, pwm_function(0));  //generate PWM
#endif
 }
#endif
 if (pwm2.iomode[1] != 255)
 { //ch 1
#if defined(FUEL_INJECT)
  if (0==pwm2.compb_mode)
   pwm2_set_duty8(1, pwm_function(1)); //generate PWM
  else if (1==pwm2.compb_mode)
   pwm2_set_fl_cons(1);                //generate fuel consumption signal (flequency with 50% duty)
  else if (2==pwm2.compb_mode)
   pwm2_set_vtachom();                 //genarate pulses for tachometer (VTACHOM output)
#else
  if (0==pwm2.compb_mode)
   pwm2_set_duty8(1, pwm_function(1));
  else if (2==pwm2.compb_mode)
   pwm2_set_vtachom();                 //genarate pulses for tachometer (VTACHOM output)
#endif
 }
}

void pwm2_set_pwmfrq(uint8_t ch, uint16_t period)
{
 //period must be <= 53000 (~10Hz minimum frequency)
 //period = 1/f * 524288; 524288 = 2^19
 //78125 = 312500/4; 4 = 2^2
 pwm2.pwm_steps[ch] = ((uint32_t)(78125UL * period)) >> (19-2);
}

#ifdef DIAGNOSTICS
void pwm2_set_diag_iomode(uint8_t iomode)
{
 pwm2.iomode[1] = iomode;
}
#endif
