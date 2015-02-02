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

/** \file injector.c
 * Implementation of fuel injector control
 * (Реализация управления топливной форсункой).
 */

#ifdef FUEL_INJECT

#include "port/port.h"
#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "bitmask.h"
#include "injector.h"
#include "ioconfig.h"
#include "tables.h"

#ifndef _PLATFORM_M644_
 #error "You can not use FUEL_INJECT option without _PLATFORM_M644_"
#endif

#define INJ_ON  0   //!< Injector is turned on
#define INJ_OFF 1   //!< Injector is turned off

/**COMPB interrupt calibration*/
#define INJ_COMPB_CALIB 2

/** Define injector state variables structure*/
typedef struct
{
 volatile uint16_t inj_time;     //!< Current injection time, used in interrupts
 volatile uint8_t tmr2b_h;       //!< used in timer2 COMPB interrupt to perform 16 timing
 volatile uint8_t cyl_number;    //!< number of engine cylinders
 uint8_t  num_squirts;           //!< number of squirts per cycle
 uint8_t  cur_cyl;               //!< current cylinder
 uint8_t  sqr_cyl;               //!< current cylinder for squirt
 volatile uint8_t cyl_inc;       //!< used to calculate next number of cylinder for squirt
 volatile uint8_t fuelcut;       //!< fuelcut flag
}inj_state_t;

/** Global instance of injector state variable structure*/
inj_state_t inj;

void inject_init_state(void)
{
 inj.inj_time = 0xFFFF;
 inj.sqr_cyl = 0;
 inj.cur_cyl = 0;
 inj.fuelcut = 1;  //no fuel cut
}

void inject_init_ports(void)
{
 IOCFG_INIT(IOP_INJ_OUT0, INJ_OFF);          //injector 1 is turned off
 IOCFG_INIT(IOP_INJ_OUT1, INJ_OFF);          //injector 2 is turned off
 IOCFG_INIT(IOP_INJ_OUT2, INJ_OFF);          //injector 3 is turned off
 IOCFG_INIT(IOP_INJ_OUT3, INJ_OFF);          //injector 4 is turned off
}

void inject_set_cyl_number(uint8_t cylnum)
{
 _BEGIN_ATOMIC_BLOCK();
 inj.cyl_number = cylnum;
 inj.cyl_inc = cylnum / inj.num_squirts;
 _END_ATOMIC_BLOCK();
}

void inject_set_num_squirts(uint8_t numsqr)
{
 inj.num_squirts = numsqr;
 inj.cyl_inc = inj.cyl_number / numsqr;
}

void inject_set_inj_time(uint16_t time)
{
 if (time < 16)                              //restrict minimum injection time to 50uS
  time = 16;
 time = (time >> 1) - INJ_COMPB_CALIB;       //subtract calibration ticks
 if (0==_AB(time, 0))                        //avoid strange bug which appears when OCR2B is set to the same value as TCNT2
  (_AB(time, 0))++;

 _BEGIN_ATOMIC_BLOCK();
 inj.inj_time = time;
 _END_ATOMIC_BLOCK();
}

void inject_set_fuelcut(uint8_t state)
{
 inj.fuelcut = state;
}

void inject_start_inj(void)
{
 if (!inj.fuelcut)
  return; //fuel is OFF
 if (inj.cur_cyl == inj.sqr_cyl)
 {
  //interrupts must be disabled!
  _BEGIN_ATOMIC_BLOCK();
  OCR2B = TCNT2 + _AB(inj.inj_time, 0);
  inj.tmr2b_h = _AB(inj.inj_time, 1);
  IOCFG_SET(IOP_INJ_OUT0, INJ_ON);            //turn on injector 1
  IOCFG_SET(IOP_INJ_OUT1, INJ_ON);            //turn on injector 2
  IOCFG_SET(IOP_INJ_OUT2, INJ_ON);            //turn on injector 3
  IOCFG_SET(IOP_INJ_OUT3, INJ_ON);            //turn on injector 4

  SETBIT(TIMSK2, OCIE2B);
  SETBIT(TIFR2, OCF2B);                       //reset possible pending interrupt flag
  _END_ATOMIC_BLOCK();
  inj.sqr_cyl+= inj.cyl_inc;
  if (inj.sqr_cyl >= inj.cyl_number)
   inj.sqr_cyl = 0;
 }
 if (++inj.cur_cyl == inj.cyl_number)
  inj.cur_cyl = 0;
}

void inject_open_inj(uint16_t time)
{
  if (0==time) return;
  time = (time >> 1) - INJ_COMPB_CALIB;
  if (0==_AB(time, 0))
   (_AB(time, 0))++;
  _BEGIN_ATOMIC_BLOCK();
  OCR2B = TCNT2 + _AB(time, 0);
  inj.tmr2b_h = _AB(time, 1);
  IOCFG_SET(IOP_INJ_OUT0, INJ_ON);            //turn on injector 1
  IOCFG_SET(IOP_INJ_OUT1, INJ_ON);            //turn on injector 2
  IOCFG_SET(IOP_INJ_OUT2, INJ_ON);            //turn on injector 3
  IOCFG_SET(IOP_INJ_OUT3, INJ_ON);            //turn on injector 4
  SETBIT(TIMSK2, OCIE2B);
  SETBIT(TIFR2, OCF2B);                       //reset possible pending interrupt flag
  _END_ATOMIC_BLOCK();
}

/**Interrupt for controlling of first injector*/
ISR(TIMER2_COMPB_vect)
{
 if (inj.tmr2b_h)
 {
  --inj.tmr2b_h;
 }
 else
 {
  IOCFG_SET(IOP_INJ_OUT0, INJ_OFF);          //turn off injector 1
  IOCFG_SET(IOP_INJ_OUT1, INJ_OFF);          //turn off injector 2
  IOCFG_SET(IOP_INJ_OUT2, INJ_OFF);          //turn off injector 3
  IOCFG_SET(IOP_INJ_OUT3, INJ_OFF);          //turn off injector 4
  CLEARBIT(TIMSK2, OCIE2B);                  //disable this interrupt
 }
}

//todo: we can try to free TIMER2_COMPA_VECT for second injector by switching cooling fan PWM to TIMER0_COMPB
//ISR(TIMER2_COMPA_vect)
//{
//}

#endif
