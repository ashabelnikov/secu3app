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

/**COMPB interrupt calibration*/
#define INJ_COMPB_CALIB 2

/** Define injector state variables structure*/
typedef struct
{
 volatile uint16_t inj_time;     //!< Current injection time, used in interrupts
 volatile uint8_t tmr2b_h;       //!< used in timer2 COMPB interrupt to perform 16 timing
}inj_state_t;

/** Global instance of injector state variable structure*/
inj_state_t inj;

void inject_init_state(void)
{
 inj.inj_time = 0xFFFF;
}

void inject_init_ports(void)
{
 IOCFG_INIT(IOP_INJ_OUT0, 0);                //injector is turned off
}

void inject_set_inj_time(uint16_t time)
{
 if (time < 16)                              //restrict minimum injection time to 50uS
  time = 16;
 time = (time >> 1) - INJ_COMPB_CALIB - 1;   //calibration ticks and 1 tick to ensure correct behaviour when low byte is 0
 _BEGIN_ATOMIC_BLOCK();
 inj.inj_time = time;
 _END_ATOMIC_BLOCK();
}

void inject_start_inj(void)
{
 //interrupts must be disabled!
 _BEGIN_ATOMIC_BLOCK();
 OCR2B = TCNT2 + _AB(inj.inj_time, 0) + 1;
 inj.tmr2b_h = _AB(inj.inj_time, 1);
 IOCFG_SET(IOP_INJ_OUT0, 1);                 //turn on injector
 SETBIT(TIMSK2, OCIE2B);
 SETBIT(TIFR2, OCF2B);                       //reset possible pending interrupt flag
 _END_ATOMIC_BLOCK();
}

/**Interrupt fo controlling of first injector*/
ISR(TIMER2_COMPB_vect)
{
 if (inj.tmr2b_h)
 {
  --inj.tmr2b_h;
 }
 else
 {
  IOCFG_SET(IOP_INJ_OUT0, 0);                //turn off injector
  CLEARBIT(TIMSK2, OCIE2B);                  //disable this interrupt
 }
}

//todo: we can try to free TIMER2_COMPA_VECT for second injector by switching cooling fan PWM to TIMER0_COMPB
//ISR(TIMER2_COMPA_vect)
//{
//}

#endif
