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

/** \file vstimer.c
 * \author Alexey A. Shabelnikov
 * Implementation of virtual system timers
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "ce_errors.h"
#include "ioconfig.h" //for SM_CONTROL
#include "tables.h"
#include "vstimer.h"
#include "knock.h" //for knock_start_expander_latching()

/**Reload count for system timer's divider, to obtain approximately 10 ms from 1.6384 ms,
 frequency will be divided by 6 */
#define DIVIDER_RELOAD       5

static volatile uint16_t sys_counter = 0;                 //!< system tick counter, 1 tick = 10ms
static volatile uint8_t diagnostics = 0;                  //!< diagnostics flag
static uint8_t divider = DIVIDER_RELOAD;                  //!< for division, to achieve 10ms, because timer overflovs each 2 ms
static uint16_t strokes_since_start = 0;                  //!< number of strokes since engine start

#ifdef SM_CONTROL
//See smcontrol.c
extern volatile uint16_t sm_steps;
extern volatile uint8_t sm_latch;
extern volatile uint16_t sm_steps_b;
extern uint8_t sm_pulse_state;
extern volatile uint16_t sm_steps_cnt;
extern volatile uint8_t sm_freq;
uint8_t sm_divider = 0;
#endif

#ifdef GD_CONTROL
//See gdcontrol.c
extern volatile uint16_t gdsm_steps;
extern volatile uint8_t gdsm_latch;
extern uint16_t gdsm_steps_b;
extern uint8_t gdsm_pulse_state;
extern volatile uint16_t gdsm_steps_cnt;
extern volatile uint8_t gd_freq;
uint8_t gd_divider = 0;
#endif

#ifdef CARB_AFR
//See carb_afr.c
extern uint8_t cafr_iv_comp;
extern volatile uint8_t cafr_iv_duty;
extern uint8_t cafr_pv_comp;
extern volatile uint8_t cafr_pv_duty;
extern uint8_t cafr_soft_cnt;
#endif

#if defined(EVAP_CONTROL) && !defined(SECU3T)
//see evap.c for more information
extern uint8_t evap_comp;
extern volatile uint8_t evap_duty;
extern uint8_t evap_soft_cnt;
#endif

#if defined(COOLINGFAN_PWM) && !defined(SECU3T)
//see ventilator.c for more information
extern uint8_t vent_comp;
extern volatile uint8_t vent_duty;
extern uint8_t vent_soft_cnt;
#endif

uint8_t divider_sens = 0;

/**Interrupt routine which called when T/C 2 overflovs - used for counting time intervals in system
 *(for generic usage). Called each 1.6384ms. System tick is 10ms, and so we divide frequency by 6
 */
ISR(TIMER2_OVF_vect)
{
 _ENABLE_INTERRUPT();

#ifdef DIAGNOSTICS
if (!diagnostics) {
#endif

#ifdef SM_CONTROL
 if (sm_divider > 0)
  --sm_divider;
 else
 {
  sm_divider = sm_freq;

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
    ++sm_steps_cnt; //count processed steps
   }
  }
 }
#endif

#ifdef GD_CONTROL
 if (gd_divider > 0)
  --gd_divider;
 else
 {
  gd_divider = gd_freq;

  if (!gdsm_pulse_state && gdsm_latch) {
   gdsm_steps_b = gdsm_steps;
   gdsm_latch = 0;
  }

  if (gdsm_steps_b)
  {
   if (!gdsm_pulse_state) {
    IOCFG_SET(IOP_GD_STP, 1); //falling edge
    gdsm_pulse_state = 1;
   }
   else {//The step occurs on the rising edge of ~CLOCK signal
    IOCFG_SET(IOP_GD_STP, 0); //rising edge
    gdsm_pulse_state = 0;
    --gdsm_steps_b;
    ++gdsm_steps_cnt; //count processed steps
   }
  }
 }
#endif

#ifdef CARB_AFR
 cafr_soft_cnt = (cafr_soft_cnt + 1) & 0x3F; //increment modulo 64
 if (cafr_soft_cnt == 0)
 {
  cafr_iv_comp = cafr_iv_duty;
  cafr_pv_comp = cafr_pv_duty;
  IOCFG_SET(IOP_IE, 1); //ON
  IOCFG_SET(IOP_FE, 1); //ON
 }
 if (cafr_iv_comp == cafr_soft_cnt)
  IOCFG_SET(IOP_IE, 0); //OFF
 if (cafr_pv_comp == cafr_soft_cnt)
  IOCFG_SET(IOP_FE, 0); //OFF
#endif

#ifdef DIAGNOSTICS
}
#endif

//PWM for canister purge valve control (~19 Hz, 32 discretes)
#if defined(EVAP_CONTROL) && !defined(SECU3T)
 if (evap_duty)
 {
  evap_soft_cnt = (evap_soft_cnt + 1) & 0x1F; //increment modulo 32
  if (evap_soft_cnt == 0)
  {
   evap_comp = evap_duty;
   IOCFG_SET(IOP_EVAP_O, 1); //ON
  }
  if (evap_comp == evap_soft_cnt)
   IOCFG_SET(IOP_EVAP_O, 0); //OFF
 }
#endif

//Low frequency for cooling fan's PWM
#if defined(COOLINGFAN_PWM) && !defined(SECU3T)
 if (vent_duty)
 {
  vent_soft_cnt = (vent_soft_cnt + 1) & 0x0F; //increment modulo 16, frequency will be 39Hz
  if (vent_soft_cnt == 0)
  {
   vent_comp = vent_duty;
   IOCFG_SET(IOP_ECF, 1); //OFF
  }
  if (vent_comp == vent_soft_cnt)
   IOCFG_SET(IOP_ECF, 0); //ON
 }
#endif

 if (divider > 0)
  --divider;
 else
 {//each 10 ms
  divider = DIVIDER_RELOAD;
  ++sys_counter;
 }

 _DISABLE_INTERRUPT();
#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
 knock_start_expander_latching();
#endif

 if (divider_sens == 0)
  adc_begin_measure();   //each 6.56ms
 divider_sens = (divider_sens + 1) & 0x03; //increment modulo 4
}

void s_timer_init(void)
{
 TCCR2B|= _BV(CS22)|_BV(CS20); //clock = 156.25kHz (tick = 6.4us)
 TCNT2 = 0;
 TIMSK2|= _BV(TOIE2);
}

void s_timer_set(s_timer16_t* ctx, uint16_t time)
{
 _BEGIN_ATOMIC_BLOCK();
 ctx->start = sys_counter;
 _END_ATOMIC_BLOCK();
 ctx->time = time;
 ctx->action = 0;
}

uint8_t s_timer_is_action(s_timer16_t* ctx)
{
 if (ctx->action)
  return 1; //already fired

 uint16_t current;
 _BEGIN_ATOMIC_BLOCK();
 current = sys_counter;
 _END_ATOMIC_BLOCK();

 if ((current - ctx->start) > ctx->time)
 {
  ctx->action = 1;
  return 1; //fire!
 }

 return 0; //still not fired
}

uint16_t s_timer_gtc(void)
{
 uint16_t result;
 _BEGIN_ATOMIC_BLOCK();
 result = sys_counter;
 _END_ATOMIC_BLOCK();
 return result;
}

#ifdef DIAGNOSTICS
void s_timer_enter_diag(void)
{
 diagnostics = 1;
}
#endif

uint16_t s_timer_sss(void)
{
 return strokes_since_start;
}

void s_timer_stroke_event_notification(void)
{
 if (strokes_since_start < 65535)
  ++strokes_since_start; //update strokes counter (counts up to 65535 and stops)
}

void s_timer_eng_stopped_notification(void)
{
 strokes_since_start = 0; //reset strokes counter
}
