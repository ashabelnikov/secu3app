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

/** \file uni_out.c
 * Implementation of a universal programmable output.
 */

#ifdef UNI_OUTPUT

#include "port/port.h"
#include <string.h>
#include "ioconfig.h"
#include "secu3.h"
#include "vstimer.h"

/**Defines state variables for condition*/
typedef struct
{
 uint8_t state;              //!< current logic state of condition execution
 uint8_t other;              //!< state of other condition (used for linking two conditions together)
 uint16_t tmr;               //!< used for timer(s) implementation
 uint8_t sm;                 //!< for state machine
}out_state_t;

/**Internal state variables */
typedef struct
{
 struct
 {
  out_state_t ctx1;          //!< state variables for condition 1
  out_state_t ctx2;          //!< state variables for condition 2
 }states[UNI_OUTPUT_NUMBER];
}uni_out_state_t;

/**Instance of internal state variables structure*/
static uni_out_state_t uni;

/** Condition function for coolant temperature sensor
 * \param d pointer to ECU data structure
 * \param on_thrd ON threshold, must be in temperature units
 * \param off_thrd OFF threshold, must be in temperature units
 * \param state Current logic state (0 or 1)
 * \return updated logic state (0 or 1)
 */
static uint8_t cond_cts(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if ((int16_t)on_thrd > (int16_t)off_thrd)
 {
  if (d->sens.temperat >= (int16_t)on_thrd)
   p_ctx->state = 1; //ON
  if (d->sens.temperat <= (int16_t)off_thrd)
   p_ctx->state = 0; //OFF
 }
 else
 { //has same effect as inversion
  if (d->sens.temperat <= (int16_t)on_thrd)
   p_ctx->state = 1; //ON
  if (d->sens.temperat >= (int16_t)off_thrd)
   p_ctx->state = 0; //OFF
 }
 return p_ctx->state;
}

/**Condition function for RPM*/
static uint8_t cond_rpm(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.inst_frq >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.inst_frq <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for MAP sensor*/
static uint8_t cond_map(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.map >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.map <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for board voltage*/
static uint8_t cond_volt(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.voltage >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.voltage <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for carburetor's throttle limit switch*/
static uint8_t cond_carb(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.carb == on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.carb == off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for vehicle speed*/
static uint8_t cond_vspd(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
#ifdef SPEED_SENSOR
 if (d->sens.speed >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.speed <= off_thrd)
  p_ctx->state = 0; //OFF
#endif
 return p_ctx->state;
}

/**Condition function for air flow*/
static uint8_t cond_airfl(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->airflow >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->airflow <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for timer (starts to run after triggering of other(first) condition).
 * this function is used only as second condition */
static uint8_t cond_tmr(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 switch(p_ctx->sm)
 {
  case 0:
   if (p_ctx->other)
   {
    p_ctx->tmr = s_timer_gtc();
    p_ctx->sm = 1;
   }
   p_ctx->state = 0;
   break;
  case 1:
   if ((s_timer_gtc() - p_ctx->tmr) >= on_thrd)
   {
    p_ctx->state = 1;  //ON
    if (off_thrd != 60000)  //do not turn off it if OFF threshold set to maximum
    {
     p_ctx->tmr = s_timer_gtc();
     p_ctx->sm = 2;
    }
   }
   break;
  case 2:
   if ((s_timer_gtc() - p_ctx->tmr) >= off_thrd)
   {
    p_ctx->state = 0;  //OFF
   }
   break;
 }

 if (!p_ctx->other)
  p_ctx->sm = 0;

 return p_ctx->state;
}

/**Condition function for timer which starts to run after ignition turn on*/
static uint8_t cond_ittmr(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 switch(p_ctx->sm)
 {
  case 0:
   p_ctx->tmr = s_timer_gtc();
   p_ctx->sm = 1;
   p_ctx->state = 0;
   break;
  case 1:
   if ((s_timer_gtc() - p_ctx->tmr) >= on_thrd)
   {
    p_ctx->state = 1;  //ON
    if (off_thrd != 60000)  //do not turn off it if OFF threshold set to maximum
    {
     p_ctx->tmr = s_timer_gtc();
     p_ctx->sm = 2;
    }
   }
   break;
  case 2:
   if ((s_timer_gtc() - p_ctx->tmr) >= off_thrd)
   {
    p_ctx->state = 0;  //OFF
   }
   break;
 }

 return p_ctx->state;
}

/**Condition function for timer which starts to run after starting of engine*/
static uint8_t cond_estmr(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 switch(p_ctx->sm)
 {
  case 0:
   if (d->st_block)
   { //engine is running
    p_ctx->tmr = s_timer_gtc();
    p_ctx->sm = 1;
   }
   p_ctx->state = 0;
   break;
  case 1:
   if ((s_timer_gtc() - p_ctx->tmr) >= on_thrd)
   {
    p_ctx->state = 1;  //ON
    if (off_thrd != 60000)  //do not turn off it if OFF threshold set to maximum
    {
     p_ctx->tmr = s_timer_gtc();
     p_ctx->sm = 2;
    }
   }
   break;
  case 2:
   if ((s_timer_gtc() - p_ctx->tmr) >= off_thrd)
   {
    p_ctx->state = 0;  //OFF
   }
   break;
 }

 if (!d->st_block)     //Has engine stopped?
  p_ctx->sm = 0;

 return p_ctx->state;
}

/**Condition function for choke/idling actuator position */
static uint8_t cond_cpos(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->choke_pos >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->choke_pos <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for advance angle */
static uint8_t cond_aang(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->corr.curr_angle >= (int16_t)on_thrd)
  p_ctx->state = 1; //ON
 if (d->corr.curr_angle <= (int16_t)off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for knock signal level */
static uint8_t cond_klev(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.knock_k >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.knock_k <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for TPS */
static uint8_t cond_tps(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.tps >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.tps <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/** Condition function for intake air temperature sensor
 * \param d pointer to ECU data structure
 * \param on_thrd ON threshold, must be in temperature units
 * \param off_thrd OFF threshold, must be in temperature units
 * \param state Current logic state (0 or 1)
 * \return updated logic state (0 or 1)
 */
static uint8_t cond_ats(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
#if defined(AIRTEMP_SENS) && defined(SECU3T)
 if ((int16_t)on_thrd > (int16_t)off_thrd)
 {
  if (d->sens.air_temp >= (int16_t)on_thrd)
   p_ctx->state = 1; //ON
  if (d->sens.air_temp <= (int16_t)off_thrd)
   p_ctx->state = 0; //OFF
 }
 else
 { //has same effect as inversion
  if (d->sens.air_temp <= (int16_t)on_thrd)
   p_ctx->state = 1; //ON
  if (d->sens.air_temp >= (int16_t)off_thrd)
   p_ctx->state = 0; //OFF
 }
#endif
 return p_ctx->state;
}

/**Condition function for ADD_I1 analog input */
static uint8_t cond_ai1(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.add_i1 >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.add_i1 <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for ADD_I2 analog input */
static uint8_t cond_ai2(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.add_i2 >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.add_i2 <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for gas valve input*/
static uint8_t cond_gasv(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->sens.gas == on_thrd)
  p_ctx->state = 1; //ON
 if (d->sens.gas == off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Condition function for injector PW */
static uint8_t cond_ipw(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
#ifdef FUEL_INJECT
 if (d->inj_pw >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->inj_pw <= off_thrd)
  p_ctx->state = 0; //OFF
#endif
 return p_ctx->state;
}

/**Condition function for CE state */
static uint8_t cond_ce(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, out_state_t* p_ctx)
{
 if (d->ce_state >= on_thrd)
  p_ctx->state = 1; //ON
 if (d->ce_state <= off_thrd)
  p_ctx->state = 0; //OFF
 return p_ctx->state;
}

/**Function pointer type used in function pointers tables (conditions)*/
typedef uint8_t (*cond_fptr_t)(struct ecudata_t*, uint16_t, uint16_t, out_state_t*);

/**Number of function pointers in table*/
#define COND_FPTR_TABLE_SIZE 20

/**Table containing pointers to condition functions */
PGM_DECLARE(static cond_fptr_t cond_fptr[COND_FPTR_TABLE_SIZE]) =
 {&cond_cts, &cond_rpm, &cond_map, &cond_volt, &cond_carb, &cond_vspd, &cond_airfl, &cond_tmr, &cond_ittmr,
  &cond_estmr, &cond_cpos, &cond_aang, &cond_klev, &cond_tps, &cond_ats, &cond_ai1, &cond_ai2, &cond_gasv,
  &cond_ipw, &cond_ce};

void uniout_init_ports(void)
{
 IOCFG_INIT(IOP_UNI_OUT0, 0); //turned OFF
 IOCFG_INIT(IOP_UNI_OUT1, 0); //turned OFF
 IOCFG_INIT(IOP_UNI_OUT2, 0); //turned OFF
}

void uniout_init(void)
{
 //Default state of all outputs is OFF
 memset(&uni.states, 0, sizeof(uni.states));
}

/** Processes (executes assigned conditions and updates state) specified output
 * \param d pointer to ECU data structure
 * \param index output index
 */
static void process_output(struct ecudata_t *d, uint8_t index)
{
 uni_output_t* p_out_param = &d->param.uni_output[index];
 uint8_t cond_1 = (p_out_param->condition1 < COND_FPTR_TABLE_SIZE) ? p_out_param->condition1 : 0;
 uint8_t cond_2 = (p_out_param->condition2 < COND_FPTR_TABLE_SIZE) ? p_out_param->condition2 : 0;
 uint8_t state1, state2;

 //execute specified conditions
 state1 = cond_fptr[cond_1](d, p_out_param->on_thrd_1, p_out_param->off_thrd_1, &uni.states[index].ctx1);
 if (15 != (p_out_param->flags >> 4))
 {
  uni.states[index].ctx2.other = state1;
  state2 = cond_fptr[cond_2](d, p_out_param->on_thrd_2, p_out_param->off_thrd_2, &uni.states[index].ctx2);
 }

 //apply inversion flags
 state1^=p_out_param->flags & 0x1;
 state2^=(p_out_param->flags >> 1) & 0x2;

 //apply specified logic function and update output state
 switch(p_out_param->flags >> 4)
 {
  case 0:  IOCFG_SET(index + IOP_UNI_OUT0, state1 | state2); break; //OR
  case 1:  IOCFG_SET(index + IOP_UNI_OUT0, state1 & state2); break; //AND
  case 2:  IOCFG_SET(index + IOP_UNI_OUT0, state1 ^ state2); break; //XOR
  case 15: IOCFG_SET(index + IOP_UNI_OUT0, state1); break; //first condition only
 }
}

void uniout_control(struct ecudata_t *d)
{
 uint8_t i = 0;
 //Process 3 outputs, we process them only if they are active (remapped to real I/O)
 for(; i < UNI_OUTPUT_NUMBER; ++i)
  if (IOCFG_CHECK(IOP_UNI_OUT0 + i))
   process_output(d, i);
}

#endif //UNI_OUTPUT
