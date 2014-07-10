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
#include "ioconfig.h"
#include "secu3.h"

/**Internal state variables */
typedef struct
{
 //states for hysteresis
 struct
 {
  uint8_t state1;             //!< current logic state of condition 1 execution
  uint8_t state2;             //!< current logic state of condition 2 execution
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
static uint8_t cond_cts(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, uint8_t state)
{
 if ((int16_t)on_thrd > (int16_t)off_thrd)
 {
  if (d->sens.temperat >= (int16_t)on_thrd)
   state = 1; //ON
  if (d->sens.temperat <= (int16_t)off_thrd)
   state = 0; //OFF
 }
 else
 { //has same effect as inversion
  if (d->sens.temperat <= (int16_t)on_thrd)
   state = 1; //ON
  if (d->sens.temperat >= (int16_t)off_thrd)
   state = 0; //OFF
 }
 return state;
}

/**Condition function for RPM*/
static uint8_t cond_rpm(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, uint8_t state)
{
 if (d->sens.inst_frq >= on_thrd)
  state = 1; //ON
 if (d->sens.inst_frq <= off_thrd)
  state = 0; //OFF
 return state;
}

/**Condition function for MAP sensor*/
static uint8_t cond_map(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, uint8_t state)
{
 if (d->sens.map >= on_thrd)
  state = 1; //ON
 if (d->sens.map <= off_thrd)
  state = 0; //OFF
 return state;
}

/**Condition function for board voltage*/
static uint8_t cond_volt(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, uint8_t state)
{
 if (d->sens.voltage >= on_thrd)
  state = 1; //ON
 if (d->sens.voltage <= off_thrd)
  state = 0; //OFF
 return state;
}

/**Condition function for carburetor's throttle limit switch*/
static uint8_t cond_carb(struct ecudata_t *d, uint16_t on_thrd, uint16_t off_thrd, uint8_t state)
{
 if (d->sens.carb == on_thrd)
  state = 1; //ON
 if (d->sens.carb == off_thrd)
  state = 0; //OFF
 return state;
}

/**Function pointer type used in function pointers tables (conditions)*/
typedef uint8_t (*cond_fptr_t)(struct ecudata_t*, uint16_t, uint16_t, uint8_t);

/**Number of function pointers in table*/
#define COND_FPTR_TABLE_SIZE 5

/**Table containing pointers to condition functions */
PGM_DECLARE(static cond_fptr_t cond_fptr[COND_FPTR_TABLE_SIZE]) = {&cond_cts, &cond_rpm, &cond_map, &cond_volt, &cond_carb};

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
 uni.states[index].state1 = state1 = cond_fptr[cond_1](d, p_out_param->on_thrd_1, p_out_param->off_thrd_1, uni.states[index].state1);
 if (15 != (p_out_param->flags >> 4))
  uni.states[index].state2 = state2 = cond_fptr[cond_2](d, p_out_param->on_thrd_2, p_out_param->off_thrd_2, uni.states[index].state2);

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
