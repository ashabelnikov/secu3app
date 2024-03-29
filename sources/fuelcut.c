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

/** \file fuelcut.c
 * \author Alexey A. Shabelnikov
 * Implementation of controling of Idle Cut-off valve control (Carburetor) or fuel cut control (Fuel injection).
 */

#if !defined(CARB_AFR) || defined(GD_CONTROL)
#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ecudata.h"
#include "fuelcut.h"
#include "ioconfig.h"
#include "vstimer.h"
#endif //!CARB_AFR || GD_CONTROL

/**Idle cut off delay after cranking, 5 seconds*/
#define ICOCD 500

/**Idle cut off timer. Used by idle cut off controlling algorithm */
s_timer16_t epxx_delay_time_counter = {0,0,1}; //already fired!

#if !defined(CARB_AFR) || defined(GD_CONTROL)
/**Used to disable idle cut off during some time after start of engine*/
s_timer16_t idlecutoff_crnk_delay = {0,ICOCD,0};
#endif

#if defined(CARB_AFR) && defined(FUEL_INJECT)
 #error "You can not use FUEL_INJECT option together with CARB_AFR"
#endif

/** Get fuel cut low threshold depending on gas_v input
 * \return low threshold
 */
static uint16_t get_fc_lot(void)
{
 return d.sens.gas ? d.param.ie_lot_g : d.param.ie_lot;
}

/** Get fuel cut high threshold depending on gas_v input
 * \return high threshold
 */
static uint16_t get_fc_hit(void)
{
 return d.sens.gas ? d.param.ie_hit_g : d.param.ie_hit;
}

#if (defined(FUEL_INJECT) && defined(GD_CONTROL)) || (!defined(FUEL_INJECT) && (!defined(CARB_AFR) || defined(GD_CONTROL)))
/** Fuel cut control logic for carburetor or gas doser
 * Uses d ECU data structure
 * \param apply Apply to phisical output - 1, or not - 0 (IE will be turned off)
 */
static void simple_fuel_cut(uint8_t apply)
{
 if (s_timer_is_action(&idlecutoff_crnk_delay))
 {
  //if throttle is opened, then open valve, reload timer and exit from condition
  if (d.sens.carb
#ifdef SPEED_SENSOR
 || (IOCFG_CHECK(IOP_SPDSENS) && (d.sens.vss_speed < PGM_GET_WORD(&fw_data.exdata.fuelcut_vss_thrd)))
#endif
     )
  {
   d.ie_valve = 1;
   s_timer_set(&epxx_delay_time_counter, d.param.shutoff_delay);
  }
  //if throttle is closed, then state of valve depends on RPM,previous state of valve,timer and type of fuel
  else
  {
   d.ie_valve = (s_timer_is_action(&epxx_delay_time_counter)&&
                (((d.sens.rpm > get_fc_lot())&&(!d.ie_valve))||(d.sens.rpm > get_fc_hit())))?0:1;
  }
 }
 else
 {
  d.ie_valve = 1; //always ON while delay is not over
 }

 if (apply)
 {
  if (d.floodclear)   //Turn off carburetor's idle cut off valve if flood clear mode is active. Here we rely that apply=1 only if gas dosator is NOT active.
   d.ie_valve = 0;
  IOCFG_SETF(IOP_IE, d.ie_valve);
 }
 else
  IOCFG_SETF(IOP_IE, 0); //turn off valve
}
#endif


#ifdef FUEL_INJECT

/**State variable for delay counter*/
static uint8_t state = 0;

void fuelcut_init_ports(void)
{
 //empty
}

void fuelcut_control(void)
{
#ifdef GD_CONTROL
 if (d.sens.gas && IOCFG_CHECK(IOP_GD_STP))
 {
  simple_fuel_cut(0);   //fuel cut off for gas doser
  goto revlim;
 }
#endif

 if (s_timer_is_action(&idlecutoff_crnk_delay))
 {
  //fuel cut off for fuel injection
  if (d.sens.rpm > get_fc_hit())
  {
   //When RPM > hi threshold, then check TPS, CTS and MAP
   if ((!d.sens.carb) && (d.sens.temperat > d.param.fuelcut_cts_thrd) && (d.sens.map < d.param.fuelcut_map_thrd)
#ifdef SPEED_SENSOR
    && (!IOCFG_CHECK(IOP_SPDSENS) || (d.sens.vss_speed > PGM_GET_WORD(&fw_data.exdata.fuelcut_vss_thrd)))
#endif
      )
   {
    if (0==state)
    {
     s_timer_set(&epxx_delay_time_counter, d.param.shutoff_delay);
     state = 1;
    }
    else
    {
     if (s_timer_is_action(&epxx_delay_time_counter))
      d.ie_valve = 0;  //Cut fuel
    }
   }
   else
   {
    d.ie_valve = 1;   //normal operation
    state = 0;
   }
  }
  else if (d.sens.rpm < get_fc_lot() || d.sens.carb)
  { //always turn on fuel when RPM < low threshold or throttle is opened
   d.ie_valve = 1;
   state = 0;
  }
 }
 else
 { //always ON while delay is not over
  d.ie_valve = 1;
  state = 0;
 }

#ifdef UNI_OUTPUT
  if (d.param.fuelcut_uni != 0x0F)
   d.ie_valve = d.ie_valve && d.uniout[d.param.fuelcut_uni]; //use condition result from selected univ.out (use AND function)
#endif

#ifdef GD_CONTROL
revlim:
#endif
 //simple Rev. limitter
 if (d.sens.rpm > d.param.revlim_hit)
 {
  d.fc_revlim = 1; //cut fuel
 }
 else if (d.sens.rpm < d.param.revlim_lot)
 {
  d.fc_revlim = 0; //restore fuel
 }
}

#else //Carburetor (Idle Cut-off valve control)

#if !defined(CARB_AFR) || defined(GD_CONTROL) //Carb. AFR control supersede idle cut-off functionality

void fuelcut_init_ports(void)
{
#ifndef CARB_AFR
 IOCFG_INIT(IOP_IE, 1); //valve is turned on
#endif
}

//Implementation of Idle Cut-off valve control. If throttle gate is closed AND frq > [up.threshold] OR
//throttle gate is closed AND frq > [lo.threshold] BUT valve is already closed, THEN turn off
//fuel supply by stopping to apply voltage to valve's coil. ELSE - fuel supply is turned on.
void fuelcut_control(void)
{
#ifdef CARB_AFR
 //In case of carb. AFR control, fuel cut for gas doser must be used only 
 //when gas doser is activated and fuel type is gas
 if (d.sens.gas && IOCFG_CHECK(IOP_GD_STP))
 {
#endif
  uint8_t off_icv_on_gas = IOCFG_CHECK(IOP_GD_STP); //turn off carburetor's idle cut off valve when gas doser is active
  simple_fuel_cut((!d.sens.gas || !off_icv_on_gas));

#ifdef GD_CONTROL
  //simple Rev. limitter used only for gas doser
  if (d.sens.rpm > d.param.revlim_hit)
  {
   d.fc_revlim = 1; //cut fuel
  }
  else if (d.sens.rpm < d.param.revlim_lot)
  {
   d.fc_revlim = 0; //restore fuel
  }
#endif

#ifdef CARB_AFR
 }
#endif
}

#endif //!CARB_AFR || GD_CONTROL

#endif

#if !defined(CARB_AFR) || defined(GD_CONTROL) //Carb. AFR control supersede idle cut-off functionality
void fuelcut_eng_stopped_notification(void)
{
 s_timer_set(&idlecutoff_crnk_delay, ICOCD);
}
#endif