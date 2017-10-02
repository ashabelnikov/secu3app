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

/** \file knklogic.c
 * \author Alexey A. Shabelnikov
 * The module implements all the regulation logic of advance angle by detonation
 */

#include "port/avrio.h"
#include "port/port.h"
#include "ce_errors.h"
#include "ecudata.h"
#include "knklogic.h"
#include "magnitude.h"
#include "mathemat.h"

/**Delay in strokes*/
#define KNK_STRT_DELAY 150

uint8_t knklogic_detect(retard_state_t* p_rs)
{
 if (d.sens.frequen > d.param.starter_off && d.sens.temperat > TEMPERATURE_MAGNITUDE(70.0))
 {
  if (0==p_rs->sd_counter)
  {
   //This is a very simple algorithm to determine is knock present. We must
   //improve it in the nearest future.
   p_rs->knock_flag = (d.sens.knock_k > d.param.knock_threshold);
  }
  else
   --p_rs->sd_counter;
 }
 else
  p_rs->knock_flag = 0; //Do not detect knock at the startup of engine


 //if knock detected set corresponding CE error
 if (p_rs->knock_flag)
  ce_set_error(ECUERROR_KNOCK_DETECTED);
 else
  ce_clear_error(ECUERROR_KNOCK_DETECTED);

 return p_rs->knock_flag;
}

void knklogic_init(retard_state_t* p_rs)
{
 p_rs->delay_counter = 0;
 p_rs->knock_flag = 0;
 p_rs->sd_counter = KNK_STRT_DELAY;
}

void knklogic_retard(retard_state_t* p_rs)
{
 if (p_rs->knock_flag)
 { //detonation is present
  d.corr.knock_retard+= d.param.knock_retard_step;//retard
  p_rs->knock_flag = 0;
  p_rs->delay_counter = d.param.knock_recovery_delay; //reset delay
 }
 else
 { //detonation is absent
  if (p_rs->delay_counter == 0)
  {
   d.corr.knock_retard-= d.param.knock_advance_step;//advance
   p_rs->delay_counter = d.param.knock_recovery_delay;
  }
 }
 //restrict knock retard value
 restrict_value_to(&d.corr.knock_retard, 0, d.param.knock_max_retard);

 if (p_rs->delay_counter != 0)
  p_rs->delay_counter--;
}
