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

/** \file ltft.c
 * \author Alexey A. Shabelnikov
 * Implementation of the long term fuel trim algorithm
 */

#ifdef FUEL_INJECT

#include "port/port.h"
#include "port/pgmspace.h"
#include <stdlib.h>
#include "ltft.h"
#include "ecudata.h"
#include "eeprom.h"
#include "suspendop.h"
#include "funconv.h"
#include "lambda.h"
#include "mathemat.h"
#include "bitmask.h"

//See ltft-kosh.c for more information about implementation
extern void kosh_ltft_control(uint8_t Channel);
extern void kosh_circular_buffer_update(void);

/**Describes data for each LTFT channel*/
typedef struct
{
 uint8_t ltft_state;  //!< SM state
 uint16_t stat_tmr;   //!< timer
 uint8_t ltft_idx_r;  //!< rpm index of current work point
 uint8_t ltft_idx_l;  //!< load index of current work point
 int16_t ltft_corr;   //!< value of actual correction
 uint8_t idx_l;       //!< index for iteration throught load axis
 uint8_t idx_r;       //!< index for iteration throught rpm axis
 uint8_t strokes;     //!< counter of eng. strokes
}ltft_t;

ltft_t ltft[2] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};

/** Does LTFT "learning"
 * \param i number of EGO sensor channel
 */
void aas_ltft_control(uint8_t i)
{
 if (d.sens.rpm < PGM_GET_WORD(&fw_data.exdata.ltft_learn_rpm[0]) || d.sens.rpm > PGM_GET_WORD(&fw_data.exdata.ltft_learn_rpm[1]))
  return;
 if (d.load < PGM_GET_WORD(&fw_data.exdata.ltft_learn_load[0]) || d.load > PGM_GET_WORD(&fw_data.exdata.ltft_learn_load[1]))
  return;

 switch(ltft[i].ltft_state)
 {
  case 0:
  { //wait for work point to enter restricted band around a cell
   uint8_t r = ltft_check_rpm_hit();
   uint8_t l = ltft_check_load_hit();
   if (r != 255 && l != 255)
   {
    lambda_reset_swt_counter(i);
    ltft[i].stat_tmr = s_timer_gtc();
    ltft[i].strokes = 0;
    ltft[i].ltft_state++;
   }
   else
    break;
  }

  case 1:
  {
   uint8_t r = ltft_check_rpm_hit();
   uint8_t l = ltft_check_load_hit();
   if (r == 255 || l == 255)
   { //work point came out restricted band - reset SM state
    ltft[i].ltft_state = 0;
    break;
   }

   uint8_t stab_time_ready = 0;

   if (0==PGM_GET_BYTE(&fw_data.exdata.ltft_stab_str))
    stab_time_ready = ((s_timer_gtc() - ltft[i].stat_tmr) >= PGM_GET_BYTE(&fw_data.exdata.ltft_stab_time)); //use time
   else
    stab_time_ready = ltft[i].strokes >= PGM_GET_BYTE(&fw_data.exdata.ltft_stab_str); //use eng. strokes

   if (stab_time_ready && lambda_get_swt_counter(i) >= PGM_GET_BYTE(&fw_data.exdata.ltft_sigswt_num))
   {
    int16_t ltft_curr = i ? d.inj_ltft2[l][r] : d.inj_ltft1[l][r];
    int16_t new_val = ltft_curr + d.corr.lambda[i];
    restrict_value_to(&new_val, (int16_t)PGM_GET_BYTE(&fw_data.exdata.ltft_min), (int16_t)PGM_GET_BYTE(&fw_data.exdata.ltft_max));
    if (i)
     d.inj_ltft2[l][r] = new_val;     //apply correction to current cell of LTFT 2
    else
     d.inj_ltft1[l][r] = new_val;     //apply correction to current cell of LTFT 1
    ltft[i].ltft_corr = new_val - ltft_curr;
    d.corr.lambda[i]-=ltft[i].ltft_corr;       //reduce current lambda by actual value of correction (taking into account possible min/max restriction)
    ltft[i].ltft_idx_r = r, ltft[i].ltft_idx_l = l; //remember indexes of current work point
    ltft[i].idx_l = 0, ltft[i].idx_r = 0;
    ltft[i].ltft_state++;
   }
   else
   {
    break;
   }
  }

  case 2: //perform correction of neighbour cells
  {
   uint8_t r = PGM_GET_BYTE(&fw_data.exdata.ltft_neigh_rad);
   uint8_t idx_l = ltft[i].idx_l, idx_r = ltft[i].idx_r;
   if ((abs8((int8_t)idx_r - ltft[i].ltft_idx_r) <= r) && (abs8((int8_t)idx_l - ltft[i].ltft_idx_l) <= r)) //skip cells which lay out of radius
   {
    if (ltft[i].ltft_idx_r != idx_r || ltft[i].ltft_idx_l != idx_l) //skip already corrected (current) cell
    {
     int8_t dist_l = abs8((int8_t)ltft[i].ltft_idx_l - idx_l);
     int8_t dist_r = abs8((int8_t)ltft[i].ltft_idx_r - idx_r);
     int8_t dist = (dist_l > dist_r) ? dist_l : dist_r; //find maximum distance
     int16_t new_val = ((int16_t)(i ? d.inj_ltft2[idx_l][idx_r] : d.inj_ltft1[idx_l][idx_r])) + (((((int32_t)ltft[i].ltft_corr) * PGM_GET_BYTE(&fw_data.exdata.ltft_learn_grad)) >> 8) / dist);
     restrict_value_to(&new_val, (int16_t)PGM_GET_BYTE(&fw_data.exdata.ltft_min), (int16_t)PGM_GET_BYTE(&fw_data.exdata.ltft_max));
     if (i)
      d.inj_ltft2[idx_l][idx_r] = new_val;
     else
      d.inj_ltft1[idx_l][idx_r] = new_val;
    }
   }

   ltft[i].idx_r++;
   if (ltft[i].idx_r == INJ_VE_POINTS_F)
   {
    ltft[i].idx_r = 0;
    ltft[i].idx_l++;
    if (ltft[i].idx_l == INJ_VE_POINTS_L)
     ltft[i].ltft_state = 0; //all 256 cells updated, finish
   }
  }
 }
}

void ltft_control(void)
{
 uint8_t ee_opcode = eeprom_get_pending_opcode();
 if (ee_opcode == OPCODE_RESET_LTFT || ee_opcode == OPCODE_SAVE_LTFT)
  return; //do not write to LTFT map during saving to EEPROM

 if (d.sens.temperat < ((int16_t)PGM_GET_WORD(&fw_data.exdata.ltft_learn_clt)) || d.sens.temperat > ((int16_t)PGM_GET_WORD(&fw_data.exdata.ltft_learn_clt_up)))
  return; //CLT is too low or too high for learning

 if (d.sens.air_temp > ((int16_t)PGM_GET_WORD(&fw_data.exdata.ltft_learn_iat_up)))
  return; //Intake air temperature is too high for learning

#ifndef SECU3T
 if (d.sens.map2 < PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpa))
  return; //gas pressure is below threshold

 if (PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpd) && ((d.sens.map2 - d.sens.map) < PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpd)))
  return; //differential gas pressure is below threshold
#endif

 if (!ltft_is_active())
  return; //LTFT functionality turned off or not active for current fuel

 if (!d.sens.carb && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_IDLCORR))
  return; //Lambda correction is disabled on idling

 if (!d.sens.carb && !PGM_GET_BYTE(&fw_data.exdata.ltft_on_idling))
  return; //LTFT updating on idling is disabled

 uint8_t chnum = (0x00!=d.param.lambda_selch) && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN) ? 2 : 1;
 uint8_t chbeg = (0xFF==d.param.lambda_selch) && !CHECKBIT(d.param.inj_lambda_flags, LAMFLG_MIXSEN);

 for (uint8_t i = chbeg; i < chnum; ++i)
 { //do learning:
  if (0==PGM_GET_BYTE(&fw_data.exdata.ltft_algo))
   aas_ltft_control(i); //default alrorithm
  else
   kosh_ltft_control(i); //'Kosh' algorithm
 }
}

uint8_t ltft_is_active(void)
{
 if (PGM_GET_BYTE(&fw_data.exdata.ltft_mode)==0)
 {
  return 0; //LTFT functionality turned off
 }
 else if (PGM_GET_BYTE(&fw_data.exdata.ltft_mode)==1)
 {
  if (1==d.sens.gas)
   return 0; // LTFT enabled only for petrol
 }
 else if (PGM_GET_BYTE(&fw_data.exdata.ltft_mode)==2)
 {
  if (0==d.sens.gas)
   return 0; // LTFT enabled only for gas
 }
 return 1;
}

void ltft_stroke_event_notification(void)
{
 if (0==PGM_GET_BYTE(&fw_data.exdata.ltft_algo))
 {
  ++ltft[0].strokes;
  ++ltft[1].strokes;
 }
 else
 {
  kosh_circular_buffer_update(); //'Kosh' algorithm
 }
}

#endif
