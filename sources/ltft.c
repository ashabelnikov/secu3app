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

#define LTFT_MIN -126 //!< Min. value in LTFT map; -126 / 512 = -0.246 (-24.6%)
#define LTFT_MAX  126 //!< Max. value in LTFT map;  126 / 512 =  0.246 ( 24.6%)

/**Number of successive switches of signal*/
#define LAMBDA_SWT_NUM 4

uint8_t ltft_state = 0;  //!< SM state
uint16_t stat_tmr = 0;   //!< timer
uint8_t ltft_idx_r = 0;  //!< rpm index of current work point
uint8_t ltft_idx_l = 0;  //!< load index of current work point
int16_t ltft_corr = 0;   //!< value of actual correction
uint8_t idx_l = 0;       //!< index for iteration throught load axis
uint8_t idx_r = 0;       //!< index for iteration throught rpm axis

void ltft_control(void)
{
 uint8_t ee_opcode = eeprom_get_pending_opcode();
 if (ee_opcode == OPCODE_RESET_LTFT || ee_opcode == OPCODE_SAVE_LTFT)
  return; //do not write to LTFT map during saving to EEPROM

 if (d.sens.temperat < ((int16_t)PGM_GET_WORD(&fw_data.exdata.ltft_learn_clt)))
  return; //CLT is too low for learning

 if (d.sens.map2 < PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpa))
  return; //gas pressure is below threshold

 if (PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpd) && ((d.sens.map2 - d.sens.map) < PGM_GET_WORD(&fw_data.exdata.ltft_learn_gpd)))
  return; //differential gas pressure is below threshold

 if (!ltft_is_active())
  return; //LTFT functionality turned off or not active for current fuel

 //do learning:
 switch(ltft_state)
 {
  case 0:
  { //wait for work point to enter restricted band around a cell
   uint8_t r = ltft_check_rpm_hit();
   uint8_t l = ltft_check_load_hit();
   if (r != 255 && l != 255)
   {
    lambda_reset_swt_counter();
    stat_tmr = s_timer_gtc();
    ltft_state++;
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
    ltft_state = 0;
    break;
   }

   if (((s_timer_gtc() - stat_tmr) >= PGM_GET_BYTE(&fw_data.exdata.ltft_stab_time)) && lambda_get_swt_counter() >= LAMBDA_SWT_NUM)
   {
    int16_t ltft_curr = d.inj_ltft[l][r];
    int16_t new_val = ltft_curr + d.corr.lambda;
    restrict_value_to(&new_val, LTFT_MIN, LTFT_MAX);
    d.inj_ltft[l][r] = new_val;     //apply correction to current cell
    ltft_corr = new_val - ltft_curr;
    d.corr.lambda-=ltft_corr;       //reduce current lambda by actual value of correction (taking into account possible min/max restriction)
    ltft_idx_r = r, ltft_idx_l = l; //remember indexes of current work point
    idx_l = 0, idx_r = 0;
    ltft_state++;
   }
   else
   {
    break;
   }
  }

  case 2: //perform correction of neighbour cells
  {
   uint8_t r = PGM_GET_BYTE(&fw_data.exdata.ltft_neigh_rad);
   if ((abs8((int8_t)idx_r - ltft_idx_r) <= r) && (abs8((int8_t)idx_l - ltft_idx_l) <= r)) //skip cells which lay out of radius
   {
    if (ltft_idx_r != idx_r || ltft_idx_l != idx_l) //skip already corrected (current) cell
    {
     int8_t dist_l = abs(ltft_idx_l - idx_l);
     int8_t dist_r = abs(ltft_idx_r - idx_r);
     int8_t dist = (dist_l > dist_r) ? dist_l : dist_r; //find maximum distance
     int16_t new_val = ((int16_t)d.inj_ltft[idx_l][idx_r]) + (((((int32_t)ltft_corr) * PGM_GET_BYTE(&fw_data.exdata.ltft_learn_grad)) >> 8) / dist);
     restrict_value_to(&new_val, LTFT_MIN, LTFT_MAX);
     d.inj_ltft[idx_l][idx_r] = new_val;
    }
   }

   idx_r++;
   if (idx_r == INJ_VE_POINTS_F)
   {
    idx_r = 0;
    idx_l++;
    if (idx_l == INJ_VE_POINTS_L)
     ltft_state = 0; //all 256 cells updated, finish
   }
  }
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

#endif
