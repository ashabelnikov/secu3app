/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#include <inavr.h>
#include <ioavr.h>
#include "ce_errors.h"
#include "knklogic.h"
#include "secu3.h"
#include "funconv.h"

uint8_t knklogic_detect(struct ecudata_t* d, retard_state_t* p_rs)
{
 p_rs->knock_flag = (d->sens.knock_k > d->param.knock_threshold);
 
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
}

void knklogic_retard(struct ecudata_t* d, retard_state_t* p_rs)
{
 if (p_rs->delay_counter != 0)
  p_rs->delay_counter--;
 else
 {
  if (p_rs->knock_flag)
  { //detonation present
   d->knock_retard+= d->param.knock_retard_step;//retard
   p_rs->knock_flag = 0;
  }
  else
  {//detonation is absent
   d->knock_retard-= d->param.knock_advance_step;//advance
  } 
  restrict_value_to(&d->knock_retard, 0, d->param.knock_max_retard);
  
  p_rs->delay_counter = d->param.knock_recovery_delay;   
  if (0!=(p_rs->delay_counter)) 
    --(p_rs->delay_counter);
 }     
}
