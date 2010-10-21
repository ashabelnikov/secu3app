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

#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include <stdint.h>
#include "vstimer.h"

int16_t simple_interpolation(int16_t x,int16_t a1,int16_t a2,int16_t x_s,int16_t x_l);
int16_t bilinear_interpolation(int16_t x,int16_t y,int16_t a1,int16_t a2,int16_t a3,int16_t a4,int16_t x_s,int16_t y_s,int16_t x_l,int16_t y_l);

struct ecudata_t;

int16_t start_function(struct ecudata_t* d);
int16_t idling_function(struct ecudata_t* d);
int16_t work_function(struct ecudata_t* d, uint8_t i_update_airflow_only);
int16_t coolant_function(struct ecudata_t* d);
uint8_t knock_attenuator_function(struct ecudata_t* d);
void idling_regulator_init(void);
int16_t idling_pregulator(struct ecudata_t* d, volatile s_timer8_t* io_timer);
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m);
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit);

#endif //_FUNCONV_H_
