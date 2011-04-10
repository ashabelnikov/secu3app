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
              http://secu-3.org
              email: shabelnikov@secu-3.org
*/

/** \file funconv.h
 * Core mathematics and regulation logic.
 * (Основная часть математического аппарата и логики регулирования).
 */

#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include <stdint.h>
#include "vstimer.h"

/** f(x) liniar interpolation
 * \param x
 * \param a1
 * \param a2
 * \param x_s
 * \param x_l
 * \return interpolated value of function * 16
 */
int16_t simple_interpolation(int16_t x,int16_t a1,int16_t a2,int16_t x_s,int16_t x_l);

/** f(x,y) liniar interpolation
 * \param x
 * \param y
 * \param a1
 * \param a2
 * \param a3
 * \param a4
 * \param x_s
 * \param y_s
 * \param x_l
 * \param y_l
 * \return interpolated value of function * 16
 */
int16_t bilinear_interpolation(int16_t x,int16_t y,int16_t a1,int16_t a2,int16_t a3,int16_t a4,int16_t x_s,int16_t y_s,int16_t x_l,int16_t y_l);

struct ecudata_t;

/** Calculates advance angle from "start" map
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t start_function(struct ecudata_t* d);

/** Calculates advance angle from "idle" map
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t idling_function(struct ecudata_t* d);

/** Calculates advance angle from "work" map
 * \param d pointer to ECU data structure
 * \param i_update_airflow_only
 * \return value of advance angle * 32
 */
int16_t work_function(struct ecudata_t* d, uint8_t i_update_airflow_only);

/** Calculates anvance angle correction using coolant temperature
 * \param d pointer to ECU data structure
 * \return value of advance angle * 32
 */
int16_t coolant_function(struct ecudata_t* d);

/**
 * \param d pointer to ECU data structure
 * \return
 */
uint8_t knock_attenuator_function(struct ecudata_t* d);

/**Initialization of idling regulator's data structures */
void idling_regulator_init(void);

/**
 * \param d pointer to ECU data structure
 * \param io_timer
 * \return value of advance angle * 32
 */
int16_t idling_pregulator(struct ecudata_t* d, volatile s_timer8_t* io_timer);

/**
 * \param new_advance_angle
 * \param ip_prev_state
 * \param intstep_p
 * \param intstep_m
 * \return value of advance angle * 32
 */
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m);

/**
 * \param io_value
 * \param i_bottom_limit
 * \param i_top_limit
 */
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit);

#ifdef COIL_REGULATION
/** Calculates current accumulation time (coil regulation) using current board voltage
 * \param d pointer to ECU data structure
 * \return accumulation time in timer's ticks (1 tick = 4uS)
 */
uint16_t accumulation_time(struct ecudata_t* d);
#endif

#endif //_FUNCONV_H_
