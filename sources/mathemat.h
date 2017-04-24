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

/** \file mathemat.h
 * \author Alexey A. Shabelnikov
 * Core mathematic functions.
 */

#ifndef _MATHEMAT_H_
#define _MATHEMAT_H_

#include <stdint.h>

/** f(x) liniar interpolation for function with single argument
 * \param x argument value
 * \param a1 function value at the beginning of interval
 * \param a2 function value at the end of interval
 * \param x_s argument value at the beginning of interval
 * \param x_l length of interval in x
 * \param m function multiplier
 * \return interpolated value of function * m
 */
int16_t simple_interpolation(int16_t x,int16_t a1,int16_t a2,int16_t x_s,int16_t x_l, uint8_t m);

/** f(x,y) liniar interpolation for function with two arguments
 * \param x first argument value
 * \param y second argument value
 * \param a1 function value at the beginning of interval (1 corner)
 * \param a2 function value at the beginning of interval (2 corner)
 * \param a3 function value at the beginning of interval (3 corner)
 * \param a4 function value at the beginning of interval (4 corner)
 * \param x_s first argument value at the beginning of interval
 * \param y_s second argument value at the beginning of interval
 * \param x_l length of interval in x
 * \param y_l length of interval in y
 * \return interpolated value of function * 16
 */
int16_t bilinear_interpolation(int16_t x,int16_t y,int16_t a1,int16_t a2,int16_t a3,int16_t a4,int16_t x_s,int16_t y_s,int16_t x_l,int16_t y_l);

/** Restricts specified value to specified limits
 * \param io_value pointer to value to be restricted. This parameter will also receive result.
 * \param i_bottom_limit bottom limit
 * \param i_top_limit upper limit
 */
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit);

#if defined(FUEL_INJECTION) || defined(GD_CONTROL)
/**Calculate 1/x function using Newton-Raphson method, 2 iterations
 * \param x  8...24, value * 1024
 * \return 1/x * 32768
 */
uint16_t nr_1x_afr(uint16_t x);
#endif

#if defined(FUEL_INJECTION) || defined(GD_CONTROL)
/** Square root calculation
 * \param input Input value
 * \return SQRT(input)
 */
uint16_t ui32_sqrt(uint32_t input);
#endif

#endif //_MATHEMAT_H_
