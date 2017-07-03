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

/** \file mathemat.c
 * \author Alexey A. Shabelnikov
 * Implementation of core mathematic functions.
 */

#include "port/port.h"
#include <stdlib.h>
#include "mathemat.h"

int16_t bilinear_interpolation(int16_t x, int16_t y, int16_t a1, int16_t a2, int16_t a3, int16_t a4,
                               int16_t x_s, int16_t y_s, int16_t x_l, int16_t y_l)
{
 int16_t a23,a14;
 a23 = ((a2 * 16) + (((int32_t)(a3 - a2) * 16) * (x - x_s)) / x_l);
 a14 = (a1 * 16) + (((int32_t)(a4 - a1) * 16) * (x - x_s)) / x_l;
 return (a14 + ((((int32_t)(a23 - a14)) * (y - y_s)) / y_l));
}

int16_t simple_interpolation(int16_t x, int16_t a1, int16_t a2, int16_t x_s, int16_t x_l, uint8_t m)
{
 return ((a1 * m) + (((int32_t)(a2 - a1) * m) * (x - x_s)) / x_l);
}

void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit)
{
 if (*io_value > i_top_limit)
  *io_value = i_top_limit;
 if (*io_value < i_bottom_limit)
  *io_value = i_bottom_limit;
}

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint16_t nr_1x_afr(uint16_t x)
{
 uint16_t r;
 if (x > 20480) //20 * 1024
  r = 1507; //0.046 * 32768
 else if (x > (16179)) //15.8 * 1024
  r = 1835; //0.058 * 32768
 else if (x > (11469)) //11.2 * 1024
  r = 2392; //0.073 * 32768
 else
  r = 3375; //0.103 * 32768

 uint8_t i = 2;

 while(i--)
 {
//  r = ((uint32_t)(r * ((2*32768) - (((uint32_t)(x * r)) >> 10)))) >> 15;
//  r = ((uint32_t)r * (2*32768 - (uint16_t)(((uint32_t)x * r) >> 10))) >> 15;
    r = ((uint32_t)r * (uint16_t)((uint16_t)0 - (uint16_t)(((uint32_t)x * r) >> 10))) >> 15;
 }

 return r;
}

#endif //FUEL_INJECT || GD_CONTROL

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
uint16_t ui32_sqrt(uint32_t input)
{
 unsigned long mask = 0x40000000, sqr = 0, temp;
 do
 {
  temp = sqr | mask;
  sqr >>= 1;
  if(temp <= input)
  {
   sqr |= mask;
   input -= temp;
  }
 } while(mask >>= 2);

 return (uint16_t)sqr;
}

#endif //FUEL_INJECT || GD_CONTROL
