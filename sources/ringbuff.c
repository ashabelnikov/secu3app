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

/** \file ringbuff.c
 * \author Alexey A. Shabelnikov
 * Implementation of the ring buffer
 */

#include "port/port.h"
#include "ringbuff.h"

void init_buffer(struct ringbuff_t* p_rb, uint8_t avnum)
{
 p_rb->avnum = avnum;
 p_rb->size = 0;
}

void update_buffer(struct ringbuff_t* p_rb, uint16_t value)
{
 p_rb->sum-= p_rb->buff[p_rb->idx]; //remove old value from the sum
 p_rb->sum+= value;                 //add a new value

 p_rb->buff[p_rb->idx++] = value;   //update buffer with new value
 if (p_rb->idx >= p_rb->avnum)
  p_rb->idx = 0;

 if (p_rb->size < p_rb->avnum)      //update actual size
  ++p_rb->size;
}

uint16_t average_buffer(struct ringbuff_t* p_rb)
{
 if (p_rb->size > 0)
 {
  if (p_rb->size==4)               //if possible we use shifts instead of division.
   return p_rb->sum >> 2;
  if (p_rb->size==8)
   return p_rb->sum >> 3;
  return p_rb->sum / p_rb->size;
 }
 else
 {
  return 0;                        //buffer is empty or has zero size
 }
}
