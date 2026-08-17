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
#include <string.h>

void init_buffer(struct ringbuff_t* p_rb, uint8_t avnum)
{
 p_rb->avnum = avnum;
 p_rb->size = 0;
 p_rb->idx = 0;
 p_rb->sum = 0;
 p_rb->buff[0] = 0;
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

//MEDIAN FILTER
static uint16_t m_buff[CIRCBUFFMAX];   //!< buffer for processing median filter

/* Sort array using insertion sort 
 * \param p_arr Array of items to sort
 * \param n Number of items in array
 */
static void insertion_sort(uint16_t* p_arr, uint8_t n)
{
 for (uint8_t i = 1; i < n; ++i)
 {
  uint16_t key = p_arr[i];
  int8_t j = i - 1;
  //move items
  while (j >= 0 && p_arr[j] > key)
  {
   p_arr[j + 1] = p_arr[j];
   j = j - 1;
  }
  p_arr[j + 1] = key;
 }
}

void init_median(struct medifilt_t* p_mf, uint8_t avnum)
{
 p_mf->avnum = avnum; //size of the filter
 p_mf->size = 0;      //actual size of the filter
 p_mf->idx = 0;
 p_mf->buff[0] = 0;
}

void update_median(struct medifilt_t* p_mf, uint16_t value)
{
 p_mf->buff[p_mf->idx++] = value;   //update buffer with new value
 if (p_mf->idx >= p_mf->avnum)
  p_mf->idx = 0;

 if (p_mf->size < p_mf->avnum)      //update actual size
  ++p_mf->size;
}

uint16_t calc_median(struct medifilt_t* p_mf)
{
 if (p_mf->size > 0)
 {
  memcpy(m_buff, p_mf->buff, sizeof(uint16_t) * p_mf->size); //copy array before sorting, because we have to leave order in the original array intact
  insertion_sort(m_buff, p_mf->size); //sort array
  return m_buff[p_mf->size/2]; //find median value
 }
 else
 {
  return 0;                        //buffer is empty or has zero size
 }
}
