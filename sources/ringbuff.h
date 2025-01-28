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

/** \file ringbuff.h
 * \author Alexey A. Shabelnikov
 *  Ring buffer
 */

#ifndef _RINGBUFF_H_
#define _RINGBUFF_H_

#include <stdint.h>

#define CIRCBUFFMAX    32                       //!< Maximum size of ring buffer in items

/**Describes ring buffer for one input*/
typedef struct ringbuff_t
{
 uint16_t buff[CIRCBUFFMAX];                    //!< Ring buffer
 uint8_t avnum;                                 //!< size of the circular buffer
 uint8_t size;                                  //!< actual size, may be less or equal to size
 uint8_t idx;                                   //!< index in buffer
 uint32_t sum;                                  //!< sum for averaging
}ringbuff_t;

/** Initializes the ring buffer
 * \param p_rb Pointer to the ring buffer
 * \param avnum Size of the ring buffer, this value must not be greater than CIRCBUFFMAX
 */
void init_buffer(struct ringbuff_t* p_rb, uint8_t avnum);

/** Updates the ring buffer with specified value
 * \param p_rb Pointer to the ring buffer
 * \param value Value, buffer to be updated with
 */
void update_buffer(struct ringbuff_t* p_rb, uint16_t value);

/** Averages the ring buffer
 * \param p_rb Pointer to the ring buffer
 * \return Averaged value
 */
uint16_t average_buffer(struct ringbuff_t* p_rb);

#endif //_RINGBUFF_H_
