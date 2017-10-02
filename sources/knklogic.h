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

/** \file knklogic.h
 * \author Alexey A. Shabelnikov
 * The module contains all the regulation logic of advance angle by detonation
 */

#ifndef _KNKLOGIC_H_
#define _KNKLOGIC_H_

#include <stdint.h>

/** Contains state variables used by retard algorithm and others */
typedef struct retard_state_t
{
 uint8_t delay_counter; //!< used to count time in retard algorithm
 uint8_t knock_flag;    //!< indicates that detonation is present
 uint8_t sd_counter;    //!< used to count time after engine startup
}retard_state_t;

/** Implements alrogithms for knock detection
 * Uses d ECU data structure
 * \param p_rs poiter to state variables used by algorithm
 * \return: 0 - detonation is absent, 1 - detonation is present
 */
uint8_t knklogic_detect(retard_state_t* p_rs);

/** Initialization of state variables
 * \param p_rs poiter to state variables used by algorithm
 */
void knklogic_init(retard_state_t* p_rs);

/** Called in each work stroke
 * Uses d ECU data structure
 * \param p_rs poiter to state variables used by algorithm
 */
void knklogic_retard(retard_state_t* p_rs);

#endif //_KNKLOGIC_H_
