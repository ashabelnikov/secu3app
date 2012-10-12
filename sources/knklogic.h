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

/** \file knklogic.h
 * The module contains all the regulation logic of advance angle by detonation
 * (Модуль содержащий всю логику регулирования УОЗ по детонации).
 */

#ifndef _KNKLOGIC_H_
#define _KNKLOGIC_H_

#include <stdint.h>

/** Contains state variables used by retard algorithm and others */
typedef struct retard_state_t
{
 uint8_t delay_counter; //!< used to count time in retard algorithm
 uint8_t knock_flag;    //!< indicates that detonation is present
}retard_state_t;

struct ecudata_t;

/** Implements alrogithms for knock detection
 * \param d pointer to ECU data structure
 * \param p_rs poiter to state variables used by algorithm
 * \return: 0 - detonation is absent, 1 - detonation is present
 */
uint8_t knklogic_detect(struct ecudata_t* d, retard_state_t* p_rs);

/** Initialization of state variables (инициализация переменных состояния)
 * \param p_rs poiter to state variables used by algorithm
 */
void knklogic_init(retard_state_t* p_rs);

/** Called in each work stroke (вызывается в каждом рабочем такте)
 * \param d pointer to ECU data structure
 * \param p_rs poiter to state variables used by algorithm
 */
void knklogic_retard(struct ecudata_t* d, retard_state_t* p_rs);

#endif //_KNKLOGIC_H_
