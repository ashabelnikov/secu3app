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

/** \file vstimer.h
 * Virtual system timers
 * (Виртуальные системные таймеры).
 */

#ifndef _VSTIMER_H_
#define _VSTIMER_H_

#include <stdint.h>
//инструментарий для реализации виртуальных таймерв

//Типы объектов таймеров. 8-ми разрядный тамйер может отсчитывать периоды
//до 2.56 сек. 16-ти разрядный таймер может отсчитывать периоды до 655 сек.

/** Describes a timer. Used by 8-bit timers */
typedef struct
{
 uint16_t timeout;    //!< timeout
 uint16_t start_val;  //!< start value, used for counddown
}s_timer16_t;

/** Describes a timer. Used by 16-bit timers */
typedef struct
{
 uint8_t timeout;     //!< timeout
 uint8_t start_val;   //!< start value, used for counddown
}s_timer8_t;

/**Initialization specified timer state (8-bit version). One tick = 10ms
 *(инициализация состояния указанного таймера. Один тик таймера равен 10 мс).
 * \param timer pointer to timer's descriptor
 * \param value timeout value used for countdown
 */
void s_timer_set8(s_timer8_t* timer, uint8_t value);

/**Initialization specified timer state (16-bit version). One tick = 10ms
 *(инициализация состояния указанного таймера. Один тик таймера равен 10 мс).
 * \param timer pointer to timer's descriptor
 * \param value timeout value used for countdown
 */
void s_timer_set16(s_timer16_t* timer, uint16_t value);

/**Checks whenever specified timer is expired (Проверяет сработал ли указанный таймер)
 * (8-bit version)
 * \param timer pointer to timer's descriptor
 * \return 1 - expired, 0 - not expired
 */
uint8_t s_timer_isexp8(const s_timer8_t* timer);

/**Checks whenever specified timer is expired (Проверяет сработал ли указанный таймер)
 * (16-bit version)
 * \param timer pointer to timer's descriptor
 * \return 1 - expired, 0 - not expired
 */
uint8_t s_timer_isexp16(const s_timer16_t* timer);

/**Get value of the system 10ms counter (8-bit version)
 * \return current value of 10ms 8-bit system tick counter
 */
uint8_t s_timer_gtc8(void);

/**Get value of the system 10ms counter (16-bit version)
 * \return current value of 10ms 16-bit system tick counter
 */
uint16_t s_timer_gtc16(void);

/**Initialization of system timers */
void s_timer_init(void);

#endif //_VSTIMER_H_
