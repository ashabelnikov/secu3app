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

/** \file ventilator.h
 * Cooling fan's control related functions.
 * (‘ункции дл€ управлени€ вентил€тором).
 */

#ifndef _VENTILATOR_H_
#define _VENTILATOR_H_

struct ecudata_t;

/**Initialization of used I/O ports (инициализаци€ используемых портов)*/
void vent_init_ports(void);

/**Control of cooling fan (управление вентил€тором охлаждени€ двигател€).
 * \param d pointer to ECU data structure
 */
void vent_control(struct ecudata_t *d);

/**Initialization of internal state (инициализаци€ состо€ни€)*/
void vent_init_state(void);

#endif //_VENTILATOR_H_
