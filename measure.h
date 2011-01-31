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

#ifndef _MEASURE_H_
#define _MEASURE_H_

struct ecudata_t;

void meas_update_values_buffers(struct ecudata_t* d);
void meas_average_measured_values(struct ecudata_t* d);
void meas_initial_measure(struct ecudata_t* d);

//производит считывание дискретных входов системы и переключение 
//типа топлива (набор таблиц).
void meas_take_discrete_inputs(struct ecudata_t *d);

#endif //_MEASURE_H_
