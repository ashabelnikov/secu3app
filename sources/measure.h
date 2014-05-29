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

/** \file measure.h
 * Process (averaging, corrections etc) data comes from ADC and sensors
 * (Обработка (усреднение, корректировки и т.д.) данных поступающих от АЦП и данчиков).
 */

#ifndef _MEASURE_H_
#define _MEASURE_H_

#include <stdint.h>

struct ecudata_t;

/**Initialization of used input ports*/
void meas_init_ports(void);

/**Update ring buffers with new data given from sensors and ADC
 * \param d pointer to ECU data structure
 * \param rpm_only if != 0, then only RPM related buffers will be updated
 */
void meas_update_values_buffers(struct ecudata_t* d, uint8_t rpm_only);

/**Perform avaraging using data from ring buffers
 * \param d pointer to ECU data structure
 */
void meas_average_measured_values(struct ecudata_t* d);

/**Initialization of ring buffers. Performs initial measurements. Used before start of engine
 * \param d pointer to ECU data structure
 */
void meas_initial_measure(struct ecudata_t* d);

/**Performs reading of discrete inputs and switching of fuel type
 * производит считывание дискретных входов системы и переключение
 * типа топлива (набор таблиц).
 * \param d pointer to ECU data structure
 */
void meas_take_discrete_inputs(struct ecudata_t *d);

#endif //_MEASURE_H_
