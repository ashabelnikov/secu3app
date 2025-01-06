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

/** \file measure.h
 * \author Alexey A. Shabelnikov
 * Processing (averaging, corrections etc) of data which comes from ADC and sensors
 */

#ifndef _MEASURE_H_
#define _MEASURE_H_

#include <stdint.h>

struct ce_sett_t;

/**Initialization of used input ports*/
void meas_init_ports(void);

/**Update ring buffers with new data given from sensors and ADC
 * Uses d ECU data structure
 * \param cesd Pointer to the data structure describing CE settings
 */
void meas_update_values_buffers(ce_sett_t *cesd);
void meas_update_values_buffers_map(ce_sett_t *cesd);

/**Perform avaraging using data from ring buffers
 * Uses d ECU data structure
 * \param cesd Pointer to CE settings data structure
 */
void meas_average_measured_values(ce_sett_t *cesd);

/**Initialization of ring buffers. Performs initial measurements. Used before start of engine
 * Uses d ECU data structure
 */
void meas_init(void);

/**Performs reading of discrete inputs and switching of fuel type (sets of maps)
 * Uses d ECU data structure
 */
void meas_take_discrete_inputs(void);

#endif //_MEASURE_H_
