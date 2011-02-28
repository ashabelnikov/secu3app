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

/** \file knock.h
 * Knock chip related functions.
 * Service of HIP9011 knock signal processing chip
 * (Обслуживание чипа HIP9011 обрабатывающего сигнал детонации).
 */

#ifndef _KNOCK_H_
#define _KNOCK_H_

#include <stdint.h>

//Parameters of functions receive data according to registers format of
//HIP9011. Retuning of parameters allowed only in HOLD mode!

//These three functions may be called in real time and at any turning angles of
//the crankshaft. However, application of value being set will occur only after
//a call of knock_start_settings_latching()

/**Set center frequency of bandpass filter
 * \param freq code of bandpass frequency (according to datasheet)
 */
void knock_set_band_pass(uint8_t freq);

/**Set attenuation gain of digitally programmable gain
 * \param gain code of programmable gain (according to datasheet)
 */
void knock_set_gain(uint8_t gain);

/**Set time constant of integrator
 * \param inttime code of time constant (according to datasheet)
 */
void knock_set_int_time_constant(uint8_t inttime);


/**Starts the process of transferring the settings into the signal processor. Must
 * be invoked under certain turning angles of the crankshaft, at which the signal
 * processor is in HOLD mode. If at the time of calling of this function latching
 * process is not finished yet, the old process will be aborted and started a new
 * one, but it will set a sign of error.
 */
void knock_start_settings_latching(void);

/**\return value > 0 if at the current moment latching operation is not in process */
uint8_t knock_is_latching_idle(void);

/**\return 1 if was an error (chip was not responding or data corruption was detected) */
uint8_t knock_is_error(void);

/**Reset an error */
void knock_reset_error(void);

/**Initialization of knock channel and its testing.
 * \return 1 - if testing performed succesfully, otherwise 0.
 */
uint8_t knock_module_initialize(void);

#define KNOCK_INTMODE_INT  1 //!< code for integration mode (used with knock_set_integration_mode())
#define KNOCK_INTMODE_HOLD 0 //!< code for hold mode (used with knock_set_integration_mode())

/**Affects INT/HOLD input of HIP9011, setting in such a way either integration
 * or hold mode
 * \param mode code of mode (KNOCK_INTMODE_INT or KNOCK_INTMODE_HOLD)
 */
void knock_set_integration_mode(uint8_t mode);

/**Initialization of used I/O ports */
void knock_init_ports(void);

#endif //_KNOCK_H_
