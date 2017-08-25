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

/** \file knock.h
 * \author Alexey A. Shabelnikov
 * Knock chip related functions.
 * Service of the HIP9011 knock signal processing chip
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

//channel selection values
#define KSP_CHANNEL_0          0x00   //!< code for select 0 channel
#define KSP_CHANNEL_1          0x01   //!< code for select 1 channel

/**Set channel number
 * \param Channel number to set active (0,1)
 */
void knock_set_channel(uint8_t channel);

/**Starts the process of transferring the settings into the signal processor. Must
 * be invoked under certain turning angles of the crankshaft, at which the signal
 * processor is in HOLD mode. If at the time of calling of this function latching
 * process is not finished yet, the old process will be aborted and started a new
 * one, but it will set a sign of error.
 */
void knock_start_settings_latching(void);

#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
void knock_start_expander_latching(void);
#endif

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

#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---

/** Initialization of expander chip MCP23S17
 * \return 1 - if testing performed succesfully (chip works), otherwise 0.
 */
uint8_t knock_expander_initialize(void);
#endif

#ifdef OBD_SUPPORT
struct can_t;
/** Push CAN message for sending
 * \param msg message to be sent
 */
void knock_push_can_message(struct can_t* msg);
#endif

#define KNOCK_INTMODE_INT  1 //!< code for integration mode (used with knock_set_integration_mode())
#define KNOCK_INTMODE_HOLD 0 //!< code for hold mode (used with knock_set_integration_mode())

/**Affects INT/HOLD input of HIP9011, setting in such a way either integration
 * or hold mode
 * \param mode code of mode (KNOCK_INTMODE_INT or KNOCK_INTMODE_HOLD)
 */
void knock_set_integration_mode(uint8_t mode);

/**Initialization of used I/O ports */
void knock_init_ports(void);

#ifdef TPIC8101
/** Get ADC value read from TPIC8101
 * \return 10-bit ADC value
 */
uint16_t knock_get_adc_value(void);
#endif

#endif //_KNOCK_H_
