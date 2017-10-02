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

/** \file params.h
 * \author Alexey A. Shabelnikov
 * Functionality for work with parameters (save/restore/check)
 */

#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <stdint.h>

#define SAVE_PARAM_TIMEOUT_VALUE      3000  //!< timeout value used to count time before automatic saving of parameters

/**Write data to EEPROM - the process is very slow. It will take place in parallel with
 * the execution of the program. Storing data in the EEPROM will only happen if for a given
 * time, there was not a single receive operation of parameters via UART, and saved settings
 * are different from current.
 * Uses d ECU data structure
 */
void save_param_if_need(void);

/**Loads the parameters from the EEPROM, and verifies the integrity of the data if they spoiled
 * it takes a backup instance from the FLASH.
 * Call this function only when EEPROM is idle!
 * Uses d ECU data structure
 */
void load_eeprom_params(void);

#ifdef REALTIME_TABLES
/** Loads tables into RAM depending on specified index
 *  Call this function only when EEPROM is idle!
 * Uses d ECU data structure
 * \param index index of tables set to load into RAM
 */
void load_specified_tables_into_ram(uint8_t index);
#endif

/** Resets EEPROM contents to default values and resets device, this function has same effect as
 *  closed jumper "Default EEPROM"
 * Uses d ECU data structure
 */
void reset_eeprom_params(void);

#endif //_PARAMS_H_
