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

/** \file params.h
 * Functionality for work with parameters (save/restore/check)
 * (Функциональность для работы с параметрами (сохранение/восстановление/проверка)).
 */

#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <stdint.h>

struct ecudata_t;

/**Write data to EEPROM - the process is very slow. It will take place in parallel with
 * the execution of the program. Storing data in the EEPROM will only happen if for a given
 * time, there was not a single receive operation of parameters via UART, and saved settings
 * are different from current.
 * Запись данных в EEPROM - процесс очень медленный. Он будет проходить параллельно с выполнением программы.
 * Сохранение данных в EEPROM произойдет только если за заданное время не произошло ни одной операции приема параметров
 * из UART-a и сохраненные параметры отличаются от текущих.
 * \param d pointer to ECU data structure
 */
void save_param_if_need(struct ecudata_t* d);

/**Loads the parameters from the EEPROM, and verifies the integrity of the data if they spoiled
 * it takes a backup instance from the FLASH.
 * Call this function only when EEPROM is idle!
 * Загружает параметры из EEPROM, проверяет целостность данных и если они испорчены то
 * берет резервную копию из FLASH.
 * \param d pointer to ECU data structure
 */
void load_eeprom_params(struct ecudata_t* d);

#ifdef REALTIME_TABLES
/** Loads tables into RAM depending on current fuel type and index of selected table (selected in parameters).
 *  Call this function only when EEPROM is idle!
 * \param d pointer to ECU data structure
 */
void load_selected_tables_into_ram(struct ecudata_t* d);

/** Loads tables into RAM depending on specified fuel type and index
 *  Call this function only when EEPROM is idle!
 * \param d pointer to ECU data structure
 * \param fuel_type type of fuel (0 - gasoline, 1 - gas)
 * \param index index of tables set to load into RAM
 */
void load_specified_tables_into_ram(struct ecudata_t* d, uint8_t fuel_type, uint8_t index);
#endif

/** Cache for buffering parameters used during suspended EEPROM operations */
extern uint8_t eeprom_parameters_cache[];

/** Resets EEPROM contents to default values and resets device, this function has same effect as
 *  closed jumper "Default EEPROM"
 * \param d pointer to ECU data structure
 */
void reset_eeprom_params(struct ecudata_t* d);

#endif //_PARAMS_H_
