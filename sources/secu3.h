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

/** \file secu3.h
 * Main module of the firmware.
 * This file contains main data structures used in the firmware.
 * (Главный модуль прошивки. Этот файл содержит основные структуры данных используемые в прошивке).
 */

#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SAVE_PARAM_TIMEOUT_VALUE      3000  //!< timeout value used to count time before automatic saving of parameters
#define FORCE_MEASURE_TIMEOUT_VALUE   20    //!< timeout value used to perform measurements when engine is stopped
#define CE_CONTROL_STATE_TIME_VALUE   50    //!< used for CE (flashing)
#define ENGINE_ROTATION_TIMEOUT_VALUE 20    //!< timeout value used to determine that engine is stopped (this value must not exceed 25)
#define IDLE_PERIOD_TIME_VALUE        50    //!< used by idling regulator

#ifdef DIAGNOSTICS
/**Describes diagnostics inputs data */
typedef struct diagnost_inp_t
{
 uint16_t voltage;                       //!< board voltage
 uint16_t map;                           //!< MAP sensor
 uint16_t temp;                          //!< coolant temperature
 uint16_t add_io1;                       //!< additional input 1 (analog)
 uint16_t add_io2;                       //!< additional input 2 (analog)
 uint16_t carb;                          //!< carburetor switch, throttle position sensor (analog)
 uint8_t  bits;                          //!< bits describing states of: gas valve, CKP sensor, VR type cam sensor, Hall-effect cam sensor, BL jmp, DE jmp
 uint16_t ks_1;                          //!< knock sensor 1
 uint16_t ks_2;                          //!< knock sensor 2
}diagnost_inp_t;
#endif

/**Describes all system's inputs - theirs derivative and integral magnitudes
 * описывает все входы системы - их производные и интегральные величины
 */
typedef struct sensors_t
{
 uint16_t map;                           //!< Input Manifold Pressure (давление во впускном коллекторе (усредненное))
 uint16_t voltage;                       //!< Board voltage (напряжение бортовой сети (усредненное))
 int16_t  temperat;                      //!< Coolant temperature (температура охлаждающей жидкости (усредненная))
 uint16_t frequen;                       //!< Averaged RPM (частота вращения коленвала (усредненная))
 uint16_t inst_frq;                      //!< Instant RPM - not averaged  (мгновенная частота вращения)
 uint8_t  carb;                          //!< State of carburetor's limit switch (состояние концевика карбюратора)
 uint8_t  gas;                           //!< State of gas valve (состояние газового клапана)
 uint16_t frequen4;                      //!< RPM averaged by using only 4 samples (частота усредненная всего по 4-м выборкам)
 uint16_t knock_k;                       //!< Knock signal level (уровень сигнала детонации)

 //сырые значения датчиков (дискреты АЦП с компенсированными погрешностями)
 int16_t  map_raw;                       //!< raw ADC value from MAP sensor
 int16_t  voltage_raw;                   //!< raw ADC value from voltage
 int16_t  temperat_raw;                  //!< raw ADC value from coolant temperature sensor

}sensors_t;

/**Describes system's data (main ECU data structure)
 * описывает данные системы, обеспечивает единый интерфейс данных
 */
typedef struct ecudata_t
{
 struct params_t  param;                 //!< --parameters (параметры)
 struct sensors_t sens;                  //!< --sensors (сенсоры)

 uint8_t  ie_valve;                      //!< State of Idle Economizer valve (состояние клапана ЭПХХ)
 uint8_t  fe_valve;                      //!< State of Fuel Economizer valve (состояние клапана ЭМР)
 uint8_t  ce_state;                      //!< State of CE lamp (состояние лампы "CE")
 uint8_t  airflow;                       //!< Air flow (расход воздуха)
 int16_t  curr_angle;                    //!< Current advance angle (текущий угол опережения)
 int16_t  knock_retard;                  //!< Correction of advance angle from knock detector (поправка УОЗ от регулятора по детонации)


#ifndef REALTIME_TABLES 
 f_data_t _PGM *fn_dat;                  //!< Pointer to the set of tables (указатель на набор характеристик)
#else
 f_data_t tables_ram[2];                 //!< set of tables in RAM for petrol(0) and gas(1)
 f_data_t* fn_dat;                       //!< pointer to current set of tables in RAM
 uint8_t  fn_gas_prev;                   //!< previous index of tables set used for gas
 uint8_t  fn_gasoline_prev;              //!< previous index of tables set used for petrol
#endif

 uint16_t op_comp_code;                  //!< Contains code of operation for packet being sent - OP_COMP_NC (содержит код который посылается через UART (пакет OP_COMP_NC))
 uint16_t op_actn_code;                  //!< Contains code of operation for packet being received - OP_COMP_NC (содержит код который принимается через UART (пакет OP_COMP_NC))
 uint16_t ecuerrors_for_transfer;        //!< Buffering of error codes being sent via UART in real time (буферизирует коды ошибок передаваемые через UART в реальном времени)
 uint16_t ecuerrors_saved_transfer;      //!< Buffering of error codes for read/write from/to EEPROM which is being sent/received (буферизирует коды ошибок для чтения/записи в EEPROM, передаваемые/принимаемые через UART)
 uint8_t  use_knock_channel_prev;        //!< Previous state of knock channel's usage flag (предыдущее состояние признака использования канала детонации)

 //TODO: To reduce memory usage it is possible to use crc or some simple hash algorithms to control state of memory(changes). So this variable becomes unneeded.
 uint8_t* eeprom_parameters_cache;       //!< Pointer to an array of EEPROM parameters cache (reduces redundant write operations)

 uint8_t engine_mode;                    //!< Current engine mode(start, idle, work) (текущий режим двигателя (пуск, ХХ, нагрузка))

#ifdef DIAGNOSTICS
 diagnost_inp_t diag_inp;                //!< diagnostic mode: values of inputs 
 uint16_t       diag_out;                //!< diagnostic mode: values of outputs
#endif
}ecudata_t;


#endif  //_SECU3_H_
