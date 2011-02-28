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

#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SAVE_PARAM_TIMEOUT_VALUE      3000
#define FORCE_MEASURE_TIMEOUT_VALUE   50
#define CE_CONTROL_STATE_TIME_VALUE   50
#define ENGINE_ROTATION_TIMEOUT_VALUE 15
#define IDLE_PERIOD_TIME_VALUE        50

//описывает все входы системы - их производные и интегральные величины
typedef struct sensors_t
{
 uint16_t map;                           //давление во впускном коллекторе (усредненное)
 uint16_t voltage;                       //напряжение бортовой сети (усредненное)
 int16_t  temperat;                      //температура охлаждающей жидкости (усредненная)
 uint16_t frequen;                       //частота вращения коленвала (усредненная)
 uint16_t inst_frq;                      //мгновенная частота вращения
 uint8_t  carb;                          //состояние концевика карбюратора
 uint8_t  gas;                           //состояние газового клапана
 uint16_t frequen4;                      //частота усредненная всего по 4-м выборкам
 uint16_t knock_k;                       //уровень сигнала детонации

 //сырые значения датчиков (дискреты АЦП с компенсированными погрешностями)
 int16_t  map_raw;
 int16_t  voltage_raw;
 int16_t  temperat_raw;

}sensors_t;

//описывает данные системы, обеспечивает единый интерфейс данных
typedef struct ecudata_t
{
 struct params_t  param;                //--параметры
 struct sensors_t sens;                 //--сенсоры

 uint8_t  ie_valve;                     //состояние клапана ЭПХХ
 uint8_t  fe_valve;                     //состояние клапана ЭМР
 uint8_t  ce_state;                     //состояние лампы "CE"
 uint8_t  airflow;                      //расход воздуха
 int16_t  curr_angle;                   //текущий угол опережения
 int16_t  knock_retard;                 //поправка УОЗ от регулятора по детонации

 __flash f_data_t*  fn_dat;             //указатель на набор характеристик

 uint8_t  op_comp_code;                 //содержит код который посылается через UART (пакет OP_COMP_NC)
 uint8_t  op_actn_code;                 //содержит код который принимается через UART (пакет OP_COMP_NC)
 uint16_t ecuerrors_for_transfer;       //буферизирует коды ошибок передаваемые через UART в реальном времени.
 uint16_t ecuerrors_saved_transfer;     //буферизирует коды ошибок для чтения/записи в EEPROM, передаваемые/принимаемые через UART.
 uint8_t  use_knock_channel_prev;       //предыдущее состояние признака использования канала детонации

 uint8_t* eeprom_parameters_cache;

 uint8_t engine_mode;                  //текущий режим двигателя (пуск, ХХ, нагрузка)
}ecudata_t;


#endif  //_SECU3_H_
