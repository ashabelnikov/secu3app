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

#ifndef _SUSPOP_H_
#define _SUSPOP_H_

#include <stdint.h>

#define SOP_NA                       255
#define SOP_SAVE_PARAMETERS          0
#define SOP_SAVE_CE_MERGED_ERRORS    1
#define SOP_SEND_NC_PARAMETERS_SAVED 2
#define SOP_SAVE_CE_ERRORS           3
#define SOP_SEND_NC_CE_ERRORS_SAVED  4
#define SOP_READ_CE_ERRORS           5
#define SOP_TRANSMIT_CE_ERRORS       6
#define SOP_SEND_FW_SIG_INFO         7

//Эти константы не должны быть равны 0
#define OPCODE_EEPROM_PARAM_SAVE     1
#define OPCODE_CE_SAVE_ERRORS        2
#define OPCODE_READ_FW_SIG_INFO      3

struct ecudata_t;

//установка указанной рперации в очередь на выполнение
void sop_set_operation(uint8_t opcode);

//проверка - ждет выполнения или выполняется операция
uint8_t sop_is_operation_active(uint8_t opcode);

//инициализация модуля
void sop_init_operations(void);

//обработка очереди отложенных операций
void sop_execute_operations(struct ecudata_t* d);

#endif //#define _SUSPOP_H_
