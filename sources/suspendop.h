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

/** \file suspendop.h
 * Execution of suspended operations
 * (Выполнение отложенных операций).
 */

#ifndef _SUSPOP_H_
#define _SUSPOP_H_

#include <stdint.h>

#define SOP_NA                       255  //!< not defined
#define SOP_SAVE_PARAMETERS          0    //!< save parameters
#define SOP_SAVE_CE_MERGED_ERRORS    1    //!< save merged bits of CE errors
#define SOP_SEND_NC_PARAMETERS_SAVED 2    //!< notify that parameters has been saved
#define SOP_SAVE_CE_ERRORS           3    //!< save CE errors into EEPROM
#define SOP_SEND_NC_CE_ERRORS_SAVED  4    //!< notify that CE errors has been saved
#define SOP_READ_CE_ERRORS           5    //!< read CE errors
#define SOP_TRANSMIT_CE_ERRORS       6    //!< transmit CE errors
#define SOP_SEND_FW_SIG_INFO         7    //!< send signature information about firmware
#define SOP_NEW_TABLSET_SELECTED     8    //!< send information about new tables set has being selected

//Эти константы не должны быть равны 0
#define OPCODE_EEPROM_PARAM_SAVE     1    //!< save EEPROM parameters
#define OPCODE_CE_SAVE_ERRORS        2    //!< save CE errors
#define OPCODE_READ_FW_SIG_INFO      3    //!< read signature information about firmware
#define OPCODE_NEW_TABLSET_SELECTED  4    //!< new tables set has being selected

struct ecudata_t;

/**Set specified operation to execution queue (установка указанной рперации в очередь на выполнение)
 * \param opcode code of operation to be executed
 */
void sop_set_operation(uint8_t opcode);

/**Check for specified operation is pending (проверка - ждет выполнения или выполняется операция)
 * \param opcode code of operation
 * \return 1 - operation is active, 0 - not in queque (or finished)
 */
uint8_t sop_is_operation_active(uint8_t opcode);

/**Module initialization (инициализация модуля) */
void sop_init_operations(void);

/**Process queue of suspended operations (обработка очереди отложенных операций) 
 * \param d pointer to ECU data structure
 */
void sop_execute_operations(struct ecudata_t* d);

#endif //#define _SUSPOP_H_
