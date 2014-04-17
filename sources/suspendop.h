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

//#define SOP_NA                     255  //!< not defined
#define SOP_SAVE_PARAMETERS          0    //!< save parameters
#define SOP_SAVE_CE_MERGED_ERRORS    1    //!< save merged bits of CE errors
#define SOP_SEND_NC_PARAMETERS_SAVED 2    //!< notify that parameters has been saved
#define SOP_SAVE_CE_ERRORS           3    //!< save CE errors into EEPROM
#define SOP_SEND_NC_CE_ERRORS_SAVED  4    //!< notify that CE errors has been saved
#define SOP_READ_CE_ERRORS           5    //!< read CE errors
#define SOP_TRANSMIT_CE_ERRORS       6    //!< transmit CE errors
#define SOP_SEND_FW_SIG_INFO         7    //!< send signature information about firmware
#ifdef REALTIME_TABLES
#define SOP_SELECT_TABLSET           8    //!< select new set of tables (when selected in parameters)
#define SOP_LOAD_TABLSET             9    //!< load new set of tables
#define SOP_SEND_NC_TABLSET_LOADED  10    //!< notify that new tables set has being selected
#define SOP_SAVE_TABLSET            11    //!< save table set for selected fuel from RAM to EEPROM
#define SOP_SEND_NC_TABLSET_SAVED   12    //!< notify that table set for selected fuel has been saved
#endif
#ifdef DEBUG_VARIABLES
#define SOP_DBGVAR_SENDING          13    //!< send out some of firmware variables
#endif
#ifdef DIAGNOSTICS
#define SOP_SEND_NC_ENTER_DIAG      14    //!< notify that device has entered diagnostic mode
#define SOP_SEND_NC_LEAVE_DIAG      15    //!< notify that device has left diagnostic mode
#endif
#define SOP_SEND_NC_RESET_EEPROM    16    //!< notify that device has entered into the EEPROM resetting mode

//Эти константы не должны быть равны 0
#define OPCODE_EEPROM_PARAM_SAVE     1    //!< save EEPROM parameters
#define OPCODE_CE_SAVE_ERRORS        2    //!< save CE errors
#define OPCODE_READ_FW_SIG_INFO      3    //!< read signature information about firmware
#ifdef REALTIME_TABLES
#define OPCODE_LOAD_TABLSET          4    //!< new set of tables must be loaded or notify that it has been loaded
#define OPCODE_SAVE_TABLSET          5    //!< save table set for selected fuel from RAM to EEPROM or notify that it has been saved
#endif
#ifdef DIAGNOSTICS
#define OPCODE_DIAGNOST_ENTER        6    //!< enter diagnostic mode
#define OPCODE_DIAGNOST_LEAVE        7    //!< leave diagnostic mode
#endif
#define OPCODE_RESET_EEPROM       0xCF    //!< reset EEPROM, second byte must be 0xAA
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
