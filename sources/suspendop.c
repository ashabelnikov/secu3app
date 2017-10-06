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

/** \file suspendop.c
 * \author Alexey A. Shabelnikov
 * Implementation of execution of suspended operations
 */

#include "port/port.h"
#include <string.h>
#include "bitmask.h"
#include "ce_errors.h"
#include "crc16.h"
#include "ecudata.h"
#include "eeprom.h"
#include "params.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "wdt.h"

/**Maximum allowed number of suspended operations */
#define SUSPENDED_OPERATIONS_SIZE 32

/**Contains queue of suspended operations. Each operation can appear one time */
uint8_t suspended_opcodes[SUSPENDED_OPERATIONS_SIZE];

/*INLINE*/
void sop_set_operation(uint8_t opcode)
{
 suspended_opcodes[(opcode)] = (opcode);
}

/*INLINE*/
void sop_reset_operation(uint8_t opcode)
{
 suspended_opcodes[opcode] = SOP_NA;
}

/*INLINE*/
uint8_t sop_is_operation_active(uint8_t opcode)
{
 return (suspended_opcodes[(opcode)] == (opcode));
}

void sop_init_operations(void)
{
 memset(suspended_opcodes, SOP_NA, SUSPENDED_OPERATIONS_SIZE);
}

/**Delay 25ms*/
void delay_25ms(void)
{
 _DELAY_US(8000);    //8ms
 wdt_reset_timer();
 _DELAY_US(8000);    //8ms
 wdt_reset_timer();
 _DELAY_US(8000);    //8ms
 wdt_reset_timer();
 _DELAY_US(1000);    //1ms
}

//Обработка операций которые могут требовать или требуют оложенного выполнения.
void sop_execute_operations(void)
{
 if (sop_is_operation_active(SOP_SAVE_PARAMETERS))
 {
  //мы не можем начать сохранение параметров, так как EEPROM на данный момент занято - сохранение
  //откладывается и будет осуществлено когда EEPROM освободится и будет вновь вызвана эта функция.
  if (eeprom_is_idle())
  {
   //для обеспечения атомарности данные будут скопированы в отдельный буфер и из него потом записаны в EEPROM.
   memcpy(eeprom_parameters_cache,&d.param,sizeof(params_t));
   ((params_t*)eeprom_parameters_cache)->crc=crc16(eeprom_parameters_cache,sizeof(params_t)-PAR_CRC_SIZE); //calculate check sum
   eeprom_start_wr_data(OPCODE_EEPROM_PARAM_SAVE, EEPROM_PARAM_START, eeprom_parameters_cache, sizeof(params_t));

   //если была соответствующая ошибка, то она теряет смысл после того как в EEPROM будут
   //записаны новые параметры с корректной контрольной суммой
   ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SAVE_PARAMETERS);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_CE_MERGED_ERRORS))
 {
  //Если EEPROM не занято, то необходимо сохранить массив с кодами ошибок Cehck Engine.
  //Для сбережения ресурса EEPROM cохранение ошибки произойдет только в том случае, если
  //она еще не была сохранена. Для этого производится чтение и сравнение.
  if (eeprom_is_idle())
  {
   ce_save_merged_errors(0);

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SAVE_CE_MERGED_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_PARAMETERS_SAVED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_EEPROM_PARAM_SAVE;
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SEND_NC_PARAMETERS_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   ce_save_merged_errors(&d.ecuerrors_saved_transfer);

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SAVE_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_CE_ERRORS_SAVED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_CE_SAVE_ERRORS;
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SEND_NC_CE_ERRORS_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_READ_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   eeprom_read(&d.ecuerrors_saved_transfer, EEPROM_ECUERRORS_START, sizeof(uint16_t));
   sop_set_operation(SOP_TRANSMIT_CE_ERRORS);
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_READ_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_TRANSMIT_CE_ERRORS))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(CE_SAVED_ERR);    //теперь передатчик озабочен передачей данных
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_TRANSMIT_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_FW_SIG_INFO))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(FWINFO_DAT);    //теперь передатчик озабочен передачей данных
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SEND_FW_SIG_INFO);
  }
 }

#ifdef REALTIME_TABLES
 if (sop_is_operation_active(SOP_SEND_NC_TABLSET_LOADED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_LOAD_TABLSET;
   _AB(d.op_comp_code, 1) = 0; //not used
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SEND_NC_TABLSET_LOADED);
  }
 }

 if (sop_is_operation_active(SOP_LOAD_TABLSET))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   //bits: aaaabbbb
   // aaaa - not used
   // bbbb - index of tables set to load from, begins from FLASH's indexes
   uint8_t index = (_AB(d.op_actn_code, 1) & 0xF);
   load_specified_tables_into_ram(index);
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_LOAD_TABLSET);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_TABLSET_SAVED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_SAVE_TABLSET;
   _AB(d.op_comp_code, 1) = 0; //not used
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SEND_NC_TABLSET_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_TABLSET))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   eeprom_start_wr_data(OPCODE_SAVE_TABLSET, EEPROM_REALTIME_TABLES_START, &d.tables_ram, sizeof(f_data_t));

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   sop_reset_operation(SOP_SAVE_TABLSET);
  }
 }

#endif

#ifdef DEBUG_VARIABLES
 if (sop_is_operation_active(SOP_DBGVAR_SENDING))
 {
  //Is sender busy (передатчик занят)?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(DBGVAR_DAT);    //send packet with debug information
   //"delete" this operation from list because it has already completed
   sop_reset_operation(SOP_DBGVAR_SENDING);
  }
 }
#endif

#ifdef DIAGNOSTICS
 if (sop_is_operation_active(SOP_SEND_NC_ENTER_DIAG))
 {
  //Is sender busy (передатчик занят)?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_DIAGNOST_ENTER;
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных
   //"delete" this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_ENTER_DIAG);
  }
 }
 if (sop_is_operation_active(SOP_SEND_NC_LEAVE_DIAG))
 {
  //Is sender busy (передатчик занят)?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_DIAGNOST_LEAVE;
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных
   delay_25ms();       //wait 25ms because of pending UART packet
   wdt_reset_device(); //wait for death :-)
  }
 }
#endif

 if (sop_is_operation_active(SOP_SEND_NC_RESET_EEPROM))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_RESET_EEPROM;
   _AB(d.op_comp_code, 1) = 0x55;
   uart_send_packet(OP_COMP_NC);    //теперь передатчик озабочен передачей данных
   delay_25ms();           //wait 25ms because of pending UART packet
   reset_eeprom_params();  //no back way!
  }
 }

 //если есть завершенная операция EEPROM, то сохраняем ее код для отправки нотификации
 switch(eeprom_take_completed_opcode()) //TODO: review assembler code -take!
 {
  case OPCODE_EEPROM_PARAM_SAVE:
   sop_set_operation(SOP_SEND_NC_PARAMETERS_SAVED);
   break;

  case OPCODE_CE_SAVE_ERRORS:
   sop_set_operation(SOP_SEND_NC_CE_ERRORS_SAVED);
   break;

#ifdef REALTIME_TABLES
  case OPCODE_SAVE_TABLSET:
   sop_set_operation(SOP_SEND_NC_TABLSET_SAVED);
   break;
#endif
 }
}

void sop_send_gonna_bl_start(void)
{
 //send confirmation that firmware is ready to start boot loader
 _AB(d.op_comp_code, 0) = OPCODE_BL_CONFIRM;
 _AB(d.op_comp_code, 1) = 0xBC;
 uart_send_packet(OP_COMP_NC);
 //delay 25ms
 delay_25ms();
}
