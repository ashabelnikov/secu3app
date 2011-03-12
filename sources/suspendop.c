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

/** \file suspendop.c
 * Implementation of execution of suspended operations
 * (Реализация выполнения отложенных операций).
 */

#include <ioavr.h>
#include <string.h>
#include "suspendop.h"
#include "eeprom.h"
#include "crc16.h"
#include "uart.h"
#include "ufcodes.h"
#include "ce_errors.h"
#include "secu3.h"

/**Maximum allowed number of suspended operations */
#define SUSPENDED_OPERATIONS_SIZE 16

/**Contains queue of suspended operations. Each operation can appear one time */
uint8_t suspended_opcodes[SUSPENDED_OPERATIONS_SIZE];

/*#pragma inline*/
void sop_set_operation(uint8_t opcode)
{
 suspended_opcodes[(opcode)] = (opcode);
}

#pragma inline
uint8_t sop_is_operation_active(uint8_t opcode)
{
 return (suspended_opcodes[(opcode)] == (opcode));
}

void sop_init_operations(void)
{
 memset(suspended_opcodes, SOP_NA, SUSPENDED_OPERATIONS_SIZE);
}

//Обработка операций которые могут требовать или требуют оложенного выполнения.
void sop_execute_operations(struct ecudata_t* d)
{
 if (sop_is_operation_active(SOP_SAVE_PARAMETERS))
 {
  //мы не можем начать сохранение параметров, так как EEPROM на данный момент занято - сохранение
  //откладывается и будет осуществлено когда EEPROM освободится и будет вновь вызвана эта функция.
  if (eeprom_is_idle())
  {
   //для обеспечения атомарности данные будут скопированы в отдельный буфер и из него потом записаны в EEPROM.
   memcpy(d->eeprom_parameters_cache,&d->param,sizeof(params_t));
   ((params_t*)d->eeprom_parameters_cache)->crc=crc16(d->eeprom_parameters_cache,sizeof(params_t)-PAR_CRC_SIZE); //считаем контролбную сумму
   eeprom_start_wr_data(OPCODE_EEPROM_PARAM_SAVE, EEPROM_PARAM_START, d->eeprom_parameters_cache, sizeof(params_t));

   //если была соответствующая ошибка, то она теряет смысл после того как в EEPROM будут
   //записаны новые параметры с корректной контрольной суммой
   ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_SAVE_PARAMETERS] = SOP_NA;
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
   suspended_opcodes[SOP_SAVE_CE_MERGED_ERRORS] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_PARAMETERS_SAVED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   d->op_comp_code = OPCODE_EEPROM_PARAM_SAVE;
   uart_send_packet(d, OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_SEND_NC_PARAMETERS_SAVED] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_SAVE_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   ce_save_merged_errors(&d->ecuerrors_saved_transfer);

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_SAVE_CE_ERRORS] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_CE_ERRORS_SAVED))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   d->op_comp_code = OPCODE_CE_SAVE_ERRORS;
   uart_send_packet(d, OP_COMP_NC);    //теперь передатчик озабочен передачей данных

   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_SEND_NC_CE_ERRORS_SAVED] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_READ_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   eeprom_read(&d->ecuerrors_saved_transfer, EEPROM_ECUERRORS_START, sizeof(uint16_t));
   sop_set_operation(SOP_TRANSMIT_CE_ERRORS);
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_READ_CE_ERRORS] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_TRANSMIT_CE_ERRORS))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(d, CE_SAVED_ERR);    //теперь передатчик озабочен передачей данных
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_TRANSMIT_CE_ERRORS] = SOP_NA;
  }
 }

 if (sop_is_operation_active(SOP_SEND_FW_SIG_INFO))
 {
  //передатчик занят?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(d, FWINFO_DAT);    //теперь передатчик озабочен передачей данных
   //"удаляем" эту операцию из списка так как она уже выполнилась.
   suspended_opcodes[SOP_SEND_FW_SIG_INFO] = SOP_NA;
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
 }
}
