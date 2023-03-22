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
#include "funconv.h"
#include "params.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "spdsens.h"
#include "wdt.h"

/**Maximum allowed number of suspended operations */
#define SUSPENDED_OPERATIONS_SIZE 32

/**Contains queue of suspended operations. Each operation can appear one time */
uint8_t suspended_opcodes[SUSPENDED_OPERATIONS_SIZE];

/**Buffered value of the VSS distance used for saving*/
static uint32_t vss_int_dist_buff = 0;

/**Buffered value of the consumed fuel used for saving*/
static uint32_t consfuel_int_buff = 0;

/*inline*/
void sop_set_operation(uint8_t opcode)
{
 suspended_opcodes[(opcode)] = (opcode);
}

/*inline*/
void sop_reset_operation(uint8_t opcode)
{
 suspended_opcodes[opcode] = SOP_NA;
}

/*inline*/
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

//Processing of operations which require deferred actions
void sop_execute_operations(void)
{
 if (sop_is_operation_active(SOP_SAVE_PARAMETERS))
 {
  //we can't start saving parameters because EEPROM is busy at the moment - some other saving is pending,
  //so we postpone new saving and it will be executed when the EEPROM become idle and this function is called again.
  if (eeprom_is_idle())
  {
   //to ensure atomicity, the data will be copied to a separate buffer and then written from it to EEPROM.
   memcpy(&eeprom_parameters_cache, &d.param, sizeof(params_t));
   eeprom_parameters_cache.crc = crc16((uint8_t*)&eeprom_parameters_cache, sizeof(params_t)-PAR_CRC_SIZE); //calculate check sum
   eeprom_start_wr_data(OPCODE_EEPROM_PARAM_SAVE, EEPROM_PARAM_START, &eeprom_parameters_cache, sizeof(params_t));

   //if there was a corresponding error, then it becomes unneeded after EEPROM contains
   //new parameters written with correct checksum
   ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_PARAMETERS);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_CE_MERGED_ERRORS))
 {
  //If EEPROM is not busy, then we need to save an array with 'Check Engine' error codes.
  //To save the EEPROM resource, the error will only be saved if
  //it hasn't been saved yet. This is done by reading and comparing.
  if (eeprom_is_idle())
  {
   ce_save_merged_errors(0);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_CE_MERGED_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_PARAMETERS_SAVED))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_EEPROM_PARAM_SAVE;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_PARAMETERS_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   ce_save_merged_errors(&d.ecuerrors_saved_transfer);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_CE_ERRORS_SAVED))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_CE_SAVE_ERRORS;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_CE_ERRORS_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_READ_CE_ERRORS))
 {
  if (eeprom_is_idle())
  {
   eeprom_read(&d.ecuerrors_saved_transfer, EEPROM_ECUERRORS_START, sizeof(uint32_t));
   sop_set_operation(SOP_TRANSMIT_CE_ERRORS);
   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_READ_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_TRANSMIT_CE_ERRORS))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(CE_SAVED_ERR);    //sender become busy now
   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_TRANSMIT_CE_ERRORS);
  }
 }

 if (sop_is_operation_active(SOP_SEND_FW_SIG_INFO))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(FWINFO_DAT);    //sender become busy now
   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_FW_SIG_INFO);
  }
 }

#ifdef REALTIME_TABLES
 if (sop_is_operation_active(SOP_SEND_NC_TABLSET_LOADED))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_LOAD_TABLSET;
   _AB(d.op_comp_code, 1) = 0; //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
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
   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_LOAD_TABLSET);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_TABLSET_SAVED))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_SAVE_TABLSET;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_TABLSET_SAVED);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_TABLSET))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   d.tables_ram.checksum = crc16_b((uint8_t*)&d.tables_ram, sizeof(f_data_t)-sizeof(uint16_t));
   eeprom_start_wr_data(OPCODE_SAVE_TABLSET, EEPROM_REALTIME_TABLES_START, &d.tables_ram, sizeof(f_data_t));

   //clear possibly present error because after saving checksum become correct
   ce_clear_error(ECUERROR_EEPROM_TABL_BROKEN);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_TABLSET);
  }
 }

#endif

#ifdef DEBUG_VARIABLES
 if (sop_is_operation_active(SOP_DBGVAR_SENDING))
 {
  //Is sender busy?
  if (!uart_is_sender_busy())
  {
   uart_send_packet(DBGVAR_DAT);    //send packet with debug information
   //remove this operation from list because it has already completed
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
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now
   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_ENTER_DIAG);
  }
 }
 if (sop_is_operation_active(SOP_SEND_NC_LEAVE_DIAG))
 {
  //Is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_DIAGNOST_LEAVE;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now
   delay_25ms();       //wait 25ms because of pending UART packet
   wdt_reset_device(); //wait for death :-)
  }
 }
#endif

 if (sop_is_operation_active(SOP_SEND_NC_RESET_EEPROM))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_RESET_EEPROM;
   _AB(d.op_comp_code, 1) = 0x55;
   uart_send_packet(OP_COMP_NC);    //sender become busy now
   delay_25ms();           //wait 25ms because of pending UART packet
   reset_eeprom_params();  //no back way!
  }
 }

#ifdef FUEL_INJECT
 if (sop_is_operation_active(SOP_RESET_LTFT))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   memset(&d.inj_ltft[0][0], 0, INJ_VE_POINTS_L*INJ_VE_POINTS_F); //reset contents of LTFT map in RAM

   d.inj_ltft_crc = crc16_b((uint8_t*)&d.inj_ltft[0][0], INJ_VE_POINTS_L*INJ_VE_POINTS_F);
   eeprom_start_wr_data(OPCODE_RESET_LTFT, EEPROM_LTFT_TABLES_START, &d.inj_ltft[0][0], (INJ_VE_POINTS_L*INJ_VE_POINTS_F)+sizeof(uint16_t));

   //clear possibly present error because after saving checksum become correct
   ce_clear_error(ECUERROR_EEPROM_LTFT_BROKEN);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_RESET_LTFT);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_LTFT_RESET))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_RESET_LTFT;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_LTFT_RESET);
  }
 }

 if (sop_is_operation_active(SOP_SAVE_LTFT))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   sop_start_saving_ltft();

   //clear possibly present error because after saving checksum become correct
   ce_clear_error(ECUERROR_EEPROM_LTFT_BROKEN);

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_LTFT);
  }
 }

 if (sop_is_operation_active(SOP_SEND_NC_LTFT_SAVE))
 {
  //is sender busy?
  if (!uart_is_sender_busy())
  {
   _AB(d.op_comp_code, 0) = OPCODE_SAVE_LTFT;
   _AB(d.op_comp_code, 1) = 0;      //not used
   uart_send_packet(OP_COMP_NC);    //sender become busy now

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SEND_NC_LTFT_SAVE);
  }
 }
#endif

#ifdef SPEED_SENSOR
 if (sop_is_operation_active(SOP_SAVE_ODOMET))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   sop_start_saving_odometer();

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_ODOMET);
  }
 }
#endif

#ifdef FUEL_INJECT
 if (sop_is_operation_active(SOP_SAVE_CONSFUEL))
 {
  //TODO: d.op_actn_code may become overwritten while we are waiting here...
  if (eeprom_is_idle())
  {
   sop_start_saving_consfuel();

   //remove this operation from list because it has already completed
   sop_reset_operation(SOP_SAVE_CONSFUEL);
  }
 }
#endif

 //if completed operation exist, then save its code for sending notification
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

#ifdef FUEL_INJECT
  case OPCODE_RESET_LTFT:
   sop_set_operation(SOP_SEND_NC_LTFT_RESET);
   break;

  case OPCODE_SAVE_LTFT:
   sop_set_operation(SOP_SEND_NC_LTFT_SAVE);
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

#ifdef SPEED_SENSOR
void sop_start_saving_odometer(void)
{
 uint16_t pulse_count = spdsens_get_pulse_count(1); //reset
 d.sens.vss_int_dist+= calc_dist(pulse_count); //accumulate ramaining distance
 //save odometer's data
 vss_int_dist_buff = d.sens.vss_int_dist;
 eeprom_start_wr_data(0, EEPROM_ODOMETER_START, &vss_int_dist_buff, sizeof(uint32_t));
}
#endif

#ifdef FUEL_INJECT
void sop_start_saving_consfuel(void)
{
 d.cons_fuel_int+= (d.cons_fuel_imm >> 9); //accumulate ramaining value of consumed fuel
 d.cons_fuel_imm = 0;
 //save odometer's data
 consfuel_int_buff = d.cons_fuel_int;
 eeprom_start_wr_data(0, EEPROM_ODOMETER_START+5, &consfuel_int_buff, sizeof(uint32_t));
}
#endif

#ifdef FUEL_INJECT
void sop_start_saving_ltft(void)
{
 d.inj_ltft_crc = crc16_b((uint8_t*)&d.inj_ltft[0][0], INJ_VE_POINTS_L*INJ_VE_POINTS_F);
 eeprom_start_wr_data(OPCODE_SAVE_LTFT, EEPROM_LTFT_TABLES_START, &d.inj_ltft[0][0], (INJ_VE_POINTS_L*INJ_VE_POINTS_F)+sizeof(uint16_t));
}
#endif
