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

/** \file eeprom.h
 * \author Alexey A. Shabelnikov
 * EEPROM related functions (API).
 * Functions for read/write EEPROM and related functionality
 * (Функции для для чтения/записи EEPROM и связанная с ним функциональность)
 */

#ifndef _SECU3_EEPROM_H_
#define _SECU3_EEPROM_H_

#include "port/pgmspace.h"
#include <stdint.h>

/**Address of parameters structure in EEPROM (адрес структуры параметров в EEPROM) */
#define EEPROM_PARAM_START     0x001

/**Address of errors's array (Check Engine) in EEPROM (адрес массива ошибок (Check Engine) в EEPROM) */
#define EEPROM_ECUERRORS_START (EEPROM_PARAM_START+(sizeof(params_t)))

/**Address of tables which can be edited in real time */
#define EEPROM_REALTIME_TABLES_START (EEPROM_ECUERRORS_START + 8)

/**Address of magic number in EEPROM (last 4 bytes) */
#define EEPROM_MAGIC_START (E2END-3)

//Interface of module (интерфейс модуля)

/**Start writing process of EEPROM for selected block of data
 * (запускает процесс записи в EEPROM указанного блока данных)
 * \param opcode some code which will be remembered and can be retrieved when process finishes
 * \param eeaddr address in the EEPROM for write into
 * \param sramaddr address of block of data in RAM
 * \param size number of bytes in RAM to write (size of block)
 */
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeaddr, void* sramaddr, uint16_t size);

/**Checks if EEPROM is busy
 * (возвращает не 0 если в текущий момент никакая операция не выполняется).
 * \return 0 - busy, > 0 - idle
 */
uint8_t eeprom_is_idle(void);

/**Reads specified block of data from EEPROM to RAM (without using of interrupts)
 * читает указанный блок данных из EEPROM (без использования прерываний)
 * \param sram_dest address of buffer in the RAM which will receive data
 * \param eeaddr address in the EEPROM for read from
 * \param size size of data block to read
 */
void eeprom_read(void* sram_dest, uint16_t eeaddr, uint16_t size);

/**Writes specified block of data from RAM into EEPROM (without using of interrupts)
 * записывает указанный блок данных в EEPROM (без использования прерываний)
 * \param sram_src address of buffer in the RAM which contains data to write
 * \param eeaddr address in the EEPROM for write into
 * \param size size of block of data to write
 */
void eeprom_write(const void* sram_src, uint16_t eeaddr, uint16_t size);

/**Writes specified block of data from FLASH into EEPROM (without using of interrupts)
 * записывает указанный блок данных в EEPROM (без использования прерываний)
 * \param pgm_src address of buffer in the FLASH wich contains data to write
 * \param eeaddr address in the EEPROM for write into
 * \param size size of block of data to write
 */
void eeprom_write_P(void _PGM *pgm_src, uint16_t eeaddr, uint16_t size);

/**Returns code of last finished operation (code which was passed into eeprom_start_wr_data())
 * возвращает код выполненной операции (код переданный в функцию eeprom_start_wr_data())
 * \return code of last finished operation
 */
uint8_t eeprom_take_completed_opcode(void);

#endif //_SECU3_EEPROM_H_
