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

/** \file crc16.h
 * CRC16 related functions.
 * Functions for calculate CRC16 of data in RAM and in the ROM
 * (Функции для вычисления контрольной суммы для данных в ОЗУ и в ПЗУ).
 */

#ifndef _CRC16_H_
#define _CRC16_H_

#include "port/pgmspace.h"
#include <stdint.h>

/** Calculates CRC16 for given block of data in RAM
 * (Вычисляет контрольную сумму CRC16 для блока данных в ОЗУ).
 * \param buf pointer to block of data (RAM) (указатель на байтовый буфер)
 * \param num size of block to process (размер буфера в байтах)
 * \return calculated CRC16 (контрольная сумма CRC16)
 */
uint16_t crc16(uint8_t *buf, uint16_t num);

/** Calculates CRC16 for given block of data in ROM
 * (Вычисляет контрольную сумму CRC16 для блока данных в ПЗУ).
 * \param buf pointer to block of data (ROM) (указатель на байтовый буфер)
 * \param num size of block to process (размер буфера в байтах)
 * \return calculated CRC16 (контрольная сумма CRC16)
 */
uint16_t crc16f(uint8_t _PGM *buf, uint16_t num);

#endif //_CRC16_H_
