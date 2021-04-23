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

/** \file crc16.h
 * \author Alexey A. Shabelnikov
 * CRC16 related functions.
 * Functions for calculate CRC16 of data in RAM and in the ROM
 */

#ifndef _CRC16_H_
#define _CRC16_H_

#include "port/pgmspace.h"
#include <stdint.h>

/** Calculates CRC16 for a given block of data in RAM
 * \param buf Pointer to a block of data (RAM)
 * \param num Size of block to process in bytes
 * \return Calculated CRC16
 */
uint16_t crc16(uint8_t *buf, uint16_t num);
uint16_t crc16_b(uint8_t *buf, uint16_t num);

/** Calculates CRC16 for a given block of data in ROM
 * \param buf Pointer to a block of data (ROM)
 * \param num Size of block to process in bytes
 * \return Calculated CRC16
 */
uint16_t crc16f(uint8_t _HPGM *buf, pgmsize_t num);
uint16_t crc16f_b(uint8_t _HPGM *buf, pgmsize_t num);

/** Calculates CRC8 for given byte using given seed (previous CRC value)
 * The polynomial is X8 + X5 + X4 + 1 (1-Wire bus)
 * \param data Byte which CRC will be calculated
 * \param crc Previous CRC value or zero
 * \return calculated value of CRC8
 */
uint8_t update_crc8(uint8_t data, uint8_t crc);

/** Calculates and updates CRC16 for given block of data in ROM
 * \param crc Previous value of the CRC to be updated (or zero)
 * \param buf Pointer to block of data (ROM)
 * \param num Size of block in bytes to be processed
 * \return updated value on the crc
 */
uint16_t upd_crc16f(uint16_t crc, uint8_t _HPGM *buf, uint16_t num);
uint16_t upd_crc16(uint16_t crc, uint8_t *buf, uint16_t num);

#endif //_CRC16_H_
