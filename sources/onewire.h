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

/** \file onewire.h
 * \author Alexey A. Shabelnikov
 * 1-wire protocol API
 * (API протокола 1-wire).
 */

#ifndef _ONEWIRE_H_
#define _ONEWIRE_H_

#ifdef IMMOBILIZER

#include <stdint.h>

#define OWCMD_READ_ROM      0x33     //!< Read ROM command

//Note that interrupts must be disabled when using 1-wire functions.

/** Call this function before using any other 1-wire functions,
 * it will save values of I/O registers
 */
void onewire_save_io_registers(void);

/** Call this function to restore previously saved values of I/O registers
 */
void onewire_restore_io_registers(void);

/** Reset 1-wire bus and check device presence
 * \return non-zero if device is present, otherwise - 0
 */
uint8_t onewire_reset(void);

/** Writes one byte to 1-wire bus
 * \param data Byte to write to 1-wire bus
 */
void onewire_write_byte(uint8_t data);

/** Reads one byte from 1-wire bus
 * \return Byte read from 1-wire bus
 */
uint8_t onewire_read_byte(void);

#endif //IMMOBILIZER

#endif //_ONEWIRE_H_
