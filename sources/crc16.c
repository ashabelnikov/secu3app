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

/** \file crc16.c
 * \author Alexey A. Shabelnikov
 * Implementation of CRC16 related functions.
 * Functions for calculate CRC16 of data in RAM and in the ROM
 */

#include "port/port.h"
#include "crc16.h"
#include "wdt.h"

#define      P_16   0xA001     //!< polynomial

//variant for RAM
uint16_t crc16(uint8_t *buf, uint16_t num)
{
 uint8_t i;
 uint16_t crc = 0xffff;

 while(num--)
 {
  crc ^= *buf++;
  i = 8;
  do
  {
   if (crc & 1)
    crc = (crc >> 1) ^ P_16;
   else
    crc >>= 1;
  } while(--i);
 }
 return(crc);
}

//variant for FLASH
uint16_t crc16f(uint8_t _HPGM *buf, pgmsize_t num)
{
 uint8_t i;
 uint16_t crc = 0xffff;

 while(num--)
 {
  crc ^= PGM_GET_BYTE(buf++);
  i = 8;
  do
  {
   if (crc & 1)
    crc = (crc >> 1) ^ P_16;
   else
    crc >>= 1;
  } while(--i);
 }

 return(crc);
}

uint8_t update_crc8(uint8_t data, uint8_t crc)
{
 uint8_t i = 8;
 do
 {
  if ((crc ^ data) & 0x01)
   crc = ((crc ^ 0x18) >> 1) | 0x80;
  else
   crc >>= 1;
  data >>= 1;
 }
 while(--i);

 return crc;
}

uint16_t upd_crc16f(uint16_t crc, uint8_t _HPGM *buf, uint16_t num)
{
 uint8_t i;
 while(num--)
 {
  crc ^= PGM_GET_BYTE(buf++);
  i = 8;
  do
  {
   if (crc & 1)
    crc = (crc >> 1) ^ P_16;
   else
    crc >>= 1;
  } while(--i);
 }
 return crc;
}

uint16_t upd_crc16(uint16_t crc, uint8_t *buf, uint16_t num)
{
 uint8_t i;
 while(num--)
 {
  crc ^= *buf++;
  i = 8;
  do
  {
   if (crc & 1)
    crc = (crc >> 1) ^ P_16;
   else
    crc >>= 1;
  } while(--i);
 }
 return crc;
}

uint16_t crc16f_b(uint8_t _HPGM *buf, pgmsize_t num)
{
 uint16_t crc = 0xFFFF;
 uint16_t blockSize = 128;
 num+= ((uint32_t)buf);

 while(1)
 {
  wdt_reset_timer();
  uint16_t d = num - ((uint32_t)buf);
  if (d > blockSize)
  {
   crc = upd_crc16f(crc, buf, blockSize);
  }
  else
   return upd_crc16f(crc, buf, d);
  buf+=blockSize;
 }
}

uint16_t crc16_b(uint8_t *buf, uint16_t num)
{
 uint16_t crc = 0xFFFF;
 uint16_t blockSize = 128;
 num+=((uint16_t)buf);

 while(1)
 {
  wdt_reset_timer();
  uint16_t d = num - ((uint16_t)buf);
  if (d > blockSize)
  {
   crc = upd_crc16(crc, buf, blockSize);
  }
  else
   return upd_crc16(crc, buf, d);
  buf+=blockSize;
 }
}
