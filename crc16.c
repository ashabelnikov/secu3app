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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#include "crc16.h"

#define      P_16   0xA001     //полином


/*  Контрольная сумма crc16
*    uint8_t *buf - указатель на байтовый буфер
*    uint16_t num - размер буфера в байтах
*  Возвращает
*    uint16_t crc16 буфера
*/
uint16_t crc16( uint8_t *buf, uint16_t num )
{
uint16_t i;
uint16_t crc = 0xffff;

  while ( num-- )
  { 
    crc ^= *buf++;
    i = 8;
    do
    { 
      if ( crc & 1 )
        crc = ( crc >> 1 ) ^ P_16;
      else
        crc >>= 1;
    } while ( --i );
  }  
  return( crc );
}

//вариант для данных во flash
uint16_t crc16f(uint8_t __flash *buf, uint16_t num )
{
uint16_t i;
uint16_t crc = 0xffff;

  while ( num-- )
  { 
    crc ^= *buf++;   
    i = 8;
    do
    { 
      if ( crc & 1 )
        crc = ( crc >> 1 ) ^ P_16;
      else
        crc >>= 1;
    } while ( --i );
  }
  
  return( crc );
}
