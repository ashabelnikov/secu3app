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

/** \file bitmask.h
 * \author Alexey A. Shabelnikov
 * Support for bit operations.
 * Helpful macros for perform bit operations. Note: You must define corresponding *_ENDIAN_DATA_FORMAT
 * symbol to ensure correct work of mentioned macros.
 * (Макросы для работы с битами).
 */

#ifndef _BITMASK_H_
#define _BITMASK_H_

/** Set y bit in the byte x (установка бита y в байте x) */
#define SETBIT(x,y)   ((x) |= (1<<(y)))

/** Clear y bit in the byte x (сброс бита y в байте x) */
#define CLEARBIT(x,y) ((x) &= (~(1<<(y))))

/** Check y bit in the byte x (проверка бита y в байте x) */
#define CHECKBIT(x,y) ((x) & (1<<(y)))

/** Write specified v value to y bit in the byte x (запись значения v в бит y в байте x) */
#define WRITEBIT(x, y, v) if (v)  SETBIT(x,y); else CLEARBIT(x, y);

#ifdef LITTLE_ENDIAN_DATA_FORMAT //little-endian data store format (Intel)
 #define _AB(src,rel) *(((unsigned char*)&(src)+(rel)))                 /**< Access Nth byte in variable */
#else                            //big-endian data store format (Motorola)
 #define _AB(src,rel) *(((unsigned char*)&(src)+sizeof((src))-1-(rel))) /**< Access Nth byte in variable */
#endif

/**Converts a bit number into a 1-byte value. */
#ifndef _BV
 #define _BV(bit) (1 << (bit))
#endif

/**Converts a bit number into a 4-byte value */
#ifndef _BV32
 #define _BV32(bit) (((uint32_t)1) << (bit))
#endif

/**Converts a bit number into a 1-byte value with specifying of a bit value */
#ifndef _CBV8
 #define _CBV8(val,bit) (((uint8_t)val) << (bit))
#endif

/**Converts a bit number into a 4-byte value with specifying of a bit value */
#ifndef _CBV32
 #define _CBV32(val,bit) (((uint32_t)val) << (bit))
#endif

#endif //_BITMASK_H_
