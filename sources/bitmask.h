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

/** \file bitmask.h
 * Support for bit operations.
 * Helpful macros for perform bit operations. Note: You must define corresponding *_ENDIAN_DATA_FORMAT
 * symbol to ensure correct work of mentioned macros.
 * (ћакросы дл€ работы с битами).
 */

#ifndef _BITMASK_H_
#define _BITMASK_H_

/** Set y bit in the byte x (установка бита y в байте x) */
#define SETBIT(x,y)   ((x) |= (1<<(y)))

/** Clear y bit in the byte x (сброс бита y в байте x) */
#define CLEARBIT(x,y) ((x) &= (~(1<<(y))))

/** Check y bit in the byte x (проверка бита y в байте x) */
#define CHECKBIT(x,y) ((x) & (1<<(y)))     

#ifdef LITTLE_ENDIAN_DATA_FORMAT //little-endian data store format (Intel)
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+(rel)))                 /**< Get Nth byte from variable */
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+(rel)))                 /**< Set Nth byte of variable */
#else                            //big-endian data store format (Motorola) 
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+sizeof((src))-1-(rel))) /**< Get Nth byte from variable */
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+sizeof((des))-1-(rel))) /**< Set Nth byte of variable */
#endif  

#endif //_BITMASK_H_
