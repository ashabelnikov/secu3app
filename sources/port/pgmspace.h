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

/** \file pgmspace.h
 * \author Alexey A. Shabelnikov
 * Promram space access macros
 */

#ifndef _SECU3_PGMSPACE_H_
#define _SECU3_PGMSPACE_H_

#ifdef __ICCAVR__
 #include <pgmspace.h>
 #include <stdint.h>

#if defined(__ATmega1284__)
 #define __flash__  __farflash
 #define __hugeflash__  __hugeflash
 typedef uint32_t pgmsize_t;
#else //644
 #define __flash__  __flash
 #define __hugeflash__  __flash
 typedef uint16_t pgmsize_t;
#endif

 //Declare variable in FLASH at fixed address
 #define PGM_FIXED_ADDR_OBJ(variable, sect_name) _Pragma("object_attribute=__root") \
__flash__ variable@sect_name

 //Declare variable in FLASH
 #define PGM_DECLARE(x) __flash__ x

 #define PGM_GET_BYTE(addr) *(addr)
 #define PGM_GET_WORD(addr) *(addr)
 #define PGM_GET_DWORD(addr) *(addr)
 #define MEMCPY_P(dest, src, len) memcpy_P(dest, src, len)

 #define _PGM __flash__
 #define _HPGM __hugeflash__

#else //AVR GCC
 #include <avr/pgmspace.h>

 //Declare variable in FLASH at fixed address
 #define PGM_FIXED_ADDR_OBJ(variable, sect_name) variable __attribute__((section (sect_name)))

 //Declare variable in FLASH
 #define PGM_DECLARE(x) const x __attribute__((__progmem__))

 #if defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  #define PGM_GET_BYTE(addr) pgm_read_byte_far(addr)
  #define PGM_GET_WORD(addr) pgm_read_word_far(addr)
  #define PGM_GET_DWORD(addr) pgm_read_dword_far(addr)
  #define MEMCPY_P(dest, src, len) memcpy_PF(dest, (uint_farptr_t)(src), len)
 #else //644
  #define PGM_GET_BYTE(addr) pgm_read_byte(addr)
  #define PGM_GET_WORD(addr) pgm_read_word(addr)
  #define PGM_GET_DWORD(addr) pgm_read_dword(addr)
  #define MEMCPY_P(dest, src, len) memcpy_P(dest, src, len)
 #endif

 //GCC doesn't support Harvard's architecture, so use const at least to indicate
 //that variable will be read-only
 #define _PGM const
 #define _HPGM const __memx

 #if defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  typedef uint32_t pgmsize_t;
 #else //644
  typedef uint16_t pgmsize_t;
 #endif

#endif

#endif //_SECU3_PGMSPACE_H_
