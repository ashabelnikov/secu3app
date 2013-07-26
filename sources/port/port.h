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

#ifndef _SECU3_PORT_H_
#define _SECU3_PORT_H_

#ifdef __ICCAVR__
 #define MAIN() __C_task void main(void) 

 //convert compiler-specific symbols to common symbols
 #ifdef __ATmega16__
  #define _PLATFORM_M16_
  #define F_CPU 16000000UL
 #elif  __ATmega32__
  #define _PLATFORM_M32_
  #define F_CPU 16000000UL
 #elif  __ATmega64__
  #define _PLATFORM_M64_
  #define F_CPU 16000000UL
 #else
  #error "avrio.h: Wrong platform identifier!"
 #endif

 #define INLINE _Pragma("inline")

#elif defined(__GNUC__) // GNU Compiler
 //main() can be void if -ffreestanding compiler option specified.
 #define MAIN() __attribute__ ((OS_main)) void main(void)

 //convert compiler-specific symbols to common symbols
 #if defined (__AVR_ATmega16__)
  #define _PLATFORM_M16_
  #define F_CPU 16000000UL
 #elif defined (__AVR_ATmega32__)
  #define _PLATFORM_M32_
  #define F_CPU 16000000UL
 #elif defined (__AVR_ATmega64__)
  #define _PLATFORM_M64_
  #define F_CPU 16000000UL
 #else
  #error "avrio.h: Wrong platform identifier!"
 #endif

 #define INLINE inline
#else //Unknown compiler!
 #error "port.h: Unknown C compiler!"
#endif

#endif //_SECU3_PORT_H_
