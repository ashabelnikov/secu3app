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

/** \file intrinsic.h
 * \author Alexey A. Shabelnikov
 * Intrinsic functions and macros
 */

#ifndef _SECU3_INTRINSIC_H_
#define _SECU3_INTRINSIC_H_

#ifdef __ICCAVR__
 #include <inavr.h> //IAR's intrinsics

 //abstracting intrinsics
 #define _ENABLE_INTERRUPT() __enable_interrupt()
 #define _DISABLE_INTERRUPT() __disable_interrupt()
 #define _SAVE_INTERRUPT() __save_interrupt()
 #define _RESTORE_INTERRUPT(s) __restore_interrupt(s)
 #define _NO_OPERATION() __no_operation()
 #define _DELAY_CYCLES(cycles) __delay_cycles(cycles)
 #define _DELAY_US(us) __delay_cycles((us) * (F_CPU / 1000000UL))
 #define _WATCHDOG_RESET() __watchdog_reset()

 //accepts byte address!
 #define CALL_ADDRESS(addr) ((void (*)())((addr)/2))()

#else //AVR GCC
 #include <avr/eeprom.h>       //__EEGET(), __EEPUT() etc

 //Old versions of avr-gcc have only _EEGET() and _EEPUT() defined
 #ifndef __EEGET
  #define __EEGET(val, addr) _EEGET(val, addr)
 #endif

 #ifndef __EEPUT
  #define __EEPUT(addr, val) _EEPUT(addr, val)
 #endif

 //Following helper function is used by _DELAY_CYCLES() macro.
 //The loop executes 4 CPU cycles per iteration
 static inline void _delay_4cpi(uint16_t __count)
 {
  __asm__ volatile (
  "1: sbiw %0,1" "\n\t"
  "brne 1b"
  : "=w" (__count)
  : "0" (__count)
  );
 }

 //Following function implements ICALL AVR instruction.
 //Function works with word address
 static inline void _icall_ins(uint16_t __address)
 {
  __asm__ volatile (
  "icall"
  :: "z" (__address)
  );
 }

 //abstracting intrinsics
 #define _ENABLE_INTERRUPT() sei()
 #define _DISABLE_INTERRUPT() cli()
 #define _SAVE_INTERRUPT() SREG
 #define _RESTORE_INTERRUPT(s) SREG = (s)
 #define _NO_OPERATION() __asm__ __volatile__ ("nop" ::)
 #define _DELAY_CYCLES(cycles) _delay_4cpi(cycles / 4)
 #define _DELAY_US(us) _delay_4cpi((us) * (F_CPU / 4000000UL))
 #define _WATCHDOG_RESET() __asm__ __volatile__ ("wdr")

 //accepts byte address!
 #define CALL_ADDRESS(addr) _icall_ins(((addr)/2))

#endif

#endif //_SECU3_INTRINSIC_H_
