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

#ifndef _SECU3_INTERRUPT_H_
#define _SECU3_INTERRUPT_H_

#ifdef __ICCAVR__
 //Use C99 _Pragma, which can be used inside macro. Next, define macro ISR
 //to counterfeit GCC
 #define PRAGMA_QUOTE(x) _Pragma(#x)
 #define ISR(vec) PRAGMA_QUOTE(vector=vec) __interrupt void isr_##vec(void) 

#else //GCC
 #include <avr/interrupt.h>

#endif

//For using instead of __monitor
#define _BEGIN_ATOMIC_BLOCK()\
uint8_t _t = _SAVE_INTERRUPT();\
_DISABLE_INTERRUPT()

#define _END_ATOMIC_BLOCK() _RESTORE_INTERRUPT(_t)

#endif //_SECU3_INTERRUPT_H_
