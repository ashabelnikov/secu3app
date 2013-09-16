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

#ifndef _SECU3_AVRIO_H_
#define _SECU3_AVRIO_H_

#ifdef __ICCAVR__
 #include <ioavr.h>     //device IO

//Mega64 compatibility
#ifdef __ATmega64__
#ifndef RXEN
 #define RXEN  RXEN0
#endif
#ifndef TXEN
 #define TXEN  TXEN0
#endif
#ifndef UDRE
 #define UDRE  UDRE0
#endif
#ifndef RXC
 #define RXC   RXC0
#endif
#ifndef RXCIE
 #define RXCIE RXCIE0
#endif
#ifndef UDRIE
 #define UDRIE UDRIE0
#endif
#ifndef UCSZ0
 #define UCSZ0 UCSZ00
#endif
#ifndef UCSZ1
 #define UCSZ1 UCSZ01
#endif
#ifndef U2X
 #define U2X U2X0
#endif
 #define UDR   UDR0
 #define UBRRL UBRR0L
 #define UBRRH UBRR0H
 #define UCSRA UCSR0A
 #define UCSRB UCSR0B
 #define UCSRC UCSR0C
 #define USART_UDRE_vect USART0_UDRE_vect
 #define USART_RXC_vect USART0_RXC_vect
#endif

#ifdef __ATmega16__
 #define EIMSK GICR
#elif __ATmega32__
 #define EIMSK GICR
#endif

#else //AVR GCC
 #include <avr/io.h>    //device IO

 //Who can tell me why I must do this?
 #ifdef __AVR_ATmega64__
  #ifndef EE_RDY_vect
   #define EE_RDY_vect EE_READY_vect
  #endif
 #endif

 //Again...
 #ifdef __AVR_ATmega64__
  #ifndef USART0_RXC_vect
   #define USART0_RXC_vect USART0_RX_vect
  #endif
 #endif

//Mega64 compatibility
#ifdef __AVR_ATmega64__
#ifndef RXEN
 #define RXEN  RXEN0
#endif
#ifndef TXEN
 #define TXEN  TXEN0
#endif
#ifndef UDRE
 #define UDRE  UDRE0
#endif
#ifndef RXC
 #define RXC   RXC0
#endif
#ifndef RXCIE
 #define RXCIE RXCIE0
#endif
#ifndef UDRIE
 #define UDRIE UDRIE0
#endif
#ifndef UCSZ0
 #define UCSZ0 UCSZ00
#endif
#ifndef UCSZ1
 #define UCSZ1 UCSZ01
#endif
#ifndef U2X
 #define U2X U2X0
#endif
 #define UDR   UDR0
 #define UBRRL UBRR0L
 #define UBRRH UBRR0H
 #define UCSRA UCSR0A
 #define UCSRB UCSR0B
 #define UCSRC UCSR0C
 #define USART_UDRE_vect USART0_UDRE_vect
 #define USART_RXC_vect USART0_RXC_vect
#endif

#ifdef __AVR_ATmega16__
 #define EIMSK GICR
#elif __AVR_ATmega32__
 #define EIMSK GICR
#endif

#endif

#endif //_SECU3_AVRIO_H_
