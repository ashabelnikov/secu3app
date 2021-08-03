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

/** \file avrio.h
 * \author Alexey A. Shabelnikov
 * Define I/O registers and bits
 */

#ifndef _SECU3_AVRIO_H_
#define _SECU3_AVRIO_H_

#ifdef __ICCAVR__
 #include <ioavr.h>     //device IO

//UART registers
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

//EEPROM
#ifndef EEPE
 #define EEPE EEWE
#endif
#ifndef EEMPE
 #define EEMPE EEMWE
#endif

//SPI
#ifndef SPIE
 #define SPIE SPIE0
#endif
#ifndef SPE
 #define SPE SPE0
#endif
#ifndef MSTR
 #define MSTR MSTR0
#endif
#ifndef SPR0
 #define SPR0 SPR00
#endif
#ifndef CPHA
 #define CPHA CPHA0
#endif
#ifndef SPIF
 #define SPIF SPIF0
#endif
#ifndef CPOL
 #define CPOL CPOL0
#endif

 //EE_READY_vect is defined instead of EE_RDY_vect in iom644.h
 #if defined(__ATmega644__) || defined(__ATmega1284__)
  #ifndef EE_RDY_vect
   #define EE_RDY_vect EE_READY_vect
  #endif
 #endif

 //USART0_RX_vect is defined instead of USART0_RXC_vect in iom644.h
 #if defined(__ATmega644__) || defined(__ATmega1284__)
  #ifndef USART0_RXC_vect
   #define USART0_RXC_vect USART0_RX_vect
  #endif
 #endif

//some bits of the GTCCR register:
#ifndef TSM
 #define TSM 7
#endif

#ifndef PSRSYNC
 #define PSRSYNC 0
#endif

#else //AVR GCC
 #include <avr/io.h>    //device IO

 //Who can tell me why I must do this?
 #if defined(__AVR_ATmega64__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  #ifndef EE_RDY_vect
   #define EE_RDY_vect EE_READY_vect
  #endif
 #endif

 //Again...
 #if defined(__AVR_ATmega64__)  || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  #ifndef USART0_RXC_vect
   #define USART0_RXC_vect USART0_RX_vect
  #endif
 #endif

//UART registers
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

//EEPROM
#ifndef EEPE
 #define EEPE EEWE
#endif
#ifndef EEMPE
 #define EEMPE EEMWE
#endif

#endif

//PORTA
#ifndef PA0
#define PA0 PORTA0
#endif

#ifndef PA1
#define PA1 PORTA1
#endif

#ifndef PA2
#define PA2 PORTA2
#endif

#ifndef PA3
#define PA3 PORTA3
#endif

#ifndef PA4
#define PA4 PORTA4
#endif

#ifndef PA5
#define PA5 PORTA5
#endif

#ifndef PA6
#define PA6 PORTA6
#endif

#ifndef PA7
#define PA7 PORTA7
#endif

//PORTB
#ifndef PB0
#define PB0 PORTB0
#endif

#ifndef PB1
#define PB1 PORTB1
#endif

#ifndef PB2
#define PB2 PORTB2
#endif

#ifndef PB3
#define PB3 PORTB3
#endif

#ifndef PB4
#define PB4 PORTB4
#endif

#ifndef PB5
#define PB5 PORTB5
#endif

#ifndef PB6
#define PB6 PORTB6
#endif

#ifndef PB7
#define PB7 PORTB7
#endif

//PORTC
#ifndef PC0
#define PC0 PORTC0
#endif

#ifndef PC1
#define PC1 PORTC1
#endif

#ifndef PC2
#define PC2 PORTC2
#endif

#ifndef PC3
#define PC3 PORTC3
#endif

#ifndef PC4
#define PC4 PORTC4
#endif

#ifndef PC5
#define PC5 PORTC5
#endif

#ifndef PC6
#define PC6 PORTC6
#endif

#ifndef PC7
#define PC7 PORTC7
#endif

//PORTD
#ifndef PD0
#define PD0 PORTD0
#endif

#ifndef PD1
#define PD1 PORTD1
#endif

#ifndef PD2
#define PD2 PORTD2
#endif

#ifndef PD3
#define PD3 PORTD3
#endif

#ifndef PD4
#define PD4 PORTD4
#endif

#ifndef PD5
#define PD5 PORTD5
#endif

#ifndef PD6
#define PD6 PORTD6
#endif

#ifndef PD7
#define PD7 PORTD7
#endif


//DDRA
#ifndef DDA0
#define DDA0 DDRA0
#endif

#ifndef DDA1
#define DDA1 DDRA1
#endif

#ifndef DDA2
#define DDA2 DDRA2
#endif

#ifndef DDA3
#define DDA3 DDRA3
#endif

#ifndef DDA4
#define DDA4 DDRA4
#endif

#ifndef DDA5
#define DDA5 DDRA5
#endif

#ifndef DDA6
#define DDA6 DDRA6
#endif

#ifndef DDA7
#define DDA7 DDRA7
#endif

//DDRB
#ifndef DDB0
#define DDB0 DDRB0
#endif

#ifndef DDB1
#define DDB1 DDRB1
#endif

#ifndef DDB2
#define DDB2 DDRB2
#endif

#ifndef DDB3
#define DDB3 DDRB3
#endif

#ifndef DDB4
#define DDB4 DDRB4
#endif

#ifndef DDB5
#define DDB5 DDRB5
#endif

#ifndef DDB6
#define DDB6 DDRB6
#endif

#ifndef DDB7
#define DDB7 DDRB7
#endif

//DDRC
#ifndef DDC0
#define DDC0 DDRC0
#endif

#ifndef DDC1
#define DDC1 DDRC1
#endif

#ifndef DDC2
#define DDC2 DDRC2
#endif

#ifndef DDC3
#define DDC3 DDRC3
#endif

#ifndef DDC4
#define DDC4 DDRC4
#endif

#ifndef DDC5
#define DDC5 DDRC5
#endif

#ifndef DDC6
#define DDC6 DDRC6
#endif

#ifndef DDC7
#define DDC7 DDRC7
#endif

//DDRD
#ifndef DDD0
#define DDD0 DDRD0
#endif

#ifndef DDD1
#define DDD1 DDRD1
#endif

#ifndef DDD2
#define DDD2 DDRD2
#endif

#ifndef DDD3
#define DDD3 DDRD3
#endif

#ifndef DDD4
#define DDD4 DDRD4
#endif

#ifndef DDD5
#define DDD5 DDRD5
#endif

#ifndef DDD6
#define DDD6 DDRD6
#endif

#ifndef DDD7
#define DDD7 DDRD7
#endif

#endif //_SECU3_AVRIO_H_
