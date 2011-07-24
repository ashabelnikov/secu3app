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

#ifndef _SECU3_PORTBITS_H_
#define _SECU3_PORTBITS_H_

#ifndef __ICCAVR__

#include <stdint.h>
typedef struct 
{
 unsigned char bit0:1, bit1:1, bit2:1, bit3:1,
               bit4:1, bit5:1, bit6:1, bit7:1;
}__bit_field_t;

#ifdef PORTA
#define PORTA_Bit0 (*((volatile __bit_field_t*) (&PORTA))).bit0
#define PORTA_Bit1 (*((volatile __bit_field_t*) (&PORTA))).bit1
#define PORTA_Bit2 (*((volatile __bit_field_t*) (&PORTA))).bit2
#define PORTA_Bit3 (*((volatile __bit_field_t*) (&PORTA))).bit3
#define PORTA_Bit4 (*((volatile __bit_field_t*) (&PORTA))).bit4
#define PORTA_Bit5 (*((volatile __bit_field_t*) (&PORTA))).bit5
#define PORTA_Bit6 (*((volatile __bit_field_t*) (&PORTA))).bit6
#define PORTA_Bit7 (*((volatile __bit_field_t*) (&PORTA))).bit7
#endif //PORTA

#ifdef PORTB
#define PORTB_Bit0 (*((volatile __bit_field_t*) (&PORTB))).bit0
#define PORTB_Bit1 (*((volatile __bit_field_t*) (&PORTB))).bit1
#define PORTB_Bit2 (*((volatile __bit_field_t*) (&PORTB))).bit2
#define PORTB_Bit3 (*((volatile __bit_field_t*) (&PORTB))).bit3
#define PORTB_Bit4 (*((volatile __bit_field_t*) (&PORTB))).bit4
#define PORTB_Bit5 (*((volatile __bit_field_t*) (&PORTB))).bit5
#define PORTB_Bit6 (*((volatile __bit_field_t*) (&PORTB))).bit6
#define PORTB_Bit7 (*((volatile __bit_field_t*) (&PORTB))).bit7
#endif //PORTB

#ifdef PORTC
#define PORTC_Bit0 (*((volatile __bit_field_t*) (&PORTC))).bit0
#define PORTC_Bit1 (*((volatile __bit_field_t*) (&PORTC))).bit1
#define PORTC_Bit2 (*((volatile __bit_field_t*) (&PORTC))).bit2
#define PORTC_Bit3 (*((volatile __bit_field_t*) (&PORTC))).bit3
#define PORTC_Bit4 (*((volatile __bit_field_t*) (&PORTC))).bit4
#define PORTC_Bit5 (*((volatile __bit_field_t*) (&PORTC))).bit5
#define PORTC_Bit6 (*((volatile __bit_field_t*) (&PORTC))).bit6
#define PORTC_Bit7 (*((volatile __bit_field_t*) (&PORTC))).bit7
#endif //PORTC

#ifdef PORTD
#define PORTD_Bit0 (*((volatile __bit_field_t*) (&PORTD))).bit0
#define PORTD_Bit1 (*((volatile __bit_field_t*) (&PORTD))).bit1
#define PORTD_Bit2 (*((volatile __bit_field_t*) (&PORTD))).bit2
#define PORTD_Bit3 (*((volatile __bit_field_t*) (&PORTD))).bit3
#define PORTD_Bit4 (*((volatile __bit_field_t*) (&PORTD))).bit4
#define PORTD_Bit5 (*((volatile __bit_field_t*) (&PORTD))).bit5
#define PORTD_Bit6 (*((volatile __bit_field_t*) (&PORTD))).bit6
#define PORTD_Bit7 (*((volatile __bit_field_t*) (&PORTD))).bit7
#endif //PORTD

#ifdef PINA
#define PINA_Bit0 (*((volatile __bit_field_t*) (&PINA))).bit0
#define PINA_Bit1 (*((volatile __bit_field_t*) (&PINA))).bit1
#define PINA_Bit2 (*((volatile __bit_field_t*) (&PINA))).bit2
#define PINA_Bit3 (*((volatile __bit_field_t*) (&PINA))).bit3
#define PINA_Bit4 (*((volatile __bit_field_t*) (&PINA))).bit4
#define PINA_Bit5 (*((volatile __bit_field_t*) (&PINA))).bit5
#define PINA_Bit6 (*((volatile __bit_field_t*) (&PINA))).bit6
#define PINA_Bit7 (*((volatile __bit_field_t*) (&PINA))).bit7
#endif //PINA

#ifdef PINB
#define PINB_Bit0 (*((volatile __bit_field_t*) (&PINB))).bit0
#define PINB_Bit1 (*((volatile __bit_field_t*) (&PINB))).bit1
#define PINB_Bit2 (*((volatile __bit_field_t*) (&PINB))).bit2
#define PINB_Bit3 (*((volatile __bit_field_t*) (&PINB))).bit3
#define PINB_Bit4 (*((volatile __bit_field_t*) (&PINB))).bit4
#define PINB_Bit5 (*((volatile __bit_field_t*) (&PINB))).bit5
#define PINB_Bit6 (*((volatile __bit_field_t*) (&PINB))).bit6
#define PINB_Bit7 (*((volatile __bit_field_t*) (&PINB))).bit7
#endif //PINB

#ifdef PINC
#define PINC_Bit0 (*((volatile __bit_field_t*) (&PINC))).bit0
#define PINC_Bit1 (*((volatile __bit_field_t*) (&PINC))).bit1
#define PINC_Bit2 (*((volatile __bit_field_t*) (&PINC))).bit2
#define PINC_Bit3 (*((volatile __bit_field_t*) (&PINC))).bit3
#define PINC_Bit4 (*((volatile __bit_field_t*) (&PINC))).bit4
#define PINC_Bit5 (*((volatile __bit_field_t*) (&PINC))).bit5
#define PINC_Bit6 (*((volatile __bit_field_t*) (&PINC))).bit6
#define PINC_Bit7 (*((volatile __bit_field_t*) (&PINC))).bit7
#endif //PINC

#endif //__ICCAVR__

#endif //_SECU3_PORTBITS_H_
