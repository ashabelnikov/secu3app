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

/** \file ioconfig.c
 * \author Alexey A. Shabelnikov
 * Implementation of I/O configuration. Allows I/O functions remapping
 * (Реализация переназначения выводов).
 */

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include <stdint.h>

void iocfg_i_ign_out1(uint8_t value)
{
 WRITEBIT(PORTD, PD4, value);
 DDRD |= _BV(DDD4);
}

void iocfg_i_ign_out1i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTD, PD4, !value);
 DDRD |= _BV(DDD4);
}

void iocfg_i_ign_out2(uint8_t value)
{
 WRITEBIT(PORTD, PD5, value);
 DDRD |= _BV(DDD5);
}

void iocfg_i_ign_out2i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTD, PD5, !value);
 DDRD |= _BV(DDD5);
}

void iocfg_s_ign_out1(uint8_t value)
{
 WRITEBIT(PORTD, PD4, value);
}

void iocfg_s_ign_out1i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTD, PD4, !value);
}

void iocfg_s_ign_out2(uint8_t value)
{
 WRITEBIT(PORTD, PD5, value);
}

void iocfg_s_ign_out2i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTD, PD5, !value);
}

void iocfg_i_ign_out3(uint8_t value)
{
 WRITEBIT(PORTC, PC0, value);
 DDRC |= _BV(DDC0);
}

void iocfg_i_ign_out3i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTC, PC0, !value);
 DDRC |= _BV(DDC0);
}

void iocfg_s_ign_out3(uint8_t value)
{
 WRITEBIT(PORTC, PC0, value);
}

void iocfg_s_ign_out3i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTC, PC0, !value);
}

void iocfg_i_ign_out4(uint8_t value)
{
 WRITEBIT(PORTC, PC1, value);
 DDRC |= _BV(DDC1);
}

void iocfg_i_ign_out4i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTC, PC1, !value);
 DDRC |= _BV(DDC1);
}

void iocfg_s_ign_out4(uint8_t value)
{
 WRITEBIT(PORTC, PC1, value);
}

void iocfg_s_ign_out4i(uint8_t value)  //inverted version
{
 WRITEBIT(PORTC, PC1, !value);
}

void iocfg_i_add_io1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
 DDRC |= _BV(DDC5);
}

void iocfg_i_add_io1i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTC, PC5, !value);
 DDRC |= _BV(DDC5);
}

void iocfg_s_add_io1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
}

void iocfg_s_add_io1i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTC, PC5, !value);
}

void iocfg_i_add_io2(uint8_t value)
{
 WRITEBIT(PORTA, PA4, value);
 DDRA |= _BV(DDA4);
}

void iocfg_i_add_io2i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTA, PA4, !value);
 DDRA |= _BV(DDA4);
}

void iocfg_s_add_io2(uint8_t value)
{
 WRITEBIT(PORTA, PA4, value);
}

void iocfg_s_add_io2i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTA, PA4, !value);
}

void iocfg_i_ecf(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, value);
#else
 WRITEBIT(PORTD, PD7, !(value));
#endif
 DDRD |= _BV(DDD7);
}

void iocfg_i_ecfi(uint8_t value)       //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, !value);
#else
 WRITEBIT(PORTD, PD7, value);
#endif
 DDRD |= _BV(DDD7);
}

void iocfg_s_ecf(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, value);
#else
 WRITEBIT(PORTD, PD7, !(value));
#endif
}

void iocfg_s_ecfi(uint8_t value)       //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, !value);
#else
 WRITEBIT(PORTD, PD7, value);
#endif
}

void iocfg_i_st_block(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, value);
#else
 WRITEBIT(PORTB, PB1, !(value));
#endif
 DDRB |= _BV(DDB1);
}

void iocfg_i_st_blocki(uint8_t value)  //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, !value);
#else
 WRITEBIT(PORTB, PB1, value);
#endif
 DDRB |= _BV(DDB1);
}

void iocfg_s_st_block(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, value);
#else
 WRITEBIT(PORTB, PB1, !(value));
#endif
}

void iocfg_s_st_blocki(uint8_t value)  //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, !value);
#else
 WRITEBIT(PORTB, PB1, value);
#endif
}

void iocfg_i_ie(uint8_t value)
{
 WRITEBIT(PORTB, PB0, value);
 DDRB |= _BV(DDB0);
}

void iocfg_i_iei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTB, PB0, !value);
 DDRB |= _BV(DDB0);
}

void iocfg_s_ie(uint8_t value)
{
 WRITEBIT(PORTB, PB0, value);
}

void iocfg_s_iei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTB, PB0, !value);
}

void iocfg_i_fe(uint8_t value)
{
 WRITEBIT(PORTC, PC7, value);
 DDRC |= _BV(DDC7);
}

void iocfg_i_fei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC7, !value);
 DDRC |= _BV(DDC7);
}

void iocfg_s_fe(uint8_t value)
{
 WRITEBIT(PORTC, PC7, value);
}

void iocfg_s_fei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC7, !value);
}

void iocfg_s_stub(uint8_t nil)
{
 //this is a stub!
}

void iocfg_i_ce(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB2, value);
#else
 WRITEBIT(PORTB, PB2, !value);
#endif
 DDRB |= _BV(DDB2);
}

void iocfg_i_cei(uint8_t value)        //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB2, !value);
#else
 WRITEBIT(PORTB, PB2, value);
#endif
 DDRB |= _BV(DDB2);
}

void iocfg_s_ce(uint8_t value)
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB2, value);
#else
 WRITEBIT(PORTB, PB2, !value);
#endif
}

void iocfg_s_cei(uint8_t value)        //inverted version
{
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB2, !value);
#else
 WRITEBIT(PORTB, PB2, value);
#endif
}

void iocfg_i_bl(uint8_t value)
{
 WRITEBIT(PORTC, PC3, value);
 DDRC |= _BV(DDC3);
}

void iocfg_i_bli(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC3, !value);
 DDRC |= _BV(DDC3);
}

void iocfg_s_bl(uint8_t value)
{
 WRITEBIT(PORTC, PC3, value);
}

void iocfg_s_bli(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC3, !value);
}

void iocfg_i_de(uint8_t value)
{
 WRITEBIT(PORTC, PC2, value);
 DDRC |= _BV(DDC2);
}

void iocfg_i_dei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC2, !value);
 DDRC |= _BV(DDC2);
}

void iocfg_s_de(uint8_t value)
{
 WRITEBIT(PORTC, PC2, value);
}

void iocfg_s_dei(uint8_t value)        //inverted version
{
 WRITEBIT(PORTC, PC2, !value);
}

void iocfg_i_ps(uint8_t value)
{
 WRITEBIT(PORTD, PD3, value);          //controls pullup resistor
 DDRD &= ~_BV(DDD3);                   //input
}

void iocfg_i_psi(uint8_t value)
{
 WRITEBIT(PORTD, PD3, value);          //controlls pullup resistor
 DDRD &= ~_BV(DDD3);                   //input
}

uint8_t iocfg_g_ps(void)
{
 return !!CHECKBIT(PIND, PIND3);
}

uint8_t iocfg_g_psi(void)              //inverted version
{
 return !CHECKBIT(PIND, PIND3);
}

void iocfg_i_add_i1(uint8_t value)
{
 WRITEBIT(PORTA, PA6, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA6);                 //input
}

void iocfg_i_add_i1i(uint8_t value)
{
 WRITEBIT(PORTA, PA6, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA6);                 //input
}

uint8_t iocfg_g_add_i1(void)
{
 return !!CHECKBIT(PINA, PINA6);
}

uint8_t iocfg_g_add_i1i(void)          //inverted version
{
 return !CHECKBIT(PINA, PINA6);
}

void iocfg_i_add_i2(uint8_t value)
{
 WRITEBIT(PORTA, PA5, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA5);                 //input
}

void iocfg_i_add_i2i(uint8_t value)
{
 WRITEBIT(PORTA, PA5, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA5);                 //input
}

uint8_t iocfg_g_add_i2(void)
{
 return !!CHECKBIT(PINA, PINA5);
}

uint8_t iocfg_g_add_i2i(void)          //inverted version
{
 return !CHECKBIT(PINA, PINA5);
}

void iocfg_i_ref_s(uint8_t value)
{
 WRITEBIT(PORTD, PD2, value);          //controlls pullup resistor
 CLEARBIT(DDRD, DDD2);                 //input
}

void iocfg_i_ref_si(uint8_t value)     //inverted version
{
 WRITEBIT(PORTD, PD2, value);          //controlls pullup resistor
 CLEARBIT(DDRD, DDD2);                 //input
}

uint8_t iocfg_g_ref_s(void)
{
 return !!CHECKBIT(PIND, PIND2);
}

uint8_t iocfg_g_ref_si(void)           //inverted version
{
 return !CHECKBIT(PIND, PIND2);
}

void iocfg_i_gas_v(uint8_t value)
{
 WRITEBIT(PORTC, PC6, value);          //controlls pullup resistor
 CLEARBIT(DDRC, DDC6);                 //input
}

void iocfg_i_gas_vi(uint8_t value)     //inverted version
{
 WRITEBIT(PORTC, PC6, value);          //controlls pullup resistor
 CLEARBIT(DDRC, DDC6);                 //input
}

uint8_t iocfg_g_gas_v(void)
{
 return !!CHECKBIT(PINC, PINC6);
}

uint8_t iocfg_g_gas_vi(void)           //inverted version
{
 return !CHECKBIT(PINC, PINC6);
}

void iocfg_i_ckps(uint8_t value)
{
 WRITEBIT(PORTD, PD6, value);          //controlls pullup resistor
 CLEARBIT(DDRD, DDD6);                 //input
}

void iocfg_i_ckpsi(uint8_t value)     //inverted version
{
 WRITEBIT(PORTD, PD6, value);          //controlls pullup resistor
 CLEARBIT(DDRD, DDD6);                 //input
}

uint8_t iocfg_g_ckps(void)
{
 return !!CHECKBIT(PIND, PIND6);
}

uint8_t iocfg_g_ckpsi(void)           //inverted version
{
 return !CHECKBIT(PIND, PIND6);
}


uint8_t iocfg_g_stub(void)
{
 //this is a stub! Always return 0
 return 0;
}
