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

/** \file ioconfig.с
 * Implementation of I/O configuration. Allows I/O functions remapping
 * (Реализация переназначения выводов).
 */

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include <stdint.h>

void iocfg_i_ecf(uint8_t value)
{
#ifdef SECU3T /*SECU-3T*/
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, value);
#else
 WRITEBIT(PORTD, PD7, !(value));
#endif
 DDRD |= _BV(DDD7);
#else         /*SECU-3*/
 WRITEBIT(PORTB, PB1, value);
 DDRB |= _BV(DDB1);
#endif
}

void iocfg_s_ecf(uint8_t value)
{
#ifdef SECU3T /*SECU-3T*/
#ifdef REV9_BOARD
 WRITEBIT(PORTD, PD7, value);
#else
 WRITEBIT(PORTD, PD7, !(value));
#endif
#else         /*SECU-3*/
 WRITEBIT(PORTB, PB1, value);
#endif
}

void iocfg_i_st_block(uint8_t value)
{
#ifdef SECU3T /*SECU-3T*/
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, value);
#else
 WRITEBIT(PORTB, PB1, !(value));
#endif
 DDRB |= _BV(DDB1);
#else         /*SECU-3*/
 WRITEBIT(PORTD, PD7, !(value));
 DDRD |= _BV(DDD7);
#endif
}

void iocfg_s_st_block(uint8_t value)
{
#ifdef SECU3T /*SECU-3T*/
#ifdef REV9_BOARD
 WRITEBIT(PORTB, PB1, value);
#else
 WRITEBIT(PORTB, PB1, !(value));
#endif
#else         /*SECU-3*/
 WRITEBIT(PORTD, PD7, !(value));
#endif
}

void iocfg_i_ign_out3(uint8_t value)
{
 WRITEBIT(PORTC, PC0, value);
 DDRC |= _BV(DDC0);
}

void iocfg_s_ign_out3(uint8_t value)
{
 WRITEBIT(PORTC, PC0, value);
}

void iocfg_i_ign_out4(uint8_t value)
{
 WRITEBIT(PORTC, PC1, value);
 DDRC |= _BV(DDC1);
}

void iocfg_s_ign_out4(uint8_t value)
{
 WRITEBIT(PORTC, PC1, value);
}

#ifdef SECU3T
void iocfg_i_add_io1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
 DDRC |= _BV(DDC5);
}

void iocfg_s_add_io1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
}

void iocfg_i_add_io2(uint8_t value)
{
 WRITEBIT(PORTA, PA4, value);
 DDRA |= _BV(DDA4);
}

void iocfg_s_add_io2(uint8_t value)
{
 WRITEBIT(PORTA, PA4, value);
}
#endif

void iocfg_i_ie(uint8_t value)
{
 WRITEBIT(PORTB, PB0, value);
 DDRB |= _BV(DDB0);
}

void iocfg_s_ie(uint8_t value)
{
 WRITEBIT(PORTB, PB0, value);
}

void iocfg_i_fe(uint8_t value)
{
 WRITEBIT(PORTC, PC7, value);
 DDRC |= _BV(DDC7);
}

void iocfg_s_fe(uint8_t value)
{
 WRITEBIT(PORTC, PC7, value);
}

void iocfg_s_stub(uint8_t nil)
{
 //this is a stub!
}

//Following I/Os can not be remapped
void iocfg_i_ign_out1(uint8_t value)
{
 WRITEBIT(PORTD, PD4, value);
 DDRD |= _BV(DDD4);
}

void iocfg_i_ign_out2(uint8_t value)
{
 WRITEBIT(PORTD, PD5, value);
 DDRD |= _BV(DDD5);
}

void iocfg_s_ign_out1(uint8_t value)
{
 WRITEBIT(PORTD, PD4, value);
}

void iocfg_s_ign_out2(uint8_t value)
{
 WRITEBIT(PORTD, PD5, value);
}
