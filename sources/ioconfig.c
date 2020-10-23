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
 */

#include "port/avrio.h"
#include "port/port.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "bitmask.h"
#include <stdint.h>
#include "ioconfig.h"

#ifdef IOCFG_FUNC_INIT
// Can save up to 200 bytes of program memory, but loosing a little in speed
#include "port/pgmspace.h"
#include "tables.h"

void IOCFG_INIT(uint8_t io_id, uint8_t io_state)
{
 ((iocfg_pfn_init)_IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]))(io_state);
}

uint8_t IOCFG_CHECK(uint8_t io_id)
{
 return (_IOREM_GPTR(&fw_data.cddata.iorem.s_stub) != _IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]));
}

void IOCFG_SETF(uint8_t io_id, uint8_t io_value)
{
 ((iocfg_pfn_set)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(io_value);
}

uint8_t IOCFG_GET(uint8_t io_id)
{
 return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))();
}
#endif

uint8_t iocfg_g_stub(void)
{
 //this is a stub! Always return 0
 return 0;
}

void iocfg_s_stub(uint8_t nil)
{
 //this is a stub!
}


#ifdef SECU3T //---SECU-3T---

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

void iocfg_i_add_o1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
 DDRC |= _BV(DDC5);
}

void iocfg_i_add_o1i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTC, PC5, !value);
 DDRC |= _BV(DDC5);
}

void iocfg_s_add_o1(uint8_t value)
{
 WRITEBIT(PORTC, PC5, value);
}

void iocfg_s_add_o1i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTC, PC5, !value);
}

void iocfg_i_add_o2(uint8_t value)
{
#ifndef PA4_INP_IGNTIM
 WRITEBIT(PORTA, PA4, value);
 DDRA |= _BV(DDA4);
#endif
}

void iocfg_i_add_o2i(uint8_t value)   //inverted version
{
#ifndef PA4_INP_IGNTIM
 WRITEBIT(PORTA, PA4, !value);
 DDRA |= _BV(DDA4);
#endif
}

void iocfg_s_add_o2(uint8_t value)
{
#ifndef PA4_INP_IGNTIM
 WRITEBIT(PORTA, PA4, value);
#endif
}

void iocfg_s_add_o2i(uint8_t value)   //inverted version
{
#ifndef PA4_INP_IGNTIM
 WRITEBIT(PORTA, PA4, !value);
#endif
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

#ifdef MCP3204
 #error "MCP3204 supported only in SECU-3i, please compile without SECU3T option"
#endif

#else //---SECU-3i---

uint8_t  spi_PORTA = 0x80; //!< Bits of inputs read by SPI. 7 bit is reserved as output, by default = 1
uint8_t  spi_PORTB = 0;   //!< Bits of outputs controlled by SPI
uint8_t  spi_IODIRA = 0;  //!< Direction control bits for SPI PORTA (inputs). 7 bit is reserved and configured as output
uint8_t  spi_IODIRB = 0;  //!< Direction control bits for SPI PORTB (outputs)
uint8_t  spi_GPPUA = 0x80; //!< Pull-up resistors control register A. 7 bit is reserved as output
uint8_t  spi_GPPUB = 0;   //!< Pull-up resistors control register B

#ifdef MCP3204
uint16_t spiadc_chan[SPIADC_CHNUM] = {0};
#endif

//Outputs
void iocfg_i_ign_out1(uint8_t value)    //!< init IGN_O1
{
 WRITEBIT(PORTD, PD4, value);
 DDRD |= _BV(DDD4);
}

void iocfg_i_ign_out1i(uint8_t value)   //!< init IGN_O1           (inverted)
{
 WRITEBIT(PORTD, PD4, !value);
 DDRD |= _BV(DDD4);
}

void iocfg_s_ign_out1(uint8_t value)    //!< set  IGN_O1
{
 WRITEBIT(PORTD, PD4, value);
}

void iocfg_s_ign_out1i(uint8_t value)   //!< set  IGN_O1           (inverted)
{
 WRITEBIT(PORTD, PD4, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ign_out2(uint8_t value)    //!< init IGN_O2
{
 WRITEBIT(PORTD, PD5, value);
 DDRD |= _BV(DDD5);
}

void iocfg_i_ign_out2i(uint8_t value)   //!< init IGN_O2           (inverted)
{
 WRITEBIT(PORTD, PD5, !value);
 DDRD |= _BV(DDD5);
}

void iocfg_s_ign_out2(uint8_t value)    //!< set  IGN_O2
{
 WRITEBIT(PORTD, PD5, value);
}

void iocfg_s_ign_out2i(uint8_t value)   //!< set  IGN_O2           (inverted)
{
 WRITEBIT(PORTD, PD5, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ign_out3(uint8_t value)    //!< init IGN_O3
{
 WRITEBIT(PORTC, PC0, value);
 DDRC |= _BV(DDC0);
}

void iocfg_i_ign_out3i(uint8_t value)   //!< init IGN_O3           (inverted)
{
 WRITEBIT(PORTC, PC0, !value);
 DDRC |= _BV(DDC0);
}

void iocfg_s_ign_out3(uint8_t value)    //!< set  IGN_O3
{
 WRITEBIT(PORTC, PC0, value);
}

void iocfg_s_ign_out3i(uint8_t value)   //!< set  IGN_O3           (inverted)
{
 WRITEBIT(PORTC, PC0, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ign_out4(uint8_t value)    //!< init IGN_O4
{
 WRITEBIT(PORTC, PC1, value);
 DDRC |= _BV(DDC1);
}

void iocfg_i_ign_out4i(uint8_t value)   //!< init IGN_O4           (inverted)
{
 WRITEBIT(PORTC, PC1, !value);
 DDRC |= _BV(DDC1);
}

void iocfg_s_ign_out4(uint8_t value)    //!< set  IGN_O4
{
 WRITEBIT(PORTC, PC1, value);
}

void iocfg_s_ign_out4i(uint8_t value)   //!< set  IGN_O4           (inverted)
{
 WRITEBIT(PORTC, PC1, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ign_out5(uint8_t value)    //!< init IGN_O5
{
 WRITEBIT(PORTC, PC5, value);
 DDRC |= _BV(DDC5);
}

void iocfg_i_ign_out5i(uint8_t value)   //!< init IGN_O5           (inverted)
{
 WRITEBIT(PORTC, PC5, !value);
 DDRC |= _BV(DDC5);
}

void iocfg_s_ign_out5(uint8_t value)    //!< set  IGN_O5
{
 WRITEBIT(PORTC, PC5, value);
}

void iocfg_s_ign_out5i(uint8_t value)   //!< set  IGN_O5           (inverted)
{
 WRITEBIT(PORTC, PC5, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ecf(uint8_t value)         //!< init ECF
{
 WRITEBIT(PORTD, PD7, value);
 DDRD |= _BV(DDD7);
}

void iocfg_i_ecfi(uint8_t value)        //!< init ECF              (inverted)
{
 WRITEBIT(PORTD, PD7, !value);
 DDRD |= _BV(DDD7);
}

void iocfg_s_ecf(uint8_t value)         //!< set  ECF
{
 WRITEBIT(PORTD, PD7, value);
}

void iocfg_s_ecfi(uint8_t value)        //!< set  ECF              (inverted)
{
 WRITEBIT(PORTD, PD7, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_inj_out1(uint8_t value)    //!< init INJ_O1
{
 WRITEBIT(PORTB, PB1, value);
 DDRB |= _BV(DDB1);
}

void iocfg_i_inj_out1i(uint8_t value)   //!< init INJ_O1           (inverted)
{
 WRITEBIT(PORTB, PB1, !value);
 DDRB |= _BV(DDB1);
}

void iocfg_s_inj_out1(uint8_t value)    //!< set  INJ_O1
{
 WRITEBIT(PORTB, PB1, value);
}

void iocfg_s_inj_out1i(uint8_t value)   //!< set  INJ_O1           (inverted)
{
 WRITEBIT(PORTB, PB1, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_inj_out2(uint8_t value)    //!< init INJ_O2
{
 WRITEBIT(PORTC, PC6, value);
 DDRC |= _BV(DDC6);
}

void iocfg_i_inj_out2i(uint8_t value)   //!< init INJ_O2           (inverted)
{
 WRITEBIT(PORTC, PC6, !value);
 DDRC |= _BV(DDC6);
}

void iocfg_s_inj_out2(uint8_t value)    //!< set  INJ_O2
{
 WRITEBIT(PORTC, PC6, value);
}

void iocfg_s_inj_out2i(uint8_t value)   //!< set  INJ_O2           (inverted)
{
 WRITEBIT(PORTC, PC6, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_inj_out3(uint8_t value)    //!< init INJ_O3
{
 WRITEBIT(PORTB, PB2, value);
 DDRB |= _BV(DDB2);
}

void iocfg_i_inj_out3i(uint8_t value)   //!< init INJ_O3           (inverted)
{
 WRITEBIT(PORTB, PB2, !value);
 DDRB |= _BV(DDB2);
}

void iocfg_s_inj_out3(uint8_t value)    //!< set  INJ_O3
{
 WRITEBIT(PORTB, PB2, value);
}

void iocfg_s_inj_out3i(uint8_t value)   //!< set  INJ_O3           (inverted)
{
 WRITEBIT(PORTB, PB2, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_inj_out4(uint8_t value)    //!< init INJ_O4
{
 WRITEBIT(PORTC, PC7, value);
 DDRC |= _BV(DDC7);
}

void iocfg_i_inj_out4i(uint8_t value)   //!< init INJ_O4           (inverted)
{
 WRITEBIT(PORTC, PC7, !value);
 DDRC |= _BV(DDC7);
}

void iocfg_s_inj_out4(uint8_t value)    //!< set  INJ_O4
{
 WRITEBIT(PORTC, PC7, value);
}

void iocfg_s_inj_out4i(uint8_t value)   //!< set  INJ_O4           (inverted)
{
 WRITEBIT(PORTC, PC7, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_inj_out5(uint8_t value)    //!< init INJ_O5
{
 WRITEBIT(PORTB, PB0, value);
 DDRB |= _BV(DDB0);
}

void iocfg_i_inj_out5i(uint8_t value)   //!< init INJ_O5           (inverted)
{
 WRITEBIT(PORTB, PB0, !value);
 DDRB |= _BV(DDB0);
}

void iocfg_s_inj_out5(uint8_t value)    //!< set  INJ_O5
{
 WRITEBIT(PORTB, PB0, value);
}

void iocfg_s_inj_out5i(uint8_t value)   //!< set  INJ_O5           (inverted)
{
 WRITEBIT(PORTB, PB0, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_bl(uint8_t value)          //!< init BL
{
 WRITEBIT(PORTC, PC3, value);
 DDRC |= _BV(DDC3);
}

void iocfg_i_bli(uint8_t value)         //!< init BL               (inverted)
{
 WRITEBIT(PORTC, PC3, !value);
 DDRC |= _BV(DDC3);
}

void iocfg_s_bl(uint8_t value)          //!< set  BL
{
 WRITEBIT(PORTC, PC3, value);
}

void iocfg_s_bli(uint8_t value)         //!< set  BL               (inverted)
{
 WRITEBIT(PORTC, PC3, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_de(uint8_t value)          //!< init DE
{
 WRITEBIT(PORTC, PC2, value);
 DDRC |= _BV(DDC2);
}

void iocfg_i_dei(uint8_t value)         //!< init DE               (inverted)
{
 WRITEBIT(PORTC, PC2, !value);
 DDRC |= _BV(DDC2);
}

void iocfg_s_de(uint8_t value)          //!< set  DE
{
 WRITEBIT(PORTC, PC2, value);
}

void iocfg_s_dei(uint8_t value)         //!< set  DE               (inverted)
{
 WRITEBIT(PORTC, PC2, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_st_block(uint8_t value)    //!< init STBL_O
{
 WRITEBIT(spi_PORTB, 1, value);
 CLEARBIT(spi_IODIRB, 1);
}

void iocfg_i_st_blocki(uint8_t value)   //!< init STBL_O           (inverted)
{
 WRITEBIT(spi_PORTB, 1, !value);
 CLEARBIT(spi_IODIRB, 1);
}

void iocfg_s_st_block(uint8_t value)    //!< set  STBL_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 1, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_st_blocki(uint8_t value)   //!< set  STBL_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 1, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_ce(uint8_t value)          //!< init CEL_O
{
 WRITEBIT(spi_PORTB, 5, value);
 CLEARBIT(spi_IODIRB, 5);
}

void iocfg_i_cei(uint8_t value)         //!< init CEL_O            (inverted)
{
 WRITEBIT(spi_PORTB, 5, !value);
 CLEARBIT(spi_IODIRB, 5);
}

void iocfg_s_ce(uint8_t value)          //!< set  CEL_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 5, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_cei(uint8_t value)         //!< set  CEL_O            (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 5, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_fpmp_o(uint8_t value)      //!< init FPMP_O
{
 WRITEBIT(spi_PORTB, 3, value);
 CLEARBIT(spi_IODIRB, 3);
}

void iocfg_i_fpmp_oi(uint8_t value)     //!< init FPMP_O           (inverted)
{
 WRITEBIT(spi_PORTB, 3, !value);
 CLEARBIT(spi_IODIRB, 3);
}

void iocfg_s_fpmp_o(uint8_t value)      //!< set  FPMP_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 3, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_fpmp_oi(uint8_t value)     //!< set  FPMP_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 3, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_pwrr_o(uint8_t value)      //!< init PWRR_O
{
 WRITEBIT(spi_PORTB, 2, value);
 CLEARBIT(spi_IODIRB, 2);
}

void iocfg_i_pwrr_oi(uint8_t value)     //!< init PWRR_O           (inverted)
{
 WRITEBIT(spi_PORTB, 2, !value);
 CLEARBIT(spi_IODIRB, 2);
}

void iocfg_s_pwrr_o(uint8_t value)      //!< set  PWRR_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 2, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_pwrr_oi(uint8_t value)     //!< set  PWRR_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 2, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_evap_o(uint8_t value)      //!< init EVAP_O
{
 WRITEBIT(spi_PORTB, 6, value);
 CLEARBIT(spi_IODIRB, 6);
}

void iocfg_i_evap_oi(uint8_t value)     //!< init EVAP_O           (inverted)
{
 WRITEBIT(spi_PORTB, 6, !value);
 CLEARBIT(spi_IODIRB, 6);
}

void iocfg_s_evap_o(uint8_t value)      //!< set  EVAP_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 6, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_evap_oi(uint8_t value)     //!< set  EVAP_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 6, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_o2sh_o(uint8_t value)      //!< init O2SH_OP
{
 WRITEBIT(spi_PORTB, 7, value);
 CLEARBIT(spi_IODIRB, 7);
}

void iocfg_i_o2sh_oi(uint8_t value)     //!< init O2SH_O           (inverted)
{
 WRITEBIT(spi_PORTB, 7, !value);
 CLEARBIT(spi_IODIRB, 7);
}

void iocfg_s_o2sh_o(uint8_t value)      //!< set  O2SH_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 7, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_o2sh_oi(uint8_t value)     //!< set  O2SH_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 7, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_cond_o(uint8_t value)      //!< init COND_O
{
 WRITEBIT(spi_PORTB, 4, value);
 CLEARBIT(spi_IODIRB, 4);
}

void iocfg_i_cond_oi(uint8_t value)     //!< init COND_O           (inverted)
{
 WRITEBIT(spi_PORTB, 4, !value);
 CLEARBIT(spi_IODIRB, 4);
}

void iocfg_s_cond_o(uint8_t value)      //!< set  COND_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 4, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_cond_oi(uint8_t value)     //!< set  COND_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 4, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_add_o2(uint8_t value)      //!< init ADD_O2
{
 WRITEBIT(spi_PORTB, 0, value);
 CLEARBIT(spi_IODIRB, 0);
}

void iocfg_i_add_o2i(uint8_t value)     //!< init ADD_O2           (inverted)
{
 WRITEBIT(spi_PORTB, 0, !value);
 CLEARBIT(spi_IODIRB, 0);
}

void iocfg_s_add_o2(uint8_t value)      //!< set  ADD_O2
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 0, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_add_o2i(uint8_t value)     //!< set  ADD_O2           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTB, 0, !value);
 _END_ATOMIC_BLOCK();
}
//-----------------------------------------------------------------------------
void iocfg_i_tach_o(uint8_t value)      //!< init TACH_O
{
 WRITEBIT(PORTC, PC4, value);
 DDRC |= _BV(DDC4);
}

void iocfg_i_tach_oi(uint8_t value)     //!< init TACH_O           (inverted)
{
 WRITEBIT(PORTC, PC4, !value);
 DDRC |= _BV(DDC4);
}

void iocfg_s_tach_o(uint8_t value)      //!< set  TACH_O
{
 WRITEBIT(PORTC, PC4, value);
}

void iocfg_s_tach_oi(uint8_t value)     //!< set  TACH_O           (inverted)
{
 WRITEBIT(PORTC, PC4, !value);
}
//-----------------------------------------------------------------------------
void iocfg_i_ksp_cs(uint8_t value)      //!< init KSP_CS
{
 WRITEBIT(PORTB, PB4, value);
 DDRB |= _BV(DDB4);
}

void iocfg_i_ksp_csi(uint8_t value)     //!< init KSP_CS           (inverted)
{
 WRITEBIT(PORTB, PB4, !value);
 DDRB |= _BV(DDB4);
}

void iocfg_s_ksp_cs(uint8_t value)      //!< set  KSP_CS
{
 WRITEBIT(PORTB, PB4, value);
}

void iocfg_s_ksp_csi(uint8_t value)     //!< set  KSP_CS           (inverted)
{
 WRITEBIT(PORTB, PB4, !value);
}
//-----------------------------------------------------------------------------

//Inputs
void iocfg_i_ps(uint8_t value)          //!< init PS input
{
 WRITEBIT(PORTD, PD3, value);           //controls pullup resistor
 DDRD &= ~_BV(DDD3);                    //input
}

void iocfg_i_psi(uint8_t value)         //!< init PS input           (inverted)
{
 WRITEBIT(PORTD, PD3, value);           //controls pullup resistor
 DDRD &= ~_BV(DDD3);                    //input
}

uint8_t iocfg_g_ps(void)                //!< get PS input value
{
 return !!CHECKBIT(PIND, PIND3);
}

uint8_t iocfg_g_psi(void)               //!< get PS input value      (inverted)
{
 return !CHECKBIT(PIND, PIND3);
}
//-----------------------------------------------------------------------------
void iocfg_i_ref_s(uint8_t value)       //!< init REF_S input
{
 WRITEBIT(PORTD, PD2, value);           //controlls pullup resistor
 CLEARBIT(DDRD, DDD2);                  //input
}

void iocfg_i_ref_si(uint8_t value)      //!< init REF_S input        (inverted)
{
 WRITEBIT(PORTD, PD2, value);           //controlls pullup resistor
 CLEARBIT(DDRD, DDD2);                  //input
}

uint8_t iocfg_g_ref_s(void)             //!< get REF_S input
{
 return !!CHECKBIT(PIND, PIND2);
}

uint8_t iocfg_g_ref_si(void)            //!< get REF_S input         (inverted)
{
 return !CHECKBIT(PIND, PIND2);
}
//-----------------------------------------------------------------------------
void iocfg_i_ckps(uint8_t value)        //!< init CKPS input
{
 WRITEBIT(PORTD, PD6, value);           //controlls pullup resistor
 CLEARBIT(DDRD, DDD6);                  //input
}

void iocfg_i_ckpsi(uint8_t value)       //!< init CKPS input         (inverted)
{
 WRITEBIT(PORTD, PD6, value);           //controlls pullup resistor
 CLEARBIT(DDRD, DDD6);                  //input
}

uint8_t iocfg_g_ckps(void)              //!< get CKPS input
{
 return !!CHECKBIT(PIND, PIND6);
}

uint8_t iocfg_g_ckpsi(void)             //!< get CKPS input          (inverted)
{
 return !CHECKBIT(PIND, PIND6);
}
//-----------------------------------------------------------------------------
void iocfg_i_add_i1(uint8_t value)      //!< init ADD_I1 input
{
 WRITEBIT(PORTA, PA6, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA6);                  //input
}

void iocfg_i_add_i1i(uint8_t value)     //!< init ADD_I1 input       (inverted)
{
 WRITEBIT(PORTA, PA6, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA6);                  //input
}

uint8_t iocfg_g_add_i1(void)            //!< set  ADD_I1 input
{
 return !!CHECKBIT(PINA, PINA6);
}

uint8_t iocfg_g_add_i1i(void)           //!< set  ADD_I1 input       (inverted)
{
 return !CHECKBIT(PINA, PINA6);
}
//-----------------------------------------------------------------------------
void iocfg_i_add_i2(uint8_t value)      //!< init ADD_I2 input
{
 WRITEBIT(PORTA, PA5, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA5);                  //input
}

void iocfg_i_add_i2i(uint8_t value)     //!< init ADD_I2 input       (inverted)
{
 WRITEBIT(PORTA, PA5, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA5);                  //input
}

uint8_t iocfg_g_add_i2(void)            //!< set  ADD_I2 input
{
 return !!CHECKBIT(PINA, PINA5);
}

uint8_t iocfg_g_add_i2i(void)           //!< set  ADD_I2 input       (inverted)
{
 return !CHECKBIT(PINA, PINA5);
}
//-----------------------------------------------------------------------------
void iocfg_i_add_i3(uint8_t value)      //!< init ADD_I3 input
{
 WRITEBIT(PORTA, PA4, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA4);                  //input
}

void iocfg_i_add_i3i(uint8_t value)     //!< init ADD_I3 input       (inverted)
{
 WRITEBIT(PORTA, PA4, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA4);                  //input
}

uint8_t iocfg_g_add_i3(void)            //!< set  ADD_I3 input
{
 return !!CHECKBIT(PINA, PINA4);
}

uint8_t iocfg_g_add_i3i(void)           //!< set  ADD_I3 input       (inverted)
{
 return !CHECKBIT(PINA, PINA4);
}
//-----------------------------------------------------------------------------
void iocfg_i_add_i4(uint8_t value)      //!< init ADD_I4 input
{
 WRITEBIT(PORTA, PA3, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                  //input
}

void iocfg_i_add_i4i(uint8_t value)     //!< init ADD_I4 input       (inverted)
{
 WRITEBIT(PORTA, PA3, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                  //input
}

uint8_t iocfg_g_add_i4(void)            //!< set  ADD_I4 input
{
 return !!CHECKBIT(PINA, PINA3);
}

uint8_t iocfg_g_add_i4i(void)           //!< set  ADD_I4 input       (inverted)
{
 return !CHECKBIT(PINA, PINA3);
}
//-----------------------------------------------------------------------------
void iocfg_i_gas_v(uint8_t value)       //!< init GAS_V input
{
 WRITEBIT(spi_GPPUA, 0, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 0);                 //input
}

void iocfg_i_gas_vi(uint8_t value)      //!< init GAS_V input        (inverted)
{
 WRITEBIT(spi_GPPUA, 0, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 0);                 //input
}

uint8_t iocfg_g_gas_v(void)             //!< get GAS_V input value
{
 return !!CHECKBIT(spi_PORTA, 0);
}

uint8_t iocfg_g_gas_vi(void)            //!< get GAS_V input value   (inverted)
{
 return !CHECKBIT(spi_PORTA, 0);
}
//-----------------------------------------------------------------------------
void iocfg_i_ign(uint8_t value)         //!< init IGN_I input
{
 WRITEBIT(spi_GPPUA, 3, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 3);                 //input
}

void iocfg_i_igni(uint8_t value)        //!< init IGN_I input        (inverted)
{
 WRITEBIT(spi_GPPUA, 3, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 3);                 //input
}

uint8_t iocfg_g_ign(void)               //!< get IGN_I input value
{
 return !!CHECKBIT(spi_PORTA, 3);
}

uint8_t iocfg_g_igni(void)              //!< get IGN_I input value   (inverted)
{
 return !CHECKBIT(spi_PORTA, 3);
}
//-----------------------------------------------------------------------------
void iocfg_i_cond_i(uint8_t value)      //!< init IGN_I input
{
 WRITEBIT(spi_GPPUA, 2, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 2);                 //input
}

void iocfg_i_cond_ii(uint8_t value)     //!< init IGN_I input        (inverted)
{
 WRITEBIT(spi_GPPUA, 2, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 2);                 //input
}

uint8_t iocfg_g_cond_i(void)            //!< get IGN_I input value
{
 return !!CHECKBIT(spi_PORTA, 2);
}

uint8_t iocfg_g_cond_ii(void)           //!< get IGN_I input value   (inverted)
{
 return !CHECKBIT(spi_PORTA, 2);
}
//-----------------------------------------------------------------------------
void iocfg_i_epas_i(uint8_t value)      //!< init EPAS_I input
{
 WRITEBIT(spi_GPPUA, 1, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 1);                 //input
}

void iocfg_i_epas_ii(uint8_t value)     //!< init EPAS_I input       (inverted)
{
 WRITEBIT(spi_GPPUA, 1, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 1);                 //input
}

uint8_t iocfg_g_epas_i(void)            //!< get EPAS_I input value
{
 return !!CHECKBIT(spi_PORTA, 1);
}

uint8_t iocfg_g_epas_ii(void)           //!< get EPAS_I input value  (inverted)
{
 return !CHECKBIT(spi_PORTA, 1);
}
//-----------------------------------------------------------------------------

void iocfg_i_oilp_i(uint8_t value)      //!< init OILP_I input
{
 WRITEBIT(spi_GPPUA, 4, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 4);                 //input
}

void iocfg_i_oilp_ii(uint8_t value)     //!< init OILP_I input       (inverted)
{
 WRITEBIT(spi_GPPUA, 4, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 4);                 //input
}

uint8_t iocfg_g_oilp_i(void)            //!< get OILP_I input value
{
 return !!CHECKBIT(spi_PORTA, 4);
}

uint8_t iocfg_g_oilp_ii(void)           //!< get OILP_I input value  (inverted)
{
 return !CHECKBIT(spi_PORTA, 4);
}
//-----------------------------------------------------------------------------

void iocfg_i_gens_i(uint8_t value)      //!< init GENS_I input
{
 WRITEBIT(spi_GPPUA, 5, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 5);                 //input
}

void iocfg_i_gens_ii(uint8_t value)     //!< init GENS_I input       (inverted)
{
 WRITEBIT(spi_GPPUA, 5, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 5);                 //input
}

uint8_t iocfg_g_gens_i(void)            //!< get GENS_I input value
{
 return !!CHECKBIT(spi_PORTA, 5);
}

uint8_t iocfg_g_gens_ii(void)           //!< get GENS_I input value  (inverted)
{
 return !CHECKBIT(spi_PORTA, 5);
}
//-----------------------------------------------------------------------------

void iocfg_i_add_i5(uint8_t value)
{
 //stub
}

void iocfg_i_add_i5i(uint8_t value)     //inverted version
{
 //stub
}

uint8_t iocfg_g_add_i5(void)
{
#ifdef MCP3204
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = spiadc_chan[0];
 _END_ATOMIC_BLOCK();
 return (value > 2048);
#else
 return 0; //stub
#endif
}

uint8_t iocfg_g_add_i5i(void)          //inverted version
{
#ifdef MCP3204
 uint16_t value;
 _BEGIN_ATOMIC_BLOCK();
 value = spiadc_chan[0];
 _END_ATOMIC_BLOCK();
 return (value < 2048);
#else
 return 0; //stub
#endif
}

//-----------------------------------------------------------------------------

#endif
