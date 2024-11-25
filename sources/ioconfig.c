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
#include "port/pgmspace.h"
#include "bitmask.h"
#include <stdint.h>
#include "ioconfig.h"
#include "adc.h"
#include "ce_errors.h"
#include "tables.h"

#ifdef IOCFG_FUNC_INIT
// Can save up to 200 bytes of program memory, but loosing a little in speed

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

uint16_t IOCFG_GET(uint8_t io_id)
{
 return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(0); //get digital value from input
}

uint16_t IOCFG_GETA(uint8_t io_id)
{
 return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(1); //get analog value from input
}

uint16_t IOCFG_GETE(uint8_t io_id)
{
 return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(2); //get flag of the related CE error
}

uint8_t IOCFG_CMP(uint8_t ios_id, uint8_t iop_id)
{
 return (IOCFG_CMPN(ios_id, iop_id) || IOCFG_CMPI(ios_id, iop_id));
}
#endif

uint16_t IOCFG_GETAS(uint8_t ios_id)
{
 uint8_t ioi = IOCFG_GET_IOI(ios_id);
 if (ioi==127)
  return 0; //not mapped to any input

 if (ioi & 0x80)
  return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_slotsi[ios_id]))(1);
 else
  return ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_slots[ios_id]))(1);
}

//from measure.c
extern int16_t iocfg_map_s;
extern int16_t iocfg_add_i1;
extern int16_t iocfg_add_i2;
#ifndef SECU3T
extern int16_t iocfg_add_i3;
#endif
#ifdef TPIC8101
extern int16_t iocfg_add_i4;
#endif
#ifdef MCP3204
extern int16_t iocfg_add_i5;
extern int16_t iocfg_add_i6;
extern int16_t iocfg_add_i7;
extern int16_t iocfg_add_i8;
#endif

uint16_t iocfg_g_stub(uint8_t doa)
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
 WRITEBIT(PORTA, PA4, value);
 DDRA |= _BV(DDA4);
}

void iocfg_i_add_o2i(uint8_t value)   //inverted version
{
 WRITEBIT(PORTA, PA4, !value);
 DDRA |= _BV(DDA4);
}

void iocfg_s_add_o2(uint8_t value)
{
 WRITEBIT(PORTA, PA4, value);
}

void iocfg_s_add_o2i(uint8_t value)   //inverted version
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

uint16_t iocfg_g_ps(uint8_t doa)
{
 if (doa)
  return 0x8000 | (!!CHECKBIT(PIND, PIND3)); //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND3);
}

uint16_t iocfg_g_psi(uint8_t doa)      //inverted version
{
 if (doa)
  return 0x8000 | (!CHECKBIT(PIND, PIND3)); //analog mode is not supported on this input
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

uint16_t iocfg_g_add_i1(uint8_t doa)
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA6);      //digital
 else if (1==doa)
  return iocfg_add_i1;
 return ce_is_error(ECUERROR_ADD_I1_SENSOR);
}

uint16_t iocfg_g_add_i1i(uint8_t doa)  //inverted version
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA6);        //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i1;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I1_SENSOR);
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

uint16_t iocfg_g_add_i2(uint8_t doa)
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA5);      //digital
 else if (1==doa)
  return iocfg_add_i2;                 //analog
 return ce_is_error(ECUERROR_ADD_I2_SENSOR);
}

uint16_t iocfg_g_add_i2i(uint8_t doa)  //inverted version
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA5);       //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i2;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I2_SENSOR);
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

uint16_t iocfg_g_ref_s(uint8_t doa)
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PIND, PIND2); //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND2);
}

uint16_t iocfg_g_ref_si(uint8_t doa)   //inverted version
{
 if (doa)
  return 0x8000 | !CHECKBIT(PIND, PIND2); //analog mode is not supported on this input
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

uint16_t iocfg_g_gas_v(uint8_t doa)
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PINC, PINC6); //analog mode is not supported on this input
 return !!CHECKBIT(PINC, PINC6);
}

uint16_t iocfg_g_gas_vi(uint8_t doa)   //inverted version
{
 if (doa)
  return 0x8000 | !CHECKBIT(PINC, PINC6); //analog mode is not supported on this input
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

uint16_t iocfg_g_ckps(uint8_t doa)
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PIND, PIND6); //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND6);
}

uint16_t iocfg_g_ckpsi(uint8_t doa)    //inverted version
{
 if (doa)
  return 0x8000 | !CHECKBIT(PIND, PIND6); //analog mode is not supported on this input
 return !CHECKBIT(PIND, PIND6);
}

void iocfg_i_map_s(uint8_t value)
{
 WRITEBIT(PORTA, PA2, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA2);                 //input
}

void iocfg_i_map_si(uint8_t value)
{
 WRITEBIT(PORTA, PA2, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA2);                 //input
}

uint16_t iocfg_g_map_s(uint8_t doa)
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA2);      //digital
 else if (1==doa)
  return iocfg_map_s;                   //analog
 return ce_is_error(ECUERROR_MAP_SENSOR_FAIL);
}

uint16_t iocfg_g_map_si(uint8_t doa)   //inverted version
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA2);       //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_map_s;//do inversion
  return r < 0 ? 0 : r;                //analog
 }
 return ce_is_error(ECUERROR_MAP_SENSOR_FAIL);
}

void iocfg_i_add_i4(uint8_t value)
{
#ifdef TPIC8101
 WRITEBIT(PORTA, PA3, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                 //input
#endif
}

void iocfg_i_add_i4i(uint8_t value)
{
#ifdef TPIC8101
 WRITEBIT(PORTA, PA3, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                 //input
#endif
}

uint16_t iocfg_g_add_i4(uint8_t doa)
{
#ifdef TPIC8101
 if (0==doa)
  return !!CHECKBIT(PINA, PINA3);      //digital
 else if (1==doa)
  return iocfg_add_i4;                  //analog
 return ce_is_error(ECUERROR_ADD_I4_SENSOR);
#else
 return 0;                             //stub!
#endif
}

uint16_t iocfg_g_add_i4i(uint8_t doa)  //inverted version
{
#ifdef TPIC8101
 if (0==doa)
  return !CHECKBIT(PINA, PINA3);       //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i4;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I4_SENSOR);
#else
 return 0;                             //stub!
#endif
}

#ifdef MCP3204
 #error "MCP3204 supported only in SECU-3i, please compile without SECU3T option"
#endif

#else //---SECU-3i---

uint8_t  spi_PORTA = 0x80; //!< Bits of inputs read by SPI. 7,6 bits are reserved as outputs, 7 bit by default = 1
uint8_t  spi_PORTB = 0;   //!< Bits of outputs controlled by SPI
uint8_t  spi_IODIRA = 0;  //!< Direction control bits for SPI PORTA (inputs). 7 bit is reserved and configured as output
uint8_t  spi_IODIRB = 0;  //!< Direction control bits for SPI PORTB (outputs)
uint8_t  spi_GPPUA = 0x80;//!< Pull-up resistors control register A. 7,6 bits are reserved as outputs
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
void iocfg_i_gpa6_o(uint8_t value)      //!< init GPA6_O
{
 WRITEBIT(spi_PORTA, 6, value);
 CLEARBIT(spi_IODIRA, 6);
}

void iocfg_i_gpa6_oi(uint8_t value)     //!< init GPA6_O           (inverted)
{
 WRITEBIT(spi_PORTA, 6, !value);
 CLEARBIT(spi_IODIRA, 6);
}

void iocfg_s_gpa6_o(uint8_t value)      //!< set  GPA6_O
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTA, 6, value);
 _END_ATOMIC_BLOCK();
}

void iocfg_s_gpa6_oi(uint8_t value)     //!< set  GPA6_O           (inverted)
{
 _BEGIN_ATOMIC_BLOCK(); //TODO: In the ATmega1284 we can use I/O register, which is atomic and this line becomes unneeded
 WRITEBIT(spi_PORTA, 6, !value);
 _END_ATOMIC_BLOCK();
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

uint16_t iocfg_g_ps(uint8_t doa)        //!< get PS input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PIND, PIND3);; //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND3);
}

uint16_t iocfg_g_psi(uint8_t doa)       //!< get PS input value      (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(PIND, PIND3); //analog mode is not supported on this input
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

uint16_t iocfg_g_ref_s(uint8_t doa)     //!< get REF_S input
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PIND, PIND2); //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND2);
}

uint16_t iocfg_g_ref_si(uint8_t doa)    //!< get REF_S input         (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(PIND, PIND2); //analog mode is not supported on this input
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

uint16_t iocfg_g_ckps(uint8_t doa)      //!< get CKPS input
{
 if (doa)
  return 0x8000 | !!CHECKBIT(PIND, PIND6); //analog mode is not supported on this input
 return !!CHECKBIT(PIND, PIND6);
}

uint16_t iocfg_g_ckpsi(uint8_t doa)     //!< get CKPS input          (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(PIND, PIND6); //analog mode is not supported on this input
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

uint16_t iocfg_g_add_i1(uint8_t doa)    //!< set  ADD_I1 input
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA6);       //digital
 else if (1==doa)
  return iocfg_add_i1;                  //analog
 return ce_is_error(ECUERROR_ADD_I1_SENSOR);
}

uint16_t iocfg_g_add_i1i(uint8_t doa)   //!< set  ADD_I1 input       (inverted)
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA6);        //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i1;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I1_SENSOR);
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

uint16_t iocfg_g_add_i2(uint8_t doa)    //!< set  ADD_I2 input
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA5);       //digital
 else if (1==doa)
  return iocfg_add_i2;                  //analog
 return ce_is_error(ECUERROR_ADD_I2_SENSOR);
}

uint16_t iocfg_g_add_i2i(uint8_t doa)   //!< set  ADD_I2 input       (inverted)
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA5);        //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i2;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I2_SENSOR);
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

uint16_t iocfg_g_add_i3(uint8_t doa)    //!< set  ADD_I3 input
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA4);       //digital
 else if (1==doa)
  return iocfg_add_i3;                  //analog
 return ce_is_error(ECUERROR_ADD_I3_SENSOR);
}

uint16_t iocfg_g_add_i3i(uint8_t doa)   //!< set  ADD_I3 input       (inverted)
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA4);        //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i3;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I3_SENSOR);
}
//-----------------------------------------------------------------------------
void iocfg_i_add_i4(uint8_t value)      //!< init ADD_I4 input
{
#ifdef TPIC8101
 WRITEBIT(PORTA, PA3, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                  //input
#endif
}

void iocfg_i_add_i4i(uint8_t value)     //!< init ADD_I4 input       (inverted)
{
#ifdef TPIC8101
 WRITEBIT(PORTA, PA3, value);           //controlls pullup resistor
 CLEARBIT(DDRA, DDA3);                  //input
#endif
}

uint16_t iocfg_g_add_i4(uint8_t doa)    //!< set  ADD_I4 input
{
#ifdef TPIC8101
 if (0==doa)
  return !!CHECKBIT(PINA, PINA3);       //digital
 else if (1==doa)
  return iocfg_add_i4;                  //analog
 return ce_is_error(ECUERROR_ADD_I4_SENSOR);
#else
 return 0;                              //stub!
#endif
}

uint16_t iocfg_g_add_i4i(uint8_t doa)   //!< set  ADD_I4 input       (inverted)
{
#ifdef TPIC8101
 if (0==doa)
  return !CHECKBIT(PINA, PINA3);        //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i4;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I4_SENSOR);
#else
 return 0;                              //stub!
#endif
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

uint16_t iocfg_g_gas_v(uint8_t doa)     //!< get GAS_V input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 0); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 0);
}

uint16_t iocfg_g_gas_vi(uint8_t doa)    //!< get GAS_V input value   (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 0); //analog mode is not supported on this input
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

uint16_t iocfg_g_ign(uint8_t doa)       //!< get IGN_I input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 3); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 3);
}

uint16_t iocfg_g_igni(uint8_t doa)      //!< get IGN_I input value   (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 3); //analog mode is not supported on this input
 return !CHECKBIT(spi_PORTA, 3);
}
//-----------------------------------------------------------------------------
void iocfg_i_cond_i(uint8_t value)      //!< init COND_I input
{
 WRITEBIT(spi_GPPUA, 2, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 2);                 //input
}

void iocfg_i_cond_ii(uint8_t value)     //!< init COND_I input        (inverted)
{
 WRITEBIT(spi_GPPUA, 2, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 2);                 //input
}

uint16_t iocfg_g_cond_i(uint8_t doa)    //!< get COND_I input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 2); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 2);
}

uint16_t iocfg_g_cond_ii(uint8_t doa)   //!< get COND_I input value   (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 2); //analog mode is not supported on this input
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

uint16_t iocfg_g_epas_i(uint8_t doa)    //!< get EPAS_I input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 1); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 1);
}

uint16_t iocfg_g_epas_ii(uint8_t doa)   //!< get EPAS_I input value  (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 1); //analog mode is not supported on this input
 return !CHECKBIT(spi_PORTA, 1);
}
//-----------------------------------------------------------------------------

void iocfg_i_gpa4_i(uint8_t value)      //!< init GPA4_I input
{
 WRITEBIT(spi_GPPUA, 4, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 4);                 //input
}

void iocfg_i_gpa4_ii(uint8_t value)     //!< init GPA4_I input       (inverted)
{
 WRITEBIT(spi_GPPUA, 4, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 4);                 //input
}

uint16_t iocfg_g_gpa4_i(uint8_t doa)    //!< get GPA4_I input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 4); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 4);
}

uint16_t iocfg_g_gpa4_ii(uint8_t doa)   //!< get GPA4_I input value  (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 4); //analog mode is not supported on this input
 return !CHECKBIT(spi_PORTA, 4);
}
//-----------------------------------------------------------------------------

void iocfg_i_gpa5_i(uint8_t value)      //!< init GPA5_I input
{
 WRITEBIT(spi_GPPUA, 5, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 5);                 //input
}

void iocfg_i_gpa5_ii(uint8_t value)     //!< init GPA5_I input       (inverted)
{
 WRITEBIT(spi_GPPUA, 5, value);         //controlls pullup resistor
 SETBIT(spi_IODIRA, 5);                 //input
}

uint16_t iocfg_g_gpa5_i(uint8_t doa)    //!< get GPA5_I input value
{
 if (doa)
  return 0x8000 | !!CHECKBIT(spi_PORTA, 5); //analog mode is not supported on this input
 return !!CHECKBIT(spi_PORTA, 5);
}

uint16_t iocfg_g_gpa5_ii(uint8_t doa)   //!< get GPA5_I input value  (inverted)
{
 if (doa)
  return 0x8000 | !CHECKBIT(spi_PORTA, 5); //analog mode is not supported on this input
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

uint16_t iocfg_g_add_i5(uint8_t doa)
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[0];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) > 2048);
 }
 else if (1==doa)
  return iocfg_add_i5;                    //analog
 return ce_is_error(ECUERROR_ADD_I5_SENSOR);
#else
 return 0; //stub
#endif
}

uint16_t iocfg_g_add_i5i(uint8_t doa)    //inverted version
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[0];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) < 2048);
 }
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i5;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I5_SENSOR);
#else
 return 0; //stub
#endif
}

//-----------------------------------------------------------------------------
void iocfg_i_add_i6(uint8_t value)
{
 //stub
}

void iocfg_i_add_i6i(uint8_t value)     //inverted version
{
 //stub
}

uint16_t iocfg_g_add_i6(uint8_t doa)
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[1];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) > 2048);
 }
 else if (1==doa)
  return iocfg_add_i6;                  //analog
 return ce_is_error(ECUERROR_ADD_I6_SENSOR);
#else
 return 0; //stub
#endif
}

uint16_t iocfg_g_add_i6i(uint8_t doa)   //inverted version
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[1];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) < 2048);
 }
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i6;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I6_SENSOR);
#else
 return 0; //stub
#endif
}

//-----------------------------------------------------------------------------
void iocfg_i_add_i7(uint8_t value)
{
 //stub
}

void iocfg_i_add_i7i(uint8_t value)     //inverted version
{
 //stub
}

uint16_t iocfg_g_add_i7(uint8_t doa)
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[2];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) > 2048);
 }
 else if (1==doa)
  return iocfg_add_i7;                  //analog
 return ce_is_error(ECUERROR_ADD_I7_SENSOR);
#else
 return 0; //stub
#endif
}

uint16_t iocfg_g_add_i7i(uint8_t doa)   //inverted version
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[2];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) < 2048);
 }
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i7;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I7_SENSOR);
#else
 return 0; //stub
#endif
}

//-----------------------------------------------------------------------------
void iocfg_i_add_i8(uint8_t value)
{
 //stub
}

void iocfg_i_add_i8i(uint8_t value)     //inverted version
{
 //stub
}

uint16_t iocfg_g_add_i8(uint8_t doa)
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[3];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) > 2048);
 }
 else if (1==doa)
  return iocfg_add_i8;
 return ce_is_error(ECUERROR_ADD_I8_SENSOR);
#else
 return 0; //stub
#endif
}

uint16_t iocfg_g_add_i8i(uint8_t doa)    //inverted version
{
#ifdef MCP3204
 if (0==doa)
 { //digital
  uint16_t value;
  _BEGIN_ATOMIC_BLOCK();
  value = spiadc_chan[3];
  _END_ATOMIC_BLOCK();
  return ((value & 0xFFF) < 2048);
 }
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_add_i8;//do inversion
  return r < 0 ? 0 : r;                 //analog
 }
 return ce_is_error(ECUERROR_ADD_I8_SENSOR);
#else
 return 0; //stub
#endif
}

//-----------------------------------------------------------------------------
void iocfg_i_map_s(uint8_t value)
{
 WRITEBIT(PORTA, PA2, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA2);                 //input
}

void iocfg_i_map_si(uint8_t value)
{
 WRITEBIT(PORTA, PA2, value);          //controlls pullup resistor
 CLEARBIT(DDRA, DDA2);                 //input
}

uint16_t iocfg_g_map_s(uint8_t doa)
{
 if (0==doa)
  return !!CHECKBIT(PINA, PINA2);      //digital
 else if (1==doa)
  return iocfg_map_s;                  //analog
 return ce_is_error(ECUERROR_MAP_SENSOR_FAIL);
}

uint16_t iocfg_g_map_si(uint8_t doa)          //inverted version
{
 if (0==doa)
  return !CHECKBIT(PINA, PINA2);       //digital
 else if (1==doa)
 {
  int16_t r = ADC_DISCRNUM - iocfg_map_s;//do inversion
  return r < 0 ? 0 : r;                //analog
 }
 return ce_is_error(ECUERROR_MAP_SENSOR_FAIL);
}

//-----------------------------------------------------------------------------

#endif
