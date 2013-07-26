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

/** \file onewire.c
 * Implementation of 1-wire protocol API.
 * (Реализация API протокола 1-wire).
 */

#include "port/avrio.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "onewire.h"

//1-wire net uses PC3
#define OW_DQ   PC3                //!< I/O data line bit number
#define OW_PIN  PINC               //!< read register
#define OW_PORT PORTC              //!< write register
#define OW_DDR  DDRC               //!< direction register

uint8_t ow_port;                   //!< contains saved value of OW_PORT register
uint8_t ow_ddr;                    //!< contains saved value of OW_DDR register

//Save I/O registers
void onewire_save_io_registers(void)
{
 ow_port = OW_PORT;                // save port register
 ow_ddr = OW_DDR;                  // save data direction register
}

//Restore I/O registers
void onewire_restore_io_registers(void)
{
 OW_PORT = ow_port;                // restore port register
 OW_DDR = ow_ddr;                  // restore data direction register
}

//Reset 1-wire bus
uint8_t onewire_reset(void)
{
 uint8_t answer;
 CLEARBIT(OW_PORT, OW_DQ);         // start reset pulse (low level at data line)
 SETBIT(OW_DDR, OW_DQ);
 _DELAY_US(480);
 CLEARBIT(OW_DDR, OW_DQ);          // finish reset pulse (high level at data line)

 _DELAY_US(70);                    // wait before sample (tMSP)

 answer = !(OW_PIN & _BV(OW_DQ));  // save presence flag (answer)

 _DELAY_US(410);                   // complete the reset sequence recovery (tRSTH)
 return answer;                    // 1 - present, 0 - not present
}

//Write one bit
static void onewire_write_bit(uint8_t bit)
{
 CLEARBIT(OW_PORT, OW_DQ);         // set low level at data line
 SETBIT(OW_DDR, OW_DQ);
 _DELAY_US(10);
 if(bit) CLEARBIT(OW_DDR, OW_DQ);  // write value of bit
 _DELAY_US(75);
 CLEARBIT(OW_DDR, OW_DQ);
}

//Read one bit
static uint8_t onewire_read_bit(void)
{
 CLEARBIT(OW_PORT, OW_DQ);
 SETBIT(OW_DDR, OW_DQ);
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 _NO_OPERATION();
 CLEARBIT(OW_DDR, OW_DQ);
 _DEALAY_US(10);
 return (OW_PIN & _BV(OW_DQ));     // read and return bit value
}

//Write one byte
void onewire_write_byte(uint8_t data)
{
 uint8_t i = 8;
 do
 {
  onewire_write_bit(data & 1);     //send out one bit
  data >>= 1;                      //shift to next bit
 }
 while( --i );
 _DELAY_US(120);
}

//Read one byte
uint8_t onewire_read_byte(void)
{
 uint8_t i = 0, data = 0;
 for(; i < 8; ++i)
 {
  if(onewire_read_bit())
   data |= _BV(i);                 // read out one bit
  _DELAY_US(120);
 }

 return data;
}
