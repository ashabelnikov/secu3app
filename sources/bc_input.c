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

/** \file bc_input.c
 * \author Alexey A. Shabelnikov
 * Implementation of CE errors information output using blink codes
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bitmask.h"
#include "eeprom.h"
#include "ce_errors.h"
#include "ioconfig.h"
#include "knock.h"   //for knock_read_expander() and knock_write_expander()
#include "starter.h"
#include "tables.h"  //for IOCFG_
#include "ventilator.h"
#include "wdt.h"

/**Declare blink codes. Indexes in the array must correspond to numbers of bits of corresponding CE errors in ce_errors.h */
PGM_DECLARE(uint8_t blink_codes[16]) =
 {0x21, 0x13, 0x14, 0x31, 0x32, 0x22, 0x23, 0x24, 0x41, 0x25, 0x26, 0x27, 0x28, 0x51, 0x52, 0};


void bc_init_ports(void)
{
 IOCFG_INIT(IOP_BC_INPUT, 0); //don't use internal pull up resistor
}

/**Delay in hundreds of milliseconds
 * \param hom value in hundreds of milliseconds
 */
void delay_hom(uint8_t hom)
{
 do
 {
  uint8_t i = 10;
  do {
   _DELAY_US(10000);       //10ms
   wdt_reset_timer();
  }while(--i);
 }while(--hom);
}

void blink(void)
{
 ce_set_state(CE_STATE_ON);
#if !defined(SECU3T)
 knock_write_expander();
#endif
 delay_hom(2);
 ce_set_state(CE_STATE_OFF);
#if !defined(SECU3T)
 knock_write_expander();
#endif
 delay_hom(2);
}

/** Displays single 2 digit blink code
 * \param bc blink code 0...99 (in Hex!)
 */
void disp_code(uint8_t bc)
{
 uint8_t i = 0;

 //Hi. digit
 i = bc >> 4;
 do
 {
  blink();
 }while(--i);

 delay_hom(10);

 //Lo. digit
 i = bc & 0xF;
 do
 {
  blink();
 }while(--i);
}

/** Displays start marker (4 long flashes) */
void disp_start(void)
{
 uint8_t i = 4;
 do
 {
  ce_set_state(CE_STATE_ON);
#if !defined(SECU3T)
  knock_write_expander();
#endif
  delay_hom(8);
  ce_set_state(CE_STATE_OFF);
#if !defined(SECU3T)
  knock_write_expander();
#endif
  delay_hom(3);
 }while(--i);
}

void ckps_init_ports(void);
#ifdef FUEL_INJECT
void inject_init_ports(void);
#endif

void bc_indication_mode(void)
{
 uint8_t i = 5;
 if (!IOCFG_CHECK(IOP_BC_INPUT))
  return; //normal program execution

 _DISABLE_INTERRUPT();

 //Check 5 times
 do
 {
#if !defined(SECU3T)
  knock_read_expander();
#endif
  if (IOCFG_GET(IOP_BC_INPUT))
   return; //normal program execution
 }while(--i);

 //We are entered to the blink codes indication mode
 ckps_init_ports();
#ifdef FUEL_INJECT
 inject_init_ports();
#endif
 ce_set_state(CE_STATE_OFF);

 vent_turnoff();                 //turn off ventilator
 starter_set_blocking_state(1);  //block starter
 IOCFG_INIT(IOP_FL_PUMP, 0);     //turn off fuel pump
 IOCFG_INIT(IOP_IE, 0);          //turn off IE valve solenoid
 IOCFG_INIT(IOP_FE, 0);          //turn off power valve solenoid
 IOCFG_SETF(IOP_FL_PUMP, 0);     //turn off fuel pump

#if !defined(SECU3T)
 knock_write_expander();
#endif

 wdt_reset_timer();

 //delay 2 sec.
 delay_hom(20);

 //main loop
 for(;;)
 {
  uint16_t errors = 0;
  disp_start();

  delay_hom(7);

  //read errors
  eeprom_read(&errors, EEPROM_ECUERRORS_START, sizeof(uint16_t));

  for(i = 0; i < 16; ++i)
  {
   if (0 == PGM_GET_BYTE(&blink_codes[i]))
    continue;
   if (errors & (1 << i))
   {
    disp_code(PGM_GET_BYTE(&blink_codes[i]));
    delay_hom(20);
   }
  }

  delay_hom(20);
  wdt_reset_timer();
 }
}
