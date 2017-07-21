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
 * (Реализация выдачи информации об ошибках СЕ используя блинк коды).
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
#include "starter.h"
#include "tables.h"  //for IOCFG_
#include "ventilator.h"
#include "wdt.h"

/**Определяем блинк коды. Индекс в массиве должен соответствовать номеру бита 
 * соответствующей ошибки в ф. ce_errors.h */
PGM_DECLARE(uint8_t blink_codes[16]) =
 {0x21, 0x13, 0x14, 0x31, 0x32, 0x22, 0x23, 0x24, 0x41, 0x25, 0x26, 0x27, 0x28, 0, 0, 0};

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
 delay_hom(2);
 ce_set_state(CE_STATE_OFF);
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
  delay_hom(8);
  ce_set_state(CE_STATE_OFF);
  delay_hom(3);
 }while(--i);
}

void bc_indication_mode(struct ecudata_t *d)
{
 uint8_t i = 5;
 if (!IOCFG_CHECK(IOP_BC_INPUT))
  return; //normal program execution

 //Check 5 times
 do
 {
  if (IOCFG_GET(IOP_BC_INPUT))
   return; //normal program execution
 }while(--i);

 //We are entered to the blink codes indication mode
 _DISABLE_INTERRUPT();
 ce_set_state(CE_STATE_OFF);

 vent_turnoff(d);                //turn off ventilator
 starter_set_blocking_state(1);  //block starter
 IOCFG_INIT(IOP_FL_PUMP, 0);     //turn off fuel pump
 IOCFG_INIT(IOP_IE, 0);          //turn off IE valve solenoid
 IOCFG_INIT(IOP_FE, 0);          //turn off power valve solenoid

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
