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

/** \file eeprom.c
 * \author Alexey A. Shabelnikov
 * Implementation of EEPROM related functions (API).
 * Functions for read/write EEPROM and related functionality
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "eeprom.h"
#include "wdt.h"

/**Describes information is necessary for storing of data into EEPROM
 */
typedef struct
{
 uint16_t ee_addr;             //!< Address for EEPROM
 uint8_t* sram_addr;           //!< Address of data in RAM
 uint16_t count;               //!< Number of bytes
 uint8_t eews;                 //!< State of writing process
 uint8_t opcode;               //!< code of specific operation which caused writing process
 uint8_t completed_opcode;     //!< will be equal to opcode after finish of process
}eeprom_wr_desc_t;

/**State variables */
eeprom_wr_desc_t eewd = {0,0,0,0,0,0};

/** Initiates process of byte's writing */
#define EE_START_WR_BYTE()  {EECR|= _BV(EEMPE);  EECR|= _BV(EEPE);}

uint8_t eeprom_take_completed_opcode(void)
{
 uint8_t result;
 _DISABLE_INTERRUPT();
 result = eewd.completed_opcode;
 eewd.completed_opcode = 0;
 _ENABLE_INTERRUPT();
 return result;
}

//starts the process of writing specified block of data to EEPROM
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeaddr, void* sramaddr, uint16_t size)
{
 eewd.eews = 1;
 eewd.ee_addr = eeaddr;
 eewd.sram_addr = sramaddr;
 eewd.count = size;
 eewd.opcode = opcode;
 SETBIT(EECR, EERIE);
}

//returns 0 - EEPROM is busy, 1 - EEPEOM is idle (not busy)
uint8_t eeprom_is_idle(void)
{
 return (eewd.eews) ? 0 : 1;
}

uint8_t eeprom_get_pending_opcode(void)
{
 return eewd.opcode;
}

/**Interrupt handler from EEPROM. Each time when state machine finishes we set address
 * register to zero.
 */
ISR(EE_RDY_vect)
{
 CLEARBIT(EECR, EERIE); //disable interrupt from EEPROM
 _ENABLE_INTERRUPT();
 switch(eewd.eews)
 {
  case 0:   //state machine is stopped
   break;

  case 1:   //state machine is in process of writing
   _DISABLE_INTERRUPT();
   EEAR = eewd.ee_addr;
   EEDR = *eewd.sram_addr;
   EE_START_WR_BYTE();
   SETBIT(EECR, EERIE);
   _ENABLE_INTERRUPT();
   ++eewd.sram_addr;
   ++eewd.ee_addr;
   if (--eewd.count==0)
    eewd.eews = 2;   //started to write last byte
   else
    eewd.eews = 1;
   break;

  case 2:   //last byte wrote
   EEAR=0x000;      //this will help to prevent corruption of EEPROM
   eewd.eews = 0;
   eewd.completed_opcode = eewd.opcode;
   eewd.opcode = 0;
   break;
 }//switch
}

void eeprom_read(void* sram_dest, uint16_t eeaddr, uint16_t size)
{
 uint8_t _t;
 uint8_t *dest = (uint8_t*)sram_dest;
 do
 {
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  __EEGET(*dest, eeaddr);
  _RESTORE_INTERRUPT(_t);

  eeaddr++;
  dest++;
 }while(--size);

 EEAR=0x000; //this will help to prevent corruption of EEPROM
}

void eeprom_write(const void* sram_src, uint16_t eeaddr, uint16_t size)
{
 uint8_t _t;
 uint8_t *src = (uint8_t*)sram_src;
 do
 {
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  __EEPUT(eeaddr, *src);
  _RESTORE_INTERRUPT(_t);

  wdt_reset_timer();

  eeaddr++;
  src++;
 }while(--size);

 EEAR=0x000; //this will help to prevent corruption of EEPROM
}

void eeprom_write_P(void _PGM *pgm_src, uint16_t eeaddr, uint16_t size)
{
 uint8_t _t;
 uint8_t _PGM *src = (uint8_t _PGM*)pgm_src;
 do
 {
  uint8_t byte = PGM_GET_BYTE(src);
  _t=_SAVE_INTERRUPT();
  _DISABLE_INTERRUPT();
  __EEPUT(eeaddr, byte);
  _RESTORE_INTERRUPT(_t);

  wdt_reset_timer();

  eeaddr++;
  src++;
 }while(--size);

 EEAR=0x000; //this will help to prevent corruption of EEPROM
}
