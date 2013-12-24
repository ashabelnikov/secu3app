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

/** \file eeprom.c
 * Implementation of EEPROM related functions (API).
 * Functions for read/write EEPROM and related functionality
 * (Реализация Функций для для чтения/записи EEPROM и связанная с ним функциональность)
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include "bitmask.h"
#include "eeprom.h"
#include "wdt.h"

/**Describes information is necessary for storing of data into EEPROM
 * (Описывает информацию необходимую для сохранения данных в EEPROM)
 */
typedef struct
{
 uint16_t ee_addr;             //!< Address for EEPROM (адрес для записи в EEPROM)
 uint8_t* sram_addr;           //!< Address of data in RAM (адрес данных в ОЗУ)
 uint16_t count;               //!< Number of bytes (количество байтов)
 uint8_t eews;                 //!< State of writing process (состояние процесса записи)
 uint8_t opcode;               //!< code of specific operation wich cased writing process
 uint8_t completed_opcode;     //!< will be equal to opcode after finish of process
}eeprom_wr_desc_t;

/**State variables */
eeprom_wr_desc_t eewd = {0,0,0,0,0,0};

/** Initiates process of byte's writing (инициирует процесс записи байта в EEPROM) */
#define EE_START_WR_BYTE()  {EECR|= _BV(EEMWE);  EECR|= _BV(EEWE);}

uint8_t eeprom_take_completed_opcode(void)
{
 uint8_t result;
 _DISABLE_INTERRUPT();
 result = eewd.completed_opcode;
 eewd.completed_opcode = 0;
 _ENABLE_INTERRUPT();
 return result;
}

//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeaddr, void* sramaddr, uint16_t size)
{
 eewd.eews = 1;
 eewd.ee_addr = eeaddr;
 eewd.sram_addr = sramaddr;
 eewd.count = size;
 eewd.opcode = opcode;
 SETBIT(EECR, EERIE);
}

//возвращает не 0 если в текущий момент никакая операция не выполняется
uint8_t eeprom_is_idle(void)
{
 return (eewd.eews) ? 0 : 1;
}

/**Interrupt handler from EEPROM. Each time when state machine finishes we set address
 * register to zero.
 * (Обработчик прерывания от EEPROM при завершении работы автомата всегда заносим в
 * регистр адреса - адрес нулевой ячейки).
 */
ISR(EE_RDY_vect)
{
 CLEARBIT(EECR, EERIE); //запрещаем прерывание от EEPROM
 _ENABLE_INTERRUPT();
 switch(eewd.eews)
 {
  case 0:   //КА остановлен
   break;

  case 1:   //КА в процессе записи
   _DISABLE_INTERRUPT();
   EEAR = eewd.ee_addr;
   EEDR = *eewd.sram_addr;
   EE_START_WR_BYTE();
   SETBIT(EECR, EERIE);
   _ENABLE_INTERRUPT();
   ++eewd.sram_addr;
   ++eewd.ee_addr;
   if (--eewd.count==0)
    eewd.eews = 2;   //последний байт запущен на запись.
   else
    eewd.eews = 1;
   break;

  case 2:   //последний байт записан
   EEAR=0x000;      //this will help to prevent corruption of EEPROM
   eewd.eews = 0;
   eewd.completed_opcode = eewd.opcode;
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
  __EEGET(*dest,eeaddr);
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
