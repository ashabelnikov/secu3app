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

#include <ioavr.h>
#include <inavr.h>
#include "eeprom.h"
#include "bitmask.h"

/**Describes information is necessary for storing of data into EEPROM
 * (Описывает информацию необходимую для сохранения данных в EEPROM)
 */
typedef struct 
{
 uint16_t ee_addr;             //!< Address for EEPROM (адрес для записи в EEPROM)
 uint8_t* sram_addr;           //!< Address of data in RAM (адрес данных в ОЗУ) 
 uint8_t count;                //!< Number of bytes (количество байтов)
 uint8_t eews;                 //!< State of writing process (состояние процесса записи)
 uint8_t opcode;               //!< code of specific operation wich cased writing process
 uint8_t completed_opcode;     //!< will be equal to opcode after finish of process
}eeprom_wr_desc_t;

/**State variables */
eeprom_wr_desc_t eewd = {0,0,0,0,0,0};

/** Initiates process of byte's writing (инициирует процесс записи байта в EEPROM) */
#define EE_START_WR_BYTE()  {EECR|= (1<<EEMWE);  EECR|= (1<<EEWE);}     

uint8_t eeprom_take_completed_opcode(void)  
{
 uint8_t result;
 __disable_interrupt();
 result = eewd.completed_opcode;
 eewd.completed_opcode = 0; 
 __enable_interrupt();
 return result;
}

//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeprom_addr, uint8_t* sram_addr, uint8_t count)  
{
 eewd.eews = 1;
 eewd.ee_addr = eeprom_addr;
 eewd.sram_addr = sram_addr;
 eewd.count = count;
 eewd.opcode = opcode;
 SETBIT(EECR,EERIE);
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
#pragma vector=EE_RDY_vect
__interrupt void ee_ready_isr(void)
{ 
 switch(eewd.eews)
 {
  case 0:   //КА остановлен
   break;

  case 1:   //КА в процессе записи
   EEAR = eewd.ee_addr;       
   EEDR = *eewd.sram_addr;
   EE_START_WR_BYTE();                          
   eewd.sram_addr++;
   eewd.ee_addr++;
   if (--eewd.count==0)
    eewd.eews = 2;   //последний байт запущен на запись.
   else      
    eewd.eews = 1;   
   break;    

  case 2:   //последний байт записан
   EEAR=0x000;      
   CLEARBIT(EECR,EERIE); //запрещаем прерывание от EEPROM        
   eewd.eews = 0;
   eewd.completed_opcode = eewd.opcode;
   break;      
 }//switch  
}

void eeprom_read(void* sram_dest, int16_t eeaddr, uint16_t size)
{
 uint8_t _t;
 uint8_t *dest = (uint8_t*)sram_dest;  
 do
 {
  _t=__save_interrupt();
  __disable_interrupt();
  __EEGET(*dest,eeaddr);
  __restore_interrupt(_t);

  eeaddr++;
  dest++;
 }while(--size); 

 EEAR=0x000;      
}

void eeprom_write(const void* sram_src, int16_t eeaddr, uint16_t size)
{
 uint8_t _t;
 uint8_t *src = (uint8_t*)sram_src;  
 do
 {
  _t=__save_interrupt();
  __disable_interrupt();
  __EEPUT(eeaddr, *src);
  __restore_interrupt(_t);

  eeaddr++;
  src++;
 }while(--size); 

 EEAR=0x000;      
}
