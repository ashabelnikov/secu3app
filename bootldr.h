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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

/** \file bootldr.h
 * Functionality and information for access boot loader
 * (Функционал и информация для доступа к загрузчику).
 */

#ifndef _BOOTLDR_H_
#define _BOOTLDR_H_

#include <stdint.h>

/**Define size of boot loader's section depending on selected platform.
 * Size of boot loader which corresponds to SECONDBOOTSTART value is used everywere.
 * (Определяем размер секции бутлоадера в зависимости от выбранной платформы.
 * Везде используется размер загрузчика соответствующий значению SECONDBOOTSTART)
 */
#ifdef __ATmega16__
 #define BOOT_LOADER_SIZE  512    
#elif __ATmega32__
 #define BOOT_LOADER_SIZE  1024   
#elif __ATmega64__
 #define BOOT_LOADER_SIZE  2048
#else
 #error "Not supported platform!"  
#endif 

/**Define start address of boot loader in the firmware (in bytes),
 * FLASHEND defined in ioavr.h
 * (определяем стартовый адрес бутлоадера в прошивке (в байтах),
 * FLASHEND определено в ioavr.h)
 */
#define SECU3BOOTSTART ((((unsigned int)FLASHEND) + 1) - BOOT_LOADER_SIZE)

/**Input point of boot loader used from programm (passing by jumper checking),
 * see source code of boot loader
 * (точка входа в бутлоадер из программы (минуя проверку перемычки),
 * смотрите исходный код загрузчика).
 */ 
#define boot_loader_start() ((void (*)())((SECU3BOOTSTART+0xA)/2))()

#endif //_BOOTLDR_H_
