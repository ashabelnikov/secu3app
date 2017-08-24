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

/** \file procuart.h
 * \author Alexey A. Shabelnikov
 * Functionality for processing of pending data which is sent/received via serial interface (UART)
 * (Обработка поступающих данных для приема/передачи через последовательный интерфейс (UART)).
 */

#ifndef _PROCUART_H_
#define _PROCUART_H_

/** Process sent/received frames from UART. Should be called from main loop!
  * Uses d ECU data structure
 */
void process_uart_interface(void);

#endif //_PROCUART_H_
