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

/** \file uart.h
 * Service for performing communication via UART.
 * (Поддержка обмена данными через UART).
 */

#ifndef  _UART_H_
#define  _UART_H_

#include <stdint.h>

//Here are some values for UBRR for 16.000 mHz crystal
//
//       Speed    Value(U2X=0)  Value(U2X=1)
//       2400        0x1A0         0x340
//       4800        0xCF          0x1A0
//       9600        0x67          0xCF
//       14400       0x44          0x8A
//       19200       0x33          0x67
//       28800       0x22          0x44
//       38400       0x19          0x33
//       57600       0x10          0x22

#define  CBR_2400                0x0340 //!<  2400 baud
#define  CBR_4800                0x01A0 //!<  4800 baud
#define  CBR_9600                0x00CF //!<  9600 baud
#define  CBR_14400               0x008A //!< 14400 baud
#define  CBR_19200               0x0067 //!< 19200 baud
#define  CBR_28800               0x0044 //!< 28800 baud 
#define  CBR_38400               0x0033 //!< 38400 baud
#define  CBR_57600               0x0022 //!< 57600 baud

#define  UART_RECV_BUFF_SIZE     64 //!< Size of receiver's buffer
#define  UART_SEND_BUFF_SIZE     64 //!< Size of transmitter's buffer

// Interface of the module (интерфейс модуля)

 struct ecudata_t;

/**Builds a packet depending of type of the current descriptor and launches it on the transfer.
 * Function does not check the transmitter is busy or not, it should be done before the call
 * (Cтроит пакет в зависимости от текущего дескриптора и запускает его на передачу. Функция не
 * проверяет занят передатчик или нет, это должно быть сделано до вызова функции).
 * \param d pointer to ECU data structure
 * \param send_mode code of descriptor of packet to be send
 */
 void uart_send_packet(struct ecudata_t* d, uint8_t send_mode);

/**This function does not check was or wasn't frame received, checking must be done before the
 * call (Эта функция не проверяет, был или не был принят фрейм, проверка должна быть произведена
 * до вызова функции).
 * \param d pointer to ECU data structure
 * \return Descriptor of processed frame (Возвращает дескриптор обработанного фрейма)
 */
 uint8_t uart_recept_packet(struct ecudata_t* d);

/**Call this function to tell service that you already accepted frame (reset busy state) */
 void uart_notify_processed(void);

/**\return 1 if sender is busy, otherwise - 0 */
 uint8_t uart_is_sender_busy(void);

/**This function checks for received frame
 * \return 1 if unprocessed frame is pending
 */
 uint8_t uart_is_packet_received(void);

/** \return code of current descriptor (type of frame) */
 uint8_t uart_get_send_mode(void);

/**Sets current type of frame
 * \param descriptor code of descriptor of packet
 * \return code of passed(set) descriptor
 */
 uint8_t uart_set_send_mode(uint8_t descriptor);

/**Initialization of module
 * \param baud code of baud rate (divisor's value - see datasheet)
 */
 void uart_init(uint16_t baud);

#endif //_UART_H_
