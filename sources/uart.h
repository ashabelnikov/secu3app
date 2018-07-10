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

/** \file uart.h
 * \author Alexey A. Shabelnikov
 * Service for performing communication via UART.
 */

#ifndef  _UART_H_
#define  _UART_H_

#include <stdint.h>

//Here are some values for UBRR for 20.000 mHz crystal
//
//       Speed    Value(U2X=0)  Value(U2X=1)
//       2400        0x208         0x411
//       4800        0x103         0x208
//       9600        0x81          0x103
//       14400       0x56          0xAD
//       19200       0x40          0x81
//       28800       0x2A          0x56
//       38400       0x20          0x40
//       57600       0x15          0x2A
//       115200      0x0A          0x15

#define  CBR_2400                0x0411 //!<  2400 baud
#define  CBR_4800                0x0208 //!<  4800 baud
#define  CBR_9600                0x0103 //!<  9600 baud
#define  CBR_14400               0x00AD //!< 14400 baud
#define  CBR_19200               0x0081 //!< 19200 baud
#define  CBR_28800               0x0056 //!< 28800 baud
#define  CBR_38400               0x0040 //!< 38400 baud
#define  CBR_57600               0x002A //!< 57600 baud
#define  CBR_115200              0x0015 //!< 115200 baud

#define  UART_RECV_BUFF_SIZE     110    //!< Size of receiver's buffer
#define  UART_SEND_BUFF_SIZE     130    //!< Size of transmitter's buffer

// Interface of the module

/**Builds a packet depending of type of the current descriptor and launches it on the transfer.
 * Function does not check the transmitter is busy or not, it should be done before the call
 * Uses d ECU data structure
 * \param send_mode code of descriptor of packet to be send
 */
 void uart_send_packet(uint8_t send_mode);

/**This function does not check was or wasn't frame received, checking must be done before the
 * call.
 * Uses d ECU data structure
 * \return Descriptor of processed frame
 */
 uint8_t uart_recept_packet(void);

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
 * \param baud ID of baud rate (divisor's value - see datasheet)
 */
 void uart_init(uint16_t baud);

#endif //_UART_H_
