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

/** \file obd.h
 * OBD support
 */

#ifdef OBD_SUPPORT

#ifndef _OBD_H_
#define _OBD_H_

#include <stdint.h>

/** CAN message */
typedef struct can_t
{
#if SUPPORT_EXT_CANID
 uint32_t id;             //CAN ID
 struct
 {
  uint8_t rtr : 1;        //!< is Remote Transmit Request frame (11 or 29 bits)
  uint8_t extended : 1;   //!< is extended ID
 } flags;
#else
 uint16_t id;             //!< ID  (11 bits)
 struct
 {
  uint8_t rtr : 1;        //!< is Remote Transmit Request frame
 } flags;
#endif
 uint8_t length;          //!< length of data bytes
 uint8_t data[8];         //!< data bytes

#if SUPPORT_TIMESTAMPS
 uint16_t timestamp;
#endif
} can_t;

/***/
void obd_init(void);
/**
 * Uses d ECU data structure
 */
void obd_process(void);

#endif //_OBD_H_

#endif //OBD_SUPPORT
