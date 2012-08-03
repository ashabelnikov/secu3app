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

/** \file ioconfig.h
 * I/O configuration file. Allows I/O functions remapping
 * (Файл конфигурации линий ввода/вывода).
 */

#ifndef _IOCONFIG_H_
#define _IOCONFIG_H_

#include <stdint.h>

/**Function pointer type used for initialization of I/O */
typedef void (*iocfg_pfn_init)(uint8_t state);
/**Function pointer type used for setting value of I/O  */
typedef void (*iocfg_pfn_set)(uint8_t value);

//List all I/O plugs
#define IOP_ECF           0     //!< ECF
#define IOP_ST_BLOCK      1     //!< ST_BLOCK
#define IOP_IGN_OUT3      2     //!< IGN_OUT3
#define IOP_IGN_OUT4      3     //!< IGN_OUT4
#define IOP_ADD_IO1       4     //!< ADD_IO1     (applicable only in SECU-3T)
#define IOP_ADD_IO2       5     //!< ADD_IO2     (applicable only in SECU-3T)
#define IOP_IE            6     //!< IE
#define IOP_FE            7     //!< FE
#define IOP_FL_PUMP       8     //!< FL_PUMP
#define IOP_HALL_OUT      9     //!< HALL_OUT
#define IOP_STROBE        10    //!< STROBE

//Wrap macro from port/pgmspace.h.
#define _IOREM_GPTR(ptr) PGM_GET_WORD(ptr)

/**Init specified I/O
 * io_id - ID of I/O to be initialized
 * io_state - Initial state of I/O (On/off)
 */
#define IOCFG_INIT(io_id, io_state) ((iocfg_pfn_init)_IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]))(io_state)

/**Set value of specified I/O
 * io_id - ID of I/O to be set to specified value
 * io_value - Value for I/O (On/Off)
 */
#define IOCFG_SET(io_id, io_value) ((iocfg_pfn_set)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(io_value)

/**Checks specified I/O for availability. If specified I/O is not available it means that it is not
 * plugged into a real I/O slot.
 * returns 1 - available, 0 - not available
 */
#define IOCFG_CHECK(io_id) (_IOREM_GPTR(&fw_data.cddata.iorem.s_stub) != _IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]))

/**Get specified I/O callback address
 * io_id - ID of I/O to be set to specified value
 * returns - corresponding callback address
 */
#define IOCFG_CB(io_id) (_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))

//List all I/O functions. These functions must be used only inside tables.c
void iocfg_i_ecf(uint8_t value);         //!< init ECF
void iocfg_s_ecf(uint8_t value);         //!< set  ECF
void iocfg_i_st_block(uint8_t value);    //!< init ST_BLOCK
void iocfg_s_st_block(uint8_t value);    //!< set  ST_BLOCK
void iocfg_i_ign_out3(uint8_t value);    //!< init IGN_OUT3
void iocfg_s_ign_out3(uint8_t value);    //!< set  IGN_OUT3
void iocfg_i_ign_out4(uint8_t value);    //!< init IGN_OUT4
void iocfg_s_ign_out4(uint8_t value);    //!< set  IGN_OUT4
#ifdef SECU3T
void iocfg_i_add_io1(uint8_t value);     //!< init ADD_IO1    (applicable only in SECU-3T)
void iocfg_s_add_io1(uint8_t value);     //!< set  ADD_IO1    (applicable only in SECU-3T)
void iocfg_i_add_io2(uint8_t value);     //!< init ADD_IO2    (applicable only in SECU-3T)
void iocfg_s_add_io2(uint8_t value);     //!< set  ADD_IO2    (applicable only in SECU-3T)
#endif
void iocfg_i_ie(uint8_t value);          //!< init IE
void iocfg_s_ie(uint8_t value);          //!< set  IE
void iocfg_i_fe(uint8_t value);          //!< init FE
void iocfg_s_fe(uint8_t value);          //!< set  FE
void iocfg_s_stub(uint8_t);              //!< stub function
//Additional I/O functions which are not for remapping
void iocfg_i_ign_out1(uint8_t value);    //!< init IGN_OUT1
void iocfg_s_ign_out1(uint8_t value);    //!< set  IGN_OUT1
void iocfg_i_ign_out2(uint8_t value);    //!< init IGN_OUT2
void iocfg_s_ign_out2(uint8_t value);    //!< set  IGN_OUT2

#endif //_IOCONFIG_H_
