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
/**Function pointer type used for getting value from I/O*/
typedef uint8_t (*iocfg_pfn_get)(void);


//List all I/O plugs
#define IOP_IGN_OUT1      0     //!< IGN_OUT1        (output)
#define IOP_IGN_OUT2      1     //!< IGN_OUT2        (output)
#define IOP_IGN_OUT3      2     //!< IGN_OUT3        (output)
#define IOP_IGN_OUT4      3     //!< IGN_OUT4        (output)
#define IOP_ADD_IO1       4     //!< ADD_IO1         (output)  (applicable only in SECU-3T)
#define IOP_ADD_IO2       5     //!< ADD_IO2         (output)  (applicable only in SECU-3T)
#define IOP_ECF           6     //!< ECF             (output)
#define IOP_ST_BLOCK      7     //!< ST_BLOCK        (output)
#define IOP_IE            8     //!< IE              (output)
#define IOP_FE            9     //!< FE              (output)
#define IOP_PS           10     //!< PS              (input)
#define IOP_ADD_I1       11     //!< ADD_IO1         (input)   (applicable only in SECU-3T)
#define IOP_ADD_I2       12     //!< ADD_IO2         (input)   (applicable only in SECU-3T)
#define IOP_CE           13     //!< CE              (output)
#define IOP_BL           14     //!< "Bootloader"    (output)
#define IOP_DE           15     //!< "Default EEPROM"(output)
#define IOP_GAS_V        16     //!< GAS_V           (input)
#define IOP_REF_S        17     //!< REF_S           (input)   (applicable only in SECU-3T)
#define IOP_RESERVED1    18     //!< reserved slot   ()
#define IOP_RESERVED2    19     //!< reserved slot   ()
#define IOP_RESERVED3    20     //!< reserved slot   ()
#define IOP_RESERVED4    21     //!< reserved slot   ()
#define IOP_RESERVED5    22     //!< reserved slot   ()
#define IOP_RESERVED6    23     //!< reserved slot   ()
#define IOP_RESERVED7    24     //!< reserved slot   ()
#define IOP_RESERVED8    25     //!< reserved slot   ()
#define IOP_RESERVED9    26     //!< reserved slot   ()
#define IOP_RESERVED10   27     //!< reserved slot   ()
#define IOP_RESERVED11   28     //!< reserved slot   ()
#define IOP_RESERVED12   29     //!< reserved slot   ()
#define IOP_RESERVED13   30     //!< reserved slot   ()
#define IOP_RESERVED14   31     //!< reserved slot   ()
#define IOP_RESERVED15   32     //!< reserved slot   ()
#define IOP_RESERVED16   33     //!< reserved slot   ()
#define IOP_RESERVED17   34     //!< reserved slot   ()
#define IOP_RESERVED18   35     //!< reserved slot   ()
#define IOP_RESERVED19   36     //!< reserved slot   ()
//Next definitions correspond to plugs only
#define IOP_FL_PUMP      37     //!< FL_PUMP         (output)
#define IOP_HALL_OUT     38     //!< HALL_OUT        (output)
#define IOP_STROBE       39     //!< STROBE          (output)
#define IOP_PWRRELAY     40     //!< PWRRELAY        (output)
#define IOP_IGN          41     //!< IGN             (input)
#define IOP_IGN_OUT7     42     //!< IGN_OUT7        (output)
#define IOP_IGN_OUT8     43     //!< IGN_OUT8        (output)
#define IOP_BC_INPUT     44     //!< BC_INPUT        (input)
#define IOP_SM_DIR       45     //!< SM_DIR          (output)
#define IOP_SM_STP       46     //!< SM_STP          (output)
#define IOP_MAPSEL0      47     //!< MAPSEL0         (input)
#define IOP_SPDSENS      48     //!< SPD_SENS        (input)
#define IOP_INTK_HEAT    49     //!< INTK_HEAT       (output)
#define IOP_LAMBDA       50     //!< LAMBDA          (input)
#define IOP_AIR_TEMP     51     //!< AIR_TEMP        (input)
#define IOP_UNI_OUT0     52     //!< UNI_OUT0        (output)
#define IOP_UNI_OUT1     53     //!< UNI_OUT1        (output)
#define IOP_UNI_OUT2     54     //!< UNI_OUT2        (output)
#define IOP_INJ_OUT0     55     //!< INJ_OUT0        (output)
#define IOP_INJ_OUT1     56     //!< INJ_OUT1        (output)
#define IOP_INJ_OUT2     57     //!< INJ_OUT2        (output)
#define IOP_INJ_OUT3     58     //!< INJ_OUT3        (output)
#define IOP_RESERVED20   59     //!< reserved plug   ()
#define IOP_RESERVED21   60     //!< reserved plug   ()
#define IOP_RESERVED22   61     //!< reserved plug   ()
#define IOP_RESERVED23   62     //!< reserved plug   ()
#define IOP_RESERVED24   63     //!< reserved plug   ()
#define IOP_RESERVED25   64     //!< reserved plug   ()
#define IOP_RESERVED26   65     //!< reserved plug   ()
#define IOP_RESERVED27   66     //!< reserved plug   ()
#define IOP_RESERVED28   67     //!< reserved plug   ()

/**Wrap macro from port/pgmspace.h. for getting function pointers from program memory */
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

/**Get value of specified I/O. Applicable only for plugs which are inputs.
 * io_id - ID of I/O to be set to specified value
 */
#define IOCFG_GET(io_id) ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))()

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
void iocfg_i_ign_out1(uint8_t value);    //!< init IGN_OUT1
void iocfg_i_ign_out1i(uint8_t value);   //!< init IGN_OUT1           (inverted)
void iocfg_s_ign_out1(uint8_t value);    //!< set  IGN_OUT1
void iocfg_s_ign_out1i(uint8_t value);   //!< set  IGN_OUT1           (inverted)
void iocfg_i_ign_out2(uint8_t value);    //!< init IGN_OUT2
void iocfg_i_ign_out2i(uint8_t value);   //!< init IGN_OUT2           (inverted)
void iocfg_s_ign_out2(uint8_t value);    //!< set  IGN_OUT2
void iocfg_s_ign_out2i(uint8_t value);   //!< set  IGN_OUT2           (inverted)
void iocfg_i_ign_out3(uint8_t value);    //!< init IGN_OUT3
void iocfg_i_ign_out3i(uint8_t value);   //!< init IGN_OUT3           (inverted)
void iocfg_s_ign_out3(uint8_t value);    //!< set  IGN_OUT3
void iocfg_s_ign_out3i(uint8_t value);   //!< set  IGN_OUT3           (inverted)
void iocfg_i_ign_out4(uint8_t value);    //!< init IGN_OUT4
void iocfg_i_ign_out4i(uint8_t value);   //!< init IGN_OUT4           (inverted)
void iocfg_s_ign_out4(uint8_t value);    //!< set  IGN_OUT4
void iocfg_s_ign_out4i(uint8_t value);   //!< set  IGN_OUT4           (inverted)
#ifdef SECU3T
void iocfg_i_add_io1(uint8_t value);     //!< init ADD_IO1 output  (applicable only in SECU-3T)
void iocfg_i_add_io1i(uint8_t value);    //!< init ADD_IO1 output  (applicable only in SECU-3T) (inverted)
void iocfg_s_add_io1(uint8_t value);     //!< set  ADD_IO1 output  (applicable only in SECU-3T)
void iocfg_s_add_io1i(uint8_t value);    //!< set  ADD_IO1 output  (applicable only in SECU-3T) (inverted)
void iocfg_i_add_io2(uint8_t value);     //!< init ADD_IO2 output  (applicable only in SECU-3T)
void iocfg_i_add_io2i(uint8_t value);    //!< init ADD_IO2 output  (applicable only in SECU-3T) (inverted)
void iocfg_s_add_io2(uint8_t value);     //!< set  ADD_IO2 output  (applicable only in SECU-3T)
void iocfg_s_add_io2i(uint8_t value);    //!< set  ADD_IO2 output  (applicable only in SECU-3T) (inverted)
#endif
void iocfg_i_ecf(uint8_t value);         //!< init ECF
void iocfg_i_ecfi(uint8_t value);        //!< init ECF                (inverted)
void iocfg_s_ecf(uint8_t value);         //!< set  ECF
void iocfg_s_ecfi(uint8_t value);        //!< set  ECF                (inverted)
void iocfg_i_st_block(uint8_t value);    //!< init ST_BLOCK
void iocfg_i_st_blocki(uint8_t value);   //!< init ST_BLOCK           (inverted)
void iocfg_s_st_block(uint8_t value);    //!< set  ST_BLOCK
void iocfg_s_st_blocki(uint8_t value);   //!< set  ST_BLOCK           (inverted)
void iocfg_i_ie(uint8_t value);          //!< init IE
void iocfg_i_iei(uint8_t value);         //!< init IE                 (inverted)
void iocfg_s_ie(uint8_t value);          //!< set  IE
void iocfg_s_iei(uint8_t value);         //!< set  IE                 (inverted)
void iocfg_i_fe(uint8_t value);          //!< init FE
void iocfg_i_fei(uint8_t value);         //!< init FE                 (inverted)
void iocfg_s_fe(uint8_t value);          //!< set  FE
void iocfg_s_fei(uint8_t value);         //!< set  FE                 (inverted)
void iocfg_s_stub(uint8_t);              //!< stub function for outputs

void iocfg_i_ce(uint8_t value);          //!< init CE
void iocfg_i_cei(uint8_t value);         //!< init CE                 (inverted)
void iocfg_s_ce(uint8_t value);          //!< set  CE
void iocfg_s_cei(uint8_t value);         //!< set  CE                 (inverted)
void iocfg_i_bl(uint8_t value);          //!< init BL
void iocfg_i_bli(uint8_t value);         //!< init BL                 (inverted)
void iocfg_s_bl(uint8_t value);          //!< set  BL
void iocfg_s_bli(uint8_t value);         //!< set  BL                 (inverted)
void iocfg_i_de(uint8_t value);          //!< init DE
void iocfg_i_dei(uint8_t value);         //!< init DE                 (inverted)
void iocfg_s_de(uint8_t value);          //!< set  DE
void iocfg_s_dei(uint8_t value);         //!< set  DE                 (inverted)

//Inputs
void iocfg_i_ps(uint8_t value);          //!< init PS input
void iocfg_i_psi(uint8_t value);         //!< init PS input           (inverted)
uint8_t iocfg_g_ps(void);                //!< get PS input value
uint8_t iocfg_g_psi(void);               //!< get PS input value      (inverted)
#ifdef SECU3T
void iocfg_i_add_i1(uint8_t value);      //!< init ADD_IO1 input   (applicable only in SECU-3T)
void iocfg_i_add_i1i(uint8_t value);     //!< init ADD_IO1 input   (applicable only in SECU-3T) (inverted)
uint8_t iocfg_g_add_i1(void);            //!< set  ADD_IO1 input   (applicable only in SECU-3T)
uint8_t iocfg_g_add_i1i(void);           //!< set  ADD_IO1 input   (applicable only in SECU-3T) (inverted)
void iocfg_i_add_i2(uint8_t value);      //!< init ADD_IO2 input   (applicable only in SECU-3T)
void iocfg_i_add_i2i(uint8_t value);     //!< init ADD_IO2 input   (applicable only in SECU-3T) (inverted)
uint8_t iocfg_g_add_i2(void);            //!< set  ADD_IO2 input   (applicable only in SECU-3T)
uint8_t iocfg_g_add_i2i(void);           //!< set  ADD_IO2 input   (applicable only in SECU-3T) (inverted)
void iocfg_i_ref_s(uint8_t value);       //!< init REF_S input     (applicable only in SECU-3T)
void iocfg_i_ref_si(uint8_t value);      //!< init REF_S input     (applicable only in SECU-3T) (inverted)
uint8_t iocfg_g_ref_s(void);             //!< get REF_S input      (applicable only in SECU-3T)
uint8_t iocfg_g_ref_si(void);            //!< get REF_S input      (applicable only in SECU-3T) (inverted)
#endif
void iocfg_i_gas_v(uint8_t value);       //!< init GAS_V input
void iocfg_i_gas_vi(uint8_t value);      //!< init GAS_V input        (inverted)
uint8_t iocfg_g_gas_v(void);             //!< get GAS_V input value
uint8_t iocfg_g_gas_vi(void);            //!< get GAS_V input value   (inverted)

uint8_t iocfg_g_stub(void);              //!< stub function for inputs

#endif //_IOCONFIG_H_
