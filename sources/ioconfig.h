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
 * \author Alexey A. Shabelnikov
 * I/O configuration file. Allows I/O functions remapping
 */

#ifndef _IOCONFIG_H_
#define _IOCONFIG_H_

#include <stdint.h>

/**Wrap macro from port/pgmspace.h. for getting function pointers from program memory */
#define _IOREM_GPTR(ptr) PGM_GET_WORD(ptr)

/**Init specified I/O
 * io_id - ID of I/O to be initialized
 * io_state - Initial state of I/O (On/off)
 */
#ifdef IOCFG_FUNC_INIT
void IOCFG_INIT(uint8_t io_id, uint8_t io_state);
#else
#define IOCFG_INIT(io_id, io_state) ((iocfg_pfn_init)_IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]))(io_state)
#endif


/**Set value of specified I/O
 * io_id - ID of I/O to be set to specified value
 * io_value - Value for I/O (On/Off)
 */
#define IOCFG_SET(io_id, io_value) ((iocfg_pfn_set)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))(io_value)
#ifdef IOCFG_FUNC_INIT
void IOCFG_SETF(uint8_t io_id, uint8_t io_value);
#else
#define IOCFG_SETF(io_id, io_value) IOCFG_SET(io_id, io_value)
#endif

/**Get value of specified I/O. Applicable only for plugs which are inputs.
 * io_id - ID of I/O to be set to specified value
 */
#ifdef IOCFG_FUNC_INIT
uint8_t IOCFG_GET(uint8_t io_id);
#else
#define IOCFG_GET(io_id) ((iocfg_pfn_get)_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))()
#endif

/**Checks specified I/O for availability. If specified I/O is not available it means that it is not
 * plugged into a real I/O slot.
 * returns 1 - available, 0 - not available
 */
#ifdef IOCFG_FUNC_INIT
uint8_t IOCFG_CHECK(uint8_t io_id);
#else
#define IOCFG_CHECK(io_id) (_IOREM_GPTR(&fw_data.cddata.iorem.s_stub) != _IOREM_GPTR(&fw_data.cddata.iorem.i_plugs[io_id]))
#endif

/**Get specified I/O callback address
 * io_id - ID of I/O for which a callback address will be obtained
 * returns - corresponding callback address
 */
#define IOCFG_CB(io_id) (_IOREM_GPTR(&fw_data.cddata.iorem.v_plugs[io_id]))


/**Function pointer type used for initialization of I/O */
typedef void (*iocfg_pfn_init)(uint8_t state);
/**Function pointer type used for setting value of I/O  */
typedef void (*iocfg_pfn_set)(uint8_t value);
/**Function pointer type used for getting value from I/O*/
typedef uint8_t (*iocfg_pfn_get)(void);

uint8_t iocfg_g_stub(void);              //!< stub function for inputs
void iocfg_s_stub(uint8_t);              //!< stub function for outputs

#ifdef SECU3T //---SECU-3T---

//List all I/O plugs
#define IOP_IGN_OUT1      0     //!< IGN_OUT1        (output)
#define IOP_IGN_OUT2      1     //!< IGN_OUT2        (output)
#define IOP_IGN_OUT3      2     //!< IGN_OUT3        (output)
#define IOP_IGN_OUT4      3     //!< IGN_OUT4        (output)
#define IOP_IGN_OUT5      4     //!< ADD_IO1         (output)
#define IOP_IGN_OUT6      5     //!< ADD_IO2         (output)
#define IOP_ECF           6     //!< ECF             (output)
#define IOP_ST_BLOCK      7     //!< ST_BLOCK        (output)
#define IOP_IE            8     //!< IE              (output)
#define IOP_FE            9     //!< FE              (output)
#define IOP_PS           10     //!< PS              (input)
#define IOP_ADD_I1       11     //!< ADD_IO1         (input)
#define IOP_ADD_I2       12     //!< ADD_IO2         (input)
#define IOP_CE           13     //!< CE              (output)
#define IOP_BL           14     //!< "Bootloader"    (output)
#define IOP_DE           15     //!< "Default EEPROM"(output)
#define IOP_GAS_V        16     //!< GAS_V           (input)
#define IOP_REF_S        17     //!< REF_S           (input)
#define IOP_CKPS         18     //!< CKPS            (input)
#define IOP_RESERVED0    19     //!< reserved slot   ()
#define IOP_RESERVED1    20     //!< reserved slot   ()
#define IOP_RESERVED2    21     //!< reserved slot   ()
#define IOP_RESERVED3    22     //!< reserved slot   ()
#define IOP_RESERVED4    23     //!< reserved slot   ()
#define IOP_RESERVED5    24     //!< reserved slot   ()
#define IOP_RESERVED6    25     //!< reserved slot   ()
#define IOP_RESERVED7    26     //!< reserved slot   ()
#define IOP_RESERVED8    27     //!< reserved slot   ()
#define IOP_RESERVED9    28     //!< reserved slot   ()
#define IOP_RESERVED10   29     //!< reserved slot   ()
#define IOP_RESERVED11   30     //!< reserved slot   ()
#define IOP_RESERVED12   31     //!< reserved slot   ()
#define IOP_RESERVED13   32     //!< reserved slot   ()
#define IOP_RESERVED14   33     //!< reserved slot   ()
#define IOP_RESERVED15   34     //!< reserved slot   ()
#define IOP_RESERVED16   35     //!< reserved slot   ()
#define IOP_RESERVED17   36     //!< reserved slot   ()
#define IOP_RESERVED18   37     //!< reserved slot   ()
#define IOP_RESERVED19   38     //!< reserved slot   ()
#define IOP_RESERVED20   39     //!< reserved slot   ()
#define IOP_RESERVED21   40     //!< reserved slot   ()
#define IOP_RESERVED22   41     //!< reserved slot   ()
#define IOP_RESERVED23   42     //!< reserved slot   ()
#define IOP_RESERVED24   43     //!< reserved slot   ()
#define IOP_RESERVED25   44     //!< reserved slot   ()
#define IOP_RESERVED26   45     //!< reserved slot   ()
#define IOP_RESERVED27   46     //!< reserved slot   ()
#define IOP_RESERVED28   47     //!< reserved slot   ()
#define IOP_RESERVED29   48     //!< reserved slot   ()
//Next definitions correspond to plugs only
#define IOP_FL_PUMP      49     //!< FL_PUMP         (output)
#define IOP_HALL_OUT     50     //!< HALL_OUT        (output)
#define IOP_STROBE       51     //!< STROBE          (output)
#define IOP_PWRRELAY     52     //!< PWRRELAY        (output)
#define IOP_IGN          53     //!< IGN             (input)
#define IOP_IGN_OUT7     54     //!< IGN_OUT7        (output)
#define IOP_IGN_OUT8     55     //!< IGN_OUT8        (output)
#define IOP_BC_INPUT     56     //!< BC_INPUT        (input)
#define IOP_SM_DIR       57     //!< SM_DIR          (output)
#define IOP_SM_STP       58     //!< SM_STP          (output)
#define IOP_MAPSEL0      59     //!< MAPSEL0         (input)
#define IOP_SPDSENS      60     //!< SPD_SENS        (input)
#define IOP_INTK_HEAT    61     //!< INTK_HEAT       (output)
#define IOP_LAMBDA       62     //!< LAMBDA          (input)
#define IOP_AIR_TEMP     63     //!< AIR_TEMP        (input)
#define IOP_UNI_OUT0     64     //!< UNI_OUT0        (output)
#define IOP_UNI_OUT1     65     //!< UNI_OUT1        (output)
#define IOP_UNI_OUT2     66     //!< UNI_OUT2        (output)
#define IOP_INJ_OUT1     67     //!< INJ_OUT1        (output)
#define IOP_INJ_OUT2     68     //!< INJ_OUT2        (output)
#define IOP_INJ_OUT3     69     //!< INJ_OUT3        (output)
#define IOP_INJ_OUT4     70     //!< INJ_OUT4        (output)
#define IOP_IAC_PWM      71     //!< IAC_PWM         (output)
#define IOP_GD_DIR       72     //!< GD_DIR          (output)
#define IOP_GD_STP       73     //!< GD_STP          (output)
#define IOP_GD_PWM       74     //!< PWM gas valve   (output)
#define IOP_INJ_OUT5     75     //!< INJ_OUT5        (output)
#define IOP_INJ_OUT6     76     //!< INJ_OUT6        (output)
#define IOP_COND_I       77     //!< COND_I          (input)
#define IOP_O2SH_O       78     //!< IOP_O2SH_O      (output)
#define IOP_RESERVED32   79     //!< reserved plug   ()
#define IOP_RESERVED33   80     //!< reserved plug   ()
#define IOP_RESERVED34   81     //!< reserved plug   ()
#define IOP_RESERVED35   82     //!< reserved plug   ()
#define IOP_RESERVED36   83     //!< reserved plug   ()
#define IOP_RESERVED37   84     //!< reserved plug   ()
#define IOP_RESERVED38   85     //!< reserved plug   ()
#define IOP_RESERVED39   86     //!< reserved plug   ()
#define IOP_RESERVED40   87     //!< reserved plug   ()
#define IOP_RESERVED41   88     //!< reserved plug   ()
#define IOP_RESERVED42   89     //!< reserved plug   ()
#define IOP_RESERVED43   90     //!< reserved plug   ()
#define IOP_RESERVED44   91     //!< reserved plug   ()

#define IOP_IGNPLG_OFF   (IOP_IGN_OUT7-(IOP_IGN_OUT6+1)) //!< needed by ckps.c
#define IOP_INJPLG_OFF   (IOP_INJ_OUT5-(IOP_INJ_OUT4+1)) //!< needed by injector.c

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
void iocfg_i_add_o1(uint8_t value);      //!< init ADD_O1 output
void iocfg_i_add_o1i(uint8_t value);     //!< init ADD_O1 output      (inverted)
void iocfg_s_add_o1(uint8_t value);      //!< set  ADD_O1 output
void iocfg_s_add_o1i(uint8_t value);     //!< set  ADD_O1 output      (inverted)
void iocfg_i_add_o2(uint8_t value);      //!< init ADD_O2 output
void iocfg_i_add_o2i(uint8_t value);     //!< init ADD_O2 output      (inverted)
void iocfg_s_add_o2(uint8_t value);      //!< set  ADD_O2 output
void iocfg_s_add_o2i(uint8_t value);     //!< set  ADD_O2 output      (inverted)
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
void iocfg_i_add_i1(uint8_t value);      //!< init ADD_I1 input
void iocfg_i_add_i1i(uint8_t value);     //!< init ADD_I1 input       (inverted)
uint8_t iocfg_g_add_i1(void);            //!< set  ADD_I1 input
uint8_t iocfg_g_add_i1i(void);           //!< set  ADD_I1 input       (inverted)
void iocfg_i_add_i2(uint8_t value);      //!< init ADD_I2 input
void iocfg_i_add_i2i(uint8_t value);     //!< init ADD_I2 input       (inverted)
uint8_t iocfg_g_add_i2(void);            //!< set  ADD_I2 input
uint8_t iocfg_g_add_i2i(void);           //!< set  ADD_I2 input       (inverted)
void iocfg_i_ref_s(uint8_t value);       //!< init REF_S input
void iocfg_i_ref_si(uint8_t value);      //!< init REF_S input        (inverted)
uint8_t iocfg_g_ref_s(void);             //!< get REF_S input
uint8_t iocfg_g_ref_si(void);            //!< get REF_S input         (inverted)
void iocfg_i_gas_v(uint8_t value);       //!< init GAS_V input
void iocfg_i_gas_vi(uint8_t value);      //!< init GAS_V input        (inverted)
uint8_t iocfg_g_gas_v(void);             //!< get GAS_V input value
uint8_t iocfg_g_gas_vi(void);            //!< get GAS_V input value   (inverted)
void iocfg_i_ckps(uint8_t value);        //!< init CKPS input
void iocfg_i_ckpsi(uint8_t value);       //!< init CKPS input         (inverted)
uint8_t iocfg_g_ckps(void);              //!< get CKPS input
uint8_t iocfg_g_ckpsi(void);             //!< get CKPS input          (inverted)


#else //---SECU-3i---

//List all I/O plugs
#define IOP_IGN_OUT1      0     //!< IGN_O1          (output) *
#define IOP_IGN_OUT2      1     //!< IGN_O2          (output) *
#define IOP_IGN_OUT3      2     //!< IGN_O3          (output) *
#define IOP_IGN_OUT4      3     //!< IGN_O4          (output) *
#define IOP_IGN_OUT5      4     //!< IGN_O5          (output) *
#define IOP_ECF           5     //!< ECF             (output) *
#define IOP_INJ_OUT1      6     //!< INJ_O1          (output)
#define IOP_INJ_OUT2      7     //!< INJ_O2          (output)
#define IOP_INJ_OUT3      8     //!< INJ_O3          (output)
#define IOP_INJ_OUT4      9     //!< INJ_O4          (output)
#define IOP_INJ_OUT5     10     //!< INJ_O5          (output)
#define IOP_BL           11     //!< BL (SM_AO1)     (output) *
#define IOP_DE           12     //!< DE (SM_AO2)     (output) *
#define IOP_ST_BLOCK     13     //!< STBL_O          (output)   spi
#define IOP_CE           14     //!< CEL_O           (output)   spi
#define IOP_FL_PUMP      15     //!< FPMP_O          (output)   spi
#define IOP_PWRRELAY     16     //!< PWRR_O          (output)   spi
#define IOP_EVAP_O       17     //!< EVAP_O          (output)   spi
#define IOP_O2SH_O       18     //!< O2SH_O          (output)   spi
#define IOP_COND_O       19     //!< COND_O          (output)   spi
#define IOP_ADD_O2       20     //!< ADD_O2          (output)   spi
//inputs
#define IOP_PS           21     //!< PS              (input) *
#define IOP_REF_S        22     //!< REF_S           (input) *
#define IOP_CKPS         23     //!< CKPS            (input) *
#define IOP_ADD_I1       24     //!< ADD_I1          (input) *
#define IOP_ADD_I2       25     //!< ADD_I2          (input) *
#define IOP_ADD_I3       26     //!< ADD_I3          (input)
#define IOP_GAS_V        27     //!< GAS_V           (input)    spi
#define IOP_IGN          28     //!< IGN_I           (input)    spi
#define IOP_COND_I       29     //!< COND_I          (input)    spi
#define IOP_EPAS_I       30     //!< EPAS_I          (input)    spi
#define IOP_ADD_I4       31     //!< ADD_I4          (input)
#define IOP_TACH_O       32     //!< TACH_O          (output)
#define IOP_KSP_CS       33     //!< KSP_CS          (output)
#define IOP_ADD_I5       34     //!< ADD_I5          (input)    spiadc
#define IOP_OILP_I       35     //!< OILP_I          (input)    spi
#define IOP_GENS_I       36     //!< GENS_I          (input)    spi
//reserved slots
#define IOP_RESERVED0    37     //!< reserved slot   ()
#define IOP_RESERVED1    38     //!< reserved slot   ()
#define IOP_RESERVED2    39     //!< reserved slot   ()
#define IOP_RESERVED3    40     //!< reserved slot   ()
#define IOP_RESERVED4    41     //!< reserved slot   ()
#define IOP_RESERVED5    42     //!< reserved slot   ()
#define IOP_RESERVED6    43     //!< reserved slot   ()
#define IOP_RESERVED7    44     //!< reserved slot   ()
#define IOP_RESERVED8    45     //!< reserved slot   ()
#define IOP_RESERVED9    46     //!< reserved slot   ()
#define IOP_RESERVED10   47     //!< reserved slot   ()
#define IOP_RESERVED11   48     //!< reserved slot   ()
//Next definitions correspond to plugs only
#define IOP_IGN_OUT6     49     //!< IGN_O6          (output)
#define IOP_IGN_OUT7     50     //!< IGN_O7          (output)
#define IOP_IGN_OUT8     51     //!< IGN_O8          (output)
#define IOP_SM_DIR       52     //!< SM_DIR          (output)
#define IOP_SM_STP       53     //!< SM_STP          (output)
#define IOP_GD_DIR       54     //!< GD_DIR          (output)
#define IOP_GD_STP       55     //!< GD_STP          (output)
#define IOP_GD_PWM       56     //!< PWM gas valve   (output)
#define IOP_IAC_PWM      57     //!< IAC_PWM         (output)
#define IOP_HALL_OUT     58     //!< HALL_OUT        (output)
#define IOP_STROBE       59     //!< STROBE          (output)
#define IOP_INTK_HEAT    60     //!< INTK_HEAT       (output)
#define IOP_UNI_OUT0     61     //!< UNI_OUT0        (output)
#define IOP_UNI_OUT1     62     //!< UNI_OUT1        (output)
#define IOP_UNI_OUT2     63     //!< UNI_OUT2        (output)
#define IOP_IE           64     //!< IE              (output)
#define IOP_FE           65     //!< FE              (output)
#define IOP_BC_INPUT     66     //!< BC_INPUT        (input)
#define IOP_MAPSEL0      67     //!< MAPSEL0         (input)
#define IOP_SPDSENS      68     //!< SPD_SENS        (input)
#define IOP_LAMBDA       69     //!< LAMBDA          (input)
#define IOP_AIR_TEMP     70     //!< AIR_TEMP        (input)
#define IOP_CAN_CS       71     //!< CS for MCP2515  (output)
#define IOP_INJ_OUT6     72     //!< INJ_OUT6        (output)
#define IOP_INJ_OUT7     73     //!< INJ_OUT7        (output)
#define IOP_INJ_OUT8     74     //!< INJ_OUT8        (output)
#define IOP_IGNTIM       75     //!< IGNTIM          (input)
#define IOP_MAP2         76     //!< MAP2            (input)
#define IOP_TMP2         77     //!< TMP2            (input)
#define IOP_RESERVED12   78     //!< reserved plug   ()
#define IOP_RESERVED13   79     //!< reserved plug   ()
#define IOP_RESERVED14   80     //!< reserved plug   ()
#define IOP_RESERVED15   81     //!< reserved plug   ()
#define IOP_RESERVED16   82     //!< reserved plug   ()
#define IOP_RESERVED17   83     //!< reserved plug   ()
#define IOP_RESERVED18   84     //!< reserved plug   ()
#define IOP_RESERVED19   85     //!< reserved plug   ()
#define IOP_RESERVED20   86     //!< reserved plug   ()
#define IOP_RESERVED21   87     //!< reserved plug   ()
#define IOP_RESERVED22   88     //!< reserved plug   ()
#define IOP_RESERVED23   89     //!< reserved plug   ()
#define IOP_RESERVED24   90     //!< reserved plug   ()
#define IOP_RESERVED25   91     //!< reserved plug   ()

#define IOP_IGNPLG_OFF   (IOP_IGN_OUT6-(IOP_IGN_OUT5+1)) //!< needed by ckps.c
#define IOP_INJPLG_OFF   (IOP_INJ_OUT6-(IOP_INJ_OUT5+1)) //!< needed by injector.c

//List all I/O functions. These functions must be used only inside tables.c
void iocfg_i_ign_out1(uint8_t value);    //!< init IGN_O1
void iocfg_i_ign_out1i(uint8_t value);   //!< init IGN_O1           (inverted)
void iocfg_s_ign_out1(uint8_t value);    //!< set  IGN_O1
void iocfg_s_ign_out1i(uint8_t value);   //!< set  IGN_O1           (inverted)

void iocfg_i_ign_out2(uint8_t value);    //!< init IGN_O2
void iocfg_i_ign_out2i(uint8_t value);   //!< init IGN_O2           (inverted)
void iocfg_s_ign_out2(uint8_t value);    //!< set  IGN_O2
void iocfg_s_ign_out2i(uint8_t value);   //!< set  IGN_O2           (inverted)

void iocfg_i_ign_out3(uint8_t value);    //!< init IGN_O3
void iocfg_i_ign_out3i(uint8_t value);   //!< init IGN_O3           (inverted)
void iocfg_s_ign_out3(uint8_t value);    //!< set  IGN_O3
void iocfg_s_ign_out3i(uint8_t value);   //!< set  IGN_O3           (inverted)

void iocfg_i_ign_out4(uint8_t value);    //!< init IGN_O4
void iocfg_i_ign_out4i(uint8_t value);   //!< init IGN_O4           (inverted)
void iocfg_s_ign_out4(uint8_t value);    //!< set  IGN_O4
void iocfg_s_ign_out4i(uint8_t value);   //!< set  IGN_O4           (inverted)

void iocfg_i_ign_out5(uint8_t value);    //!< init IGN_O5
void iocfg_i_ign_out5i(uint8_t value);   //!< init IGN_O5           (inverted)
void iocfg_s_ign_out5(uint8_t value);    //!< set  IGN_O5
void iocfg_s_ign_out5i(uint8_t value);   //!< set  IGN_O5           (inverted)

void iocfg_i_ecf(uint8_t value);         //!< init ECF
void iocfg_i_ecfi(uint8_t value);        //!< init ECF              (inverted)
void iocfg_s_ecf(uint8_t value);         //!< set  ECF
void iocfg_s_ecfi(uint8_t value);        //!< set  ECF              (inverted)

void iocfg_i_inj_out1(uint8_t value);    //!< init INJ_O1
void iocfg_i_inj_out1i(uint8_t value);   //!< init INJ_O1           (inverted)
void iocfg_s_inj_out1(uint8_t value);    //!< set  INJ_O1
void iocfg_s_inj_out1i(uint8_t value);   //!< set  INJ_O1           (inverted)

void iocfg_i_inj_out2(uint8_t value);    //!< init INJ_O2
void iocfg_i_inj_out2i(uint8_t value);   //!< init INJ_O2           (inverted)
void iocfg_s_inj_out2(uint8_t value);    //!< set  INJ_O2
void iocfg_s_inj_out2i(uint8_t value);   //!< set  INJ_O2           (inverted)

void iocfg_i_inj_out3(uint8_t value);    //!< init INJ_O3
void iocfg_i_inj_out3i(uint8_t value);   //!< init INJ_O3           (inverted)
void iocfg_s_inj_out3(uint8_t value);    //!< set  INJ_O3
void iocfg_s_inj_out3i(uint8_t value);   //!< set  INJ_O3           (inverted)

void iocfg_i_inj_out4(uint8_t value);    //!< init INJ_O4
void iocfg_i_inj_out4i(uint8_t value);   //!< init INJ_O4           (inverted)
void iocfg_s_inj_out4(uint8_t value);    //!< set  INJ_O4
void iocfg_s_inj_out4i(uint8_t value);   //!< set  INJ_O4           (inverted)

void iocfg_i_inj_out5(uint8_t value);    //!< init INJ_O5
void iocfg_i_inj_out5i(uint8_t value);   //!< init INJ_O5           (inverted)
void iocfg_s_inj_out5(uint8_t value);    //!< set  INJ_O5
void iocfg_s_inj_out5i(uint8_t value);   //!< set  INJ_O5           (inverted)

void iocfg_i_bl(uint8_t value);          //!< init BL
void iocfg_i_bli(uint8_t value);         //!< init BL               (inverted)
void iocfg_s_bl(uint8_t value);          //!< set  BL
void iocfg_s_bli(uint8_t value);         //!< set  BL               (inverted)

void iocfg_i_de(uint8_t value);          //!< init DE
void iocfg_i_dei(uint8_t value);         //!< init DE               (inverted)
void iocfg_s_de(uint8_t value);          //!< set  DE
void iocfg_s_dei(uint8_t value);         //!< set  DE               (inverted)

void iocfg_i_st_block(uint8_t value);    //!< init STBL_O
void iocfg_i_st_blocki(uint8_t value);   //!< init STBL_O           (inverted)
void iocfg_s_st_block(uint8_t value);    //!< set  STBL_O
void iocfg_s_st_blocki(uint8_t value);   //!< set  STBL_O           (inverted)

void iocfg_i_ce(uint8_t value);          //!< init CEL_O
void iocfg_i_cei(uint8_t value);         //!< init CEL_O            (inverted)
void iocfg_s_ce(uint8_t value);          //!< set  CEL_O
void iocfg_s_cei(uint8_t value);         //!< set  CEL_O            (inverted)

void iocfg_i_fpmp_o(uint8_t value);      //!< init FPMP_O
void iocfg_i_fpmp_oi(uint8_t value);     //!< init FPMP_O           (inverted)
void iocfg_s_fpmp_o(uint8_t value);      //!< set  FPMP_O
void iocfg_s_fpmp_oi(uint8_t value);     //!< set  FPMP_O           (inverted)

void iocfg_i_pwrr_o(uint8_t value);      //!< init PWRR_O
void iocfg_i_pwrr_oi(uint8_t value);     //!< init PWRR_O           (inverted)
void iocfg_s_pwrr_o(uint8_t value);      //!< set  PWRR_O
void iocfg_s_pwrr_oi(uint8_t value);     //!< set  PWRR_O           (inverted)

void iocfg_i_evap_o(uint8_t value);      //!< init EVAP_O
void iocfg_i_evap_oi(uint8_t value);     //!< init EVAP_O           (inverted)
void iocfg_s_evap_o(uint8_t value);      //!< set  EVAP_O
void iocfg_s_evap_oi(uint8_t value);     //!< set  EVAP_O           (inverted)

void iocfg_i_o2sh_o(uint8_t value);      //!< init O2SH_O
void iocfg_i_o2sh_oi(uint8_t value);     //!< init O2SH_O           (inverted)
void iocfg_s_o2sh_o(uint8_t value);      //!< set  O2SH_O
void iocfg_s_o2sh_oi(uint8_t value);     //!< set  O2SH_O           (inverted)

void iocfg_i_cond_o(uint8_t value);      //!< init COND_O
void iocfg_i_cond_oi(uint8_t value);     //!< init COND_O           (inverted)
void iocfg_s_cond_o(uint8_t value);      //!< set  COND_O
void iocfg_s_cond_oi(uint8_t value);     //!< set  COND_O           (inverted)

void iocfg_i_add_o2(uint8_t value);      //!< init ADD_O2
void iocfg_i_add_o2i(uint8_t value);     //!< init ADD_O2           (inverted)
void iocfg_s_add_o2(uint8_t value);      //!< set  ADD_O2
void iocfg_s_add_o2i(uint8_t value);     //!< set  ADD_O2           (inverted)

void iocfg_i_tach_o(uint8_t value);      //!< init TACH_O
void iocfg_i_tach_oi(uint8_t value);     //!< init TACH_O           (inverted)
void iocfg_s_tach_o(uint8_t value);      //!< set  TACH_O
void iocfg_s_tach_oi(uint8_t value);     //!< set  TACH_O           (inverted)

void iocfg_i_ksp_cs(uint8_t value);      //!< init KSP_CS
void iocfg_i_ksp_csi(uint8_t value);     //!< init KSP_CS           (inverted)
void iocfg_s_ksp_cs(uint8_t value);      //!< set  KSP_CS
void iocfg_s_ksp_csi(uint8_t value);     //!< set  KSP_CS           (inverted)

//Inputs
void iocfg_i_ps(uint8_t value);          //!< init PS input
void iocfg_i_psi(uint8_t value);         //!< init PS input           (inverted)
uint8_t iocfg_g_ps(void);                //!< get PS input value
uint8_t iocfg_g_psi(void);               //!< get PS input value      (inverted)

void iocfg_i_ref_s(uint8_t value);       //!< init REF_S input
void iocfg_i_ref_si(uint8_t value);      //!< init REF_S input        (inverted)
uint8_t iocfg_g_ref_s(void);             //!< get REF_S input
uint8_t iocfg_g_ref_si(void);            //!< get REF_S input         (inverted)

void iocfg_i_ckps(uint8_t value);        //!< init CKPS input
void iocfg_i_ckpsi(uint8_t value);       //!< init CKPS input         (inverted)
uint8_t iocfg_g_ckps(void);              //!< get CKPS input
uint8_t iocfg_g_ckpsi(void);             //!< get CKPS input          (inverted)

void iocfg_i_add_i1(uint8_t value);      //!< init ADD_I1 input
void iocfg_i_add_i1i(uint8_t value);     //!< init ADD_I1 input       (inverted)
uint8_t iocfg_g_add_i1(void);            //!< set  ADD_I1 input
uint8_t iocfg_g_add_i1i(void);           //!< set  ADD_I1 input       (inverted)

void iocfg_i_add_i2(uint8_t value);      //!< init ADD_I2 input
void iocfg_i_add_i2i(uint8_t value);     //!< init ADD_I2 input       (inverted)
uint8_t iocfg_g_add_i2(void);            //!< set  ADD_I2 input
uint8_t iocfg_g_add_i2i(void);           //!< set  ADD_I2 input       (inverted)

void iocfg_i_add_i3(uint8_t value);      //!< init ADD_I3 input
void iocfg_i_add_i3i(uint8_t value);     //!< init ADD_I3 input       (inverted)
uint8_t iocfg_g_add_i3(void);            //!< set  ADD_I3 input
uint8_t iocfg_g_add_i3i(void);           //!< set  ADD_I3 input       (inverted)

void iocfg_i_add_i4(uint8_t value);      //!< init ADD_I4 input
void iocfg_i_add_i4i(uint8_t value);     //!< init ADD_I4 input       (inverted)
uint8_t iocfg_g_add_i4(void);            //!< set  ADD_I4 input
uint8_t iocfg_g_add_i4i(void);           //!< set  ADD_I4 input       (inverted)

void iocfg_i_gas_v(uint8_t value);       //!< init GAS_V input
void iocfg_i_gas_vi(uint8_t value);      //!< init GAS_V input        (inverted)
uint8_t iocfg_g_gas_v(void);             //!< get GAS_V input value
uint8_t iocfg_g_gas_vi(void);            //!< get GAS_V input value   (inverted)

void iocfg_i_ign(uint8_t value);         //!< init IGN_I input
void iocfg_i_igni(uint8_t value);        //!< init IGN_I input        (inverted)
uint8_t iocfg_g_ign(void);               //!< get IGN_I input value
uint8_t iocfg_g_igni(void);              //!< get IGN_I input value   (inverted)

void iocfg_i_cond_i(uint8_t value);      //!< init IGN_I input
void iocfg_i_cond_ii(uint8_t value);     //!< init IGN_I input        (inverted)
uint8_t iocfg_g_cond_i(void);            //!< get IGN_I input value
uint8_t iocfg_g_cond_ii(void);           //!< get IGN_I input value   (inverted)

void iocfg_i_epas_i(uint8_t value);      //!< init EPAS_I input
void iocfg_i_epas_ii(uint8_t value);     //!< init EPAS_I input       (inverted)
uint8_t iocfg_g_epas_i(void);            //!< get EPAS_I input value
uint8_t iocfg_g_epas_ii(void);           //!< get EPAS_I input value  (inverted)

//OILP_I
void iocfg_i_oilp_i(uint8_t value);      //!< init OILP_I input
void iocfg_i_oilp_ii(uint8_t value);     //!< init OILP_I input        (inverted)
uint8_t iocfg_g_oilp_i(void);            //!< get OILP_I input value
uint8_t iocfg_g_oilp_ii(void);           //!< get OILP_I input value   (inverted)
//GENS_I
void iocfg_i_gens_i(uint8_t value);      //!< init GENS_I input
void iocfg_i_gens_ii(uint8_t value);     //!< init GENS_I input        (inverted)
uint8_t iocfg_g_gens_i(void);            //!< get GENS_I input value
uint8_t iocfg_g_gens_ii(void);           //!< get GENS_I input value   (inverted)

void iocfg_i_add_i5(uint8_t value);      //!< init ADD_I5 input
void iocfg_i_add_i5i(uint8_t value);     //!< init ADD_I5 input       (inverted)
uint8_t iocfg_g_add_i5(void);            //!< set  ADD_I5 input
uint8_t iocfg_g_add_i5i(void);           //!< set  ADD_I5 input       (inverted)

#ifdef MCP3204
#define SPIADC_CHNUM 4
extern uint16_t spiadc_chan[SPIADC_CHNUM];
#endif

#endif

#endif //_IOCONFIG_H_
