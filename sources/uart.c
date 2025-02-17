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

/** \file uart.c
 * \author Alexey A. Shabelnikov
 * Implementation of service for performing communication via UART.
 */

#include "port/avrio.h"
#include "port/interrupt.h"
#include "port/intrinsic.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include <string.h>
#include <stddef.h>
#include "bitmask.h"
#include "dbgvar.h"
#include "etc.h"
#include "ecudata.h"
#include "eeprom.h"
#include "ioconfig.h"
#include "uart.h"
#include "ufcodes.h"
#include "wdt.h"
#include "magnitude.h"

#define ETMT_NAME_STR    0     //!< name of tables's set id
//ignition maps
#define ETMT_STRT_MAP    1     //!< start map id
#define ETMT_IDLE_MAP    2     //!< idle map id
#define ETMT_WORK_MAP    3     //!< work map id
#define ETMT_TEMP_MAP    4     //!< temp.corr. map id
#define ETMT_TEMPI_MAP   5     //!< temp.corr. map id (idling)
//fuel injection maps
#define ETMT_VE_MAP      6     //!< VE
#define ETMT_AFR_MAP     7     //!< AFR
#define ETMT_CRNK_MAP    8     //!< Cranking PW
#define ETMT_WRMP_MAP    9     //!< Warmup enrichment
#define ETMT_DEAD_MAP    10     //!< Injector's dead time
#define ETMT_IDLR_MAP    11    //!< IAC/PWM position on run
#define ETMT_IDLC_MAP    12    //!< IAC_PWM position on cranking
#define ETMT_AETPS_MAP   13    //!< AE TPS map
#define ETMT_AERPM_MAP   14    //!< AE RPM map
#define ETMT_AFTSTR_MAP  15    //!< afterstart enrichment
#define ETMT_IT_MAP      16    //!< injection timing map
#define ETMT_ITRPM_MAP   17    //!< idling RPM
#define ETMT_RIGID_MAP   18    //!< idl. regulator's rigidity map
#define ETMT_EGOCRV_MAP  19    //!< EGO curve (WBO emulation)
#define ETMT_IACC_MAP    20    //!< mixture correction vs IAC pos
#define ETMT_IACCW_MAP   21    //!< weight of misture correction vs TPS
#define ETMT_IATCLT_MAP  22    //!< IAT/CLT correction vs air flow
#define ETMT_TPSSWT_MAP  23    //!< MAP/TPS switch point
#define ETMT_GTSC_MAP    24    //!< PW correction from gas temperature
#define ETMT_GPSC_MAP    25    //!< PW correction from gas pressure
#define ETMT_ATSC_MAP    26    //!< PW correction from air temperature
#define ETMT_PWM1_MAP    27    //!< PWM duty 1
#define ETMT_PWM2_MAP    28    //!< PWM duty 2
#define ETMT_IACMAT_MAP  29    //!< IAC position's correction vs MAT
#define ETMT_VE2_MAP     30    //!< Secondary VE map
#define ETMT_TPSZON_MAP  31    //!< MAP/TPS load axis allocation
#define ETMT_CYLMULT_MAP 32    //!< Inj. multiplication
#define ETMT_CYLADD_MAP  33    //!< Inj. addition
#define ETMT_AEMAP_MAP   34    //!< AE MAP map
#define ETMT_THRASS_MAP  35    //!< Throttle assist map
#define ETMT_IVE_MAP     36    //!< Idling VE

#define ETMT_SET_END     36

//separate maps
#define ETMT_ATTENUATOR    37  //!<
#define ETMT_DWELLCNTRL    38  //!<
#define ETMT_CTS_CURVE     39  //!<
#define ETMT_ATS_CURVE     40  //!<
#define ETMT_ATS_CORR      41  //!<
#define ETMT_GASDOSE       42  //!<
#define ETMT_BAROCORR      43  //!<
#define ETMT_MANIGNTIM     44  //!<
#define ETMT_TMP2_CURVE    45  //!<
#define ETMT_CRKCLT_CORR   46  //!<
#define ETMT_EH_PAUSE      47  //!<
#define ETMT_CRANKING_THRD 48  //!<
#define ETMT_CRANKING_TIME 49  //!<
#define ETMT_SMAPABAN_THRD 50  //!<
#define ETMT_KNOCK_ZONE    51  //!<
#define ETMT_GRTS_CURVE    52  //!<
#define ETMT_GRHEAT_DUTY   53  //!<
#define ETMT_PWMIAC_UCOEF  54  //!<
#define ETMT_AFTSTR_STRK0  55  //!<
#define ETMT_AFTSTR_STRK1  56  //!<
#define ETMT_GRVDELAY      57  //!<
#define ETMT_FTLS_CURVE    58  //!<
#define ETMT_EGTS_CURVE    59  //!<
#define ETMT_OPS_CURVE     60  //!<
#define ETMT_MANINJPWC     61  //!<
#define ETMT_MAF_CURVE     62  //!<
#define ETMT_FTLSCOR       63  //!<
#define ETMT_LAMBDA_ZONE   64  //!<
#define ETMT_FTS_CURVE     65  //!<
#define ETMT_FUELDENS_CORR 66  //!<
#define ETMT_XTAU_XFACC    67  //!<
#define ETMT_XTAU_XFDEC    68  //!<
#define ETMT_XTAU_TFACC    69  //!<
#define ETMT_XTAU_TFDEC    70  //!<
#define ETMT_INJNONLINP    71  //!<
#define ETMT_INJNONLING    72  //!<
#define ETMT_EGO_DELAY     73  //!<
#define ETMT_WU_AFR0       74  //!<
#define ETMT_WU_AFR1       75  //!<
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
#define ETMT_ETC_SPRPREL   76  //!<
#define ETMT_ETC_ACCEERR   77  //!<
#define ETMT_ETC_THROPOS   78  //!<
#endif

/**Define internal state variables */
typedef struct
{
 uint8_t send_mode;                     //!< current descriptor of packets beeing send
 uint8_t recv_buf[UART_RECV_BUFF_SIZE]; //!< receiver's buffer
 uint8_t send_buf[UART_SEND_BUFF_SIZE]; //!< transmitter's buffer
 uint8_t send_buf_c[UART_SEND_BUFF_SIZE];
 uint8_t recv_buf_c[UART_RECV_BUFF_SIZE];//!< contain checked data (verified by checksum)
 volatile uint8_t send_size;            //!< size of data to be send
 uint8_t send_size_c;                   //!<
 uint8_t send_index;                    //!< index in transmitter's buffer
 volatile uint8_t recv_size;            //!< size of received data
 volatile uint8_t recv_size_c;          //!< size of received data
 uint8_t recv_index;                    //!< index in receiver's buffer
 uint8_t recv_index1;                   //!< index in receiver's buffer
 volatile uint8_t recv_head;            //!< receiver's head
 uint8_t recv_tail;                     //!< receiver's tail
 uint8_t recv_index_c;                  //!< index in the recv_buf_c[]
 uint8_t packet_crc[2];                 //!< 16-bit Fletcher checksum
}uartstate_t;

/**State variables */
uartstate_t uart = {0,{0},{0},{0},{0},0,0,0,0,0,0,0,0,0,0,{0,0}};

// There are several special reserved symbols in binary mode: 0x21, 0x40, 0x0D, 0x0A
#define FIBEGIN  0x21       //!< '!' indicates beginning of the ingoing packet
#define FOBEGIN  0x40       //!< '@' indicates beginning of the outgoing packet
#define FIOEND   0x0D       //!<'\r' indicates ending of the ingoing/outgoing packet
#define FESC     0x0A       //!<'\n' Packet escape (FESC)
// Following bytes are used only in escape sequeces and may appear in the data without any problems
#define TFIBEGIN 0x81       //!< Transposed FIBEGIN
#define TFOBEGIN 0x82       //!< Transposed FOBEGIN
#define TFIOEND  0x83       //!< Transposed FIOEND
#define TFESC    0x84       //!< Transposed FESC

/**Increments specified index of receiver's buffer (FIFO)*/
#define RECV_INC_INDEX(v) {if (++(v) == UART_RECV_BUFF_SIZE) \
                            (v) = 0;}

/**Updates Fletcher checksum with 8-bit value */
#define UPD_CHKSUM(v) uart.packet_crc[0]+=(v), uart.packet_crc[1]+=uart.packet_crc[0];

/**Reset Fletcher checksum*/
#define RST_CHKSUM() uart.packet_crc[0]=0, uart.packet_crc[1]=0;

/**Get calculated checksum*/
#define GET_CHKSUM() (*((uint16_t*)(&uart.packet_crc[0])))

/** Appends transmitter's buffer
 * \param b byte which will be used to append tx buffer
 */
/*inline*/
void append_tx_buff(uint8_t b)
{
 if (b == FOBEGIN)
 {
  uart.send_buf[uart.send_size++] = FESC;
  uart.send_buf[uart.send_size++] = TFOBEGIN;
 }
 else if ((b) == FIOEND)
 {
  uart.send_buf[uart.send_size++] = FESC;
  uart.send_buf[uart.send_size++] = TFIOEND;
 }
 else if ((b) == FESC)
 {
  uart.send_buf[uart.send_size++] = FESC;
  uart.send_buf[uart.send_size++] = TFESC;
 }
 else
  uart.send_buf[uart.send_size++] = b;
}

/** Takes out byte from receiver's buffer
 * \return byte retrieved from buffer
 */
/*inline*/
uint8_t takeout_rx_buff(void)
{
 uint8_t b1 = uart.recv_buf[uart.recv_index];
 RECV_INC_INDEX(uart.recv_index);
 ++uart.recv_index1;

 if (b1 == FESC)
 {
  uint8_t b2 = uart.recv_buf[uart.recv_index];
  RECV_INC_INDEX(uart.recv_index);
  ++uart.recv_index1;

  if (b2 == TFIBEGIN)
   return FIBEGIN;
  else if (b2 == TFIOEND)
   return FIOEND;
  else if (b2 == TFESC)
   return FESC;
  return 0; //wrong code
 }
 else
  return b1;
}

PGM_DECLARE(uint8_t lzblhs_str[4]) = "3MAN";

//--------helpful functions for building of packets-------------

/**Appends sender's buffer by sequence of bytes from program memory. This function is also used in the bluetooth module
 * note! can NOT be used for binary data! */
void build_fs(uint8_t _PGM *romBuffer, uint8_t size)
{
 while(size--) {uart.send_buf_c[uart.send_size_c++] = PGM_GET_BYTE(romBuffer++);}
}

/**Appends sender's buffer by sequence of bytes from RAM. This function is also used in the bluetooth module
 * note! can NOT be used for binary data! */
void build_rs(const uint8_t* ramBuffer, uint8_t size)
{
 while(size--) {uart.send_buf_c[uart.send_size_c++] = *ramBuffer++;}
}

/**Appends sender's buffer by two HEX bytes
 * \param i 8-bit value to be converted into hex
 */
static void build_i8h(uint8_t i)
{
 uart.send_buf_c[uart.send_size_c++] = i; //1 byte
}

/**Appends sender's buffer by 4 HEX bytes
 * \param i 16-bit value to be converted into hex
 */
static void build_i16h(uint16_t i)
{
 uart.send_buf_c[uart.send_size_c++] = _AB(i,1);    //high byte
 uart.send_buf_c[uart.send_size_c++] = _AB(i,0);    //low byte
}

static void build_i24h(uint32_t i)
{
 build_i8h(i>>16);
 build_i16h(i);
}

/**Appends sender's buffer by 8 HEX bytes
 * \param i 32-bit value to be converted into hex
 */
static void build_i32h(uint32_t i)
{
 build_i16h(i>>16);
 build_i16h(i);
}

/**Appends sender's buffer by sequence of bytes from program memory buffer
 * can be used for binary data */
/*
static void build_fb(uint8_t _PGM *romBuffer, uint8_t size)
{
 while(size--) build_i8h(PGM_GET_BYTE(romBuffer++));
}
*/

/**Appends sender's buffer by sequence of bytes from RAM buffer
 * can be used for binary data */
static void build_rb(const uint8_t* ramBuffer, uint8_t size)
{
 while(size--) build_i8h(*ramBuffer++);
}

/**Appends sender's buffer by sequence of words from RAM buffer
 * can be used for binary data
 * \param ramBuffer Points to block of data in RAM
 * \param size Number of words
 */
static void build_rw(const uint16_t* ramBuffer, uint8_t size)
{
 while(size--) build_i16h(*ramBuffer++);
}

//----------Helpful functions for parsing of packets---------
/**Recepts sequence of bytes from receiver's buffer and places it into the RAM buffer
 * can NOT be used for binary data */
static void recept_rs(uint8_t* ramBuffer, uint8_t size)
{
 while(size-- && uart.recv_index_c < uart.recv_size_c) *ramBuffer++ = uart.recv_buf_c[uart.recv_index_c++];
}

/**Retrieves from receiver's buffer 8-bit value
 * \return retrieved value
 */
static uint8_t recept_i8h(void)
{
 return uart.recv_buf_c[uart.recv_index_c++];
}

/**Retrieves from receiver's buffer 16-bit value
 * \return retrieved value
 */
static uint16_t recept_i16h(void)
{
 uint16_t i16;
 _AB(i16,1) = uart.recv_buf_c[uart.recv_index_c++]; //Hi byte
 _AB(i16,0) = uart.recv_buf_c[uart.recv_index_c++]; //Lo byte
 return i16;
}

/**Retrieves from receiver's buffer 32-bit value
 * \return retrieved value
 */
static uint32_t recept_i32h(void)
{
 uint32_t i = 0;
 i = recept_i16h();
 i = i << 16;
 i|=recept_i16h();
 return i;
}

/**Recepts sequence of bytes from receiver's buffer and places it into the RAM buffer
 * can be used for binary data */
static void recept_rb(uint8_t* ramBuffer, uint8_t size)
{
 while(size-- && uart.recv_index_c < uart.recv_size_c) *ramBuffer++ = recept_i8h();
}

/**Recepts sequence of words from receiver's buffer and places it into the RAM buffer
 * can be used for binary data
 * \param ramBuffer Pointer to block of data in RAM
 * \param size Number of words
 */
static void recept_rw(uint16_t* ramBuffer, uint8_t size)
{
 while(size-- && uart.recv_index_c < uart.recv_size_c) *ramBuffer++ = recept_i16h();
}

//--------------------------------------------------------------------

/**Makes sender to start sending */
void uart_begin_send(void)
{
 uart.send_index = 0;
 _DISABLE_INTERRUPT();
 UCSRB |= _BV(UDRIE); /* enable UDRE interrupt */
 _ENABLE_INTERRUPT();
}

void uart_send_packet(uint8_t send_mode)
{
 static uint8_t index = 0;

 uart.send_size = uart.send_size_c = 0;

 if (send_mode==0) //use current context
  send_mode = uart.send_mode;

 //common part for all packets
 uart.send_buf[uart.send_size++] = '@';
 uart.send_buf[uart.send_size++] = send_mode;

 switch(send_mode)
 {
  case TEMPER_PAR:
   build_i8h(d.param.tmp_flags);
   build_i16h(d.param.vent_on);
   build_i16h(d.param.vent_off);
   build_i16h(d.param.vent_pwmfrq);
   build_i16h(d.param.cond_pvt_on);
   build_i16h(d.param.cond_pvt_off);
   build_i16h(d.param.cond_min_rpm);
   build_i16h(d.param.vent_tmr);
   break;

  case CARBUR_PAR:
   build_i16h(d.param.ie_lot);
   build_i16h(d.param.ie_hit);
   build_i8h(d.param.carb_invers);
   build_i16h(d.param.fe_on_threshold);
   build_i16h(d.param.ie_lot_g);
   build_i16h(d.param.ie_hit_g);
   build_i8h(d.param.shutoff_delay);
   build_i16h(d.param.tps_threshold);
   build_i16h(d.param.fuelcut_map_thrd);
   build_i16h(d.param.fuelcut_cts_thrd);
   build_i16h(d.param.revlim_lot);
   build_i16h(d.param.revlim_hit);
   build_i8h(d.param.fuelcut_uni);
   build_i8h(d.param.igncut_uni);
   break;

  case IDLREG_PAR:
   build_i8h(d.param.idl_flags);   //idling flags
   build_i16h(d.param.ifac1);      //+
   build_i16h(d.param.ifac2);      //-
   build_i16h(d.param.MINEFR);
   build_i16h(d.param.idling_rpm);
   build_i16h(d.param.idlreg_min_angle);
   build_i16h(d.param.idlreg_max_angle);
   build_i16h(d.param.idlreg_turn_on_temp);
   //closed loop parameters:
   build_i8h(d.param.idl_to_run_add);
   build_i8h(d.param.rpm_on_run_add);
   build_i16h(d.param.idl_reg_p[0]);  //-
   build_i16h(d.param.idl_reg_p[1]);  //+
   build_i16h(d.param.idl_reg_i[0]);  //-
   build_i16h(d.param.idl_reg_i[1]);  //+
   build_i8h(d.param.idl_coef_thrd1);
   build_i8h(d.param.idl_coef_thrd2);
   build_i8h(d.param.idl_intrpm_lim);
   build_i16h(d.param.idl_map_value);
   build_i8h(d.param.idl_iacminpos);
   build_i8h(d.param.idl_iacmaxpos);
   build_i16h(d.param.iac_reg_db);
   build_i16h(d.param.idl_reg_d);
   build_i16h(d.param.irr_k_load);
   build_i16h(d.param.irr_k_rpm);
   break;

  case ANGLES_PAR:
   build_i16h(d.param.max_angle);
   build_i16h(d.param.min_angle);
   build_i16h(d.param.angle_corr);
   build_i16h(d.param.angle_dec_speed);
   build_i16h(d.param.angle_inc_speed);
   build_i8h(d.param.zero_adv_ang);
   build_i8h(d.param.igntim_flags);
   build_i16h(d.param.shift_igntim);
   break;

  case FUNSET_PAR:
   build_i8h(d.param.fn_gasoline);
   build_i8h(d.param.fn_gas);
   build_i16h(d.param.load_lower);
   build_i16h(d.param.load_upper);
   build_i16h(d.param.map_curve_offset);
   build_i16h(d.param.map_curve_gradient);
   build_i16h(d.param.map2_curve_offset);   //map2
   build_i16h(d.param.map2_curve_gradient); //map2
   build_i16h(d.param.tps_curve_offset);
   build_i16h(d.param.tps_curve_gradient);
   build_i8h(d.param.load_src_cfg);
   build_i8h(d.param.mapsel_uni);
   build_i8h(d.param.barocorr_type);
   build_i8h(d.param.func_flags);
   build_i8h(d.param.ve2_map_func);
   build_i8h(d.param.gas_v_uni);
   //redundant values:
   build_i8h(d.param.ckps_engine_cyl);      //used for calculations on SECU-3 Manager side
   build_i16h(d.param.inj_cyl_disp);        //used for calculations on SECU-3 Manager side
   build_i32h(d.param.mafload_const);
   build_i16h(d.sens.tps_raw);              //for TPS learning
   //
   build_i16h(d.param.gps_curve_offset);   //GPS
   build_i16h(d.param.gps_curve_gradient); //GPS
   build_i16h(d.param.fps_curve_offset);   //FPS
   build_i16h(d.param.fps_curve_gradient); //FPS
   build_i16h(d.param.apps1_curve_offset);   //APPS1
   build_i16h(d.param.apps1_curve_gradient); //APPS1
   build_i16h(d.sens.apps1_raw);            //Redundant: for APPS1 learning
   break;

  case STARTR_PAR:
   build_i16h(d.param.starter_off);
   build_i16h(d.param.smap_abandon);
   build_i16h(d.param.inj_cranktorun_time); //fuel injection
   build_i8h(d.param.inj_aftstr_strokes);   //fuel injection
   build_i16h(d.param.inj_prime_cold);      //fuel injection
   build_i16h(d.param.inj_prime_hot);       //fuel injection
   build_i8h(d.param.inj_prime_delay);      //fuel injection
   build_i16h(d.param.inj_floodclear_tps);  //fuel injection
   build_i8h(d.param.inj_aftstr_strokes1);  //fuel injection
   build_i8h(d.param.stbl_str_cnt);
   build_i8h(d.param.strt_flags);
   build_i16h(d.param.inj_cranktorun_time1);//fuel injection
   break;

  case FNNAME_DAT:
   build_i8h(TABLES_NUMBER);
#ifdef REALTIME_TABLES
   if (index < TABLES_NUMBER_PGM) //from FLASH
   {
    build_i8h(index);
    build_fs(fw_data.tables[index].name, F_NAME_SIZE);
   }
   else //from EEPROM
   {
    if (eeprom_is_idle())
    {
     build_i8h(index);
     eeprom_read(&uart.send_buf_c[uart.send_size_c], offsetof(eeprom_data_t, tables.name), F_NAME_SIZE);
     uart.send_size_c+=F_NAME_SIZE;
    }
    else //skip this item - will be transferred next time
    {
     index = TABLES_NUMBER_PGM - 1;
     build_i8h(index);
     build_fs(fw_data.tables[index].name, F_NAME_SIZE);
    }
   }
#else
   build_i8h(index);
   build_fs(fw_data.tables[index].name, F_NAME_SIZE);
#endif
   ++index;
   if (index>=(TABLES_NUMBER)) index = 0;
    break;

  case SENSOR_DAT:
   build_i16h(d.sens.rpm);               // averaged RPM
   build_i16h(d.sens.map);               // averaged MAP pressure
   build_i16h(d.sens.voltage);           // voltage (avaraged)
   build_i16h(d.sens.temperat);          // coolant temperature
   build_i16h(d.corr.curr_angle);        // advance angle
   build_i16h(d.sens.knock_k);           // knock value
   build_i16h(d.corr.knkret_aac);        // knock retard
   build_i8h(d.airflow);                 // index of the map axis curve
   //boolean values
   build_i16h(_CBV16(d.ie_valve, 0)      // IE flag
             | _CBV16(d.sens.carb, 1)    // carb. limit switch flag
             | _CBV16(d.sens.gas_raw, 2) // gas valve flag
             | _CBV16(d.fe_valve, 3)     // power valve flag
             | _CBV16 (d.ce_state, 4)    // CE flag
             | _CBV16(d.cool_fan, 5)     // cooling fan flag
             | _CBV16(d.st_block, 6)     // starter blocking flag
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
             | _CBV16(d.acceleration, 7) // acceleration enrichment flag
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
             | _CBV16(d.fc_revlim, 8)    // fuel cut rev.lim. flag
#endif
             | _CBV16(d.floodclear, 9)   // flood clear mode flag
             | _CBV16(d.sys_locked, 10)  // system locked flag (immobilizer)
             | _CBV16(d.sens.ign_i, 11)  // IGN_I flag (available also in the SECU3T)
#ifndef SECU3T
             | _CBV16(d.sens.epas_i, 13) // EPAS_I flag
#endif
             | _CBV16(d.sens.cond_i, 12) // COND_I flag (available also in the SECU3T)

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
             | _CBV16(d.aftstr_enr, 14)    // after start enrichment flag
#endif

#if defined(FUEL_INJECT)
             | _CBV16(d.iac_closed_loop, 15) // IAC closed loop flag
#endif
             );

   build_i16h(d.sens.tps);                // TPS (0...100%, x64)

   {
   uint8_t ai1sub = PGM_GET_BYTE(&fw_data.exdata.add_i1_sub);
   if (1==ai1sub)
   {
    build_i16h(d.sens.add_i1);            // averaged ADD_I1 voltage
   }
   else if (2==ai1sub)
   {
    build_i16h(d.sens.add_i2);           // send value of ADD_I2
   }
#if !defined(SECU3T)
   else if (3==ai1sub)
   {
    build_i16h(d.sens.add_i3);           // send value of ADD_I3
   }
#endif
#if defined(TPIC8101)
   else if (4==ai1sub)
   {
    build_i16h(d.sens.add_i4);           // send value of ADD_I4
   }
#endif
#if !defined(SECU3T) && defined(MCP3204)
   else if (5==ai1sub)
   {
    build_i16h(d.sens.add_i5);           // send value of ADD_I5
   }
   else if (6==ai1sub)
   {
    build_i16h(d.sens.add_i6);           // send value of ADD_I6
   }
   else if (7==ai1sub)
   {
    build_i16h(d.sens.add_i7);           // send value of ADD_I7
   }
   else
    build_i16h(d.sens.add_i8);           // send value of ADD_I8
#else
   else
    build_i16h(0);
#endif
   }

   {
   uint8_t ai2sub = PGM_GET_BYTE(&fw_data.exdata.add_i2_sub);
   if (1==ai2sub)
    build_i16h(d.sens.add_i1);            // ADD_I1 voltage
   else if (2==ai2sub)
    build_i16h(d.sens.add_i2);            // ADD_I2 voltage
#if !defined(SECU3T)
   else if (3==ai2sub)
    build_i16h(d.sens.add_i3);            //send value of ADD_I3
#endif
#if defined(TPIC8101)
   else if (4==ai2sub)
    build_i16h(d.sens.add_i4);            //send value of ADD_I4
#endif
#if !defined(SECU3T) && defined(MCP3204)
   else if (5==ai2sub)
    build_i16h(d.sens.add_i5);            //send value of ADD_I5
   else if (6==ai2sub)
    build_i16h(d.sens.add_i6);            //send value of ADD_I6
   else if (7==ai2sub)
    build_i16h(d.sens.add_i7);            //send value of ADD_I7
   else
    build_i16h(d.sens.add_i8);            //send value of ADD_I8
#else
   else
    build_i16h(0);
#endif
   }

   build_i32h(d.ecuerrors_for_transfer); // CE errors
   build_i8h(d.choke_pos);               // choke position
   build_i8h(d.gasdose_pos);             // gas dosator position
#ifdef SPEED_SENSOR
   build_i16h(d.sens.vss_speed);         // vehicle speed (2 bytes), km/h
   build_i24h(d.sens.vss_dist);          // distance (3 bytes), m
#else
   build_i16h(0);
   build_i24h(0);
#endif
#ifdef FUEL_INJECT
   build_i16h(d.inj_fff);                // instant fuel flow (frequency: 16000 pulses per 1L of burnt fuel)
#else
   build_i16h(0);
#endif
#ifdef AIRTEMP_SENS
   if (IOCFG_CHECK(IOP_AIR_TEMP))
    build_i16h(d.sens.air_temp);
   else
    build_i16h(0x7FFF);                   //<--indicates that it is not used, voltage will be shown on the dashboard
#else
   build_i16h(0);
#endif

   //corrections
   build_i16h(d.corr.strt_aalt);         // advance angle from start map
   build_i16h(d.corr.idle_aalt);         // advance angle from idle map
   build_i16h(d.corr.work_aalt);         // advance angle from work map
   build_i16h(d.corr.temp_aalt);         // advance angle from coolant temperature correction map
   build_i16h(d.corr.airt_aalt);         // advance angle from air temperature correction map
   build_i16h(d.corr.idlreg_aac);        // advance angle correction from idling RPM regulator

#ifdef PA4_INP_IGNTIM
   if ((d.corr.pa4_aac != AAV_NOTUSED) && (d.corr.octan_aac != AAV_NOTUSED))
    build_i16h(d.corr.octan_aac + d.corr.pa4_aac);// octane correction value (sum of two values: octane correction and manual ign.tim. correction)
   else if (d.corr.pa4_aac != AAV_NOTUSED)
    build_i16h(d.corr.pa4_aac);
   else if (d.corr.octan_aac != AAV_NOTUSED)
    build_i16h(d.corr.octan_aac);
   else
    build_i16h(AAV_NOTUSED);
#else
    build_i16h(d.corr.octan_aac);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   build_i16h(d.corr.lambda[0]);         // lambda correction
#else
   build_i16h(0);
#endif

#ifdef FUEL_INJECT
   build_i16h(d.inj_pw);                 // injector pulse width
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL) || (!defined(SECU3T) && defined(ELEC_THROTTLE))
   build_i16h(d.sens.tpsdot);            // TPS opening/closing speed
#else
   build_i16h(0);
#endif

   build_i16h(d.sens.map2);
#ifndef SECU3T //SECU-3i
   build_i16h(d.sens.tmp2);
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   build_i16h(d.sens.afr[0]);
#else
   build_i16h(0);
#endif

   build_i16h(d.load);
   build_i16h(d.sens.baro_press);

#ifdef FUEL_INJECT
   { //combine inj.timing (value*16) and information about inj.timing mode (begin, middle or end of pulse)
   uint16_t iit = (((uint16_t)d.corr.inj_timing) >> 1) | ((uint16_t)(d.sens.gas ? (d.param.inj_anglespec >> 4) : (d.param.inj_anglespec & 0xF))) << 14;
   build_i16h(iit);         // inj. timing
   }
#else
   build_i16h(0);
#endif

#ifdef FUEL_INJECT
   build_i8h(d.corr.rigid_arg);
#else
   build_i8h(0);
#endif

#if !defined(SECU3T) && defined(MCP3204)
   build_i16h(d.sens.grts);              //gas reducer's temperature
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   build_i16h(d.sens.rxlaf);             //RxL air flow, note: it is divided by 32
#else
   build_i16h(0);
#endif

#if !defined(SECU3T) && defined(MCP3204)
   build_i16h(d.sens.ftls);             //fuel tank level
   build_i16h(d.sens.egts);             //exhaust gas temperature
   build_i16h(d.sens.ops);              //oil pressure
#else
   build_i16h(0);
   build_i16h(0);
   build_i16h(0);
#endif

#ifdef FUEL_INJECT
   build_i8h(d.sens.inj_duty);          //injector's duty in % (value * 2)
#else
   build_i8h(0);
#endif

   build_i16h(d.sens.maf);              //Air flow (g/sec) * 64 from the MAF sensor
   build_i8h(d.vent_duty);              //PWM duty of cooling fan

#ifdef UNI_OUTPUT
   build_i8h(_CBV8(d.uniout[0], 0)      //states of universal outputs
            |_CBV8(d.uniout[1], 1)
            |_CBV8(d.uniout[2], 2)
            |_CBV8(d.uniout[3], 3)
            |_CBV8(d.uniout[4], 4)
            |_CBV8(d.uniout[5], 5));
#else
   build_i8h(0);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   build_i16h(d.sens.mapdot);            // MAPdot
#else
   build_i16h(0);
#endif

#if !defined(SECU3T) && defined(MCP3204)
   build_i16h(d.sens.fts);              //fuel temperature
#else
   build_i16h(0);
#endif

#ifdef FUEL_INJECT
   build_i24h(d.cons_fuel);             //consumed fuel, L * 1024
#else
   build_i24h(0);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL) 
   build_i16h(d.sens.afr[1]);
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   build_i16h(d.corr.lambda[1]);            // lambda correction #2
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   build_i16h(d.sens.lambda_mx);            // mix of 2 lambda sensors
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   build_i16h(d.corr.afr);                  // Current value of air to fuel ratio from AFR map
#else
   build_i16h(0);
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
   build_i16h(d.corr.tchrg);                // Corrected value of MAT
#else
   build_i16h(0);
#endif

#ifndef SECU3T //SECU-3i
   build_i16h(d.sens.gps);                  //Gas Pressure Sensor
#else
   build_i16h(0);
#endif

   build_i8h(
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
     _CBV8(d.corr.idlve, 0) |
#endif
#ifndef SECU3T
     _CBV8(d.sens.gpa4_i, 1) |             // GPA4_I input (digital)
#endif
     _CBV8(d.sens.input1, 2)               // INPUT1
   | _CBV8(d.sens.input2, 3)               // INPUT2
   | _CBV8(d.sens.auto_i, 4)               // AUTO_I
   | _CBV8(d.sens.mapsel0,5)               // MAPSEL0
#ifndef SECU3T
   | _CBV8(d.sens.refprs_i,6)              // REFPRS_I
   | _CBV8(d.sens.altrn_i,7)               // ALTRN_I
#endif
   );       //additional flags

#ifndef SECU3T
   build_i16h(d.sens.fps);                 //Fuel Pressure Sensor
#else
   build_i16h(0);
#endif
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
   build_i16h(d.sens.apps1);               //APPS1
#else
   build_i16h(0);
#endif
   break;

  case ADCCOR_PAR:
   build_i8h(d.param.adc_flags);
   build_i16h(d.param.map_adc_factor);
   build_i32h(d.param.map_adc_correction);
   build_i16h(d.param.ubat_adc_factor);
   build_i32h(d.param.ubat_adc_correction);
   build_i16h(d.param.temp_adc_factor);
   build_i32h(d.param.temp_adc_correction);
   build_i16h(d.param.tps_adc_factor);
   build_i32h(d.param.tps_adc_correction);
   build_i16h(d.param.ai1_adc_factor);
   build_i32h(d.param.ai1_adc_correction);
   build_i16h(d.param.ai2_adc_factor);
   build_i32h(d.param.ai2_adc_correction);
   build_i16h(d.param.ai3_adc_factor);
   build_i32h(d.param.ai3_adc_correction);
   build_i16h(d.param.ai4_adc_factor);
   build_i32h(d.param.ai4_adc_correction);
   build_i16h(d.param.ai5_adc_factor);
   build_i32h(d.param.ai5_adc_correction);
   build_i16h(d.param.ai6_adc_factor);
   build_i32h(d.param.ai6_adc_correction);
   build_i16h(d.param.ai7_adc_factor);
   build_i32h(d.param.ai7_adc_correction);
   build_i16h(d.param.ai8_adc_factor);
   build_i32h(d.param.ai8_adc_correction);
   break;

  case ADCRAW_DAT:
   build_i16h(d.sens.map_raw);
   build_i16h(d.sens.voltage_raw);
   build_i16h(d.sens.temperat_raw);
   build_i16h(d.sens.knock_raw);   //<-- knock signal level
   build_i16h(d.sens.tps_raw);
   build_i16h(d.sens.add_i1_raw);
   build_i16h(d.sens.add_i2_raw);
#ifndef SECU3T
   build_i16h(d.sens.add_i3_raw);
#else
   build_i16h(0);  //stub for ADD_I3
#endif
#ifdef TPIC8101
   build_i16h(d.sens.add_i4_raw);
#else
   build_i16h(0);  //stub
#endif
#if !defined(SECU3T) && defined(MCP3204)
   build_i16h(d.sens.add_i5_raw);
   build_i16h(d.sens.add_i6_raw);
   build_i16h(d.sens.add_i7_raw);
   build_i16h(d.sens.add_i8_raw);
#else
   build_i16h(0);  //stub for ADD_I5 (MCP3204)
   build_i16h(0);  //stub for ADD_I6 (MCP3204)
   build_i16h(0);  //stub for ADD_I7 (MCP3204)
   build_i16h(0);  //stub for ADD_I8 (MCP3204)
#endif
   break;

  case CKPS_PAR:
   build_i8h(d.param.ckps_cogs_btdc);
   build_i8h(d.param.ckps_ignit_cogs);
   build_i8h(d.param.ckps_engine_cyl);
   build_i8h(d.param.ckps_cogs_num);
   build_i8h(d.param.ckps_miss_num);
   build_i8h(d.param.hall_flags);
   build_i16h(d.param.hall_wnd_width);
   build_i16h(d.param.hall_degrees_btdc);
   build_i16h(d.param.ckps_mttf);
   break;

  case OP_COMP_NC:
   build_i16h(d.op_comp_code);
   break;

  case CE_ERR_CODES:
   build_i32h(d.ecuerrors_for_transfer);
   break;

  case KNOCK_PAR:
   build_i8h(d.param.knock_use_knock_channel);
   build_i8h(d.param.knock_bpf_frequency);
   build_i16h(d.param.knock_k_wnd_begin_angle);
   build_i16h(d.param.knock_k_wnd_end_angle);
   build_i8h(d.param.knock_int_time_const);

   build_i16h(d.param.knock_retard_step);
   build_i16h(d.param.knock_advance_step);
   build_i16h(d.param.knock_max_retard);
   build_i16h(d.param.knock_threshold);
   build_i8h(d.param.knock_recovery_delay);
   build_i8h(d.param.knock_selch);
   build_i16h(d.param.knkclt_thrd);
   break;

  case CE_SAVED_ERR:
   build_i32h(d.ecuerrors_saved_transfer);
   break;

  case FWINFO_DAT:
   //this check ensures that we not clibm out of buffer's bounds. 3 symbols: header and end of packet
#if ((UART_SEND_BUFF_SIZE - 3) < FW_SIGNATURE_INFO_SIZE+8)
 #error "Out of buffer!"
#endif
   build_fs(fw_data.fw_sign_info, FW_SIGNATURE_INFO_SIZE);
   build_i32h(PGM_GET_DWORD(&fw_data.cddata.config));   //<--compile-time options
   build_i8h(PGM_GET_BYTE(&fw_data.cddata.fw_version)); //<--version of the firmware
   break;

  case SIGINF_DAT:
   build_fs(fwinfo, 60);
   break;

  case MISCEL_PAR:
   build_i16h(d.param.uart_divisor);
   build_i8h(d.param.uart_period_t_ms);
   build_i8h(d.param.ign_cutoff);
   build_i16h(d.param.ign_cutoff_thrd);
   build_i16h(d.param.hop_start_ang);
   build_i16h(d.param.hop_durat_ang);
   build_i8h(d.param.flpmp_flags);   //fuel pump flags
   build_i16h(d.param.evap_afbegin);
   build_i16h(d.param.evap_afslope);
   build_i8h(d.param.fp_timeout_strt);
   build_i16h(d.param.pwmfrq[0]);
   build_i16h(d.param.pwmfrq[1]);
   build_i16h(d.param.vss_period_dist);
   break;

  case CHOKE_PAR:
   build_i16h(d.param.sm_steps);
   build_i8h(d.choke_testing);      //fake parameter (actually it is a command)
   build_i8h(0);                     //fake parameter, not used in outgoing paket
   build_i16h(d.param.choke_rpm_if);
   build_i16h(d.param.choke_corr_time[0]);
   build_i16h(d.param.choke_corr_time[1]);
   build_i8h(d.param.choke_flags); //choke flags
   build_i8h(d.param.sm_freq);
   build_i16h(d.param.inj_cranktorun_time); //fuel injection
   break;

#ifdef GD_CONTROL
  case GASDOSE_PAR:
   build_i16h(d.param.gd_steps);
   build_i8h(d.gasdose_testing);    //fake parameter (actually it is command)
   build_i8h(0);                     //fake parameter, not used in outgoing paket
   build_i8h(d.param.gd_fc_closing);
   build_i16h(d.param.gd_lambda_corr_limit_p);
   build_i16h(d.param.gd_lambda_corr_limit_m);
   build_i16h(d.param.gd_lambda_stoichval);
   build_i8h(d.param.gd_freq);
   build_i8h(d.param.gd_maxfreqinit);
   break;
#endif

  case SECUR_PAR:
   build_i8h(0);
   build_i8h(0);
   build_i8h(d.param.bt_flags);
   build_rb(d.param.ibtn_keys[0], IBTN_KEY_SIZE);  //1st iButton key
   build_rb(d.param.ibtn_keys[1], IBTN_KEY_SIZE);  //2nd iButton key
   build_i8h(d.param.bt_type);
   break;

  case UNIOUT_PAR:
  { //6 tunable outputs' parameters
   uint8_t oi = 0;
   for(; oi < UNI_OUTPUT_NUMBER; ++oi)
   {
    build_i8h(d.param.uni_output[oi].flags);
    build_i8h(d.param.uni_output[oi].condition1);
    build_i8h(d.param.uni_output[oi].condition2);
    build_i16h(d.param.uni_output[oi].on_thrd_1);
    build_i16h(d.param.uni_output[oi].off_thrd_1);
    build_i16h(d.param.uni_output[oi].on_thrd_2);
    build_i16h(d.param.uni_output[oi].off_thrd_2);
   }
   build_i8h(d.param.uniout_12lf);
   break;
  }

#ifdef FUEL_INJECT
 case INJCTR_PAR:
  build_i8h(d.param.inj_flags);
  build_i8h(d.param.inj_config[0]);
  build_i8h(d.param.inj_config[1]);
  build_i16h(d.param.inj_flow_rate[0]);
  build_i16h(d.param.inj_flow_rate[1]);
  build_i16h(d.param.inj_cyl_disp);
  build_i32h(d.param.inj_sd_igl_const[0]);
  build_i32h(d.param.inj_sd_igl_const[1]);
  build_i8h(d.param.ckps_engine_cyl);      //used for calculations on SECU-3 Manager side
  build_i16h(d.param.inj_timing[0]);
  build_i16h(d.param.inj_timing[1]);
  build_i16h(d.param.inj_timing_crk[0]);
  build_i16h(d.param.inj_timing_crk[1]);
  build_i8h(d.param.inj_anglespec);
  build_i16h(d.param.fff_const);
  build_i16h(*((uint16_t*)&d.param.inj_min_pw[0]));
  build_i32h(d.param.inj_maf_const[0]);
  build_i32h(d.param.inj_maf_const[1]);
  build_i32h(d.param.mafload_const);
  build_i16h(d.param.inj_max_pw[0]);
  build_i16h(d.param.inj_max_pw[1]);
  break;
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 case LAMBDA_PAR:
  build_i8h(d.param.inj_lambda_str_per_stp);
  build_i8h(d.param.inj_lambda_step_size_p);
  build_i8h(d.param.inj_lambda_step_size_m);
  build_i16h(d.param.inj_lambda_corr_limit_p);
  build_i16h(d.param.inj_lambda_corr_limit_m);
  build_i16h(d.param.inj_lambda_swt_point);
  build_i16h(d.param.inj_lambda_temp_thrd);
  build_i16h(d.param.inj_lambda_rpm_thrd);
  build_i8h(d.param.inj_lambda_activ_delay);
  build_i16h(d.param.inj_lambda_dead_band);
  build_i8h(d.param.inj_lambda_senstype);
  build_i8h(d.param.inj_lambda_ms_per_stp);
  build_i8h(d.param.inj_lambda_flags);
  build_i16h(d.param.gd_lambda_stoichval);
  //heating:
  build_i8h(d.param.eh_heating_time[0]);
  build_i8h(d.param.eh_heating_time[1]);
  build_i8h(d.param.eh_temper_thrd);
  build_i8h(d.param.eh_heating_act);
  build_i16h(d.param.eh_aflow_thrd);
  build_i8h(d.param.lambda_selch);
  break;
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 case ACCEL_PAR:
  build_i8h(d.param.inj_ae_tpsdot_thrd);
  build_i16h(d.param.inj_ae_coldacc_mult);
  build_i8h(d.param.inj_ae_decay_time);
  build_i8h(d.param.inj_ae_type);
  build_i8h(d.param.inj_ae_time);
  build_i8h(d.param.inj_ae_ballance);
  build_i8h(d.param.inj_ae_mapdot_thrd);
  build_i8h(d.param.inj_xtau_s_thrd);
  build_i8h(d.param.inj_xtau_f_thrd);
  build_i8h(d.param.wallwet_model);
  break;
#endif

#if defined(FUEL_INJECT)
 case LTFT_PAR:
  build_i8h(d.param.ltft_mode);
  build_i16h(d.param.ltft_learn_clt);
  build_i16h(d.param.ltft_learn_clt_up);
  build_i16h(d.param.ltft_learn_iat_up);
  build_i8h(d.param.ltft_learn_grad);
  build_i16h(d.param.ltft_learn_gpa);
  build_i16h(d.param.ltft_learn_gpd);
  build_i8h(d.param.ltft_min);
  build_i8h(d.param.ltft_max);
  build_i16h(d.param.ltft_learn_rpm[0]);
  build_i16h(d.param.ltft_learn_rpm[1]);
  build_i16h(d.param.ltft_learn_load[0]);
  build_i16h(d.param.ltft_learn_load[1]);
  build_i8h(d.param.ltft_dead_band[0]);
  build_i8h(d.param.ltft_dead_band[1]);
  break;
#endif

#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 case DBW_PAR:
  build_i16h(d.param.etc_p);
  build_i16h(d.param.etc_i);
  build_i16h(d.param.etc_d);
  build_i8h(d.param.etc_nmax_duty);
  build_i8h(d.param.etc_pmax_duty);
  build_i8h(d.param.etc_pid_period);
  build_i8h(d.param.etc_frictorq_op);
  build_i8h(d.param.etc_frictorq_cl);
  build_i8h(d.param.etc_frictorq_thrd);
  build_i16h(d.param.etc_idleadd_max);
  build_i16h(etc_get_homepos());
  break;
#endif

#ifdef REALTIME_TABLES
//Following finite state machine will transfer all table's data
  case EDITAB_PAR:
  {
   static uint8_t state = 0, wrk_index = 0;
   if (d.chmode_data!=255)
   {
    state = d.chmode_data;
   }
   else
   {
    if (state > ETMT_SET_END)
     state = 0;
   }

   build_i8h(state);  //map Id
   switch(state)
   {
    case ETMT_STRT_MAP: //start map
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.f_str, F_STR_POINTS);
     state = ETMT_IDLE_MAP;
     break;
    case ETMT_IDLE_MAP: //idle map
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.f_idl, F_IDL_POINTS);
     state = ETMT_WORK_MAP, wrk_index = 0;
     break;
    case ETMT_WORK_MAP: //work map
     build_i8h(wrk_index*F_WRK_POINTS_L);
     build_rb((uint8_t*)&d.tables_ram.f_wrk[wrk_index][0], F_WRK_POINTS_F);
     if (wrk_index >= F_WRK_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_TEMP_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_TEMP_MAP: //temper. correction.
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.f_tmp, F_TMP_POINTS);
     state = ETMT_TEMPI_MAP;
     break;
    case ETMT_TEMPI_MAP: //temper. correction. (idling)
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.f_tmp_idl, F_TMP_POINTS);
     state = ETMT_NAME_STR;
     break;
    case ETMT_NAME_STR:
     build_i8h(0); //<--not used
     build_rs(d.tables_ram.name, F_NAME_SIZE);
     state = ETMT_VE_MAP, wrk_index = 0;
     break;
    case ETMT_VE_MAP:
     build_i8h(wrk_index*INJ_VE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.tables_ram.inj_ve[wrk_index][0], (INJ_VE_POINTS_F*3)/2); //24 bytes per packet (row), INJ_VE_POINTS_L rows
     if (wrk_index >= INJ_VE_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_VE2_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_VE2_MAP:
     build_i8h(wrk_index*INJ_VE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.tables_ram.inj_ve2[wrk_index][0], (INJ_VE_POINTS_F*3)/2); //24 bytes per packet (row), INJ_VE_POINTS_L rows
     if (wrk_index >= INJ_VE_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_IVE_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_IVE_MAP:
     build_i8h(wrk_index*INJ_IVE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.tables_ram.inj_ive[wrk_index][0], (INJ_IVE_POINTS_F*3)/2); //12 bytes per packet (row), INJ_IVE_POINTS_L rows
     if (wrk_index >= INJ_IVE_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_AFR_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_AFR_MAP:
     build_i8h(wrk_index*INJ_VE_POINTS_L);
     build_rb((uint8_t*)&d.tables_ram.inj_afr[wrk_index][0], INJ_VE_POINTS_F);
     if (wrk_index >= INJ_VE_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_CRNK_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_CRNK_MAP:
     build_i8h(wrk_index*(INJ_CRANKING_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&d.tables_ram.inj_cranking[wrk_index*(INJ_CRANKING_LOOKUP_TABLE_SIZE/2)], INJ_CRANKING_LOOKUP_TABLE_SIZE/2);
     if (wrk_index >= 1)
     {
      wrk_index = 0;
      state = ETMT_WRMP_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_WRMP_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_warmup, INJ_WARMUP_LOOKUP_TABLE_SIZE);
     state = ETMT_DEAD_MAP, wrk_index = 0;
     break;
    case ETMT_DEAD_MAP:
     build_i8h(wrk_index*(INJ_DT_LOOKUP_TABLE_SIZE/4));
     build_rw((uint16_t*)&d.tables_ram.inj_dead_time[wrk_index*(INJ_DT_LOOKUP_TABLE_SIZE/4)], (INJ_DT_LOOKUP_TABLE_SIZE/4));
     if (wrk_index >= 3)
     {
      wrk_index = 0;
      state = ETMT_IDLR_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_IDLR_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_iac_run_pos, INJ_IAC_POS_TABLE_SIZE);
     state = ETMT_IDLC_MAP;
     break;
    case ETMT_IDLC_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_iac_crank_pos, INJ_IAC_POS_TABLE_SIZE);
     state = ETMT_THRASS_MAP;
     break;
    case ETMT_THRASS_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_thrass, INJ_THRASS_SIZE);
     state = ETMT_AETPS_MAP;
     break;
    case ETMT_AETPS_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_ae_tps_enr,  INJ_AE_TPS_LOOKUP_TABLE_SIZE);
     build_rb((uint8_t*)&d.tables_ram.inj_ae_tps_bins, INJ_AE_TPS_LOOKUP_TABLE_SIZE);
     state = ETMT_AERPM_MAP;
     break;
    case ETMT_AERPM_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_ae_rpm_enr,  INJ_AE_RPM_LOOKUP_TABLE_SIZE);
     build_rb((uint8_t*)&d.tables_ram.inj_ae_rpm_bins, INJ_AE_RPM_LOOKUP_TABLE_SIZE);
     state = ETMT_AFTSTR_MAP;
     break;
    case ETMT_AFTSTR_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_aftstr, INJ_AFTSTR_LOOKUP_TABLE_SIZE);
     state = ETMT_IT_MAP;
     break;
    case ETMT_IT_MAP:
     build_i8h(wrk_index*INJ_VE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.tables_ram.inj_timing[wrk_index][0], (INJ_VE_POINTS_F*3)/2); //24 bytes per packet (row), INJ_VE_POINTS_L rows
     if (wrk_index >= INJ_VE_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_ITRPM_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_ITRPM_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_target_rpm, INJ_TARGET_RPM_TABLE_SIZE);
     state = ETMT_RIGID_MAP;
     break;
    case ETMT_RIGID_MAP:
     build_i8h(0); //<-- not used, because size of table is not above 16 bytes (8 values * 2 bytes)
     build_rw((uint16_t*)&d.tables_ram.inj_idl_rigidity, INJ_IDL_RIGIDITY_SIZE);
     state = ETMT_EGOCRV_MAP;
     break;
    case ETMT_EGOCRV_MAP:
     build_i8h(wrk_index*(INJ_EGO_CURVE_SIZE/2));
     build_rw((uint16_t*)&d.tables_ram.inj_ego_curve[wrk_index*(INJ_EGO_CURVE_SIZE/2)], (wrk_index < 2) ? INJ_EGO_CURVE_SIZE/2 : 2);
     if (wrk_index >= 2)
     {
      wrk_index = 0;
      state = ETMT_IACC_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_IACC_MAP:
     build_i8h(wrk_index*INJ_IAC_CORR_SIZE);
     build_rw((uint16_t*)&d.tables_ram.inj_iac_corr[wrk_index*INJ_IAC_CORR_SIZE], (wrk_index < 1) ? INJ_IAC_CORR_SIZE : 2);
     if (wrk_index >= 1)
     {
      wrk_index = 0;
      state = ETMT_IACCW_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_IACCW_MAP:
     build_i8h(wrk_index*INJ_IAC_CORR_W_SIZE);
     build_rb((uint8_t*)&d.tables_ram.inj_iac_corr_w[wrk_index*INJ_IAC_CORR_W_SIZE], (wrk_index < 1) ? INJ_IAC_CORR_W_SIZE : 2);
     if (wrk_index >= 1)
     {
      wrk_index = 0;
      state = ETMT_IATCLT_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_IATCLT_MAP:
     build_i8h(wrk_index*INJ_IATCLT_CORR_SIZE);
     build_rw((uint16_t*)&d.tables_ram.inj_iatclt_corr[wrk_index*INJ_IATCLT_CORR_SIZE], (wrk_index < 1) ? INJ_IATCLT_CORR_SIZE : 2);
     if (wrk_index >= 1)
     {
      wrk_index = 0;
      state = ETMT_TPSSWT_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_TPSSWT_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_tpsswt, INJ_TPSSWT_SIZE);
     state = ETMT_GTSC_MAP;
     break;
    case ETMT_GTSC_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_gts_corr, INJ_GTS_CORR_SIZE);
     state = ETMT_GPSC_MAP;
     break;
    case ETMT_GPSC_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_gps_corr, INJ_GPS_CORR_SIZE+2);
     state = ETMT_ATSC_MAP;
     break;
    case ETMT_ATSC_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_ats_corr, INJ_ATS_CORR_SIZE);
     state = ETMT_IACMAT_MAP;
     break;
    case ETMT_IACMAT_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.iac_mat_corr, INJ_ATS_CORR_SIZE);
     state = ETMT_PWM1_MAP, wrk_index = 0;
     break;
    case ETMT_PWM1_MAP: //PWM1 map
     build_i8h(wrk_index*F_WRK_POINTS_L);
     build_rb((uint8_t*)&d.tables_ram.pwm_duty1[wrk_index][0], F_WRK_POINTS_F);
     if (wrk_index >= F_WRK_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_PWM2_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_PWM2_MAP: //PWM2 map
     build_i8h(wrk_index*F_WRK_POINTS_L);
     build_rb((uint8_t*)&d.tables_ram.pwm_duty2[wrk_index][0], F_WRK_POINTS_F);
     if (wrk_index >= F_WRK_POINTS_L-1 )
     {
      wrk_index = 0;
      state = ETMT_TPSZON_MAP;
     }
     else
      ++wrk_index;
     break;
    case ETMT_TPSZON_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_tpszon, INJ_TPSZON_SIZE);
     state = ETMT_CYLMULT_MAP;
     break;
    case ETMT_CYLMULT_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_cylmult, INJ_CYLADD_SIZE);
     state = ETMT_CYLADD_MAP;
     break;
    case ETMT_CYLADD_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_cyladd, INJ_CYLADD_SIZE);
     state = ETMT_AEMAP_MAP;
     break;
    case ETMT_AEMAP_MAP:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&d.tables_ram.inj_ae_map_enr,  INJ_AE_MAP_LOOKUP_TABLE_SIZE);
     build_rb((uint8_t*)&d.tables_ram.inj_ae_map_bins, INJ_AE_MAP_LOOKUP_TABLE_SIZE);
     state = ETMT_STRT_MAP;
     break;
   }
   //group of separate maps
   switch(state)
   {
    case ETMT_ATTENUATOR:
     build_i8h(wrk_index * (KC_ATTENUATOR_LOOKUP_TABLE_SIZE/8));
     build_rb(&ram_extabs.attenuator_table[wrk_index * (KC_ATTENUATOR_LOOKUP_TABLE_SIZE/8)], (KC_ATTENUATOR_LOOKUP_TABLE_SIZE/8));
     if (wrk_index < (KC_ATTENUATOR_LOOKUP_TABLE_SIZE/16) - 1)  //8 times by 16 bytes
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_DWELLCNTRL:
     build_i8h(wrk_index*(COIL_ON_TIME_LOOKUP_TABLE_SIZE/4));
     build_rw((uint16_t*)&ram_extabs.coil_on_time[wrk_index*(COIL_ON_TIME_LOOKUP_TABLE_SIZE/4)], (COIL_ON_TIME_LOOKUP_TABLE_SIZE/4));
     if (wrk_index >= 3)
      wrk_index = 0;
     else
      ++wrk_index;
     break;
    case ETMT_CTS_CURVE:
     build_i8h(wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.cts_curve[wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? THERMISTOR_LOOKUP_TABLE_SIZE/2 : 2);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_ATS_CURVE:
     build_i8h(wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.ats_curve[wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? THERMISTOR_LOOKUP_TABLE_SIZE/2 : 2);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_ATS_CORR:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.ats_corr, ATS_CORR_LOOKUP_TABLE_SIZE);
     break;
    case ETMT_GASDOSE:
     build_i8h(wrk_index*GASDOSE_POS_TPS_SIZE);
     build_rb((uint8_t*)&ram_extabs.gasdose_pos[wrk_index][0], GASDOSE_POS_RPM_SIZE);
     if (wrk_index >= GASDOSE_POS_TPS_SIZE-1 )
      wrk_index = 0;
     else
      ++wrk_index;
     break;
    case ETMT_BAROCORR:
     build_i8h(wrk_index*8);
     build_rw((uint16_t*)&ram_extabs.barocorr[wrk_index*8], (wrk_index < 1) ? 8 : 3);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_MANIGNTIM:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.pa4_igntim_corr, PA4_LOOKUP_TABLE_SIZE);
     break;
    case ETMT_TMP2_CURVE:
     build_i8h(wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.tmp2_curve[wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? THERMISTOR_LOOKUP_TABLE_SIZE/2 : 2);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_CRKCLT_CORR:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.cts_crkcorr, CTS_CRKCORR_SIZE);
     break;
    case ETMT_EH_PAUSE:
     build_i8h(wrk_index * (COIL_ON_TIME_LOOKUP_TABLE_SIZE/2));
     build_rb(&ram_extabs.eh_pause[wrk_index * COIL_ON_TIME_LOOKUP_TABLE_SIZE/2], COIL_ON_TIME_LOOKUP_TABLE_SIZE/2);
     if (wrk_index < 1)  //2 times by 16 bytes
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_CRANKING_THRD:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.cranking_thrd, CRANK_THRD_SIZE);
     break;
    case ETMT_CRANKING_TIME:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.cranking_time, CRANK_TIME_SIZE);
     break;
    case ETMT_SMAPABAN_THRD:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.smapaban_thrd, SMAPABAN_THRD_SIZE);
     break;
    case ETMT_KNOCK_ZONE:
     build_i8h(wrk_index*(KNKZONE_TPS_SIZE/4));
     build_rw((uint16_t*)&ram_extabs.knock_zones[wrk_index*(KNKZONE_TPS_SIZE/4)], (KNKZONE_TPS_SIZE/4));
     if (wrk_index < 3)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_GRTS_CURVE:
     build_i8h(wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.grts_curve[wrk_index*(THERMISTOR_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? THERMISTOR_LOOKUP_TABLE_SIZE/2 : 2);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_GRHEAT_DUTY:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.grheat_duty, F_TMP_POINTS);
     break;
    case ETMT_PWMIAC_UCOEF:
     build_i8h(wrk_index*(PWMIAC_UCOEF_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.pwmiac_ucoef[wrk_index*(PWMIAC_UCOEF_SIZE/2)], PWMIAC_UCOEF_SIZE/2);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_AFTSTR_STRK0:
     build_i8h(wrk_index*(AFTSTR_STRK_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.inj_aftstr_strk0[wrk_index*(AFTSTR_STRK_SIZE/2)], AFTSTR_STRK_SIZE/2);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_AFTSTR_STRK1:
     build_i8h(wrk_index*(AFTSTR_STRK_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.inj_aftstr_strk1[wrk_index*(AFTSTR_STRK_SIZE/2)], AFTSTR_STRK_SIZE/2);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_GRVDELAY:
     build_i8h(wrk_index*(F_TMP_POINTS/2));
     build_rw((uint16_t*)&ram_extabs.grv_delay[wrk_index*(F_TMP_POINTS/2)], F_TMP_POINTS/2);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_FTLS_CURVE:
     build_i8h(wrk_index*(FTLS_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.ftls_curve[wrk_index*(FTLS_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? FTLS_LOOKUP_TABLE_SIZE/2 : 3);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_EGTS_CURVE:
     build_i8h(wrk_index*(EGTS_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.egts_curve[wrk_index*(EGTS_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? EGTS_LOOKUP_TABLE_SIZE/2 : 3);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_OPS_CURVE:
     build_i8h(wrk_index*(OPS_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.ops_curve[wrk_index*(OPS_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? OPS_LOOKUP_TABLE_SIZE/2 : 3);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_MANINJPWC:
     build_i8h(wrk_index*(INJPWCOEF_LUT_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.injpw_coef[wrk_index*(INJPWCOEF_LUT_SIZE/2)], (wrk_index < 2) ? INJPWCOEF_LUT_SIZE/2 : 1);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_MAF_CURVE:
     build_i8h(wrk_index*(MAF_FLOW_CURVE_SIZE/8));
     build_rw((uint16_t*)&ram_extabs.maf_curve[wrk_index*(MAF_FLOW_CURVE_SIZE/8)], (wrk_index < (MAF_FLOW_CURVE_SIZE/8)) ? MAF_FLOW_CURVE_SIZE/8 : 3);
     if (wrk_index < (MAF_FLOW_CURVE_SIZE/8))
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_FTLSCOR:
     build_i8h(wrk_index*(FTLSCOR_UCOEF_SIZE/4));
     build_rw((uint16_t*)&ram_extabs.ftlscor_ucoef[wrk_index*(FTLSCOR_UCOEF_SIZE/4)], (FTLSCOR_UCOEF_SIZE/4));
     if (wrk_index < 3)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_LAMBDA_ZONE:
     build_i8h(wrk_index*(EGOZONE_LOAD_SIZE/4));
     build_rw((uint16_t*)&ram_extabs.lambda_zones[wrk_index*(EGOZONE_LOAD_SIZE/4)], (EGOZONE_LOAD_SIZE/4));
     if (wrk_index < 3)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_FTS_CURVE:
     build_i8h(wrk_index*(FTS_LOOKUP_TABLE_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.fts_curve[wrk_index*(FTS_LOOKUP_TABLE_SIZE/2)], (wrk_index < 2) ? FTS_LOOKUP_TABLE_SIZE/2 : 3);
     if (wrk_index < 2)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_FUELDENS_CORR:
     build_i8h(wrk_index*(FUELDENS_CORR_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.fueldens_corr[wrk_index*(FUELDENS_CORR_SIZE/2)], (FUELDENS_CORR_SIZE/2));
     if (wrk_index < 3)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_XTAU_XFACC:
     build_i8h(wrk_index*(XTAU_FACT_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.xtau_xfacc[wrk_index*(XTAU_FACT_SIZE/2)], (XTAU_FACT_SIZE/2));
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_XTAU_XFDEC:
     build_i8h(wrk_index*(XTAU_FACT_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.xtau_xfdec[wrk_index*(XTAU_FACT_SIZE/2)], (XTAU_FACT_SIZE/2));
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_XTAU_TFACC:
     build_i8h(wrk_index*(XTAU_FACT_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.xtau_tfacc[wrk_index*(XTAU_FACT_SIZE/2)], (XTAU_FACT_SIZE/2));
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_XTAU_TFDEC:
     build_i8h(wrk_index*(XTAU_FACT_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.xtau_tfdec[wrk_index*(XTAU_FACT_SIZE/2)], (XTAU_FACT_SIZE/2));
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_INJNONLINP:
     build_i8h(wrk_index*(INJ_NONLIN_SIZE));
     build_rw((uint16_t*)&ram_extabs.inj_nonlinp_corr[wrk_index*(INJ_NONLIN_SIZE)], INJ_NONLIN_SIZE);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_INJNONLING:
     build_i8h(wrk_index*(INJ_NONLIN_SIZE));
     build_rw((uint16_t*)&ram_extabs.inj_nonling_corr[wrk_index*(INJ_NONLIN_SIZE)], INJ_NONLIN_SIZE);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_EGO_DELAY:
     build_i8h(wrk_index*(EGO_DELAY_SIZE/2));
     build_rw((uint16_t*)&ram_extabs.inj_ego_delay[wrk_index*(EGO_DELAY_SIZE/2)], EGO_DELAY_SIZE/2);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_WU_AFR0:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.inj_wu_afr0, WU_AFR_SIZE);
     break;
    case ETMT_WU_AFR1:
     build_i8h(0); //<--not used
     build_rb((uint8_t*)&ram_extabs.inj_wu_afr1, WU_AFR_SIZE);
     break;
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
    case ETMT_ETC_SPRPREL:
     build_i8h(wrk_index*(ETC_SPRPREL_SIZE)); //note: we consider that this map is followed by bins!
     build_rw((uint16_t*)&ram_extabs.etc_sprprel_duty[wrk_index*(ETC_SPRPREL_SIZE)], ETC_SPRPREL_SIZE);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_ETC_ACCEERR:
     build_i8h(wrk_index*(ETC_ACCEPTERR_SIZE)); //note: we consider that this map is folliwed by bins!
     build_rw((uint16_t*)&ram_extabs.etc_accept_error[wrk_index*(ETC_ACCEPTERR_SIZE)], ETC_ACCEPTERR_SIZE);
     if (wrk_index < 1)
      ++wrk_index;
     else
      wrk_index = 0;
     break;
    case ETMT_ETC_THROPOS:
     build_i8h(wrk_index*ETC_POS_APPS_SIZE);
     build_rb((uint8_t*)&ram_extabs.etc_throttle_pos[wrk_index][0], ETC_POS_RPM_SIZE);
     if (wrk_index >= ETC_POS_APPS_SIZE-1 )
      wrk_index = 0;
     else
      ++wrk_index;
     break;
#endif
   }
   break;
  }

  //Transferring of RPM grid
  case RPMGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.rpm_grid_points, RPM_GRID_SIZE * sizeof(int16_t));
   break;
  //Transferring of CLT grid
  case CLTGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.clt_grid_points, CLT_GRID_SIZE * sizeof(int16_t));
   break;
  //Transferring of load grid
  case LODGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.load_grid_points, LOAD_GRID_SIZE * sizeof(int16_t));
   break;
  //Transferring idling RPM grid
  case IRPMGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.irpm_grid_points, IRPM_GRID_SIZE * sizeof(int16_t));
   break;
  //Transferring idling load grid
  case ILODGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.iload_grid_points, ILOAD_GRID_SIZE * sizeof(int16_t));
   break;
  //Transferring of VE2 load grid
  case TLODGRD_PAR:
   build_i8h(0); //<--reserved
   build_rb((uint8_t*)ram_extabs.tload_grid_points, TLOAD_GRID_SIZE * sizeof(int16_t));
   break;
#endif

  case ATTTAB_PAR:
  {
   //check that size of table is a multiple of 16
#if (KC_ATTENUATOR_LOOKUP_TABLE_SIZE % 16)
 #error "KC_ATTENUATOR_LOOKUP_TABLE_SIZE must be a number divisible by 16, if not, you have to change the code below!"
#endif
   static uint8_t tab_index = 0;
   build_i8h(tab_index * 16);
   build_rb(&ram_extabs.attenuator_table[tab_index * 16], 16);
   if (tab_index >= (KC_ATTENUATOR_LOOKUP_TABLE_SIZE / 16) - 1)
    tab_index = 0;
   else
    ++tab_index;
   break;
  }

#ifdef FUEL_INJECT
  case LTFT_DAT:
  {
   static uint8_t state = 0, ltft_index = 0;
   build_i8h(state);  //map Id
   switch(state)
   {
    case 0: //LTFT 1
     build_i8h(ltft_index*INJ_VE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.inj_ltft1[ltft_index][0], INJ_VE_POINTS_F); //16 bytes per packet (row), INJ_VE_POINTS_L rows
     if (ltft_index >= INJ_VE_POINTS_L-1 )
     {
      ltft_index = 0;
      state = 1;
     }
     else
      ++ltft_index;
     break;
    case 1: //LTFT 2
     build_i8h(ltft_index*INJ_VE_POINTS_F); //cell address
     build_rb((uint8_t*)&d.inj_ltft2[ltft_index][0], INJ_VE_POINTS_F); //16 bytes per packet (row), INJ_VE_POINTS_L rows
     if (ltft_index >= INJ_VE_POINTS_L-1 )
     {
      ltft_index = 0;
      state = 0;
     }
     else
      ++ltft_index;
     break;
   }
   break;
  }
#endif

#ifdef DEBUG_VARIABLES
  case DBGVAR_DAT:
   {
   _BEGIN_ATOMIC_BLOCK();
   uint16_t dbg_var[4] = {dbg_var1, dbg_var2, dbg_var3, dbg_var4};
   _END_ATOMIC_BLOCK();
   build_i16h(dbg_var[0]);
   build_i16h(dbg_var[1]);
   build_i16h(dbg_var[2]);
   build_i16h(dbg_var[3]);
   }
   break;
#endif
#ifdef DIAGNOSTICS
  case DIAGINP_DAT:
   build_i8h(d.diag_inp.flags);
   build_i16h(d.diag_inp.voltage);
   build_i16h(d.diag_inp.map);
   build_i16h(d.diag_inp.temp);
   build_i16h(d.diag_inp.add_i1);
   build_i16h(d.diag_inp.add_i2);
#ifndef SECU3T
   build_i16h(d.diag_inp.add_i3);
#else //SECU-3T
   build_i16h(0); //stub
#endif
#ifdef TPIC8101
   build_i16h(d.diag_inp.add_i4);
#else
   build_i16h(0); //busy by HIP9011
#endif
#if !defined(SECU3T) && defined(MCP3204)
   build_i16h(d.diag_inp.add_i5);
   build_i16h(d.diag_inp.add_i6);
   build_i16h(d.diag_inp.add_i7);
   build_i16h(d.diag_inp.add_i8);
#else
   build_i16h(0); //stubs
   build_i16h(0);
   build_i16h(0);
   build_i16h(0);
#endif
   build_i16h(d.diag_inp.carb);
   build_i16h(d.diag_inp.ks_1);
   build_i16h(d.diag_inp.ks_2);
   build_i16h(d.diag_inp.bits);
   break;
#endif
 }//switch

//checksum verification implemented only for binary mode
 //reset checksum before we will update it with data
 RST_CHKSUM();

 //calculate checksum for whole packet
 uint8_t i = 0;
 for(; i < uart.send_size_c; ++i)
  UPD_CHKSUM(uart.send_buf_c[i]);
 UPD_CHKSUM(uart.send_size_c); //update check sum with size of packet (excluding 2 bytes of check sum)

 build_i16h(GET_CHKSUM()); //append packet with 2 bytes of checksum

 //Esc. packet
 //convert packet's data (replace some bytes with esc. seq.)
 for(i = 0; i < uart.send_size_c; ++i)
  append_tx_buff(uart.send_buf_c[i]);

 //common part for all packets
 uart.send_buf[uart.send_size++] = '\r';

 //now buffer contains all of the data - start sending
 uart_begin_send();
}

// When this function finish:
// uart.recv_tail contain index of the first byte of packet (skipping '!')
// uart.recv.size contain number of raw bytes in packet (except '!' and '\r')
uint8_t uart_get_packet(void)
{
 static uint8_t state = 0;

 while(uart.recv_tail != uart.recv_head)
 {
  uint8_t chr = uart.recv_buf[uart.recv_tail];
  RECV_INC_INDEX(uart.recv_tail);
  if (0==state)        //search for a symbol indicating packet's start
  {
   if (chr == '!')     //begin of the packet?
   {
    ++state;
    uart.recv_index = uart.recv_tail; //remember begin of packet
    uart.recv_size = 0;
   }
  }
  else if (1==state)   //search for a symbol indicating the packet's end
  {
   if (chr == '\r')
   {
    --state;           //set finite state machine to the initial state
    return 1;          //packet is ready
   }
   else if (chr == '!') //reset and start again
   {
    uart.recv_index = uart.recv_tail; //remember begin of packet
    uart.recv_size = 0;
   }
   else
   {
    ++uart.recv_size;
   }
  }
 }

 return 0; //packet is not ready
}

uint8_t uart_recept_packet(void)
{
 //receiver's buffer contains packet descriptor and data
 uint8_t temp, descriptor;

 descriptor = uart.recv_buf[uart.recv_index];
 RECV_INC_INDEX(uart.recv_index);

 //convert packet's data (replace esc. seq. with original bytes)
 uart.recv_size_c = 0; uart.recv_index1 = 1;
 while(uart.recv_index1 < uart.recv_size)
  uart.recv_buf_c[uart.recv_size_c++] = takeout_rx_buff();
 uart.recv_index_c = 0;

 if (uart.recv_size_c < 3)
  return 0; //error, do nothing (just ignore damaged packet)

 if (descriptor!=LZBLHS) //disable checksum for LZBLHS packets because a lot of users already have units with boot loaders not supporting checksum
 {
  uint16_t checksum = (uart.recv_buf_c[uart.recv_size_c-2] << 8) | uart.recv_buf_c[uart.recv_size_c-1];
  uart.recv_size_c-=2; //2 bytes for check sum

  //reset checksum before it will be updated with incoming data
  RST_CHKSUM();

  //calculate checksum for whole packet (except two last bytes of checksum)
  uint8_t chkidx = 0;
  for(; chkidx < uart.recv_size_c; ++chkidx)
   UPD_CHKSUM(uart.recv_buf_c[chkidx]);
  UPD_CHKSUM(uart.recv_size_c);

  if (checksum!=GET_CHKSUM())
   return 0; //error, packet corrupted, don't accept it
 }

// TODO: implement check of uart_recv_size for each packet

 //interpret received data depending on the descriptor
 switch(descriptor)
 {
  case CHANGEMODE:
   uart_set_send_mode(recept_i8h());
   d.chmode_data = recept_i8h(); //save data
   break;

  case BOOTLOADER:
   if (recept_i8h() != 'l')
     return 0; //wrong code - don't continue
   break; //boot loader will be started in the caller function's code 

  case TEMPER_PAR:
   d.param.tmp_flags   = recept_i8h();
   d.param.vent_on   = recept_i16h();
   d.param.vent_off  = recept_i16h();
   d.param.vent_pwmfrq = recept_i16h();
   d.param.cond_pvt_on = recept_i16h();
   d.param.cond_pvt_off = recept_i16h();
   d.param.cond_min_rpm = recept_i16h();
   d.param.vent_tmr = recept_i16h();
   break;

  case CARBUR_PAR:
   d.param.ie_lot  = recept_i16h();
   d.param.ie_hit  = recept_i16h();
   d.param.carb_invers= recept_i8h();
   d.param.fe_on_threshold= recept_i16h();
   d.param.ie_lot_g = recept_i16h();
   d.param.ie_hit_g = recept_i16h();
   d.param.shutoff_delay = recept_i8h();
   d.param.tps_threshold = recept_i16h();
   d.param.fuelcut_map_thrd = recept_i16h();
   d.param.fuelcut_cts_thrd = recept_i16h();
   d.param.revlim_lot = recept_i16h();
   d.param.revlim_hit = recept_i16h();
   d.param.fuelcut_uni = recept_i8h();
   d.param.igncut_uni = recept_i8h();
   break;

  case IDLREG_PAR:
   d.param.idl_flags = recept_i8h();   //idling flags
   d.param.ifac1     = recept_i16h();  //+
   d.param.ifac2     = recept_i16h();  //-
   d.param.MINEFR    = recept_i16h();
   d.param.idling_rpm = recept_i16h();
   d.param.idlreg_min_angle = recept_i16h();
   d.param.idlreg_max_angle = recept_i16h();
   d.param.idlreg_turn_on_temp = recept_i16h();
   //closed loop parameters:
   d.param.idl_to_run_add = recept_i8h();
   d.param.rpm_on_run_add = recept_i8h();
   d.param.idl_reg_p[0] = recept_i16h();  //-
   d.param.idl_reg_p[1] = recept_i16h();  //+
   d.param.idl_reg_i[0] = recept_i16h();  //-
   d.param.idl_reg_i[1] = recept_i16h();  //+
   d.param.idl_coef_thrd1 = recept_i8h();
   d.param.idl_coef_thrd2 = recept_i8h();
   d.param.idl_intrpm_lim = recept_i8h();
   d.param.idl_map_value = recept_i16h();
   d.param.idl_iacminpos = recept_i8h();
   d.param.idl_iacmaxpos = recept_i8h();
   d.param.iac_reg_db = recept_i16h();
   d.param.idl_reg_d = recept_i16h();
   d.param.irr_k_load = recept_i16h();
   d.param.irr_k_rpm = recept_i16h();
   break;

  case ANGLES_PAR:
   d.param.max_angle = recept_i16h();
   d.param.min_angle = recept_i16h();
   d.param.angle_corr= recept_i16h();
   d.param.angle_dec_speed = recept_i16h();
   d.param.angle_inc_speed = recept_i16h();
   d.param.zero_adv_ang = recept_i8h();
   d.param.igntim_flags = recept_i8h();
   d.param.shift_igntim = recept_i16h();
   break;

  case FUNSET_PAR:
   temp = recept_i8h();
   if (temp < TABLES_NUMBER)
    d.param.fn_gasoline = temp;

   temp = recept_i8h();
   if (temp < TABLES_NUMBER)
    d.param.fn_gas = temp;

   d.param.load_lower = recept_i16h();
   d.param.load_upper = recept_i16h();
   d.param.map_curve_offset = recept_i16h();
   d.param.map_curve_gradient = recept_i16h();
   d.param.map2_curve_offset = recept_i16h();   //map2
   d.param.map2_curve_gradient = recept_i16h(); //map2
   d.param.tps_curve_offset = recept_i16h();
   d.param.tps_curve_gradient = recept_i16h();

   temp = recept_i8h();
   if (temp < 5)
    d.param.load_src_cfg = temp;

   d.param.mapsel_uni = recept_i8h();
   d.param.barocorr_type = recept_i8h();
   d.param.func_flags = recept_i8h();
   d.param.ve2_map_func = recept_i8h();
   d.param.gas_v_uni = recept_i8h();

   recept_i8h();  //stub
   recept_i16h(); //stub
   d.param.mafload_const = recept_i32h();      //calculated in the SECU-3 Manager

   d.param.gps_curve_offset = recept_i16h();   //GPS
   d.param.gps_curve_gradient = recept_i16h(); //GPS
   d.param.fps_curve_offset = recept_i16h();   //FPS
   d.param.fps_curve_gradient = recept_i16h(); //FPS
   d.param.apps1_curve_offset = recept_i16h();   //APPS1
   d.param.apps1_curve_gradient = recept_i16h(); //APPS1
   break;

  case STARTR_PAR:
   d.param.starter_off = recept_i16h();
   d.param.smap_abandon= recept_i16h();
   d.param.inj_cranktorun_time = recept_i16h(); //fuel injection
   d.param.inj_aftstr_strokes = recept_i8h();   //fuel injection
   d.param.inj_prime_cold = recept_i16h();      //fuel injection
   d.param.inj_prime_hot = recept_i16h();       //fuel injection
   d.param.inj_prime_delay = recept_i8h();      //fuel injection
   d.param.inj_floodclear_tps = recept_i16h();   //fuel injection
   d.param.inj_aftstr_strokes1 = recept_i8h();  //fuel injection
   d.param.stbl_str_cnt = recept_i8h();
   d.param.strt_flags = recept_i8h();
   d.param.inj_cranktorun_time1 = recept_i16h(); //fuel injection
   break;

  case ADCCOR_PAR:
   d.param.adc_flags          = recept_i8h();
   d.param.map_adc_factor     = recept_i16h();
   d.param.map_adc_correction = recept_i32h();
   d.param.ubat_adc_factor    = recept_i16h();
   d.param.ubat_adc_correction= recept_i32h();
   d.param.temp_adc_factor    = recept_i16h();
   d.param.temp_adc_correction= recept_i32h();
   d.param.tps_adc_factor     = recept_i16h();
   d.param.tps_adc_correction = recept_i32h();
   d.param.ai1_adc_factor     = recept_i16h();
   d.param.ai1_adc_correction = recept_i32h();
   d.param.ai2_adc_factor     = recept_i16h();
   d.param.ai2_adc_correction = recept_i32h();
   d.param.ai3_adc_factor     = recept_i16h();
   d.param.ai3_adc_correction = recept_i32h();
   d.param.ai4_adc_factor     = recept_i16h();
   d.param.ai4_adc_correction = recept_i32h();
   d.param.ai5_adc_factor     = recept_i16h();
   d.param.ai5_adc_correction = recept_i32h();
   d.param.ai6_adc_factor     = recept_i16h();
   d.param.ai6_adc_correction = recept_i32h();
   d.param.ai7_adc_factor     = recept_i16h();
   d.param.ai7_adc_correction = recept_i32h();
   d.param.ai8_adc_factor     = recept_i16h();
   d.param.ai8_adc_correction = recept_i32h();
   break;

  case CKPS_PAR:
   d.param.ckps_cogs_btdc  = recept_i8h();
   d.param.ckps_ignit_cogs = recept_i8h();
   d.param.ckps_engine_cyl = recept_i8h();
   d.param.ckps_cogs_num = recept_i8h();
   d.param.ckps_miss_num = recept_i8h();
   d.param.hall_flags = recept_i8h();
   d.param.hall_wnd_width = recept_i16h();
   d.param.hall_degrees_btdc = recept_i16h();
   d.param.ckps_mttf = recept_i16h();
   break;

  case OP_COMP_NC:
   d.op_actn_code = recept_i16h();
   break;

  case KNOCK_PAR:
   d.param.knock_use_knock_channel = recept_i8h();
   d.param.knock_bpf_frequency   = recept_i8h();
   d.param.knock_k_wnd_begin_angle = recept_i16h();
   d.param.knock_k_wnd_end_angle = recept_i16h();
   d.param.knock_int_time_const = recept_i8h();

   d.param.knock_retard_step = recept_i16h();
   d.param.knock_advance_step = recept_i16h();
   d.param.knock_max_retard = recept_i16h();
   d.param.knock_threshold = recept_i16h();
   d.param.knock_recovery_delay = recept_i8h();
   d.param.knock_selch = recept_i8h();
   d.param.knkclt_thrd = recept_i16h();
   break;

  case CE_SAVED_ERR:
   d.ecuerrors_saved_transfer = recept_i32h();
   break;

  case MISCEL_PAR:
  {
   uint16_t old_divisor = d.param.uart_divisor;
   d.param.uart_divisor = recept_i16h();
   if (d.param.uart_divisor != old_divisor)
    d.param.bt_flags|= _BV(BTF_SET_BBR); //set flag indicating that we have to set bluetooth baud rate on next reset
   d.param.uart_period_t_ms = recept_i8h();
   d.param.ign_cutoff = recept_i8h();
   d.param.ign_cutoff_thrd = recept_i16h();
   d.param.hop_start_ang = recept_i16h();
   d.param.hop_durat_ang = recept_i16h();
   d.param.flpmp_flags = recept_i8h();   //fuel pump flags
   d.param.evap_afbegin = recept_i16h();
   d.param.evap_afslope = recept_i16h();
   d.param.fp_timeout_strt = recept_i8h();
   d.param.pwmfrq[0] = recept_i16h();
   d.param.pwmfrq[1] = recept_i16h();
   d.param.vss_period_dist = recept_i16h();
  }
  break;

  case CHOKE_PAR:
   d.param.sm_steps = recept_i16h();
   d.choke_testing = recept_i8h(); //fake parameter (actually it is status)
   d.choke_manpos_d = recept_i8h();//fake parameter
   d.param.choke_rpm_if = recept_i16h();
   d.param.choke_corr_time[0] = recept_i16h();
   d.param.choke_corr_time[1] = recept_i16h();
   d.param.choke_flags = recept_i8h(); //choke flags
   d.param.sm_freq = recept_i8h();
   d.param.inj_cranktorun_time = recept_i16h(); //fuel injection
   break;

#ifdef GD_CONTROL
  case GASDOSE_PAR:
   d.param.gd_steps = recept_i16h();
   d.gasdose_testing = recept_i8h(); //fake parameter (actually it is status)
   d.gasdose_manpos_d = recept_i8h();//fake parameter
   d.param.gd_fc_closing = recept_i8h();
   d.param.gd_lambda_corr_limit_p = recept_i16h();
   d.param.gd_lambda_corr_limit_m = recept_i16h();
   d.param.gd_lambda_stoichval = recept_i16h();
   d.param.gd_freq = recept_i8h();
   d.param.gd_maxfreqinit = recept_i8h();
   break;
#endif

  case SECUR_PAR:
  {
   uint8_t old_bt_flags = d.param.bt_flags;
   d.bt_name[0] = recept_i8h();
   if (d.bt_name[0] > 8)
    d.bt_name[0] = 8;
   d.bt_pass[0] = recept_i8h();
   if (d.bt_pass[0] > 6)
    d.bt_pass[0] = 6;
   recept_rs(&d.bt_name[1], d.bt_name[0]);
   recept_rs(&d.bt_pass[1], d.bt_pass[0]);
   d.param.bt_flags = recept_i8h();
   if ((old_bt_flags & _BV(BTF_USE_BT)) != (d.param.bt_flags & _BV(BTF_USE_BT)))
    d.param.bt_flags|= _BV(BTF_SET_BBR); //set flag indicating that we have to set bluetooth baud rate on next reset
   recept_rb(d.param.ibtn_keys[0], IBTN_KEY_SIZE);  //1st iButton key
   recept_rb(d.param.ibtn_keys[1], IBTN_KEY_SIZE);  //2nd iButton key
   d.param.bt_type = recept_i8h();
  }
  break;

  case UNIOUT_PAR:
  { //6 tunable outputs' parameters
   uint8_t oi = 0;
   for(; oi < UNI_OUTPUT_NUMBER; ++oi)
   {
    d.param.uni_output[oi].flags = recept_i8h();
    d.param.uni_output[oi].condition1 = recept_i8h();
    d.param.uni_output[oi].condition2 = recept_i8h();
    d.param.uni_output[oi].on_thrd_1 = recept_i16h();
    d.param.uni_output[oi].off_thrd_1 = recept_i16h();
    d.param.uni_output[oi].on_thrd_2 = recept_i16h();
    d.param.uni_output[oi].off_thrd_2 = recept_i16h();
   }
   d.param.uniout_12lf = recept_i8h();
   break;
  }

#ifdef FUEL_INJECT
 case INJCTR_PAR:
  d.param.inj_flags = recept_i8h();
  d.param.inj_config[0] = recept_i8h();
  d.param.inj_config[1] = recept_i8h();
  d.param.inj_flow_rate[0] = recept_i16h();
  d.param.inj_flow_rate[1] = recept_i16h();
  d.param.inj_cyl_disp = recept_i16h();
  d.param.inj_sd_igl_const[0] = recept_i32h();
  d.param.inj_sd_igl_const[1] = recept_i32h();
  recept_i8h();      //stub
  d.param.inj_timing[0] = recept_i16h();
  d.param.inj_timing[1] = recept_i16h();
  d.param.inj_timing_crk[0] = recept_i16h();
  d.param.inj_timing_crk[1] = recept_i16h();
  d.param.inj_anglespec = recept_i8h();
  d.param.fff_const = recept_i16h();
  *((uint16_t*)&d.param.inj_min_pw[0]) = recept_i16h();
  d.param.inj_maf_const[0] = recept_i32h();
  d.param.inj_maf_const[1] = recept_i32h();
  d.param.mafload_const = recept_i32h();
  d.param.inj_max_pw[0] = recept_i16h();
  d.param.inj_max_pw[1] = recept_i16h();
  break;
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 case LAMBDA_PAR:
  d.param.inj_lambda_str_per_stp = recept_i8h();
  d.param.inj_lambda_step_size_p = recept_i8h();
  d.param.inj_lambda_step_size_m = recept_i8h();
  d.param.inj_lambda_corr_limit_p = recept_i16h();
  d.param.inj_lambda_corr_limit_m = recept_i16h();
  d.param.inj_lambda_swt_point = recept_i16h();
  d.param.inj_lambda_temp_thrd = recept_i16h();
  d.param.inj_lambda_rpm_thrd = recept_i16h();
  d.param.inj_lambda_activ_delay = recept_i8h();
  d.param.inj_lambda_dead_band = recept_i16h();
  d.param.inj_lambda_senstype = recept_i8h();
  d.param.inj_lambda_ms_per_stp = recept_i8h();
  d.param.inj_lambda_flags = recept_i8h();
  d.param.gd_lambda_stoichval = recept_i16h();
  //heating:
  d.param.eh_heating_time[0] = recept_i8h();
  d.param.eh_heating_time[1] = recept_i8h();
  d.param.eh_temper_thrd = recept_i8h();
  d.param.eh_heating_act = recept_i8h();
  d.param.eh_aflow_thrd = recept_i16h();
  d.param.lambda_selch = recept_i8h();
  break;
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 case ACCEL_PAR:
  d.param.inj_ae_tpsdot_thrd = recept_i8h();
  d.param.inj_ae_coldacc_mult = recept_i16h();
  d.param.inj_ae_decay_time = recept_i8h();
  d.param.inj_ae_type = recept_i8h();
  d.param.inj_ae_time = recept_i8h();
  d.param.inj_ae_ballance = recept_i8h();
  d.param.inj_ae_mapdot_thrd = recept_i8h();
  d.param.inj_xtau_s_thrd = recept_i8h();
  d.param.inj_xtau_f_thrd = recept_i8h();
  d.param.wallwet_model = recept_i8h();
  break;
#endif

#if defined(FUEL_INJECT)
 case LTFT_PAR:
  d.param.ltft_mode = recept_i8h();
  d.param.ltft_learn_clt = recept_i16h();
  d.param.ltft_learn_clt_up = recept_i16h();
  d.param.ltft_learn_iat_up = recept_i16h();
  d.param.ltft_learn_grad = recept_i8h();
  d.param.ltft_learn_gpa = recept_i16h();
  d.param.ltft_learn_gpd = recept_i16h();
  d.param.ltft_min = recept_i8h();
  d.param.ltft_max = recept_i8h();
  d.param.ltft_learn_rpm[0] = recept_i16h();
  d.param.ltft_learn_rpm[1] = recept_i16h();
  d.param.ltft_learn_load[0] = recept_i16h();
  d.param.ltft_learn_load[1] = recept_i16h();
  d.param.ltft_dead_band[0] = recept_i8h();
  d.param.ltft_dead_band[1] = recept_i8h();
  break;
#endif

#if !defined(SECU3T) && defined(ELEC_THROTTLE)
 case DBW_PAR:
  d.param.etc_p = recept_i16h();
  d.param.etc_i = recept_i16h();
  d.param.etc_d = recept_i16h();
  d.param.etc_nmax_duty = recept_i8h();
  d.param.etc_pmax_duty = recept_i8h();
  d.param.etc_pid_period = recept_i8h();
  d.param.etc_frictorq_op = recept_i8h();
  d.param.etc_frictorq_cl = recept_i8h();
  d.param.etc_frictorq_thrd = recept_i8h();
  d.param.etc_idleadd_max = recept_i16h();
  break;
#endif

#ifdef REALTIME_TABLES
  case EDITAB_PAR:
  {
   uint8_t state = recept_i8h();  //map type
   uint8_t addr = recept_i8h();   //address (address of cell)
   switch(state)
   {
    case ETMT_STRT_MAP: //start map
     recept_rb(((uint8_t*)&d.tables_ram.f_str) + addr, F_STR_POINTS); /*F_STR_POINTS max*/
     break;
    case ETMT_IDLE_MAP: //idle map
     recept_rb(((uint8_t*)&d.tables_ram.f_idl) + addr, F_IDL_POINTS); /*F_IDL_POINTS max*/
     break;
    case ETMT_WORK_MAP: //work map
     recept_rb(((uint8_t*)&d.tables_ram.f_wrk[0][0]) + addr, F_WRK_POINTS_F); /*F_WRK_POINTS_F max*/
     break;
    case ETMT_TEMP_MAP: //temper. correction map
     recept_rb(((uint8_t*)&d.tables_ram.f_tmp) + addr, F_TMP_POINTS); /*F_TMP_POINTS max*/
     break;
    case ETMT_TEMPI_MAP: //temper. correction map (idling)
     recept_rb(((uint8_t*)&d.tables_ram.f_tmp_idl) + addr, F_TMP_POINTS); /*F_TMP_POINTS max*/
     break;
    case ETMT_NAME_STR: //name
     recept_rs((d.tables_ram.name) + addr, F_NAME_SIZE); /*F_NAME_SIZE max*/
     break;
    case ETMT_VE_MAP:   //VE, 12-bit per cell
     recept_rb(((uint8_t*)&d.tables_ram.inj_ve[0][0]) + (((uint16_t)addr)+(((uint16_t)addr)>>1)), (INJ_VE_POINTS_F*3)/2); /*INJ_VE_POINTS_F*1.5 max*/
     break;
    case ETMT_VE2_MAP:   //Secondary VE, 12-bit per cell
     recept_rb(((uint8_t*)&d.tables_ram.inj_ve2[0][0]) + (((uint16_t)addr)+(((uint16_t)addr)>>1)), (INJ_VE_POINTS_F*3)/2); /*INJ_VE_POINTS_F*1.5 max*/
     break;
    case ETMT_IVE_MAP:   //Idling VE, 12-bit per cell
     recept_rb(((uint8_t*)&d.tables_ram.inj_ive[0][0]) + (((uint16_t)addr)+(((uint16_t)addr)>>1)), (INJ_IVE_POINTS_F*3)/2); /*INJ_IVE_POINTS_F*1.5 max*/
     break;
    case ETMT_AFR_MAP:  //AFR
     recept_rb(((uint8_t*)&d.tables_ram.inj_afr[0][0]) + addr, INJ_VE_POINTS_F); /*INJ_VE_POINTS_F max*/
     break;
    case ETMT_CRNK_MAP: //PW on cranking
     recept_rw(((uint16_t*)&d.tables_ram.inj_cranking) + addr, INJ_CRANKING_LOOKUP_TABLE_SIZE/2); /*INJ_CRANKING_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_WRMP_MAP: //Warmup enrichment
     recept_rb(((uint8_t*)&d.tables_ram.inj_warmup) + addr, INJ_WARMUP_LOOKUP_TABLE_SIZE); /*INJ_WARMUP_LOOKUP_TABLE_SIZE max*/
     break;
    case ETMT_DEAD_MAP: //Injector dead time
     recept_rw(((uint16_t*)&d.tables_ram.inj_dead_time) + addr, INJ_DT_LOOKUP_TABLE_SIZE/4); /*INJ_DT_LOOKUP_TABLE_SIZE/4 max*/
     break;
    case ETMT_IDLR_MAP: //IAC/PWM position on run
     recept_rb(((uint8_t*)&d.tables_ram.inj_iac_run_pos) + addr, INJ_IAC_POS_TABLE_SIZE); /*INJ_IAC_POS_TABLE_SIZE max*/
     break;
    case ETMT_IDLC_MAP: //IAC/PWM position on cranking
     recept_rb(((uint8_t*)&d.tables_ram.inj_iac_crank_pos) + addr, INJ_IAC_POS_TABLE_SIZE); /*INJ_IAC_POS_TABLE_SIZE max*/
     break;
    case ETMT_THRASS_MAP: //Throttle assistant
     recept_rb(((uint8_t*)&d.tables_ram.inj_thrass) + addr, INJ_THRASS_SIZE); /*INJ_THRASS_SIZE max*/
     break;
    case ETMT_AETPS_MAP: //AE TPS, Note! Here we consider inj_ae_tps_bins and inj_ae_tps_enr as single table
     recept_rb(((uint8_t*)&d.tables_ram.inj_ae_tps_enr) + addr, INJ_AE_TPS_LOOKUP_TABLE_SIZE*2); /*INJ_AE_TPS_LOOKUP_TABLE_SIZE*2 max*/
     break;
    case ETMT_AERPM_MAP: //AE RPM, Note! Here we consider inj_ae_rpm_bins and inj_ae_rpm_enr as single table
     recept_rb(((uint8_t*)&d.tables_ram.inj_ae_rpm_enr) + addr, INJ_AE_RPM_LOOKUP_TABLE_SIZE*2); /*INJ_AE_RPM_LOOKUP_TABLE_SIZE*2 max*/
     break;
    case ETMT_AFTSTR_MAP: //afterstart enrichment map
     recept_rb(((uint8_t*)&d.tables_ram.inj_aftstr) + addr, INJ_AFTSTR_LOOKUP_TABLE_SIZE); /*INJ_AFTSTR_LOOKUP_TABLE_SIZE max*/
     break;
    case ETMT_IT_MAP:   //Injection timing, 12 bit per cell
     recept_rb(((uint8_t*)&d.tables_ram.inj_timing[0][0]) + (((uint16_t)addr)+(((uint16_t)addr)>>1)), (INJ_VE_POINTS_F*3)/2); /*INJ_VE_POINTS_F*1.5 max*/
     break;
    case ETMT_ITRPM_MAP: //Idling RPM map
     recept_rb(((uint8_t*)&d.tables_ram.inj_target_rpm) + addr, INJ_TARGET_RPM_TABLE_SIZE); /*INJ_TARGET_RPM_TABLE_SIZE max*/
     break;
    case ETMT_RIGID_MAP: //Idling regulator's rigidity function
     recept_rw(((uint16_t*)&d.tables_ram.inj_idl_rigidity) + addr, INJ_IDL_RIGIDITY_SIZE); /*INJ_IDL_RIGIDITY_SIZE max*/
     break;
    case ETMT_EGOCRV_MAP: //EGO curve (WBO emulation)
     recept_rw(((uint16_t*)&d.tables_ram.inj_ego_curve) + addr, INJ_EGO_CURVE_SIZE/2); /*INJ_EGO_CURVE_SIZE/2 max*/
     break;
    case ETMT_IACC_MAP: //Mixture correction vs IAC pos
     recept_rw(((uint16_t*)&d.tables_ram.inj_iac_corr) + addr, INJ_IAC_CORR_SIZE); /*INJ_IAC_CORR_SIZE max*/
     break;
    case ETMT_IACCW_MAP: //Weight of mixture correction vs TPS pos
     recept_rb(((uint8_t*)&d.tables_ram.inj_iac_corr_w) + addr, INJ_IAC_CORR_W_SIZE); /*INJ_IAC_CORR_W_SIZE max*/
     break;
    case ETMT_IATCLT_MAP: //IAT/CLT correction vs air flow
     recept_rw(((uint16_t*)&d.tables_ram.inj_iatclt_corr) + addr, INJ_IATCLT_CORR_SIZE); /*INJ_IATCLT_CORR_SIZE max*/
     break;
    case ETMT_TPSSWT_MAP: //MAP/TPS switch point
     recept_rb(((uint8_t*)&d.tables_ram.inj_tpsswt) + addr, INJ_TPSSWT_SIZE); /*INJ_TPSSWT_SIZE max*/
     break;
    case ETMT_GTSC_MAP: //PW correction from gas temperature
     recept_rb(((uint8_t*)&d.tables_ram.inj_gts_corr) + addr, INJ_GTS_CORR_SIZE); /*INJ_GTS_CORR_SIZE max*/
     break;
    case ETMT_GPSC_MAP: //PW correction from gas pressure
     recept_rb(((uint8_t*)&d.tables_ram.inj_gps_corr) + addr, INJ_GPS_CORR_SIZE+2); /*INJ_GPS_CORR_SIZE+2 max*/
     break;
    case ETMT_ATSC_MAP: //PW correction from air temperature
     recept_rb(((uint8_t*)&d.tables_ram.inj_ats_corr) + addr, INJ_ATS_CORR_SIZE); /*INJ_ATS_CORR_SIZE max*/
     break;
    case ETMT_IACMAT_MAP: //IAC position's correction vs MAT
     recept_rb(((uint8_t*)&d.tables_ram.iac_mat_corr) + addr, INJ_ATS_CORR_SIZE); /*INJ_ATS_CORR_SIZE max*/
     break;
    case ETMT_PWM1_MAP: //PWM1 map
     recept_rb(((uint8_t*)&d.tables_ram.pwm_duty1[0][0]) + addr, F_WRK_POINTS_F); /*F_WRK_POINTS_F max*/
     break;
    case ETMT_PWM2_MAP: //PWM2 map
     recept_rb(((uint8_t*)&d.tables_ram.pwm_duty2[0][0]) + addr, F_WRK_POINTS_F); /*F_WRK_POINTS_F max*/
     break;
    case ETMT_TPSZON_MAP: //MAP/TPS load axis allocation
     recept_rb(((uint8_t*)&d.tables_ram.inj_tpszon) + addr, INJ_TPSZON_SIZE); /*INJ_TPSZON_SIZE max*/
     break;
    case ETMT_CYLMULT_MAP: //Inj. multiplier
     recept_rb(((uint8_t*)&d.tables_ram.inj_cylmult) + addr, INJ_CYLADD_SIZE); /*INJ_CYLADD_SIZE max*/
     break;
    case ETMT_CYLADD_MAP: //Inj. addition
     recept_rb(((uint8_t*)&d.tables_ram.inj_cyladd) + addr, INJ_CYLADD_SIZE); /*INJ_CYLADD_SIZE max*/
     break;
    case ETMT_AEMAP_MAP: //AE MAP, Note! Here we consider inj_ae_map_bins and inj_ae_map_enr as single table
     recept_rb(((uint8_t*)&d.tables_ram.inj_ae_map_enr) + addr, INJ_AE_MAP_LOOKUP_TABLE_SIZE*2); /*INJ_AE_MAP_LOOKUP_TABLE_SIZE*2 max*/
     break;
   }
   //group of separate maps
   switch(state)
   {
    case ETMT_ATTENUATOR:
     recept_rb(((uint8_t*)&ram_extabs.attenuator_table) + addr, KC_ATTENUATOR_LOOKUP_TABLE_SIZE/8); /*KC_ATTENUATOR_LOOKUP_TABLE_SIZE/8 max*/
     break;
    case ETMT_DWELLCNTRL:
     recept_rw(((uint16_t*)&ram_extabs.coil_on_time) + addr, COIL_ON_TIME_LOOKUP_TABLE_SIZE/4); /*COIL_ON_TIME_LOOKUP_TABLE_SIZE/4 max*/
     break;
    case ETMT_CTS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.cts_curve) + addr, THERMISTOR_LOOKUP_TABLE_SIZE/2); /*THERMISTOR_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_ATS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.ats_curve) + addr, THERMISTOR_LOOKUP_TABLE_SIZE/2); /*THERMISTOR_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_ATS_CORR:
     recept_rb(((uint8_t*)&ram_extabs.ats_corr) + addr, ATS_CORR_LOOKUP_TABLE_SIZE); /*ATS_CORR_LOOKUP_TABLE_SIZE max*/
     break;
    case ETMT_GASDOSE:
     recept_rb(((uint8_t*)&ram_extabs.gasdose_pos[0][0]) + addr, GASDOSE_POS_RPM_SIZE); /*GASDOSE_POS_RPM_SIZE max*/
     break;
    case ETMT_BAROCORR:
     recept_rw(((uint16_t*)&ram_extabs.barocorr) + addr, 8); /*8 max*/
     break;
    case ETMT_MANIGNTIM:
     recept_rb(((uint8_t*)&ram_extabs.pa4_igntim_corr) + addr, PA4_LOOKUP_TABLE_SIZE); /*PA4_LOOKUP_TABLE_SIZE max*/
     break;
    case ETMT_TMP2_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.tmp2_curve) + addr, THERMISTOR_LOOKUP_TABLE_SIZE/2); /*THERMISTOR_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_CRKCLT_CORR:
     recept_rb(((uint8_t*)&ram_extabs.cts_crkcorr) + addr, CTS_CRKCORR_SIZE); /*CTS_CRKCORR_SIZE max*/
     break;
    case ETMT_EH_PAUSE:
     recept_rb(((uint8_t*)&ram_extabs.eh_pause) + addr, COIL_ON_TIME_LOOKUP_TABLE_SIZE/2); /*COIL_ON_TIME_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_CRANKING_THRD:
     recept_rb(((uint8_t*)&ram_extabs.cranking_thrd) + addr, CRANK_THRD_SIZE); /*CRANK_THRD_SIZE max*/
     break;
    case ETMT_CRANKING_TIME:
     recept_rb(((uint8_t*)&ram_extabs.cranking_time) + addr, CRANK_TIME_SIZE); /*CRANK_TIME_SIZE max*/
     break;
    case ETMT_SMAPABAN_THRD:
     recept_rb(((uint8_t*)&ram_extabs.smapaban_thrd) + addr, SMAPABAN_THRD_SIZE); /*SMAPABAN_THRD_SIZE max*/
     break;
    case ETMT_KNOCK_ZONE:
     recept_rw(((uint16_t*)&ram_extabs.knock_zones) + addr, KNKZONE_TPS_SIZE/4); /*KNKZONE_TPS_SIZE/4 max*/
     break;
    case ETMT_GRTS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.grts_curve) + addr, THERMISTOR_LOOKUP_TABLE_SIZE/2); /*THERMISTOR_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_GRHEAT_DUTY:
     recept_rb(((uint8_t*)&ram_extabs.grheat_duty) + addr, F_TMP_POINTS); /*F_TMP_POINTS max*/
     break;
    case ETMT_PWMIAC_UCOEF:
     recept_rw(((uint16_t*)&ram_extabs.pwmiac_ucoef) + addr, PWMIAC_UCOEF_SIZE/2); /*PWMIAC_UCOEF_SIZE/2 max*/
     break;
    case ETMT_AFTSTR_STRK0:
     recept_rw(((uint16_t*)&ram_extabs.inj_aftstr_strk0) + addr, AFTSTR_STRK_SIZE/2); /*AFTSTR_STRK_SIZE/2 max*/
     break;
    case ETMT_AFTSTR_STRK1:
     recept_rw(((uint16_t*)&ram_extabs.inj_aftstr_strk1) + addr, AFTSTR_STRK_SIZE/2); /*AFTSTR_STRK_SIZE/2 max*/
     break;
    case ETMT_GRVDELAY:
     recept_rw(((uint16_t*)&ram_extabs.grv_delay) + addr, F_TMP_POINTS/2); /*F_TMP_POINTS/2 max*/
     break;
    case ETMT_FTLS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.ftls_curve) + addr, EGTS_LOOKUP_TABLE_SIZE/2); /*EGTS_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_EGTS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.egts_curve) + addr, EGTS_LOOKUP_TABLE_SIZE/2); /*EGTS_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_OPS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.ops_curve) + addr, OPS_LOOKUP_TABLE_SIZE/2); /*OPS_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_MANINJPWC:
     recept_rw(((uint16_t*)&ram_extabs.injpw_coef) + addr, INJPWCOEF_LUT_SIZE/2); /*INJPWCOEF_LUT_SIZE/2 max*/
     break;
    case ETMT_MAF_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.maf_curve) + addr, MAF_FLOW_CURVE_SIZE/8); /*MAF_FLOW_CURVE_SIZE/8 max*/
     break;
    case ETMT_FTLSCOR:
     recept_rw(((uint16_t*)&ram_extabs.ftlscor_ucoef) + addr, FTLSCOR_UCOEF_SIZE/4); /*FTLSCOR_UCOEF_SIZ/4 max*/
     break;
    case ETMT_LAMBDA_ZONE:
     recept_rw(((uint16_t*)&ram_extabs.lambda_zones) + addr, EGOZONE_LOAD_SIZE/4); /*EGOZONE_LOAD_SIZE/4 max*/
     break;
    case ETMT_FTS_CURVE:
     recept_rw(((uint16_t*)&ram_extabs.fts_curve) + addr, FTS_LOOKUP_TABLE_SIZE/2); /*FTS_LOOKUP_TABLE_SIZE/2 max*/
     break;
    case ETMT_FUELDENS_CORR:
     recept_rw(((uint16_t*)&ram_extabs.fueldens_corr) + addr, FUELDENS_CORR_SIZE/2); /*FUELDENS_CORR_SIZE/2 max*/
     break;
    case ETMT_XTAU_XFACC:
     recept_rw(((uint16_t*)&ram_extabs.xtau_xfacc) + addr, XTAU_FACT_SIZE/2); /*XTAU_FACT_SIZE/2 max*/
     break;
    case ETMT_XTAU_XFDEC:
     recept_rw(((uint16_t*)&ram_extabs.xtau_xfdec) + addr, XTAU_FACT_SIZE/2); /*XTAU_FACT_SIZE/2 max*/
     break;
    case ETMT_XTAU_TFACC:
     recept_rw(((uint16_t*)&ram_extabs.xtau_tfacc) + addr, XTAU_FACT_SIZE/2); /*XTAU_FACT_SIZE/2 max*/
     break;
    case ETMT_XTAU_TFDEC:
     recept_rw(((uint16_t*)&ram_extabs.xtau_tfdec) + addr, XTAU_FACT_SIZE/2); /*XTAU_FACT_SIZE/2 max*/
     break;
    case ETMT_INJNONLINP: //Note! Here we consider inj_nonlinp_corr and inj_nonlinp_bins as single table
     recept_rw(((uint16_t*)&ram_extabs.inj_nonlinp_corr) + addr, INJ_NONLIN_SIZE); /*INJ_NONLIN_SIZE max*/
     break;
    case ETMT_INJNONLING: //Note! Here we consider inj_nonling_corr and inj_nonling_bins as single table
     recept_rw(((uint16_t*)&ram_extabs.inj_nonling_corr) + addr, INJ_NONLIN_SIZE); /*INJ_NONLIN_SIZE max*/
     break;
    case ETMT_EGO_DELAY:
     recept_rw(((uint16_t*)&ram_extabs.inj_ego_delay) + addr, EGO_DELAY_SIZE/2); /*EGO_DELAY_SIZE/2 max*/
     break;
    case ETMT_WU_AFR0:
     recept_rb(((uint8_t*)&ram_extabs.inj_wu_afr0) + addr, WU_AFR_SIZE); /*WU_AFR_SIZE max*/
     break;
    case ETMT_WU_AFR1:
     recept_rb(((uint8_t*)&ram_extabs.inj_wu_afr1) + addr, WU_AFR_SIZE); /*WU_AFR_SIZE max*/
     break;
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
    case ETMT_ETC_SPRPREL: //Note! Here we consider etc_sprprel_duty and etc_sprprel_bins as single table
     recept_rw(((uint16_t*)&ram_extabs.etc_sprprel_duty) + addr, ETC_SPRPREL_SIZE); /*ETC_SPRPREL_SIZE max*/
     break;
    case ETMT_ETC_ACCEERR: //Note! Here we consider etc_accept_error and etc_accept_bins as single table
     recept_rw(((uint16_t*)&ram_extabs.etc_accept_error) + addr, ETC_ACCEPTERR_SIZE); /*ETC_ACCEPTERR_SIZE max*/
     break;
    case ETMT_ETC_THROPOS:
     recept_rb(((uint8_t*)&ram_extabs.etc_throttle_pos[0][0]) + addr, ETC_POS_RPM_SIZE); /*ETC_POS_RPM_SIZE max*/
     break;
#endif
   }
  }
  break;
#endif
#ifdef DIAGNOSTICS
  case DIAGOUT_DAT:
   d.diag_out = recept_i32h();
   d.diag_frq = recept_i16h(); //frequency
   d.diag_duty = recept_i8h(); //duty
   d.diag_chan = recept_i8h(); //selected channel (=0 no selection)
   break;
#endif

  case LZBLHS:
  {
   uint8_t buff[5]; //4 bytes - string plus 1 byte - address
   recept_rs(buff, 5);
   if (!MEMCMP_P(buff, lzblhs_str, 4))
    uart_set_send_mode(SILENT);
  }
  break;
 }//switch

 return descriptor;
}

uint8_t uart_is_sender_busy(void)
{
 return (uart.send_size > 0);
}

uint8_t uart_get_send_mode(void)
{
 return uart.send_mode;
}

uint8_t uart_set_send_mode(uint8_t descriptor)
{ //note: code of this function must follow code in uart_send_packet() !
 switch(descriptor)
 {
  case TEMPER_PAR:
  case CARBUR_PAR:
  case IDLREG_PAR:
  case ANGLES_PAR:
  case FUNSET_PAR:
  case STARTR_PAR:
  case FNNAME_DAT:
  case SENSOR_DAT:
  case ADCCOR_PAR:
  case ADCRAW_DAT:
  case CKPS_PAR:
  case OP_COMP_NC:
  case CE_ERR_CODES:
  case KNOCK_PAR:
  case CE_SAVED_ERR:
  case FWINFO_DAT:
  case MISCEL_PAR:
  case CHOKE_PAR:
#ifdef GD_CONTROL
  case GASDOSE_PAR:  //GD
#endif
  case SECUR_PAR:
  case UNIOUT_PAR:
#ifdef FUEL_INJECT
  case INJCTR_PAR:
#endif
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
  case ACCEL_PAR:
#endif
#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
  case LAMBDA_PAR:
#endif
#ifdef REALTIME_TABLES
  case EDITAB_PAR:
  case RPMGRD_PAR:
  case CLTGRD_PAR:
  case LODGRD_PAR:
  case IRPMGRD_PAR: //idling
  case ILODGRD_PAR: //idling
  case TLODGRD_PAR:
#endif
  case ATTTAB_PAR:
#ifdef DEBUG_VARIABLES
  case DBGVAR_DAT:
#endif
#ifdef DIAGNOSTICS
  case DIAGINP_DAT:
#endif
#ifdef FUEL_INJECT
  case LTFT_DAT:
#endif
#if defined(FUEL_INJECT)
  case LTFT_PAR:
#endif
#if !defined(SECU3T) && defined(ELEC_THROTTLE)
  case DBW_PAR:
#endif
  case SILENT:
  case LZBLHS:
   return uart.send_mode = descriptor;
  default:
   return uart.send_mode; //dot not set not existing context
 }
}

/** Clears sender's buffer
 */
void uart_reset_send_buff(void)
{
 uart.send_size = 0;
}

/** Append sender's buffer by one byte. This function is used in the bluetooth module
 * \param ch Byte value to be appended to the buffer
 */
void uart_append_send_buff(uint8_t ch)
{
 uart.send_buf[uart.send_size++] = ch;
}

void uart_init(uint16_t baud, uint8_t silent)
{
 // Set baud rate
 UBRRH = (uint8_t)(baud>>8);
 UBRRL = (uint8_t)baud;
 UCSRA = _BV(U2X);                                           //use speed doubling to minimize time error
 UCSRB=_BV(RXCIE)|_BV(RXEN)|_BV(TXEN);                       //receiver, receiver's interrupt and transmitter are enabled
#ifdef URSEL
 UCSRC=_BV(URSEL)/*|_BV(USBS)*/|_BV(UCSZ1)|_BV(UCSZ0);       //8 bit, 1 stop, no parity control
#else
 UCSRC=/*_BV(USBS)|*/_BV(UCSZ1)|_BV(UCSZ0);                  //8 bit, 1 stop, no parity control
#endif

 if (silent)
  uart.send_mode = SILENT;
 else
  uart.send_mode = SENSOR_DAT;

 uart.recv_head = uart.recv_tail = 0;                        //receiver's buffer is empty
}


/**Interrupt handler for the transfer of bytes through the UART (transmitter data register empty)
 */
ISR(USART_UDRE_vect)
{
 if (uart.send_size > 0)
 {
  UDR = uart.send_buf[uart.send_index++];
  --uart.send_size;
 }
 else
 {//all data have transmitted
  UCSRB &= ~_BV(UDRIE); // disable UDRE interrupt
 }
}

/**Interrupt handler for receive data through the UART */
ISR(USART_RXC_vect)
{
 uart.recv_buf[uart.recv_head] = UDR;
 RECV_INC_INDEX(uart.recv_head);

 /*
 if (uart.recv_head == uart.recv_tail)
 {
  //receive buffer overflow! What to do?
 }
 */
}

void uart_transmitter(uint8_t state)
{
 if (state)
 { //turn on transmitter
  _DISABLE_INTERRUPT();
  UCSRB|=_BV(TXEN);
  _ENABLE_INTERRUPT();
 }
 else
 { //turn off transmitter
  _DISABLE_INTERRUPT();
  UCSRB&=~_BV(TXEN);
  _ENABLE_INTERRUPT();
 }
}
