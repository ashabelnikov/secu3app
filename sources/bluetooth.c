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

/** \file bluetooth.c
 * \author Alexey A. Shabelnikov
 * Implementation of bluetooth related functionality and logic (baud rate setting, name, password).
 * Note that current implementation supports Bluetooth with HC-06 firmware.
 */

#ifdef BLUETOOTH_SUPP

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bitmask.h"
#include "bluetooth.h"
#include "ecudata.h"
#include "suspendop.h"
#include "uart.h"
#include "vstimer.h"

/**Set baud rate registers*/
#define SET_BRR(br) { UBRRH = (uint8_t)((br)>>8); UBRRL = (uint8_t)(br); }

/**Time between AT commands (Bluetooth HC-06 firmware requires it).
 * 1 discrete = 10ms
 */
#define AT_COMMAND_TIME 100

/** Start up delay, used before any AT commands will be sent
 */
#define AT_COMMAND_STRT_TIME 50

/**Get BT type*/
#define GETBTTYPE() ((d.param.bt_flags >> BTF_BT_TYPE0) & 0x3)
#define BTT_BC417   0   //!< BC417
#define BTT_BK3231  1   //!< BK3231
#define BTT_BK3231S 2   //!< BK3231S (JDY-31)

//External functions (from uart.c)
void uart_reset_send_buff(void);
void uart_append_send_buff(uint8_t ch);
void uart_begin_send(void);

/**Template for AT+BAUDx command */
PGM_DECLARE(uint8_t AT_BAUD[]) = "AT+BAUD";

/**Template for AT+NAMEx command */
PGM_DECLARE(uint8_t AT_NAME[]) = "AT+NAME";

/**Template for AT+PINx command */
PGM_DECLARE(uint8_t AT_PIN[])  = "AT+PIN";

/**Template for AT+RESET command */
PGM_DECLARE(uint8_t AT_RESET[])  = "AT+RESET";

/**Number of possible baud rate codes*/
#define BT_BAUDS_NUM 5

/**Possible baud rates*/
PGM_DECLARE(uint16_t bt_bauds[BT_BAUDS_NUM]) = {CBR_9600, CBR_19200, CBR_38400, CBR_57600, CBR_115200};

/**Define internal state variables */
typedef struct
{
 uint8_t btbr_mode;                     //!< finite state machine state for setting of BT baud rate
 uint8_t btnp_mode;                     //!< finite state machine state for setting of name and password
 uint16_t strt_t1;                      //!< used for time calculations
 uint8_t cur_baud;                      //!< index of current baud in bt_bauds array
}bts_t;

/**Instance of internal state variables */
bts_t bts;

/**Appends sender's buffer by sequence of bytes from flash.
 * note! can NOT be used for binary data! */
static void bt_build_fs(uint8_t _PGM *romBuffer, uint8_t size)
{
 while(size--) uart_append_send_buff(PGM_GET_BYTE(romBuffer++));
}

/**Appends sender's buffer by sequence of bytes from RAM.
 * note! can NOT be used for binary data! */
static void bt_build_rs(const uint8_t *ramBuffer, uint8_t size)
{
 while(size--) uart_append_send_buff(*ramBuffer++);
}

void bt_init(uint8_t en_set_baud)
{
 bts.btbr_mode = en_set_baud ? 0 : 255;
 bts.btnp_mode = 0;
}

/**Appends UART TX buffer with CRLF bytes (before AT-command)
 * Uses d ECU data structure
 */
static void build_crlf_b(void)
{
 //Use CRLF before each AT-command only for BK3231 bluetooth
 if (GETBTTYPE()==BTT_BK3231)
 {
  uart_append_send_buff('\r');
  uart_append_send_buff('\n');
 }
}

/**Appends UART TX buffer with CRLF bytes (after AT-command)
 * Uses d ECU data structure
 */
static void build_crlf_e(void)
{
 //use CRLF at the end for BK3231 and BK3231S bluetoothes
 if (GETBTTYPE()!=BTT_BC417)
 {
  uart_append_send_buff('\r');
  uart_append_send_buff('\n');
 }
}

/** Builds AT+BAUDx command in the sender's buffer
 * \param baud Baud rate code (see uart.h file for more information)
 */
static void append_tx_buff_with_at_baud_cmd(uint16_t baud)
{
 uart_reset_send_buff();
 build_crlf_b(); //use CRLF before AT command to reset possible errors
 bt_build_fs(AT_BAUD, 7);
 if (baud == CBR_9600) uart_append_send_buff('4');
 else if (baud == CBR_19200) uart_append_send_buff('5');
 else if (baud == CBR_38400) uart_append_send_buff('6');
 else if (baud == CBR_57600) uart_append_send_buff('7');
 else if (baud == CBR_115200) uart_append_send_buff('8');
 build_crlf_e();
}

/** Builds AT+RESET command in the sender's buffer
 */
static void append_tx_buff_with_at_reset_cmd(void)
{
 uart_reset_send_buff();
 build_crlf_b(); //use CRLF before AT command to reset possible errors
 bt_build_fs(AT_RESET, 8);
 build_crlf_e();
}

/** Increments SM state if timer expired (baud rate setting)
 *\return 0 - state has not been changed, 1 - state has been changed (incremented)
 */
uint8_t next_state_if_tmr_expired_br(void)
{
 if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
 {
  ++bts.btbr_mode;
  return 1;
 }
 return 0;
}

/** Increments SM state if timer expired (name & password setting)*/
void next_state_if_tmr_expired_np(void)
{
 if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
  ++bts.btnp_mode;
}

void next_state_with_new_baud(uint16_t baud)
{
 SET_BRR(baud);
 ++bts.btbr_mode;
 bts.strt_t1 = s_timer_gtc();    //set timer
}

uint8_t bt_set_baud(uint16_t baud)
{
 if (255 == bts.btbr_mode)
  return 1; //not enabled/stopped

 switch(bts.btbr_mode)
 {
  //wait some time before we start to send first AT command
  case 0:
   ++bts.btbr_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   bts.cur_baud = 0;
   return 0;
  case 1:
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_STRT_TIME)
    ++bts.btbr_mode;
   return 0;

  //Send command on current baud (try all possible baud rate)
  case 2:
   append_tx_buff_with_at_baud_cmd(baud);
   next_state_with_new_baud(PGM_GET_WORD(&bt_bauds[bts.cur_baud]));
   break;
  case 3:                          //wait some time
   if (next_state_if_tmr_expired_br())
   {
    ++bts.cur_baud;
    if (bts.cur_baud < BT_BAUDS_NUM)
    {
     bts.btbr_mode = 2; //continue...
    }
    else
    {
     if (GETBTTYPE()==BTT_BK3231S)
     {
      bts.btbr_mode = 5; //we need to perform additional operations to reset bluetooth
      bts.cur_baud = 0;
     }
    }
   }
   return 0;

  //-------------------------------------
  //Finishing...
  case 4:
   if (!uart_is_sender_busy())
   {
    next_state_with_new_baud(baud); //return back old baud rate
    bts.btbr_mode = 255; //stop state machine
    //reset flag and save parameters
    CLEARBIT(d.param.bt_flags, BTF_SET_BBR);
    sop_set_operation(SOP_SAVE_PARAMETERS);
   }
   return 0;

  //Send command on current baud (try all possible baud rate)
  case 5:
   append_tx_buff_with_at_reset_cmd();
   next_state_with_new_baud(PGM_GET_WORD(&bt_bauds[bts.cur_baud]));
   break;
  case 6:                          //wait some time
   if (next_state_if_tmr_expired_br())
   {
    ++bts.cur_baud;
    if (bts.cur_baud < BT_BAUDS_NUM)
     bts.btbr_mode = 5; //continue...
    else
     bts.btbr_mode = 4; //end
   }
   return 0;

  default:
   return 1; //stopped
 }

 //Start background process of packet sending
 uart_begin_send();
 return 0;
}


void bt_start_set_namepass(void)
{
 bts.btnp_mode = 0;
}

/** Builds AT+NAMEx command in the sender's buffer
 * \param name Buffer containing name, 1-st byte of buffer contains size of string
 */
static void append_tx_buff_with_at_name_cmd(uint8_t* name)
{
 uart_reset_send_buff();
 build_crlf_b();
 bt_build_fs(AT_NAME, 7);
 bt_build_rs(&name[1], name[0]);
 build_crlf_e();
}

/** Builds AT+PINx command in the sender's buffer
 * \param name Buffer containing password, 1-st byte of buffer contains size of string
 */
static void append_tx_buff_with_at_pass_cmd(uint8_t* pass)
{
 uart_reset_send_buff();
 build_crlf_b();
 bt_build_fs(AT_PIN, 6);
 bt_build_rs(&pass[1], pass[0]);
 build_crlf_e();
}

uint8_t bt_set_namepass(void)
{
 switch(bts.btnp_mode)
 {
  //wait some time before we start to send first AT command
  case 0:
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   return 0;
  case 1:
   next_state_if_tmr_expired_np();
   return 0;

  //Send command to change name
  case 2:
   append_tx_buff_with_at_name_cmd(d.bt_name);
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   break;                          //send!
  case 3:                          //wait some time
   next_state_if_tmr_expired_np();
   return 0;

  //Send command to change password (pin)
  case 4:
   append_tx_buff_with_at_pass_cmd(d.bt_pass);
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   break;                          //send!
  case 5:                          //wait some time
   next_state_if_tmr_expired_np();
   if (bts.btnp_mode > 5)
   {
    d.bt_name[0] = 0, d.bt_pass[0] = 0;
    return 1; //finished!
   }
   return 0;

  default:
   return 1;  //stopped
 }

 //Start background process of packet sending
 uart_begin_send();
 return 0;
}

#endif //BLUETOOTH_SUPP
