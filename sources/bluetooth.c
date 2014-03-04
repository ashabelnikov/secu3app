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

/** \file bluetooth.c
 * Implementation of bluetooth related functionality and logic (baud rate setting, name, password).
 * Note that current implementation supports Bluetooth with HC-06 firmware.
 * (Реализация логики связанной с блютузом (установка скорости, имя, пароль))
 */

#ifdef BLUETOOTH_SUPP

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bluetooth.h"
#include "secu3.h"
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

/**Define internal state variables */
typedef struct
{
 uint8_t btbr_mode;                     //!< finite state machine state for setting of BT baud rate
 uint8_t btnp_mode;                     //!< finite state machine state for setting of name and password
 uint16_t strt_t1;                      //!< used for time calculations
}bts_t;

/**Instance of internal state variables */
bts_t bts;

void bt_init(uint8_t en_set_baud)
{
 bts.btbr_mode = en_set_baud ? 0 : 255;
 bts.btnp_mode = 0;
}

/** Builds AT+BAUDx command in the sender's buffer
 * \param baud Baud rate code (see uart.h file for more information)
 */
static void append_tx_buff_with_at_baud_cmd(uint16_t baud)
{
 uint8_t i = 0;
 uart_reset_send_buff();
 for(; i < 7; ++i) uart_append_send_buff(PGM_GET_BYTE(&AT_BAUD[i]));
 if (baud == CBR_9600) uart_append_send_buff('4');
 else if (baud == CBR_19200) uart_append_send_buff('5');
 else if (baud == CBR_38400) uart_append_send_buff('6');
 else if (baud == CBR_57600) uart_append_send_buff('7');
}

/** Increments SM state if timer expired */
void next_state_if_tmr_expired(void)
{
 if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
  ++bts.btbr_mode;
}

void next_state_with_new_baud(uint16_t baud)
{
 SET_BRR(baud);
 ++bts.btbr_mode;
 bts.strt_t1 = s_timer_gtc();    //set timer
}

uint8_t bt_set_baud(struct ecudata_t *d, uint16_t baud)
{
#ifdef _PLATFORM_M644_
 baud = convert_id_to_br(baud);
#endif

 if (255 == bts.btbr_mode)
  return 1; //not enabled/stopped

 switch(bts.btbr_mode)
 {
  //wait some time before we start to send first AT command
  case 0:
   ++bts.btbr_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   return 0;
  case 1:
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_STRT_TIME)
    ++bts.btbr_mode;
   return 0;

  //Send command on 9600 baud
  case 2:
   append_tx_buff_with_at_baud_cmd(baud);
   next_state_with_new_baud(CBR_9600);
   break;
  case 3:                          //wait some time
   next_state_if_tmr_expired();
   return 0;

  //Send command on 19200 baud
  case 4:
   if (!uart_is_sender_busy())
   {
    append_tx_buff_with_at_baud_cmd(baud);
    next_state_with_new_baud(CBR_19200);
    break;
   }
   else return 0;                  //busy
  case 5:                          //wait some time
   next_state_if_tmr_expired();
   return 0;

  //Send command on 38400 baud
  case 6:
   if (!uart_is_sender_busy())
   {
    append_tx_buff_with_at_baud_cmd(baud);
    next_state_with_new_baud(CBR_38400);
    break;
   }
   else return 0;                  //busy
  case 7:                          //wait some time
   next_state_if_tmr_expired();
   return 0;

  //Send command on 57600 baud
  case 8:
   if (!uart_is_sender_busy())
   {
    append_tx_buff_with_at_baud_cmd(baud);
    next_state_with_new_baud(CBR_57600);
    break;
   }
   else return 0;                  //busy
  case 9:                          //wait some time
   next_state_if_tmr_expired();
   return 0;

  //Finishing...
  case 10:
   if (!uart_is_sender_busy())
   {
    next_state_with_new_baud(baud);//return old baud rate back
    //reset flag and save parameters
    d->param.bt_flags&=~(1 << 1);
    sop_set_operation(SOP_SAVE_PARAMETERS);
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
 uint8_t i = 0;
 uart_reset_send_buff();
 for(; i < 7; ++i) uart_append_send_buff(PGM_GET_BYTE(&AT_NAME[i]));
 for(i = 0; i < name[0]; ++i) uart_append_send_buff(name[i+1]);
}

/** Builds AT+PINx command in the sender's buffer
 * \param name Buffer containing password, 1-st byte of buffer contains size of string
 */
static void append_tx_buff_with_at_pass_cmd(uint8_t* pass)
{
 uint8_t i = 0;
 uart_reset_send_buff();
 for(; i < 6; ++i) uart_append_send_buff(PGM_GET_BYTE(&AT_PIN[i]));
 for(i = 0; i < pass[0]; ++i) uart_append_send_buff(pass[i+1]);
}

uint8_t bt_set_namepass(struct ecudata_t *d)
{
 switch(bts.btnp_mode)
 {
  //wait some time before we start to send first AT command
  case 0:
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   return 0;
  case 1:
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btnp_mode;
   return 0;

  //Send command to change name
  case 2:
   append_tx_buff_with_at_name_cmd(d->bt_name);
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   break;                          //send!
  case 3:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btnp_mode;
   return 0;

  //Send command to change password (pin)
  case 4:
   append_tx_buff_with_at_pass_cmd(d->bt_pass);
   ++bts.btnp_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   break;                          //send!
  case 5:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
   {
    ++bts.btnp_mode;
    d->bt_name[0] = 0, d->bt_pass[0] = 0;
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
