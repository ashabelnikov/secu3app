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
 * Implementation of bluetooth related functionality and logic (baud rate setting, name, password)
 * (Реализация логики связанной с блютузом (установка скорости, имя, пароль))
 */

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/port.h"
#include "bluetooth.h"
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

//External functions (fro uart.c)
void uart_reset_send_buff(void);
void uart_append_send_buff(uint8_t ch);
void uart_begin_send(void);

/**Template for AT+BAUDx command */
PGM_DECLARE(uint8_t AT_BAUD[]) = "AT+BAUD";

/**Template for AT+NAMEx command */
PGM_DECLARE(uint8_t AT_NAME[]) = "AT+NAME";

/**Template for AT+PINx command */
PGM_DECLARE(uint8_t AT_PIN[])  = "AT+PIN";

/**Define internal state variable */
typedef struct
{
 uint8_t btbr_mode;                     //!< finite state machine state for setting of BT baud rate
 uint16_t strt_t1;                      //!< used for time calculations
}bts_t;

/**Instance of internal state variables */
bts_t bts;

void bt_init(void)
{
 bts.btbr_mode = 0;
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

uint8_t bt_set_baud(uint16_t baud)
{
 switch(bts.btbr_mode)
 {
  case 0:
   ++bts.btbr_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   return 0;
  case 1:
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_STRT_TIME)
    ++bts.btbr_mode;
   return 0;

  case 2:
   SET_BRR(CBR_9600);
   append_tx_buff_with_at_baud_cmd(baud);
   ++bts.btbr_mode;
   bts.strt_t1 = s_timer_gtc();    //set timer
   break;
  case 3:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btbr_mode;
   return 0;

  case 4:
   if (!uart_is_sender_busy())
   {
    SET_BRR(CBR_19200);
    append_tx_buff_with_at_baud_cmd(baud);
    ++bts.btbr_mode;
    bts.strt_t1 = s_timer_gtc();   //set timer
    break;
   }
   else return 0;                  //busy
  case 5:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btbr_mode;
   return 0;

  case 6:
   if (!uart_is_sender_busy())
   {
    SET_BRR(CBR_38400);
    append_tx_buff_with_at_baud_cmd(baud);
    ++bts.btbr_mode;
    bts.strt_t1 = s_timer_gtc();   //set timer
    break;
   }
   else return 0;                  //busy
  case 7:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btbr_mode;
   return 0;


  case 8:
   if (!uart_is_sender_busy())
   {
    SET_BRR(CBR_57600);
    append_tx_buff_with_at_baud_cmd(baud);
    ++bts.btbr_mode;
    bts.strt_t1 = s_timer_gtc();   //set timer
    break;
   }
   else return 0;                  //busy
  case 9:                          //wait some time
   if ((s_timer_gtc() - bts.strt_t1) >= AT_COMMAND_TIME)
    ++bts.btbr_mode;
   return 0;

  case 10:
   if (!uart_is_sender_busy())
   {
    SET_BRR(baud);                 //return old baud rate back
    ++bts.btbr_mode;
    break;
   }
   else return 0;                  //busy

  default:
   return 1; //stopped
 }

 //Start background process of packet sending
 uart_begin_send();
 return 0;
}
