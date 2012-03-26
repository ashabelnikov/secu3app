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

/** \file diagnost.c
 * Implementation of hardware diagnostics
 * (Реализация диагностики аппаратной части)
 */

#ifdef DIAGNOSTICS

#include "port/avrio.h"
#include "bitmask.h"
#include "diagnost.h"
#include "procuart.h"
#include "secu3.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "wdt.h"

uint8_t diag_started = 0;

void diagnost_start(void)
{
 diag_started = 1;
 uart_set_send_mode(DIAGINP_DAT);
}

void diagnost_stop(void)
{
 sop_set_operation(SOP_SEND_NC_LEAVE_DIAG);
}

void diagnost_process(struct ecudata_t* d)
{
 if (0==diag_started)
  return; //normal mode

 //We are in diagnostic mode
 sop_set_operation(SOP_SEND_NC_ENTER_DIAG);

 //Diasable unneeded interrupts
 TIMSK&=~(_BV(OCIE2)|_BV(TICIE1)|_BV(OCIE1A)|_BV(OCIE1B)|_BV(TOIE1)|_BV(OCIE0)|_BV(TOIE0));
 
 //local loop
 while(1)
 {
  //check & execute suspended operations
  sop_execute_operations(d);
  //process data being received and sent via serial port
  process_uart_interface(d); 

  wdt_reset_timer();
 }
}

#endif //DIAGNOSTICS
