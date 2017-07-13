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

#ifdef OBD_SUPPORT

#include "port/avrio.h"
#include "port/port.h"
#include "ecudata.h"
#include "vstimer.h"
#include "knock.h"
#include "obd.h"

#define OBD_SEND_PERIOD 10   //!< 100ms

typedef struct
{
 can_t msg;            //!< CAN message
 uint16_t send_tmr;    //!< send timer
}obd_state_t;

/***/
obd_state_t obd;

void obd_init(void)
{
 obd.send_tmr = s_timer_gtc();
}

void obd_process(struct ecudata_t* d)
{
 if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD)
 {
  obd.send_tmr = s_timer_gtc();

  obd.msg.id = 0x180;   //Engine RPM
  obd.msg.flags.rtr = 0;
  obd.msg.length = 8;
  obd.msg.data[0] = d->sens.frequen >> 5; // rpm / 32
  obd.msg.data[1] = 0x00;
  obd.msg.data[2] = 0x00;
  obd.msg.data[3] = 0x00;
  obd.msg.data[4] = 0x00;
  obd.msg.data[5] = 0x00;
  obd.msg.data[6] = 0x00;
  obd.msg.data[7] = 0x00;

  knock_push_can_message(&obd.msg);
 }
}

#endif //OBD_SUPPORT
