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
#include "bitmask.h"
#include "ecudata.h"
#include "vstimer.h"
#include "knock.h"
#include "obd.h"
#include "magnitude.h"

#define OBD_SEND_PERIOD      10    //!< 100ms
#define OBD_SEND_PERIOD_MSG   2    //!< 20ms

/**Describe state variables*/
typedef struct
{
 can_t msg;            //!< CAN message
 uint16_t send_tmr;    //!< send timer
 uint8_t state;
}obd_state_t;

/**State variables*/
obd_state_t obd;

void obd_init(void)
{
 obd.send_tmr = s_timer_gtc();
 obd.state = 0;
}

#ifdef SPEED_SENSOR
/** Calculate vehicle speed (km/h)
 * \return vehicle speed, value * 32
 */
static uint16_t calc_speed(struct ecudata_t* d)
{
 if (d->sens.speed == 0xFFFF || d->sens.speed == 0)
  return 0; //return 0 if speed is too low (overflow) or SPD_SENS is not mapped to real I/O
 //TODO: use 1/speed approximation instead of division, so division will be replaced by multiplication
 return ((((uint32_t)d->param.vss_period_dist) * 1098) / d->sens.speed);   //V = (period_dist * 1125000) / period_tics, 112500 = (3600 * 312500) / 1000
}
#endif

void obd_process(struct ecudata_t* d)
{
 switch(obd.state)
 {
  case 0:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD)
   { //each 100ms
    obd.send_tmr = s_timer_gtc();
    obd.msg.id = 0x180;   //Engine RPM
    obd.msg.flags.rtr = 0;
    obd.msg.length = 8;
    obd.msg.data[0] = ((d->sens.frequen > 8100) ? 8100 : d->sens.frequen) >> 5; // limit to 8100, rpm / 32
    obd.msg.data[1] = 0x00;
    obd.msg.data[2] = 0x00;
    obd.msg.data[3] = 0x00;
    obd.msg.data[4] = 0x00;
    obd.msg.data[5] = 0x00;
    obd.msg.data[6] = 0x00;
    obd.msg.data[7] = 0x00;
    knock_push_can_message(&obd.msg);
    ++obd.state;
   }
   break;

  case 1:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD_MSG)
   {
    obd.msg.id = 0x1F9;   //Vehicle speed
    obd.msg.flags.rtr = 0;
    obd.msg.length = 8;
    obd.msg.data[0] = 0x00;
    obd.msg.data[1] = 0x00;
#ifdef SPEED_SENSOR
    obd.msg.data[2] = (((uint32_t)((calc_speed(d) > 240*32) ? 240*32 : calc_speed(d))) * 410) >> 15; // divide by 2.5, so 1 discrete = 2.5kmH, 0.4 * 1024 = 410
#else
    obd.msg.data[2] = 0;  //no speed sensor
#endif
    obd.msg.data[3] = 0x00;
    obd.msg.data[4] = 0x00;
    obd.msg.data[5] = 0x00;
    obd.msg.data[6] = 0x00;
    obd.msg.data[7] = 0x00;
    knock_push_can_message(&obd.msg);
    ++obd.state;
   }
   break;

  case 2:
   if ((s_timer_gtc() - obd.send_tmr) >= (OBD_SEND_PERIOD_MSG*2))
   {
    uint8_t FAILS = 0, BATT = _BV(0);
    WRITEBIT(FAILS, 0, d->ce_state); //CE lamp control
    WRITEBIT(FAILS, 3, d->sens.temperat > TEMPERATURE_MAGNITUDE(110.0)); //engine overheat lamp control

#ifndef SECU3T //SECU-3i
    WRITEBIT(FAILS, 2, !d->sens.oilpress_ok); //oil pressure failure
    WRITEBIT(BATT, 0, !d->sens.generator_ok); //dynamo generator failure
#endif

    obd.msg.id = 0x551;
    obd.msg.flags.rtr = 0;
    obd.msg.length = 8;
    obd.msg.data[0] = 0x00;
    obd.msg.data[1] = 0x00;
    obd.msg.data[2] = 0x00;
    obd.msg.data[3] = BATT;
    obd.msg.data[4] = FAILS;
    obd.msg.data[5] = 0x00; // ÑOUNTER
    obd.msg.data[6] = 0x00;
    obd.msg.data[7] = 0x00;
    knock_push_can_message(&obd.msg);
    obd.state = 0;
   }
   break;
 }
}

#endif //OBD_SUPPORT
