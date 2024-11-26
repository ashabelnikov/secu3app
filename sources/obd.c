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

/** \file obd.c
 * \author Alexey A. Shabelnikov
 * OBD support implementation
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
#include "funconv.h"

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
obd_state_t obd = {{0},0,0};

void obd_init(void)
{
 obd.send_tmr = s_timer_gtc();
}

void obd_process(void)
{
 if (0==PGM_GET_BYTE(&fw_data.exdata.can_dashboard))
  return; //off

 if (1==PGM_GET_BYTE(&fw_data.exdata.can_dashboard)) //Lada "Priora"
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
     obd.msg.data[0] = ((d.sens.rpm > 8100) ? 8100 : d.sens.rpm) >> 5; // limit to 8100, rpm / 32
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
     obd.msg.data[2] = (((uint32_t)((d.sens.vss_speed > VSSSPEED_MAG(240.0)) ? VSSSPEED_MAG(240.0) : d.sens.vss_speed)) * 410) >> 15; // divide by 2.5, so 1 discrete = 2.5kmH, 0.4 * 1024 = 410
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
     WRITEBIT(FAILS, 0, d.ce_state); //CE lamp control
     WRITEBIT(FAILS, 3, d.sens.temperat > TEMPERATURE_MAGNITUDE(110.0)); //engine overheat lamp control

#ifndef SECU3T //SECU-3i
     WRITEBIT(FAILS, 2, !d.sens.oilpress_ok); //oil pressure failure
     WRITEBIT(BATT, 0, d.sens.generator_ok); //dynamo generator failure
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
 else if (2==PGM_GET_BYTE(&fw_data.exdata.can_dashboard)) //Nissan Almera Classic
 {
  switch(obd.state)
  {
   case 0:
    if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD)
    { //each 100ms
     obd.send_tmr = s_timer_gtc();
     obd.msg.id = 505;   //Engine RPM
     obd.msg.flags.rtr = 0;
     obd.msg.length = 8;
     uint16_t rpm = d.sens.rpm + 500;     
     obd.msg.data[0] = 0;
     obd.msg.data[1] = 0;
     obd.msg.data[2] = ((rpm > 8100) ? 8100 : rpm) >> 5; // limit to 8100, rpm / 32
     obd.msg.data[3] = 0;
     obd.msg.data[4] = 0;
     obd.msg.data[5] = 0;
     obd.msg.data[6] = 0;
     obd.msg.data[7] = 0;
     knock_push_can_message(&obd.msg);
     ++obd.state;
    }
    break;

   case 1:
    if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD_MSG)
    {
     //map temperature range -40...120 to the range 90...170
     int16_t tg = simple_interpolation(d.sens.temperat, 90, 170, TEMPERATURE_MAGNITUDE(-40.0), TEMPERATURE_MAGNITUDE(160.0), 128) >> 7;
     restrict_value_to(&tg, 90, 170);
     obd.msg.id = 1361;   //Coolant temperature
     obd.msg.flags.rtr = 0;
     obd.msg.length = 8;
     obd.msg.data[0] = tg;    //temperature for gauge
     obd.msg.data[1] = 0x00;
     obd.msg.data[2] = 0x00;
     obd.msg.data[3] = tg;    //temperature for "Check Engine" lamp
     obd.msg.data[4] = 0x00;
     obd.msg.data[5] = 0x00;
     obd.msg.data[6] = 0x00;
     obd.msg.data[7] = 0x00;
     knock_push_can_message(&obd.msg);
     obd.state = 0;
    }
    break;
  }
 }
}

#endif //OBD_SUPPORT
