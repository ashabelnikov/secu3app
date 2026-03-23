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
 *  Implementation of communication via CAN bus with different devices (dash boards, AT/AMT etc)
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

#define OBD_SEND_PERIOD      10    //!< each 100ms send data to dash boards (several successive messages)
#define OBD_SEND_PERIOD_MSG   3    //!< 30ms between successive messages sent to dashboards

//AMT
#ifdef FUEL_INJECT
#define RPMREQSTEP PGM_GET_BYTE(&fw_data.exdata.amt_rpmalt_step)  //!< RPM increasing/decreasing step
#endif

//forward declarations
#ifdef FUEL_INJECT
void obd_at_lada_vesta();           //AMT
#endif
void obd_db_lada_priora();
void obd_db_nissan_almera_classic();

/**Describe state variables*/
typedef struct
{
 can_t msg_tx;         //!< CAN message (TX)
 can_t msg_rx;         //!< CAN message (RX)
 uint16_t send_tmr;    //!< send timer (dash board)
//uint16_t recv_tmr;   //!< recv timer (AMT)
 uint8_t state;        //!< state for dash boards
 //AMT
#ifdef FUEL_INJECT
 uint8_t amt_state[2]; //!< state for AMTs
 uint16_t amt_send_tmr[2];//!< send timer (AMT)
 int16_t torq_val;     //!< received requested torque
 uint8_t rpm_state;    //!< state for RPM increasing
 uint16_t rpm_tmr;     //!< timer for RPM increasing
 uint8_t initialized;  //!< initialization flag
#endif
}obd_state_t;

/**State variables*/
obd_state_t obd = {{0},{0},0/*,0*/,0
//AMT
#ifdef FUEL_INJECT
,{0},{0},0,0,0,0
#endif
};

void obd_init(void)
{
 obd.send_tmr = 
#ifdef FUEL_INJECT
 obd.amt_send_tmr[0] = 
 obd.amt_send_tmr[1] = 
#endif
 s_timer_gtc();
}

//called from the knock_expander_initialize(), see knock.c for more information
void obd_init_filters(void)
{
#ifdef FUEL_INJECT
 if (1==PGM_GET_BYTE(&fw_data.exdata.can_autrm))
 {
  //use buffer 0 - it will receive 0x17A only
  knock_set_can_filter(0, 0x17A);
  knock_set_can_mask(0, 0x1FF);
  //use buffer 1 - it will receive 0x17E only
  knock_set_can_filter(2, 0x17E);
  knock_set_can_mask(1, 0x1FF);
 }
#endif
}

//called from the main loop
void obd_process(void)
{
 //supported automatic transmissions (AT/AMT). Your AT may be here.
#ifdef FUEL_INJECT
 if (1==PGM_GET_BYTE(&fw_data.exdata.can_autrm))
 {
//if ((s_timer_gtc() - obd.recv_tmr) >= 1)
//{ //each 10ms
// obd.recv_tmr = s_timer_gtc();
   knock_check_can_message();   //set read request
//}

  obd_at_lada_vesta(); //process AMT "Lada Vesta"
 }
#endif

 //supported dashboards. Your dashboard may be here.
 if (PGM_GET_BYTE(&fw_data.exdata.can_dashboard) > 0)
 {
  if (1==PGM_GET_BYTE(&fw_data.exdata.can_dashboard)) //Lada Priora
   obd_db_lada_priora();
  else if (2==PGM_GET_BYTE(&fw_data.exdata.can_dashboard)) //Nissan Almera Classic
   obd_db_nissan_almera_classic();
 }
}

#ifdef FUEL_INJECT
/**AMT: Lada Vesta*/
void obd_at_lada_vesta(void)
{
 if (d.engine_mode==EM_START)
 {//cranking
  d.corr.amt_igntim = 0; //reset ign. timing correction (don't use this correction on cranking)
  d.amt_req_rpm = 0;     //no min. RPM request to IAC
  obd.rpm_state = 0;     //no increasing RPM
 }

 if (knock_get_can_message(&obd.msg_rx))  //check for pending message and read it from the CAN controller (if exist)
 {
  if (obd.msg_rx.id == 0x17A)
  { //save received requested torque value
   obd.torq_val = (((uint16_t)obd.msg_rx.data[0]) << 4) | (obd.msg_rx.data[1] >> 4);
   obd.torq_val-= (400*2);
   //control fuel cut off:
   int16_t c_torque = calc_torque();
   int16_t torque = c_torque - felcut_torque();
   int16_t min_torque = calc_minmax_torque(0)+(1*2); //+1Nm
   if (torque < min_torque)
    torque = min_torque;
   if (obd.torq_val < torque && d.sens.rpm > calc_cl_rpm() && d.sens.carb)
    d.amt_fuelcut = 0; //turn of fuel
   else
    d.amt_fuelcut = 1; //turn on fuel
   //update ignition timing correction:
   d.corr.amt_igntim = dtorq_igntim_corr(obd.torq_val - c_torque); //correction depends on the diff. between requested torque and calculated torque
   obd.initialized = 1;
  }
  else if (obd.msg_rx.id == 0x17E)
  {
   d.amt_locked = obd.msg_rx.data[5] != 0x40; //if data[5] == 0x40, then engine start is enabled, otherwise - disabled
   //increase RPM if necessary
   switch(obd.rpm_state)
   {
    case 0: //waiting for request to appear
     if (obd.msg_rx.data[2]!=0)
     {
      if (d.sens.rpm < PGM_GET_WORD(&fw_data.exdata.amt_creeping_minrpm))
      { //below
       d.amt_req_rpm = d.sens.rpm;  //start from current RPM
       ++obd.rpm_state;
      }
      else
      {//ok
       d.amt_req_rpm = PGM_GET_WORD(&fw_data.exdata.amt_creeping_minrpm); //specify minimum RPM, see calc_cl_rpm() in funconv.c for more information
       obd.rpm_state = 2;
      }
     }
     break;

    case 1: //smoothly increase RPM if it is less than required
     if (d.amt_req_rpm >= PGM_GET_WORD(&fw_data.exdata.amt_creeping_minrpm))
      ++obd.rpm_state;
     //and fall into code of the next state:

    case 2: //waiting for the request to be withdrawn
     obd.rpm_tmr = s_timer_gtc();
     if (obd.msg_rx.data[2]==0)
      obd.rpm_state = 0;
     break;
   }
  }
 }

 //send high priority messages to AMT
 switch(obd.amt_state[0])
 {
  case 0:
   if ((s_timer_gtc() - obd.amt_send_tmr[0]) >= 2 && knock_is_idle_can_tx())
   { //send 0x186 first
    if (obd.initialized) {
    obd.amt_send_tmr[0] = s_timer_gtc();
    obd.msg_tx.id = 0x186;
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    uint16_t rpm = d.sens.rpm;
    if (rpm > 8190)
     rpm = 8190;   //prevent overflow
    rpm*=8;   //x8
    obd.msg_tx.data[0] = rpm >> 8;     //hi
    obd.msg_tx.data[1] = rpm & 0xFF;   //lo
    int16_t eft = obd.torq_val;  
    restrict_value_to(&eft, calc_minmax_torque(0), calc_minmax_torque(1)); //restrict received torque_val
    eft+=(400*2);
    obd.msg_tx.data[2] = eft >> 4;
    obd.msg_tx.data[3] = (eft << 4) & 0xF0;
    obd.msg_tx.data[4] = 0;
    obd.msg_tx.data[5] = d.sens.tps / 32; //% * 2
    obd.msg_tx.data[6] = 0x24;
    obd.msg_tx.data[7] = 0;
    knock_push_can_message(&obd.msg_tx);}
    ++obd.amt_state[0];
   }
   break;

  case 1:
   if ((s_timer_gtc() - obd.amt_send_tmr[0]) >= 1 && knock_is_idle_can_tx())
   { //send 0x189 right after 0x186
    obd.msg_tx.id = 0x189;
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    int16_t torque = calc_torque();
    torque+=(400*2);
    obd.msg_tx.data[0] = torque >> 4;
    obd.msg_tx.data[1] = (torque << 4) & 0xF0;
    torque = calc_minmax_torque(0);
    torque+=(400*2);
    obd.msg_tx.data[1]|=torque >> 8;          //min
    obd.msg_tx.data[2] = torque & 0xFF;       //min
    torque = calc_minmax_torque(1);
    torque+=(400*2);
    obd.msg_tx.data[3] = torque >> 4;         //max
    obd.msg_tx.data[4] = (torque << 4) & 0xF0;//max
    uint16_t bp = (0==PGM_GET_WORD(&fw_data.exdata.amt_baro_press)) ? d.sens.baro_press : PGM_GET_WORD(&fw_data.exdata.amt_baro_press);
    restrict_value_to_u(&bp, 60*64, 111*64); //restrict in range 60...111 kPa
    bp-=(60*64);
    bp = (bp * 20) >> 8;   //divide by 12.8, because 64 / 12.8 = 5; 1/12.8 = 0.078125; 0.078125*256 = 20
    obd.msg_tx.data[5] = bp; //(kpa - 60) * 5
    obd.msg_tx.data[6] = 0;
    obd.msg_tx.data[7] = 0;
    knock_push_can_message(&obd.msg_tx);
    obd.amt_state[0] = 0;
   }
   break;
 }

 //send low priority messages to AMT each 100ms
 switch(obd.amt_state[1])
 {
  case 0:
   if ((s_timer_gtc() - obd.amt_send_tmr[1]) >= 10 && knock_is_idle_can_tx())
   { //each 100ms
    obd.amt_send_tmr[1] = s_timer_gtc();
    obd.msg_tx.id = 0x5DA;
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    int16_t clt = (d.sens.temperat / 4) + 40;
    restrict_value_to(&clt, 0, 255);    
    obd.msg_tx.data[0] = clt; //engine temperature (coolant tempterature)
    int16_t itrpm = calc_cl_rpm();
    restrict_value_to(&itrpm, 0, 2040);
    obd.msg_tx.data[1] = itrpm/8;  //target idling RPM
    obd.msg_tx.data[2] = 0;
    obd.msg_tx.data[3] = 0;
    int16_t fc_lot = get_fc_lot() / 32;
    restrict_value_to(&fc_lot, 0, 255);
    obd.msg_tx.data[4] = fc_lot;   //cut off RPM (forced idle)
    obd.msg_tx.data[5] = 0;
    obd.msg_tx.data[6] = 0;
    obd.msg_tx.data[7] = 0;
    knock_push_can_message(&obd.msg_tx);
    ++obd.amt_state[1];
   }
   break;

  case 1:
   if ((s_timer_gtc() - obd.amt_send_tmr[1]) >= 5 && knock_is_idle_can_tx())
   {//50ms
    obd.msg_tx.id = 0x65C;
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    int16_t iat = 0;
#ifdef AIRTEMP_SENS
    iat = (d.sens.air_temp / 2) + (40*2);
    restrict_value_to(&iat, 0, 255);    
#endif
    obd.msg_tx.data[0] = iat; //Intake air temperature
    obd.msg_tx.data[1] = 0;
    obd.msg_tx.data[2] = 0;
    obd.msg_tx.data[3] = 0;
    obd.msg_tx.data[4] = 0;
    obd.msg_tx.data[5] = 0;
    obd.msg_tx.data[6] = 0;
    obd.msg_tx.data[7] = 0;
    knock_push_can_message(&obd.msg_tx);
    obd.amt_state[1] = 0;
   }
   break;
 }
}
#endif

/**Dashboard: Lada Priora*/
void obd_db_lada_priora(void)
{
 switch(obd.state)
 {
  case 0:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD && knock_is_idle_can_tx())
   { //each 100ms
    obd.send_tmr = s_timer_gtc();
    obd.msg_tx.id = 0x180;   //Engine RPM
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    obd.msg_tx.data[0] = ((d.sens.rpm > 8100) ? 8100 : d.sens.rpm) >> 5; // limit to 8100, rpm / 32
    obd.msg_tx.data[1] = 0x00;
    obd.msg_tx.data[2] = 0x00;
    obd.msg_tx.data[3] = 0x00;
    obd.msg_tx.data[4] = 0x00;
    obd.msg_tx.data[5] = 0x00;
    obd.msg_tx.data[6] = 0x00;
    obd.msg_tx.data[7] = 0x00;
    knock_push_can_message(&obd.msg_tx);
    ++obd.state;
   }
   break;

  case 1:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD_MSG && knock_is_idle_can_tx())
   {
    obd.msg_tx.id = 0x1F9;   //Vehicle speed
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    obd.msg_tx.data[0] = 0x00;
    obd.msg_tx.data[1] = 0x00;
#ifdef SPEED_SENSOR
    obd.msg_tx.data[2] = (((uint32_t)((d.sens.vss_speed > VSSSPEED_MAG(240.0)) ? VSSSPEED_MAG(240.0) : d.sens.vss_speed)) * 410) >> 15; // divide by 2.5, so 1 discrete = 2.5kmH, 0.4 * 1024 = 410
#else
    obd.msg_tx.data[2] = 0;  //no speed sensor
#endif
    obd.msg_tx.data[3] = 0x00;
    obd.msg_tx.data[4] = 0x00;
    obd.msg_tx.data[5] = 0x00;
    obd.msg_tx.data[6] = 0x00;
    obd.msg_tx.data[7] = 0x00;
    knock_push_can_message(&obd.msg_tx);
    ++obd.state;
   }
   break;

  case 2:
   if ((s_timer_gtc() - obd.send_tmr) >= (OBD_SEND_PERIOD_MSG*2) && knock_is_idle_can_tx())
   {
    uint8_t FAILS = 0, BATT = _BV(0);
    WRITEBIT(FAILS, 0, d.ce_state); //CE lamp control
    WRITEBIT(FAILS, 3, d.sens.temperat > TEMPERATURE_MAGNITUDE(110.0)); //engine overheat lamp control

#ifndef SECU3T //SECU-3i
    WRITEBIT(FAILS, 2, !d.sens.oilpress_ok); //oil pressure failure
    WRITEBIT(BATT, 0, d.sens.generator_ok); //dynamo generator failure
#endif

    obd.msg_tx.id = 0x551;
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    obd.msg_tx.data[0] = 0x00;
    obd.msg_tx.data[1] = 0x00;
    obd.msg_tx.data[2] = 0x00;
    obd.msg_tx.data[3] = BATT;
    obd.msg_tx.data[4] = FAILS;
    obd.msg_tx.data[5] = 0x00; // COUNTER
    obd.msg_tx.data[6] = 0x00;
    obd.msg_tx.data[7] = 0x00;
    knock_push_can_message(&obd.msg_tx);
    obd.state = 0;
   }
   break;
 }
}

/**Dashboard: Nissan Almera Classic*/
void obd_db_nissan_almera_classic(void)
{
 switch(obd.state)
 {
  case 0:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD && knock_is_idle_can_tx())
   { //each 100ms
    obd.send_tmr = s_timer_gtc();
    obd.msg_tx.id = 505;   //Engine RPM
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    uint16_t rpm = d.sens.rpm; //+ 500;
    obd.msg_tx.data[0] = 0;
    obd.msg_tx.data[1] = 0;
    obd.msg_tx.data[2] = ((rpm > 8100) ? 8100 : rpm) >> 5; // limit to 8100, rpm / 32
    obd.msg_tx.data[3] = 0;
    obd.msg_tx.data[4] = 0;
    obd.msg_tx.data[5] = 0;
    obd.msg_tx.data[6] = 0;
    obd.msg_tx.data[7] = 0;
    knock_push_can_message(&obd.msg_tx);
    ++obd.state;
   }
   break;

  case 1:
   if ((s_timer_gtc() - obd.send_tmr) >= OBD_SEND_PERIOD_MSG && knock_is_idle_can_tx())
   {
    //map temperature range 10...115 to the range of 90...170
    int16_t tg = simple_interpolation(d.sens.temperat, 90, 170, TEMPERATURE_MAGNITUDE(10.0), TEMPERATURE_MAGNITUDE(105.0), 128) >> 7;
    restrict_value_to(&tg, 90, 170);
    obd.msg_tx.id = 1361;   //Coolant temperature
    obd.msg_tx.flags.rtr = 0;
    obd.msg_tx.length = 8;
    obd.msg_tx.data[0] = tg;    //temperature for gauge
    obd.msg_tx.data[1] = 0x00;
    obd.msg_tx.data[2] = 0x00;
    obd.msg_tx.data[3] = d.sens.temperat < TEMPERATURE_MAGNITUDE(90.0) ? 0 : tg; //temperature for "Check Engine" lamp
    obd.msg_tx.data[4] = 0x00;
    obd.msg_tx.data[5] = 0x00;
    obd.msg_tx.data[6] = 0x00;
    obd.msg_tx.data[7] = 0x00;
    knock_push_can_message(&obd.msg_tx);
    obd.state = 0;
   }
   break;
 }
}

void obd_stroke_event_notification(void)
{
#ifdef FUEL_INJECT
 //AMT
 if (obd.rpm_state == 1)
 {//smoothly increase RPM
  if (d.amt_req_rpm <= (PGM_GET_WORD(&fw_data.exdata.amt_creeping_minrpm) - RPMREQSTEP))
   d.amt_req_rpm+=RPMREQSTEP;
  else
   d.amt_req_rpm = PGM_GET_WORD(&fw_data.exdata.amt_creeping_minrpm);
 }
 else if ((s_timer_gtc() - obd.rpm_tmr) > PGM_GET_WORD(&fw_data.exdata.amt_creeping_delay) && obd.rpm_state == 0)
 {//smoothly decrease RPM
  if (d.amt_req_rpm >= RPMREQSTEP)
   d.amt_req_rpm-=RPMREQSTEP;
  else
   d.amt_req_rpm = 0;
 }
#endif
}

#endif //OBD_SUPPORT
