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

/** \file procuart.c
 * \author Alexey A. Shabelnikov
 * Implementation of functionality for processing of pending data which is sent/received via serial interface (UART)
 */

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include <stdint.h>
#include "bitmask.h"
#include "bluetooth.h"
#include "camsens.h"
#include "ce_errors.h"
#include "ckps.h"
#include "diagnost.h"
#include "eeprom.h"
#include "ecudata.h"
#include "injector.h"
#include "ioconfig.h"
#include "knock.h"
#include "params.h"
#include "procuart.h"
#include "pwm2.h"
#include "pwrrelay.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "ventilator.h"
#include "vstimer.h"
#include "smcontrol.h"
#include "gdcontrol.h"
#include "wdt.h"

uint8_t startup_packets = 5;  //number of packets before start up bit will be cleared
uint8_t silent = 0;

/**UART processing timer. Used for sending packets*/
s_timer16_t send_packet_interval_counter = {0,0,1}; //initially fired

void process_uart_interface(void)
{
 uint8_t descriptor;

#ifdef BLUETOOTH_SUPP
 //Following code executes at start up only if bluetooth is enabled and only 1 time
 if (d.param.bt_flags & _BV(BTF_USE_BT))
 {
  if (!bt_set_baud(d.param.uart_divisor))
   return;

  if (d.bt_name[0] && d.bt_pass[0])
   if (!bt_set_namepass())
    return;
 }
#endif

 if (uart_get_packet()) //did we receive new data?
 {
  descriptor = uart_recept_packet();
  switch(descriptor)
  {
   case BOOTLOADER:
    while(!eeprom_is_idle()) { wdt_reset_timer(); }
#ifdef SPEED_SENSOR
    if (IOCFG_CHECK(IOP_PWRRELAY))
     sop_start_saving_odometer();   //save value of odometer to EEPROM
#endif
#ifdef FUEL_INJECT
    while(!eeprom_is_idle()) { wdt_reset_timer(); }
    if (IOCFG_CHECK(IOP_PWRRELAY))
     sop_start_saving_consfuel();   //save value of consumed fuel to EEPROM
#endif
#ifdef FUEL_INJECT
    if (PGM_GET_BYTE(&fw_data.exdata.ltft_mode) > 0) //is LTFT used?
    {
     while(!eeprom_is_idle()) { wdt_reset_timer(); }
     sop_start_saving_ltft();       //save values of LTFT tables (1 and 2)
    }
#endif
    pwrrelay_init_steppers();       //init steppers if necessary
    vent_turnoff();
    IOCFG_SETF(IOP_FL_PUMP, 0);     //turn off fuel pump
    IOCFG_SETF(IOP_ST_BLOCK, 0);    //block starter

    //transmitter is busy, it is necessary to wait when it become free and only after that start a boot loader
    while (uart_is_sender_busy()) { wdt_reset_timer(); }

    while(!eeprom_is_idle()) { wdt_reset_timer(); }

    //send confirmation that firmware is ready to start boot loader
    sop_send_gonna_bl_start();

    //if boot loader already has a "cli" instruction, then following line may be removed.
    _DISABLE_INTERRUPT();
    ckps_init_ports();
#ifdef FUEL_INJECT
    inject_init_ports();
#endif
    wdt_turnoff_timer();
    //jump to the boot loader's code skipping check of jumper's state
    boot_loader_start();
    break;

   case TEMPER_PAR:
    vent_set_pwmfrq(d.param.vent_pwmfrq);
   case CARBUR_PAR:
   case IDLREG_PAR:
   case ANGLES_PAR:
   case STARTR_PAR:
   case ADCCOR_PAR:
   case CHOKE_PAR:
#ifdef GD_CONTROL
   case GASDOSE_PAR:
#endif
    //reset timeout counter if parameters have been changed
    param_set_save_timer();
    break;

#ifdef FUEL_INJECT
   case INJCTR_PAR:
    //TODO: redundant code fragment
    inject_set_num_squirts(d.param.inj_config[d.sens.gas] & 0xF);  //number of squirts
    inject_set_config(d.param.inj_config[d.sens.gas] >> 4, CHECKBIT(d.param.inj_flags, INJFLG_SECINJROWSWT));//type of injection
#if defined(PHASE_SENSOR) && !defined(PHASED_IGNITION)
    cams_enable_cam(
#ifdef FUEL_INJECT
    (d.param.inj_config[d.sens.gas] >> 4) == INJCFG_FULLSEQUENTIAL ||
#endif
    CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF));
#endif
   case ACCEL_PAR:
    //reset timeout counter if parameters have been changed
    param_set_save_timer();
    break;
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   case LAMBDA_PAR:
    //paramaters were altered, so reset time counter
    param_set_save_timer();
    break;
#endif

   case MISCEL_PAR:
#ifdef HALL_OUTPUT
    ckps_set_hall_pulse(d.param.hop_start_ang, d.param.hop_durat_ang);
#endif
    param_set_save_timer(); //paramaters were altered, so reset time counter
    pwm2_set_pwmfrq(0, d.param.pwmfrq[0]);
    pwm2_set_pwmfrq(1, d.param.pwmfrq[1]);
    break;

   case FUNSET_PAR:
    param_set_save_timer(); //paramaters were altered, so reset time counter
    break;

   case OP_COMP_NC:
    if (_AB(d.op_actn_code, 0) == OPCODE_EEPROM_PARAM_SAVE) //We have received command to save parameters
    {
     sop_set_operation(SOP_SAVE_PARAMETERS);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
    if (_AB(d.op_actn_code, 0) == OPCODE_CE_SAVE_ERRORS) //We have received command to read saved error codes (CE)
    {
     sop_set_operation(SOP_READ_CE_ERRORS);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
    if (_AB(d.op_actn_code, 0) == OPCODE_READ_FW_SIG_INFO) //We have received command to read and transmit information about firmware
    {
     sop_set_operation(SOP_SEND_FW_SIG_INFO);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
#ifdef REALTIME_TABLES
    if (_AB(d.op_actn_code, 0) == OPCODE_LOAD_TABLSET) //We have received command to select new set of tables
    {
     sop_set_operation(SOP_LOAD_TABLSET);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
    if (_AB(d.op_actn_code, 0) == OPCODE_SAVE_TABLSET) //We have received command to save set of tables for specified type of fuel
    {
     sop_set_operation(SOP_SAVE_TABLSET);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
#endif
#ifdef DIAGNOSTICS
    if (_AB(d.op_actn_code, 0) == OPCODE_DIAGNOST_ENTER) //'enter diagnostic mode' command has been received
    {
     //this function will send confirmation answer and start diagnostic mode (it will has its own separate loop)
     diagnost_start();
     _AB(d.op_actn_code, 0) = 0; //processed
    }
    if (_AB(d.op_actn_code, 0) == OPCODE_DIAGNOST_LEAVE) //'leave diagnostic mode' command has been received
    {
     //this function will send confirmation answer and reset device
     diagnost_stop();
     _AB(d.op_actn_code, 0) = 0; //processed
    }
#endif
    if (_AB(d.op_actn_code, 0) == OPCODE_RESET_EEPROM) //reset EEPROM command received
    {
     if (_AB(d.op_actn_code, 1) == 0xAA) //second byte must be 0xAA
      sop_set_operation(SOP_SEND_NC_RESET_EEPROM);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
#ifdef FUEL_INJECT
    if (_AB(d.op_actn_code, 0) == OPCODE_RESET_LTFT) //LTFT reset command had been received
    {
     if (_AB(d.op_actn_code, 1) == 0xBB) //second byte must be 0xBB
      sop_set_operation(SOP_RESET_LTFT);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
    if (_AB(d.op_actn_code, 0) == OPCODE_SAVE_LTFT) //LTFT save command had been received
    {
     sop_set_operation(SOP_SAVE_LTFT);
     _AB(d.op_actn_code, 0) = 0; //processed
    }
#endif
    break;

   case CE_SAVED_ERR:
    sop_set_operation(SOP_SAVE_CE_ERRORS);
    break;

   case CKPS_PAR:
    //if CKPS parameters have been changed, then immidiately apply them on the working engine and reset counter of time
    ckps_set_cyl_number(d.param.ckps_engine_cyl);  //<-- obligatory first of all!
    ckps_set_cogs_num(d.param.ckps_cogs_num, d.param.ckps_miss_num);
    ckps_set_mttf(d.param.ckps_mttf);
    ckps_set_edge_type(CHECKBIT(d.param.hall_flags, CKPF_CKPS_EDGE));     //CKPS (CKP sensor)
    cams_vr_set_edge_type(CHECKBIT(d.param.hall_flags, CKPF_REFS_EDGE));  //REF_S (Reference sensor)
    ckps_set_cogs_btdc(d.param.ckps_cogs_btdc);
    ckps_set_merge_outs(CHECKBIT(d.param.hall_flags, CKPF_MERGE_OUTS));

#ifndef DWELL_CONTROL
    ckps_set_ignition_cogs(d.param.ckps_ignit_cogs);
#else
    ckps_set_rising_spark(CHECKBIT(d.param.hall_flags, CKPF_RISING_SPARK));
#endif
    param_set_save_timer(); //paramaters were altered, so reset time counter

#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
    ckps_set_shutter_wnd_width(d.param.hall_wnd_width);
    ckps_set_degrees_btdc(d.param.hall_degrees_btdc);
#endif

#ifdef FUEL_INJECT
    if (inject_set_cyl_number(d.param.ckps_engine_cyl) != d.param.ckps_engine_cyl)
    {//reset inj. config and number of squirts to default values because they may become invalid for new number of cylinders
     d.param.inj_config[0] = (INJCFG_SIMULTANEOUS << 4) | d.param.ckps_engine_cyl; //petrol: simultaneous, num. of squirts = num. of cylinders
     d.param.inj_config[1] = (INJCFG_SIMULTANEOUS << 4) | d.param.ckps_engine_cyl; //gas
     inject_set_num_squirts(d.param.inj_config[d.sens.gas] & 0xF);  //number of squirts
     inject_set_config(d.param.inj_config[d.sens.gas] >> 4, CHECKBIT(d.param.inj_flags, INJFLG_SECINJROWSWT));//type of injection
    }
#endif

#ifdef PHASE_SENSOR
    ckps_use_cam_ref_s(CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF) && !d.param.ckps_miss_num);
#endif

#if defined(PHASE_SENSOR) && !defined(PHASED_IGNITION)
    cams_enable_cam(
#ifdef FUEL_INJECT
    (d.param.inj_config[d.sens.gas] >> 4) == INJCFG_FULLSEQUENTIAL ||
#endif
    CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF));
#endif
    break;

   case KNOCK_PAR:
    //the same is for knock control, obligatory after CKPS_PAR!
    //initialize knock IC only if user has turned on control of knock in the parameters (avoid multiple time initialization)
    if (d.param.knock_use_knock_channel)
    {
     if (!d.use_knock_channel_prev)
     {
      if (!knock_module_initialize())
      {//knock IC is not working - light up CE
       ce_set_error(ECUERROR_KSP_CHIP_FAILED);
      }
     }
    }
    else //if knock detection has turned off then reset possible error
     ce_clear_error(ECUERROR_KNOCK_DETECTED);

    knock_set_band_pass(d.param.knock_bpf_frequency);
    //gain is set in each working stroke (see main loop)
    knock_set_int_time_constant(d.param.knock_int_time_const);
    ckps_set_knock_window(d.param.knock_k_wnd_begin_angle, d.param.knock_k_wnd_end_angle);
    ckps_use_knock_channel(d.param.knock_use_knock_channel);
    ckps_set_knock_chanmap(d.param.knock_selch);

    //remember state of the flag so later it will be possible to check if it is need to initialize knock IC or not
    d.use_knock_channel_prev = d.param.knock_use_knock_channel;

    param_set_save_timer(); //paramaters were altered, so reset time counter
    break;

   case SECUR_PAR:
#ifdef BLUETOOTH_SUPP
    if (d.bt_name[0] && d.bt_pass[0])
     bt_start_set_namepass();
#endif
    param_set_save_timer(); //paramaters were altered, so reset time counter
    break;
  }
 }

 //periodically send frames with data
 if (!uart_is_sender_busy())
 {
  uint8_t desc = uart_get_send_mode();
  if (desc == SILENT && !silent)
  {
   uart_transmitter(0); //turn off transmitter
   silent = 1;
  }
  if (desc != SILENT && silent)
  {
   uart_transmitter(1); //turn on transmitter
   silent = 0;
  }

  if (s_timer_is_action(&send_packet_interval_counter) && !silent)
  {
   //----------------------------------
   if (startup_packets > 0)
    --startup_packets;
   else
    ce_clear_error(ECUERROR_SYS_START);
   //----------------------------------

   uart_send_packet(0);                  //the transmitter become busy now

#ifdef DEBUG_VARIABLES
   if (SENSOR_DAT==desc || ADCRAW_DAT==desc || CE_ERR_CODES==desc || DIAGINP_DAT==desc)
    sop_set_operation(SOP_DBGVAR_SENDING); //additionally we will send packet with debug information
#endif

   s_timer_set(&send_packet_interval_counter, d.param.uart_period_t_ms);

   //clear cache of errors after transmitting them, transmission of error bits performed only in 1 of 2 packets
   if (SENSOR_DAT==desc || CE_ERR_CODES==desc)
    d.ecuerrors_for_transfer = 0;
  }
 }
}
