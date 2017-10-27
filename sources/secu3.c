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

/** \file secu3.c
 * \author Alexey A. Shabelnikov
 * Implementation of main module of the firmware.
 */

#include "port/avrio.h"
#include "port/intrinsic.h"
#include "port/port.h"

#include "adc.h"
#include "aircond.h"
#include "bc_input.h"
#include "bitmask.h"
#include "bluetooth.h"
#include "bootldr.h"
#include "camsens.h"
#include "carb_afr.h"
#include "ce_errors.h"
#include "choke.h"
#include "ckps.h"
#include "crc16.h"
#include "diagnost.h"
#include "ecudata.h"
#include "eculogic.h"
#include "eeprom.h"
#include "evap.h"
#include "gasdose.h"
#include "pwrvalve.h"
#include "fuelpump.h"
#include "funconv.h"
#include "jumper.h"
#include "fuelcut.h"
#include "immobiliz.h"
#include "injector.h"
#include "intkheat.h"
#include "ioconfig.h"
#include "knklogic.h"
#include "knock.h"
#include "lambda.h"
#include "magnitude.h"
#include "measure.h"
#include "mathemat.h"
#include "obd.h"
#include "params.h"
#include "procuart.h"
#include "pwrrelay.h"
#include "starter.h"
#include "suspendop.h"
#include "tables.h"
#include "uart.h"
#include "uni_out.h"
#include "ventilator.h"
#include "vstimer.h"
#include "wdt.h"

#define FORCE_MEASURE_TIMEOUT_VALUE   20    //!< timeout value used to perform measurements when engine is stopped
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
#define ENGINE_ROTATION_TIMEOUT_VALUE 150   //!< timeout value used to determine that engine is stopped (used for Hall sensor)
#else
#define ENGINE_ROTATION_TIMEOUT_VALUE 20    //!< timeout value used to determine that engine is stopped (this value must not exceed 25)
#endif

/**Control of certain units of engine (управление отдельными узлами двигателя).
 * Uses d ECU data structure
 */
void control_engine_units(void)
{
#if !defined(CARB_AFR) || defined(GD_CONTROL) //Carb. AFR control supersede idle cut-off functionality
 //Idle fuel cut-off control or fuel cut-off
 fuelcut_control();
#endif

 //Starter blocking control
 starter_control();

 //Control of electric cooling fan (only if CTS is present in the system)
 vent_control();

#ifndef CARB_AFR //Carb. AFR control supersede power valve functionality
 //Power valve control
 pwrvalve_control();
#endif

#ifdef FUEL_PUMP
 //Controlling of electric fuel pump
 fuelpump_control();
#endif

 //power management
 pwrrelay_control();

#if defined(SM_CONTROL) || defined(FUEL_INJECT)
 //choke control
 choke_control();
#endif

#if defined(GD_CONTROL)
 //gas dosator control
 gasdose_control();
#endif

 //Cam sensor control
 cams_control();

#ifdef INTK_HEATING
 intkheat_control();
#endif

#ifdef UNI_OUTPUT
 //Universal programmable output control
 uniout_control();
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 lambda_control();
#endif

#ifdef CARB_AFR
 //Carburetor AFR control
 carbafr_control();
#endif

#if defined(EVAP_CONTROL) && !defined(SECU3T)
 evap_control(); //canister purge valve control
#endif

#if defined(AIRCONDIT) && !defined(SECU3T)
 aircond_control(); //air conditioner control
#endif
}

/** Check firmware integrity (CRC) and set error indication if code or data is damaged
 */
void check_firmware_integrity(void)
{
 if (crc16f(0, CODE_SIZE)!=PGM_GET_WORD(&fw_data.code_crc))
  ce_set_error(ECUERROR_PROGRAM_CODE_BROKEN);
 if (crc16f(fwinfo, FWINFOSIZE)!=0x44DB)
  check_firmware_integrity(); //Uuups!
}

/**Initialization of I/O ports
 */
void init_ports(void)
{
 jumper_init_ports();  //<--must be first!
 ckps_init_ports();
 cams_init_ports();
 vent_init_ports();
#ifndef CARB_AFR //Carb. AFR control supersede power valve functionality
 pwrvalve_init_ports();
#endif
#ifdef FUEL_PUMP
 fuelpump_init_ports();
#endif
#if !defined(CARB_AFR) || defined(GD_CONTROL) //Carb. AFR control supersede idle cut-off functionality
 fuelcut_init_ports();
#endif
 starter_init_ports();
 ce_init_ports();
 knock_init_ports();
 pwrrelay_init_ports();
#if defined(SM_CONTROL) || defined(FUEL_INJECT)
 choke_init_ports();
#endif
#if defined(GD_CONTROL)
 gasdose_init_ports();
#endif
#ifdef INTK_HEATING
 intkheat_init_ports();
#endif
 meas_init_ports();
#ifdef UNI_OUTPUT
 uniout_init_ports();
#endif
#ifdef FUEL_INJECT
 inject_init_ports();
#endif
#ifdef CARB_AFR
 carbafr_init_ports();
#endif
#if defined(EVAP_CONTROL) && !defined(SECU3T)
 evap_init_ports();
#endif
#if defined(AIRCONDIT) && !defined(SECU3T)
 aircond_init_ports();
#endif
}

/** Checks for conditions activating engine blowing mode
 * \return 1 - engine blowing should be active, 0 - not active
 */
static uint8_t engine_blowing_cond(void)
{
 return (d.sens.tps > TPS_MAGNITUDE(70.0)) && (d.engine_mode == EM_START);
}

/**Initialization of system modules
 */
void init_modules(void)
{
 //preliminary initialization of the knock signal processor
 knock_set_band_pass(d.param.knock_bpf_frequency);
 knock_set_gain(PGM_GET_BYTE(&fw_data.exdata.attenuator_table[0]));
 knock_set_int_time_constant(d.param.knock_int_time_const);
 knock_set_channel(0);
 if (d.param.knock_use_knock_channel)
  if (!knock_module_initialize())
  {//knock signal processor failure - light up CE lamp
   ce_set_error(ECUERROR_KSP_CHIP_FAILED);
  }
 d.use_knock_channel_prev = d.param.knock_use_knock_channel;

#if !defined(SECU3T) || defined(OBD_SUPPORT) //---SECU-3i---
 knock_expander_initialize();
#endif

 //Initialization of ADC
 adc_init();

 //Take away of starter blocking
 starter_set_blocking_state(0);

 //Initialization of UART
 uart_init(d.param.uart_divisor);

#ifdef BLUETOOTH_SUPP
 //Initialization of Bluetooth related module. BT baud rate can't be set if user selected to use parameters from FLASH
 bt_init(CHECKBIT(d.param.bt_flags, BTF_SET_BBR) && !CHECKBIT(d.param.bt_flags, BTF_USE_RESPAR));
#endif

 //initialization of cam module, must precede ckps initialization
 cams_init_state();

#ifdef FUEL_PUMP
 //initialization of electric fuel pump
 fuelpump_init();
#endif

 //Initialize CKPS unit
 ckps_init_state();
 ckps_set_cyl_number(d.param.ckps_engine_cyl);
 ckps_set_cogs_num(d.param.ckps_cogs_num, d.param.ckps_miss_num);
 ckps_set_edge_type(d.param.ckps_edge_type);     //CKPS edge
 cams_vr_set_edge_type(d.param.ref_s_edge_type); //REF_S edge
 ckps_set_cogs_btdc(d.param.ckps_cogs_btdc); //<--only partial initialization
#ifndef DWELL_CONTROL
 ckps_set_ignition_cogs(d.param.ckps_ignit_cogs);
#else
 ckps_set_rising_spark(CHECKBIT(d.param.hall_flags, CKPF_RISING_SPARK));
#endif
 ckps_set_knock_window(d.param.knock_k_wnd_begin_angle,d.param.knock_k_wnd_end_angle);
 ckps_use_knock_channel(d.param.knock_use_knock_channel);
 ckps_set_cogs_btdc(d.param.ckps_cogs_btdc); //<--now valid initialization
 ckps_set_merge_outs(d.param.merge_ign_outs);
#ifdef HALL_OUTPUT
 ckps_set_hall_pulse(d.param.hop_start_cogs, d.param.hop_durat_cogs);
#endif
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
 ckps_set_shutter_wnd_width(d.param.hall_wnd_width);
 ckps_set_degrees_btdc(d.param.hall_degrees_btdc);
 ckps_set_advance_angle(0);
#endif
#ifdef PHASE_SENSOR
 ckps_use_cam_ref_s(CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF) && !d.param.ckps_miss_num);
#endif

#ifdef FUEL_INJECT
 ckps_set_inj_timing(d.param.inj_timing_crk, d.inj_pw, d.param.inj_anglespec); //use inj.timing on cranking
 inject_init_state();
 inject_set_cyl_number(d.param.ckps_engine_cyl);
 inject_set_num_squirts(d.param.inj_config & 0xF);
 inject_set_config(d.param.inj_config >> 4);
#if defined(PHASE_SENSOR) && !defined(PHASED_IGNITION)
 cams_enable_cam(
#ifdef FUEL_INJECT
 (d.param.inj_config >> 4) == INJCFG_FULLSEQUENTIAL ||
#endif
 CHECKBIT(d.param.hall_flags, CKPF_USE_CAM_REF));
#endif
#endif

#ifdef CARB_AFR
 carbafr_init();
#endif

#ifdef OBD_SUPPORT
 obd_init();
#endif

 s_timer_init();
 ignlogic_init();

 vent_init_state();
 vent_set_pwmfrq(d.param.vent_pwmfrq);

 //check and enter blink codes indication mode
 bc_indication_mode();

 //Initialization of the suspended operations module
 sop_init_operations();

 //Initialize measurement's unit and perform several measure cycles for setting of ring buffers to correct values
 meas_init();

#ifdef FUEL_INJECT
 //must be called after meas_init()
 inject_set_fuelcut(!d.sys_locked && !engine_blowing_cond());
#endif
}

/**Main function of firmware - entry point. Contains initialization and main loop 
 */
MAIN()
{
 retard_state_t retard_state; //knock control

 //We need this because we might been reset by WDT
 wdt_turnoff_timer();

 //initialize knock control logic
 knklogic_init(&retard_state);

 //Perform I/O ports configuration/initialization
 init_ports();

 //If firmware code is damaged then turn on CE
 check_firmware_integrity();

 //Start watchdog timer!
 wdt_start_timer();

 //Read all system parameters
 load_eeprom_params();

#ifdef IMMOBILIZER
 //If enabled, reads and checks security keys, performs system lock/unlock
 immob_check_state();
#endif

#ifdef REALTIME_TABLES
 //load tables' set from EEPROM into RAM
 load_specified_tables_into_ram(TABLES_NUMBER - 1);
#endif

 //perform initialization of all system modules
 init_modules();

 //Enable all interrupts globally before we fall in main loop
 _ENABLE_INTERRUPT();

 //------------------------------------------------------------------------
 while(1)
 {
  if (ckps_is_cog_changed())
  {
   s_timer_set(engine_rotation_timeout_counter, ENGINE_ROTATION_TIMEOUT_VALUE);
   ignlogic_cog_changed_notification();
   #ifdef INTK_HEATING
   intkheat_cog_changed_notification();
   #endif
  }

  if (s_timer_is_action(engine_rotation_timeout_counter))
  { //engine is stopped (RPM is below critical threshold)
#ifdef DWELL_CONTROL
   ckps_init_ports();           //prevent permanent current through coils
   //TODO: Make soft cutoff of possible active current in coil to eliminate undesirable spark. How?
#endif
   ckps_init_state_variables();
   cams_init_state_variables();
   ignlogic_eng_stopped_notification(); //set cranking mode

   knklogic_init(&retard_state);

   if (d.param.knock_use_knock_channel)
    knock_start_settings_latching();

   meas_update_values_buffers(1, &fw_data.exdata.cesd);  //<-- update RPM only
   s_timer_set(engine_rotation_timeout_counter, ENGINE_ROTATION_TIMEOUT_VALUE);
  }

  //Start ADC measurements at regular intervals of time. This timer reinitialize each time of detecting of new stroke.
  //Thus, when RPM exceed spcified value, this condition will cease to be carried out.
  if (s_timer_is_action(force_measure_timeout_counter))
  {
   _DISABLE_INTERRUPT();
   adc_begin_measure(0);  //normal speed
   _ENABLE_INTERRUPT();
   s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
   meas_update_values_buffers(0, &fw_data.exdata.cesd);
  }

  //----------continious execution-----------------------------------------
  //process and execute suspended operations
  sop_execute_operations();
  //Detection and recording of errors (checking engine)
  ce_check_engine(&ce_control_time_counter);
  //processing of ingoing and outgoing data via UART
  process_uart_interface();
  //detection of changes in parameters and its saving
  save_param_if_need();
  //calculation of instant RPM
  d.sens.inst_frq = ckps_calculate_instant_freq();
  //averaging of phisical magnitudes stored in the circular buffers
  meas_average_measured_values(&fw_data.exdata.cesd);
  //read discrete inputs of the system and switching of fuel type (sets of maps)
  meas_take_discrete_inputs();
  //control peripheral devices (actuators)
  control_engine_units();
  //System's state machine core (dispatcher of modes)
  ignlogic_system_state_machine();

#ifdef DWELL_CONTROL
#if defined(HALL_SYNC) || defined(CKPS_NPLUS1)
  //Double dwell time if RPM is low and non-stable
  ckps_set_acc_time(d.st_block ? accumulation_time() : accumulation_time() << 1);
#else
  //calculate and update accumulation time (dwell control)
  ckps_set_acc_time(accumulation_time());
#endif
#endif
  if (d.sys_locked)
   ckps_enable_ignition(0);
  else
  {
   //Если разрешено, то делаем отсечку зажигания при превышении определенных оборотов
   if (d.param.ign_cutoff)
    ckps_enable_ignition(d.sens.inst_frq < d.param.ign_cutoff_thrd);
   else
    ckps_enable_ignition(1);
  }

  //------------------------------------------------------------------------

  //execute some operations, which require execution one time per engine stroke
  if (ckps_is_stroke_event_r())
  {
   meas_update_values_buffers(0, &fw_data.exdata.cesd);
   s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);

   ignlogic_stroke_event_notification();

   ce_stroke_event_notification();

#ifdef FUEL_INJECT
#ifdef GD_CONTROL
   //enable/disable fuel supply depending on fuel cut, rev.lim, sys.lock flags. Also fuel supply will be disabled if fuel type is gas and gas doser is activated
   inject_set_fuelcut(!engine_blowing_cond() && d.ie_valve && !d.sys_locked && !d.fc_revlim && pwrrelay_get_state() && !(d.sens.gas && (IOCFG_CHECK(IOP_GD_STP) || CHECKBIT(d.param.flpmp_flags, FPF_INJONGAS))));
#else
   inject_set_fuelcut(!engine_blowing_cond() && d.ie_valve && !d.sys_locked && !d.fc_revlim && pwrrelay_get_state() && !(d.sens.gas && CHECKBIT(d.param.flpmp_flags, FPF_INJONGAS)));
#endif
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
   lambda_stroke_event_notification();
#endif


#ifdef GD_CONTROL
   gasdose_stroke_event_notification();
#endif

#ifdef AIRCONDIT
   aircond_stroke_event_notification();
#endif

   //----------------------------------------------
   if (d.param.knock_use_knock_channel)
   {
    knklogic_detect(&retard_state);
    knklogic_retard(&retard_state);
    //control KS signal attenuation depending on current RPM
    knock_set_gain(knock_attenuator_function());
   }
   else
    d.corr.knock_retard = 0;
   //----------------------------------------------
  }

  //save ignition timing for applying in the nearest ignition stroke
  ckps_set_advance_angle(d.corr.curr_angle);
#ifdef FUEL_INJECT
  //set current injection time and fuel cut state
  inject_set_inj_time(d.inj_pw_raw, d.inj_dt);
  //set injection timing depending on current mode of engine
  ckps_set_inj_timing(d.corr.inj_timing, d.inj_pw, d.param.inj_anglespec);
#endif

#ifdef FUEL_INJECT
  inject_calc_fuel_flow();
#endif

#ifdef DIAGNOSTICS
  diagnost_process();
#endif

#ifdef OBD_SUPPORT
  obd_process();
#endif

  wdt_reset_timer();
 }//main loop
 //------------------------------------------------------------------------
}
