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

/** \file secu3.c
 * Implementation of main module of the firmware.
 * (���������� �������� ������ ��������).
 */

#include "port/avrio.h"
#include "port/intrinsic.h"
#include "port/port.h"

#include "adc.h"
#include "bc_input.h"
#include "bitmask.h"
#include "bootldr.h"
#include "camsens.h"
#include "ce_errors.h"
#include "choke.h"
#include "ckps.h"
#include "crc16.h"
#include "diagnost.h"
#include "eeprom.h"
#include "fuelecon.h"
#include "fuelpump.h"
#include "funconv.h"
#include "jumper.h"
#include "idlecon.h"
#include "ignlogic.h"
#include "knklogic.h"
#include "knock.h"
#include "magnitude.h"
#include "measure.h"
#include "params.h"
#include "procuart.h"
#include "pwrrelay.h"
#include "secu3.h"
#include "starter.h"
#include "suspendop.h"
#include "tables.h"
#include "uart.h"
#include "ventilator.h"
#include "vstimer.h"
#include "wdt.h"

/**ECU data structure. Contains all related data and state information */
struct ecudata_t edat;

/**Control of certain units of engine (���������� ���������� ������ ���������).
 * \param d pointer to ECU data structure
 */
void control_engine_units(struct ecudata_t *d)
{
 //���������� ������� ����.
 idlecon_control(d);

 //���������� ����������� ��������
 starter_control(d);

 //���������� ������� ������������ ���������� ���������, ��� ������� ��� ���� ������������ � �������
 vent_control(d);

 //���������� ��� (����������� ���������� �������)
 fuelecon_control(d);

#ifdef FUEL_PUMP
 //Controlling of electric fuel pump (���������� �������������������)
 fuelpump_control(d);
#endif

 //power management
 pwrrelay_control(d);

#ifdef SM_CONTROL
 //choke control
 choke_control(d);
#endif
}

/**Initialization of variables and data structures
 * \param d pointer to ECU data structure
 */
void init_ecu_data(struct ecudata_t* d)
{
 edat.op_comp_code = 0;
 edat.op_actn_code = 0;
 edat.sens.inst_frq = 0;
 edat.curr_angle = 0;
 edat.knock_retard = 0;
 edat.ecuerrors_for_transfer = 0;
 edat.eeprom_parameters_cache = &eeprom_parameters_cache[0];
 edat.engine_mode = EM_START;
 edat.ce_state = 0;
#ifdef REALTIME_TABLES
 edat.fn_gasoline_prev = 255;
 edat.fn_gas_prev = 255;
#endif
 edat.cool_fan = 0;
 edat.st_block = 0; //������� �� ������������
 edat.sens.tps = edat.sens.tps_raw = 0;
 edat.sens.add_i1 = edat.sens.add_i1_raw = 0;
 edat.sens.add_i2 = edat.sens.add_i2_raw = 0;
 edat.choke_testing = 0;
 edat.choke_pos = 0;
 edat.choke_manpos_d = 0;
}

/**Initialization of I/O ports
 */
void init_ports(void)
{
 ckps_init_ports();
 vent_init_ports();
 fuelecon_init_ports();
#ifdef FUEL_PUMP
 fuelpump_init_ports();
#endif
 idlecon_init_ports();
 starter_init_ports();
 jumper_init_ports();
 ce_init_ports();
 knock_init_ports();
 pwrrelay_init_ports();
#ifdef SM_CONTROL
 choke_init_ports();
#endif
}

/**Main function of firmware - entry point */
MAIN()
{
 int16_t calc_adv_ang = 0;
 uint8_t turnout_low_priority_errors_counter = 255;
 int16_t advance_angle_inhibitor_state = 0;
 retard_state_t retard_state;

 //���������� ��������� ������ ���������� ��������� �������
 init_ecu_data(&edat);
 knklogic_init(&retard_state);

 //������������� ����� �����/������
 init_ports();

 //���� ��� ��������� �������� - �������� ��
 if (crc16f(0, CODE_SIZE)!=PGM_GET_WORD(&fw_data.code_crc))
  ce_set_error(ECUERROR_PROGRAM_CODE_BROKEN);

 wdt_start_timer();

 //������ ���������
 load_eeprom_params(&edat);

#ifdef REALTIME_TABLES
 //load currently selected tables into RAM
 load_selected_tables_into_ram(&edat);
#endif

 knock_init(&edat.param, PGM_GET_BYTE(&fw_data.exdata.attenuator_table[0]));
 edat.use_knock_channel_prev = edat.param.knock_use_knock_channel;

 adc_init();

 //�������� ��������� ������ ��������� �������� ��� ������������� ������
 meas_initial_measure(&edat);

 //������� ���������� ��������
 starter_set_blocking_state(0);

 //�������������� UART
 uart_init(edat.param.uart_divisor);

 //initialization of cam module, must precede ckps initialization
#if defined(PHASE_SENSOR) || defined(SECU3T)
 cams_init_state();
#endif

#ifdef FUEL_PUMP
 //initialization of electric fuel pump
 fuelpump_init();
#endif

 //initialization of power management unit
 pwrrelay_init();

#ifdef SM_CONTROL
 choke_init();
#endif

 //�������������� ������ ����
 ckps_init_state();
 ckps_set_cyl_number(edat.param.ckps_engine_cyl);
 ckps_set_cogs_num(edat.param.ckps_cogs_num, edat.param.ckps_miss_num);
 ckps_set_edge_type(edat.param.ckps_edge_type);
#ifdef SECU3T
 cams_vr_set_edge_type(edat.param.ref_s_edge_type); //REF_S edge (����� ���)
#endif
 ckps_set_cogs_btdc(edat.param.ckps_cogs_btdc); //<--only partial initialization
#ifndef DWELL_CONTROL
 ckps_set_ignition_cogs(edat.param.ckps_ignit_cogs);
#endif
 ckps_set_knock_window(edat.param.knock_k_wnd_begin_angle,edat.param.knock_k_wnd_end_angle);
 ckps_use_knock_channel(edat.param.knock_use_knock_channel);
 ckps_set_cogs_btdc(edat.param.ckps_cogs_btdc); //<--now valid initialization
 ckps_set_merge_outs(edat.param.merge_ign_outs);
#ifdef HALL_OUTPUT
 ckps_set_hall_pulse(edat.param.hop_start_cogs, edat.param.hop_durat_cogs);
#endif

 s_timer_init();
 vent_init_state();

 //check and enter blink codes indication mode
 bc_indication_mode(&edat);

 //��������� ��������� ����������
 _ENABLE_INTERRUPT();

 sop_init_operations();
 //------------------------------------------------------------------------
 while(1)
 {
  if (ckps_is_cog_changed())
   s_timer_set(engine_rotation_timeout_counter, ENGINE_ROTATION_TIMEOUT_VALUE);

  if (s_timer_is_action(engine_rotation_timeout_counter))
  { //��������� ����������� (��� ������� ���� �����������)
#ifdef DWELL_CONTROL
   ckps_init_ports();           //����� IGBT �� ������� � �������� ���������
   //TODO: ������� ������ ������� ��� ���������� �� ������������� �����. ���?
#endif
   ckps_init_state_variables();
#if defined(PHASE_SENSOR) || defined(SECU3T)
   cams_init_state_variables();
#endif
   edat.engine_mode = EM_START; //����� �����

   if (edat.param.knock_use_knock_channel)
    knock_start_settings_latching();

   edat.curr_angle = calc_adv_ang;
   meas_update_values_buffers(&edat, 1);  //<-- update RPM only
  }

  //��������� ��������� ���, ����� ������ ���������� �������. ��� ����������� ������� ��������
  //����� ���� ������ ��������������������. ����� �������, ����� ������� �������� ��������� ��������
  //������������ ��������, �� ��� ������� ���������� �����������.
  if (s_timer_is_action(force_measure_timeout_counter))
  {
   if (!edat.param.knock_use_knock_channel)
   {
    _DISABLE_INTERRUPT();
    adc_begin_measure(0);  //normal speed
    _ENABLE_INTERRUPT();
   }
   else
   {
    //���� ������ ���������� �������� �������� � HIP, �� ����� ��������� �� ����������.
    while(!knock_is_latching_idle());
    _DISABLE_INTERRUPT();
    //�������� ����� �������������� � ���� ����� 20���, ���� ���������� ������ ������������� (����������
    //�� ��� ������ ������ �� ��������). � ������ ������ ��� ������ ��������� � ���, ��� �� ������ ����������
    //������������ 20-25���, ��� ��� ��� ���������� �� ����� ��������� ��������.
    knock_set_integration_mode(KNOCK_INTMODE_INT);
    _DELAY_CYCLES(350);
    knock_set_integration_mode(KNOCK_INTMODE_HOLD);
    adc_begin_measure_all(); //�������� ������ � �� ����
    _ENABLE_INTERRUPT();
   }

   s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
   meas_update_values_buffers(&edat, 0);
  }

  //----------����������� ����������-----------------------------------------
  //���������� ���������� ��������
  sop_execute_operations(&edat);
  //���������� ������������� � �������������� ����������� ������
  ce_check_engine(&edat, &ce_control_time_counter);
  //��������� ����������/�������� ������ ����������������� �����
  process_uart_interface(&edat);
  //���������� ����������� ��������
  save_param_if_need(&edat);
  //������ ���������� ������� �������� ���������
  edat.sens.inst_frq = ckps_calculate_instant_freq();
  //���������� ���������� ������� ���������� � ��������� �������
  meas_average_measured_values(&edat);
  //c�������� ���������� ����� ������� � ����������� ��� �������
  meas_take_discrete_inputs(&edat);
  //���������� ����������
  control_engine_units(&edat);
  //�� ��������� ������� (��������� ������� - ������ ��������� �����)
  calc_adv_ang = advance_angle_state_machine(&edat);
  //��������� � ��� �����-���������
  calc_adv_ang+=edat.param.angle_corr;
  //������������ ������������ ��� �������������� ���������
  restrict_value_to(&calc_adv_ang, edat.param.min_angle, edat.param.max_angle);
  //���� ����� ����� �������� ���, �� 0
  if (edat.param.zero_adv_ang)
   calc_adv_ang = 0;

#ifdef DWELL_CONTROL
  //calculate and update accumulation time (dwell control)
  ckps_set_acc_time(accumulation_time(&edat));
#endif
  //���� ���������, �� ������ ������� ��������� ��� ���������� ������������ ��������
  if (edat.param.ign_cutoff)
   ckps_enable_ignition(edat.sens.inst_frq < edat.param.ign_cutoff_thrd);
  else
   ckps_enable_ignition(1);

#ifdef DIAGNOSTICS
  diagnost_process(&edat);
#endif
  //------------------------------------------------------------------------

  //��������� �������� ������� ���������� ��������� ������ ��� ������� �������� �����.
  if (ckps_is_stroke_event_r())
  {
   meas_update_values_buffers(&edat, 0);
   s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);

   //������������ ������� ��������� ���, �� �� ����� ��������� ������ ��� �� ������������ ��������
   //�� ���� ������� ����. � ������ ����� ������ ��� ��������.
   if (EM_START == edat.engine_mode)
   {
#ifdef HALL_SYNC
    int16_t strt_map_angle = start_function(&edat);
    ckps_set_shutter_spark(0==strt_map_angle);
    edat.curr_angle = advance_angle_inhibitor_state = (0==strt_map_angle ? 0 : calc_adv_ang);
#else
    edat.curr_angle = advance_angle_inhibitor_state = calc_adv_ang;
#endif
   }
   else
  {
#ifdef HALL_SYNC
    ckps_set_shutter_spark(edat.sens.frequen < 200);
#endif
    edat.curr_angle = advance_angle_inhibitor(calc_adv_ang, &advance_angle_inhibitor_state, edat.param.angle_inc_spead, edat.param.angle_dec_spead);
  }

   //----------------------------------------------
   if (edat.param.knock_use_knock_channel)
   {
    knklogic_detect(&edat, &retard_state);
    knklogic_retard(&edat, &retard_state);
   }
   else
    edat.knock_retard = 0;
   //----------------------------------------------

   //��������� ��� ��� ���������� � ��������� �� ������� ����� ���������
   ckps_set_advance_angle(edat.curr_angle);

   //��������� ��������� ����������� � ����������� �� ��������
   if (edat.param.knock_use_knock_channel)
    knock_set_gain(knock_attenuator_function(&edat));

   // ������������� ���� ������ ���������� ��� ������ �������� ���������
   //(��� ���������� N-�� ���������� ������)
   if (turnout_low_priority_errors_counter == 1)
   {
    ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);
    ce_clear_error(ECUERROR_PROGRAM_CODE_BROKEN);
   }
   if (turnout_low_priority_errors_counter > 0)
    turnout_low_priority_errors_counter--;
  }

  wdt_reset_timer();
 }//main loop
 //------------------------------------------------------------------------
}
