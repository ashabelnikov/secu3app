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

/** \file tables.c
 * \author Alexey A. Shabelnikov
 * Tables and datastructures stored in the frmware (instances).
 * (Таблицы и структуры данных хранимые в прошивке).
 */

#include "port/port.h"
#include "adc.h"
#include "bitmask.h"
#include "compilopt.h"
#include "ioconfig.h"
#include "magnitude.h"
#include "tables.h"
#include "uart.h"

/**Helpful macro used for pointer conversion */
#define _FNC(a) ((fnptr_t)(a))

/**Helpful macro for creating of I/O remapping version number*/
#define IOREMVER(maj, min) ((maj)<<4 | ((min) & 0xf))

/**For specifying of dwell times in the lookup table*/
#define _DLV(v) ROUND(((v)*312.5))

/**For specifying of choke position in the lookup table*/
#define _CLV(v) ROUND(((v)*2.0))

/**For specifying of temperature values in the lookup table*/
#define _TLV(v) ROUND(((v)*4.0))

/**ADC compensation factor, see magnitude.h for more information*/
#define _ACF ADC_COMP_FACTOR(ADC_VREF_FACTOR)

/**ADC compensation correction, see magnitude.h for more information*/
#define _ACC ADC_COMP_CORR(ADC_VREF_FACTOR, 0.0)

/**For specifying values in the warmup enrichment lookup table*/
#define _WLV(v) ROUND((((v)/100.0)*128.0))

/**For specifying values in VE table*/
#define _VE(v) ROUND(((v)*128.0))

/**For specifying values in AFR table*/
#define _FR(v) ROUND((1.0/(v)*2048.0))

/**For setting cylinder displacement value*/
#define CYL_DISP(v) ROUND(((v)*16384.0))

/**For setting injector flow rate value*/
#define INJ_FLRT(v) ROUND(((v)*64.0))

/**For setting system time values, v in seconds*/
#define SYS_TIME_S(v) ROUND(((v)*100.0))
/**Same as SYS_TIME_S, but multiplied only by 10*/
#define SYS_TIMEX10_S(v) ROUND((v)*10)

/**For setting lambda correction values*/
#define EGO_CORR(v) ROUND((((v)/100.0)*512.0))

/**For filling afterstart enrichment lookup table */
#define _ASE(v) ROUND((((v)/100.0)*128.0))

/**For filling bins of the AE's TPS lookup table*/
#define AE_TPS_B(v) ROUND((v) / 10.0)
/**For filling values of the AE's TPS lookup table*/
#define AE_TPS_V(v) ROUND((((v)/100.0)*100.0)+55.0)
/**For filling bins of the AE's RPM lookup table*/
#define AE_RPM_B(v) ROUND((v) / 100.0)
/**For filling values of the AE's RPM lookup table*/
#define AE_RPM_V(v) ROUND(((v)/100.0)*128.0)

/**For setting AE cold accel. multiplier value*/
#define AE_CAM(v) ROUND(((v)-1.0)*128.0)

/**Cooling fan PWM frequency*/
#define _FAN_PWMFRQ(v) ROUND((1.0/(v))*524288.0)

//For encoding of gas dose actuator position value
#define _GD(v) GD_MAGNITUDE(v)

//For encoding of injection timing map values
#define _IT(v) ROUND((v) / 3.0)

/**Fill whole firmware data */
PGM_FIXED_ADDR_OBJ(fw_data_t fw_data, ".firmware_data") =
{
 /**Fill data residing in code area*/
 {
  //Add new data here

  /**I/O remapping. Match slots and plugs for default configuration*/
  {
   //normal slots (initialization)
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_add_io1), _FNC(iocfg_i_add_io2), _FNC(iocfg_i_ecf), _FNC(iocfg_i_st_block),
    _FNC(iocfg_i_ie), _FNC(iocfg_i_fe),
    _FNC(iocfg_i_ps),     //PS input initialization
    _FNC(iocfg_i_add_i1), //ADD_IO1 input initialization
    _FNC(iocfg_i_add_i2), //ADD_IO2 input initialization
    _FNC(iocfg_i_ce), _FNC(iocfg_i_bl), _FNC(iocfg_i_de),
    _FNC(iocfg_i_gas_v), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps),
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (initialization)
   {_FNC(iocfg_i_ign_out1i), _FNC(iocfg_i_ign_out2i), _FNC(iocfg_i_ign_out3i), _FNC(iocfg_i_ign_out4i),
    _FNC(iocfg_i_add_io1i), _FNC(iocfg_i_add_io2i), _FNC(iocfg_i_ecfi), _FNC(iocfg_i_st_blocki),
    _FNC(iocfg_i_iei), _FNC(iocfg_i_fei),
    _FNC(iocfg_i_psi),              //PS input initialization
    _FNC(iocfg_i_add_i1i), //ADD_IO1 input initialization
    _FNC(iocfg_i_add_i2i), //ADD_IO2 input initialization
    _FNC(iocfg_i_cei), _FNC(iocfg_i_bli), _FNC(iocfg_i_dei),
    _FNC(iocfg_i_gas_vi), _FNC(iocfg_i_ref_si), _FNC(iocfg_i_ckpsi),
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//normal slots (get/set value)
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_add_io1), _FNC(iocfg_s_add_io2), _FNC(iocfg_s_ecf), _FNC(iocfg_s_st_block),
    _FNC(iocfg_s_ie), _FNC(iocfg_s_fe),
    _FNC(iocfg_g_ps),              //PS input get value
    _FNC(iocfg_g_add_i1), //ADD_IO1 input get value
    _FNC(iocfg_g_add_i2), //ADD_IO2 input get value
    _FNC(iocfg_s_ce), _FNC(iocfg_s_bl), _FNC(iocfg_s_de),
    _FNC(iocfg_g_gas_v), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps),
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },//inverted slots (get/set value)
   {_FNC(iocfg_s_ign_out1i), _FNC(iocfg_s_ign_out2i), _FNC(iocfg_s_ign_out3i), _FNC(iocfg_s_ign_out4i),
    _FNC(iocfg_s_add_io1i), _FNC(iocfg_s_add_io2i), _FNC(iocfg_s_ecfi), _FNC(iocfg_s_st_blocki),
    _FNC(iocfg_s_iei), _FNC(iocfg_s_fei),
    _FNC(iocfg_g_psi),              //PS input get value
    _FNC(iocfg_g_add_i1i), //ADD_IO1 input get value
    _FNC(iocfg_g_add_i2i), //ADD_IO2 input get value
    _FNC(iocfg_s_cei), _FNC(iocfg_s_bli), _FNC(iocfg_s_dei),
    _FNC(iocfg_g_gas_vi), _FNC(iocfg_g_ref_si), _FNC(iocfg_g_ckpsi),
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 //<-- zero means that these slots are not implemented in this firmware
   },
   //plugs
   {_FNC(iocfg_i_ign_out1), _FNC(iocfg_i_ign_out2), _FNC(iocfg_i_ign_out3), _FNC(iocfg_i_ign_out4),
    _FNC(iocfg_i_add_io1), _FNC(iocfg_i_add_io2), _FNC(iocfg_i_ecf), _FNC(iocfg_i_st_block),
    _FNC(iocfg_i_ie), _FNC(iocfg_i_fe), _FNC(iocfg_i_ps), _FNC(iocfg_i_add_i1),
    _FNC(iocfg_i_add_i2), _FNC(iocfg_i_ce), _FNC(iocfg_i_bl), _FNC(iocfg_i_de),
    _FNC(iocfg_i_gas_v), _FNC(iocfg_i_ref_s), _FNC(iocfg_i_ckps), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //<-- mapped to slots by default
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub)
   },
   {_FNC(iocfg_s_ign_out1), _FNC(iocfg_s_ign_out2), _FNC(iocfg_s_ign_out3), _FNC(iocfg_s_ign_out4),
    _FNC(iocfg_s_add_io1), _FNC(iocfg_s_add_io2), _FNC(iocfg_s_ecf), _FNC(iocfg_s_st_block),
    _FNC(iocfg_s_ie), _FNC(iocfg_s_fe), _FNC(iocfg_g_ps), _FNC(iocfg_g_add_i1),
    _FNC(iocfg_g_add_i2), _FNC(iocfg_s_ce), _FNC(iocfg_s_bl), _FNC(iocfg_s_de),
    _FNC(iocfg_g_gas_v), _FNC(iocfg_g_ref_s), _FNC(iocfg_g_ckps), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //<-- mapped to slots by default
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_g_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), _FNC(iocfg_g_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub),
    _FNC(iocfg_s_stub), _FNC(iocfg_s_stub), _FNC(iocfg_s_stub)
   },

   _FNC(iocfg_s_stub), _FNC(iocfg_g_stub), //<-- stub, stub

   //Version of this structure - 2.4
   IOREMVER(2,4),

   //2 bytes - size of this structure
   sizeof(iorem_slots_t),
  },

  /**32-bit config data*/
  _CBV32(0/*not used*/, 0) | _CBV32(0/*not used*/, 1) | _CBV32(0/*COPT_ATMEGA64, left for compatibility*/, 2) | _CBV32(0/*COPT_ATMEGA128, left for compatibility*/, 3) |
  _CBV32(0/*not used*/, 4) | _CBV32(0/*not used*/, 5) | _CBV32(0/*not used*/, 6) | _CBV32(COPT_DWELL_CONTROL, 7) |
  _CBV32(COPT_COOLINGFAN_PWM, 8) | _CBV32(COPT_REALTIME_TABLES, 9) | _CBV32(COPT_ICCAVR_COMPILER, 10) | _CBV32(COPT_AVRGCC_COMPILER, 11) |
  _CBV32(COPT_DEBUG_VARIABLES, 12) | _CBV32(COPT_PHASE_SENSOR, 13) | _CBV32(COPT_PHASED_IGNITION, 14) | _CBV32(COPT_FUEL_PUMP, 15) |
  _CBV32(COPT_THERMISTOR_CS, 16) | _CBV32(1/*COPT_SECU3T is obsolete, left for compatibility*/, 17) | _CBV32(COPT_DIAGNOSTICS, 18) | _CBV32(COPT_HALL_OUTPUT, 19) |
  _CBV32(COPT_REV9_BOARD, 20) | _CBV32(COPT_STROBOSCOPE, 21) | _CBV32(COPT_SM_CONTROL, 22) | _CBV32(COPT_VREF_5V, 23) |
  _CBV32(COPT_HALL_SYNC, 24) | _CBV32(COPT_UART_BINARY, 25) | _CBV32(COPT_CKPS_2CHIGN, 26) | _CBV32(1/*COPT_ATMEGA644 is obsolete, left for compatibility*/, 27) |
  _CBV32(COPT_FUEL_INJECT, 28) | _CBV32(COPT_GD_CONTROL, 29) | _CBV32(COPT_CARB_AFR, 30) | _CBV32(COPT_CKPS_NPLUS1, 31),

  /**Reserved bytes*/
  {0,0,0},

  /**Version of the firmware. Do not forget to write out same value into the signature info! */
  0x45,

  /**2 bytes - size of this structure. */
  sizeof(cd_data_t)
 },

 /**Резервные параметры Fill reserve parameters with default values */
 {
  .starter_off =                 600,
  .smap_abandon =                650,

  .map_lower_pressure =          1920,
  .map_upper_pressure =          6400,
  .map_curve_offset =            54,
  .map_curve_gradient =          1089,

  .carb_invers =                 0,
  .tps_curve_offset =            160,
  .tps_curve_gradient =          0,
  .tps_threshold =               0,

  .ie_lot =                      1900,
  .ie_hit =                      2100,
  .ie_lot_g =                    1900,
  .ie_hit_g =                    2100,
  .fe_on_threshold =             392,
  .shutoff_delay =               0,
  .fuelcut_map_thrd =            1280,
  .fuelcut_cts_thrd =            60,

  .angle_dec_speed =             96,
  .angle_inc_speed =             96,
  .max_angle =                   1600,
  .min_angle =                   0,
  .angle_corr =                  0,
  .zero_adv_ang =                0,

  .fn_gasoline =                 0,
  .fn_gas =                      0,

  .idl_flags =                   _BV(IRF_USE_REGONGAS), //use regulator on gas
  .idling_rpm =                  800,
  .ifac1 =                       4,
  .ifac2 =                       4,
  .MINEFR =                      20,
  .idlreg_min_angle =           -160,
  .idlreg_max_angle =            320,
  .idlreg_turn_on_temp =         200,

  .tmp_use =                     1,
  .vent_on =                     392,
  .vent_off =                    384,
  .vent_pwm =                    0,
  .cts_use_map =                 0,
  .vent_pwmfrq =                 _FAN_PWMFRQ(5000),  //5000Hz

  .map_adc_factor =              _ACF,
  .map_adc_correction =          _ACC,
  .ubat_adc_factor =             _ACF,
  .ubat_adc_correction =         _ACC,
  .temp_adc_factor =             _ACF,
  .temp_adc_correction =         _ACC,
  .tps_adc_factor =              _ACF,
  .tps_adc_correction =          _ACC,
  .ai1_adc_factor =              _ACF,
  .ai1_adc_correction =          _ACC,
  .ai2_adc_factor =              _ACF,
  .ai2_adc_correction =          _ACC,

  .ckps_edge_type =              0,
  .ckps_cogs_btdc =              20,
  .ckps_ignit_cogs =             10,
  .ckps_engine_cyl =             4,
  .ckps_cogs_num =               60,
  .ckps_miss_num =               2,
  .ref_s_edge_type =             0,
  .hall_flags =                  0x00,
  .hall_wnd_width =              1920,

  .ign_cutoff =                  0,
  .ign_cutoff_thrd =             7500,
  .merge_ign_outs =              0,

  .hop_start_cogs =              0,
  .hop_durat_cogs =              10,

  .uart_divisor =                CBRID_57600,
  .uart_period_t_ms =            2,

  .knock_use_knock_channel =     0,
  .knock_bpf_frequency =         35,
  .knock_k_wnd_begin_angle =     0,
  .knock_k_wnd_end_angle =       800,
  .knock_int_time_const =        23,
  .knock_retard_step =           128,
  .knock_advance_step =          8,
  .knock_max_retard =            512,
  .knock_threshold =             1000,
  .knock_recovery_delay =        2,

  .sm_steps =                    800,
  .choke_rpm =                   {2000, 1200},
  .choke_startup_corr =          20,

  .choke_rpm_if =                51,
  .choke_corr_time =             300,
  .choke_corr_temp =             40,

  .bt_flags =                    0x02,
  .ibtn_keys =                   {{0,0,0,0,0,0},{0,0,0,0,0,0}},/**<--iButton keys database. Write out your own 48-bit keys here */

  .uni_output =                  {{0,0,0,220,200,220,200},{0,0,0,220,200,220,200},{0,0,0,220,200,220,200}},
  .uniout_12lf =                 15,                   //logic function between 1st and 2nd outputs

  .inj_flags =                   0,                    //
  .inj_config =                  0x12,                 //multi-point simultaneous injection, 2 squirts per cycle
  .inj_flow_rate =               INJ_FLRT(200.0),      //200 cc/min          (for management software only)
  .inj_cyl_disp =                CYL_DISP(0.375),      //0.375L (1.5/4)      (for management software only)
  .inj_sd_igl_const =            86207,                //((0.375L * 3.482 * 18750000) / 142g) * ((1 * 4) / (2 * 4)), petrol density is 0.71 g/cc, 1bank,4cyl,2squirts,4injectors

  .inj_prime_cold =              _DLV(6.0),            //6 ms at -30°C
  .inj_prime_hot =               _DLV(2.0),            //2 ms at 70°C
  .inj_prime_delay =             SYS_TIMEX10_S(2.0),   //fire prime pulse after 2 seconds

  .inj_cranktorun_time =         SYS_TIME_S(3.00),     //3 seconds
  .inj_aftstr_strokes =          75,                   //150 strokes

  .inj_lambda_str_per_stp =      8,                    //8 strokes
  .inj_lambda_step_size_p =      EGO_CORR(2.5),        //2.5%
  .inj_lambda_corr_limit_p =     EGO_CORR(30.0),       //30% max
  .inj_lambda_swt_point =        VOLTAGE_MAGNITUDE(0.5), //0.5V
  .inj_lambda_temp_thrd =        TEMPERATURE_MAGNITUDE(60.0), //60°C
  .inj_lambda_rpm_thrd =         1200,                 //1200 min-1
  .inj_lambda_activ_delay =      45,                   //activation after 45 seconds

  .inj_ae_tpsdot_thrd =          50,                   //50%/sec
  .inj_ae_coldacc_mult =         AE_CAM(1.5),          //*150% at -30°C, allowed range is 1.0...2.99

  .gd_steps =                    256,                  //256 steps, gas dose number of steps

  .inj_timing =                  0,                    //TDC

  .flpmp_flags =                 _BV(FPF_OFFONGAS),    //turn off fuel pump when GAS_V = 1

  .choke_flags =                 0,                    //do not turn off additional startup closing, RPM regulator when fuel type is gas and don't use throttle position in choke initialization

  .revlim_lot =                  8000,                 // 8000 min-1
  .revlim_hit =                  8100,                 // 8100 min-1

  .inj_timing_crk =              0,                    //TDC

  .gd_fc_closing =               GD_MAGNITUDE(30),     //close for 30%

  .inj_lambda_step_size_m =      EGO_CORR(2.5),        //2.5%
  .inj_lambda_corr_limit_m =     EGO_CORR(30.0),       //30% max

  .gd_lambda_corr_limit_p =      EGO_CORR(30.0),       //30% max
  .gd_lambda_corr_limit_m =      EGO_CORR(30.0),       //30% max

  .inj_lambda_dead_band =        VOLTAGE_MAGNITUDE(0.0), //zero dead band by default

  .load_src_cfg =                0,                    //default is MAP

  .reserved =                    {0},
  .crc =                         0
 },

 /**Дополнительные данные по умолчанию Fill additional data with default values */
 {
  /**Таблица аттенюатора, по умолчанию k = 1.000
   * attenuator's lookup table (for knock channel), by default k = 1.000 */
  {0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,
   0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E,0x0E
  },

  /**Lookup table for controlling coil's accumulation time (dwell control) */
  {//  5.4         5.8         6.2         6.6         7.0        7.4        7.8         8.2
   _DLV(15.00),_DLV(13.60),_DLV(12.50),_DLV(11.40),_DLV(10.20),_DLV(9.25),_DLV(8.30),_DLV(7.55),
   //  8.6         9.0         9.4         9.8        10.2       10.6       11.0        11.4
   _DLV(6.80), _DLV(6.30), _DLV(5.80), _DLV(5.34), _DLV(4.88), _DLV(4.53),_DLV(4.19),_DLV(3.94),
   // 11.8        12.2        12.6        13.0        13.4       13.8       14.2        14.6
   _DLV(3.69), _DLV(3.44), _DLV(3.19), _DLV(3.03), _DLV(2.90), _DLV(2.70),_DLV(2.55),_DLV(2.43),
   // 15.0        15.4        15.8        16.2        16.6       17.0       17.4        17.8
   _DLV(2.30), _DLV(2.22), _DLV(2.13), _DLV(2.00), _DLV(1.88), _DLV(1.85),_DLV(1.82),_DLV(1.80),
  },

  /**Fill coolant temperature sensor lookup table*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0)},
  ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE),

  /**Fill choke closing vs. temperarure lookup table*/
  {_CLV(100.0), _CLV(99.0), _CLV(98.0), _CLV(96.0), _CLV(95.0), _CLV(92.0), _CLV(86.0), _CLV(78.0),
   _CLV(69.0),  _CLV(56.0), _CLV(50.0), _CLV(40.0), _CLV(25.0), _CLV(12.0), _CLV(5.0),  _CLV(0)},

  /**Fill air temperature sensor lookup table (temperature vs voltage)*/
  {_TLV(120.0), _TLV(100.0), _TLV(81.0), _TLV(65.5), _TLV(56.0), _TLV(48.5), _TLV(42.0), _TLV(36.0),
   _TLV(30.0), _TLV(23.0), _TLV(18.0), _TLV(12.0), _TLV(4.0), _TLV(-4.0), _TLV(-15.0), _TLV(-37.0)},
   ROUND(0.0 / ADC_DISCRETE), ROUND(4.98 / ADC_DISCRETE),

  /**Fill air temperature lookup table for advance angle correction*/
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

  /**Fill RPM grid points lookup table*/
  {600,720,840,990,1170,1380,1650,1950,2310,2730,3210,3840,4530,5370,6360,7500},
  /**Fill PRM grid cell sizes lookup table*/
  {120,120,150,180, 210, 270, 300, 360, 420, 480, 630, 690, 840, 990, 1140},

  /** Gas dose actuator position vs (TPS,RPM)*/
  {//  600       720         840        990       1170        1380       1650       1950       2310       2730       3210      3840        4530      5370        6360      7500 (min-1)
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //100% 16
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //     15
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //     14
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //80%  13
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //     12
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //     11
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //60%  10
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      9
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      8
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //40%   7
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      6
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      5
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //20%   4
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      3
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}, //      2
   {_GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0), _GD(50.0)}  //0%    1
  },

  /**reserved bytes*/
  {0}
 },

 /**Данные в таблицах по умолчанию Fill tables with default data */
 {
  {
   {'2','1','0','8','3',' ','‘','в',' ','­','¤',' ','а','в',' ',' '},                  //name of set
   //таблицы для зажигания
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //пусковая карта
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //ХХ карта
   {
    {0x04,0x05,0x06,0x07,0x0A,0x0C,0x10,0x15,0x1A,0x20,0x24,0x27,0x28,0x28,0x28,0x28}, //карта основного режима
    {0x04,0x05,0x06,0x08,0x0B,0x0D,0x10,0x16,0x1B,0x21,0x26,0x29,0x2A,0x2A,0x2A,0x2A},
    {0x04,0x05,0x08,0x08,0x0C,0x0E,0x12,0x18,0x1E,0x23,0x28,0x2A,0x2C,0x2C,0x2C,0x2C},
    {0x04,0x06,0x08,0x0A,0x0C,0x10,0x14,0x1B,0x21,0x26,0x28,0x2A,0x2C,0x2C,0x2C,0x2D},
    {0x06,0x07,0x0A,0x0C,0x0E,0x12,0x18,0x20,0x26,0x2A,0x2C,0x2D,0x2E,0x2E,0x2F,0x30},
    {0x08,0x08,0x0A,0x0E,0x12,0x16,0x1E,0x27,0x2D,0x2F,0x30,0x31,0x33,0x34,0x35,0x36},
    {0x0A,0x0B,0x0D,0x10,0x14,0x1B,0x24,0x2D,0x32,0x33,0x35,0x37,0x39,0x3A,0x3A,0x3B},
    {0x0C,0x10,0x12,0x15,0x1A,0x21,0x29,0x32,0x36,0x37,0x39,0x3C,0x3E,0x40,0x40,0x40},
    {0x10,0x16,0x18,0x1C,0x22,0x28,0x2E,0x36,0x3A,0x3B,0x3D,0x3F,0x41,0x44,0x44,0x44},
    {0x16,0x1C,0x1E,0x22,0x27,0x2E,0x34,0x3A,0x3D,0x3E,0x40,0x42,0x44,0x48,0x48,0x48},
    {0x1C,0x21,0x22,0x26,0x2A,0x31,0x38,0x3F,0x41,0x42,0x44,0x45,0x48,0x4A,0x4A,0x4B},
    {0x1E,0x22,0x24,0x27,0x2B,0x33,0x3B,0x42,0x45,0x45,0x47,0x48,0x4A,0x4C,0x4C,0x4C},
    {0x20,0x24,0x26,0x29,0x2E,0x36,0x3D,0x45,0x47,0x47,0x48,0x49,0x4A,0x4C,0x4C,0x4D},
    {0x20,0x24,0x27,0x2B,0x31,0x37,0x3F,0x45,0x47,0x48,0x49,0x49,0x4B,0x4D,0x4D,0x4D},
    {0x1E,0x22,0x26,0x2C,0x31,0x39,0x40,0x46,0x48,0x4A,0x4A,0x4B,0x4D,0x4E,0x4E,0x4E},
    {0x1E,0x21,0x25,0x29,0x2F,0x36,0x3F,0x45,0x49,0x4B,0x4C,0x4D,0x4F,0x4F,0x4F,0x4F}
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //карта температурной коррекции УОЗ

   //Таблицы для впрыска топлива
   /**Fill VE lookup table, value can be in range 0...1.99 / Таблица задающая коэффициент наполнения цилиндра */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //16
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //15
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //14
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //13
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //12
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //11
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //10
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 9
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 8
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 7
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 6
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 5
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 4
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 3
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 2
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}  // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 / Таблица задающая соотношение воздух : топливо */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //16
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //15
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //14
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //13
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //12
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //11
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //10
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 9
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 8
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 7
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 6
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 5
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 4
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 3
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 2
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}  // 1
   },
   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature / длительность впрыска на пуске */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature / обогащение при прогреве */
   {// -30        -20        -10          0        10          20         30         40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110        120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage / время открытия форсунки (динам. производ.)*/
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) / положение ШД РХХ при работе*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) / положение ШД РХХ на пуске*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //16
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //15
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //14
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //13
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //12
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //11
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //10
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 9
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 8
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 7
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 6
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 5
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 4
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 3
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 2
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 1
   },

   /**reserved bytes */
   {0}
  },

  {
   {'2','1','0','8','3',' ','„','Ё','­',' ','¬','Ё','з','­',' ','п'},                  //name of set
   //tables used for ignition
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //start map
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //ХХ карта
   {
    {0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45}, //work map
    {0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45},
    {0x09,0x09,0x09,0x09,0x11,0x14,0x1A,0x20,0x26,0x24,0x30,0x33,0x3B,0x43,0x45,0x45},
    {0x09,0x09,0x09,0x09,0x13,0x14,0x1A,0x20,0x26,0x26,0x32,0x33,0x3F,0x43,0x45,0x45},
    {0x09,0x09,0x09,0x0A,0x13,0x16,0x1C,0x22,0x28,0x2F,0x39,0x3B,0x3F,0x47,0x49,0x49},
    {0x09,0x09,0x09,0x0B,0x15,0x28,0x2C,0x35,0x3A,0x41,0x42,0x43,0x45,0x45,0x4B,0x4B},
    {0x0B,0x0D,0x17,0x1D,0x25,0x2D,0x33,0x38,0x3F,0x45,0x4A,0x48,0x48,0x48,0x4A,0x4C},
    {0x16,0x1A,0x22,0x26,0x2D,0x33,0x39,0x3D,0x46,0x48,0x4D,0x4A,0x4A,0x4A,0x4A,0x50},
    {0x21,0x27,0x2F,0x35,0x33,0x36,0x3D,0x41,0x49,0x4B,0x4F,0x4C,0x4C,0x4C,0x4A,0x52},
    {0x28,0x2E,0x3A,0x3A,0x37,0x37,0x3D,0x45,0x4C,0x4D,0x4F,0x4F,0x4F,0x52,0x52,0x56},
    {0x2E,0x38,0x3E,0x40,0x38,0x37,0x45,0x49,0x4E,0x4F,0x51,0x50,0x50,0x54,0x58,0x58},
    {0x30,0x3E,0x42,0x40,0x38,0x3D,0x47,0x4F,0x52,0x50,0x4F,0x4E,0x4E,0x54,0x54,0x5A},
    {0x32,0x40,0x46,0x48,0x48,0x49,0x4B,0x4E,0x51,0x52,0x4E,0x4D,0x4B,0x54,0x58,0x58},
    {0x2E,0x3C,0x40,0x42,0x46,0x41,0x47,0x4B,0x4E,0x4E,0x4E,0x4D,0x45,0x54,0x54,0x56},
    {0x28,0x32,0x36,0x38,0x36,0x39,0x43,0x49,0x4B,0x4E,0x48,0x48,0x49,0x50,0x54,0x54},
    {0x24,0x28,0x28,0x28,0x30,0x35,0x3F,0x47,0x4B,0x4E,0x47,0x46,0x48,0x4C,0x50,0x50},
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //coolant temperature correction map

   //Таблицы для впрыска топлива
   /**Fill VE lookup table, value can be in range 0...1.99 / Таблица задающая коэффициент наполнения цилиндра */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //16
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //15
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //14
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //13
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //12
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //11
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //10
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 9
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 8
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 7
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 6
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 5
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 4
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 3
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 2
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}  // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 / Таблица задающая соотношение воздух : топливо */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //16
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //15
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //14
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //13
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //12
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //11
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //10
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 9
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 8
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 7
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 6
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 5
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 4
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 3
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 2
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}  // 1
   },
   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature / длительность впрыска на пуске */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature / обогащение при прогреве */
   {// -30        -20        -10          0        10          20         30         40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110       120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage / время открытия форсунки (динам. производ.)*/
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) / положение ШД РХХ при работе*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) / положение ШД РХХ на пуске*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //16
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //15
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //14
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //13
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //12
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //11
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //10
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 9
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 8
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 7
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 6
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 5
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 4
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 3
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 2
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 1
   },

   /**reserved bytes */
   {0}
  },

  {
   {'Љ','«',' ','б','б','Ё','Є',' ',' ','1','.','5',' ',' ',' ',' '},
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //ХХ карта
   {
    {0x0C,0x0C,0x0E,0x10,0x11,0x12,0x14,0x16,0x1B,0x1D,0x1F,0x21,0x22,0x24,0x27,0x27},
    {0x0E,0x0E,0x10,0x11,0x12,0x13,0x15,0x17,0x1C,0x20,0x24,0x25,0x26,0x26,0x29,0x29},
    {0x10,0x10,0x12,0x14,0x15,0x16,0x17,0x19,0x1E,0x22,0x26,0x27,0x29,0x28,0x2C,0x2C},
    {0x12,0x12,0x14,0x16,0x17,0x17,0x18,0x1D,0x21,0x26,0x2A,0x2B,0x2C,0x2D,0x2F,0x2F},
    {0x14,0x14,0x16,0x18,0x19,0x1B,0x23,0x2E,0x30,0x30,0x30,0x30,0x31,0x33,0x33,0x33},
    {0x16,0x16,0x18,0x1A,0x1B,0x23,0x2A,0x32,0x3C,0x3A,0x3A,0x3A,0x3B,0x38,0x38,0x38},
    {0x18,0x18,0x1A,0x1E,0x23,0x26,0x32,0x3A,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F},
    {0x1B,0x1B,0x20,0x20,0x24,0x2C,0x36,0x40,0x43,0x43,0x43,0x43,0x43,0x46,0x46,0x46},
    {0x1E,0x1E,0x20,0x22,0x26,0x2C,0x38,0x42,0x46,0x47,0x47,0x47,0x47,0x48,0x49,0x49},
    {0x1E,0x1E,0x20,0x22,0x26,0x32,0x3C,0x45,0x47,0x48,0x48,0x48,0x49,0x4A,0x4B,0x4B},
    {0x1E,0x1E,0x20,0x22,0x26,0x34,0x40,0x48,0x4A,0x4A,0x4A,0x4A,0x4A,0x4C,0x4C,0x4C},
    {0x1E,0x24,0x28,0x30,0x36,0x3C,0x42,0x48,0x4C,0x4C,0x4C,0x4C,0x4C,0x4E,0x4E,0x4E},
    {0x1B,0x24,0x28,0x30,0x36,0x3C,0x42,0x48,0x4C,0x4C,0x4C,0x4C,0x4C,0x4E,0x4E,0x4E},
    {0x19,0x24,0x28,0x30,0x36,0x3C,0x42,0x46,0x4A,0x4A,0x4A,0x4A,0x4A,0x4B,0x4B,0x4B},
    {0x17,0x24,0x28,0x30,0x36,0x3C,0x42,0x46,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A,0x4A},
    {0x15,0x24,0x28,0x30,0x36,0x3C,0x42,0x43,0x43,0x43,0x43,0x44,0x45,0x49,0x49,0x49},
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},

   //Таблицы для впрыска топлива
   /**Fill VE lookup table, value can be in range 0...1.99 / Таблица задающая коэффициент наполнения цилиндра */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //16
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //15
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //14
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //13
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //12
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //11
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //10
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 9
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 8
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 7
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 6
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 5
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 4
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 3
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 2
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}  // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 / Таблица задающая соотношение воздух : топливо */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //16
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //15
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //14
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //13
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //12
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //11
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //10
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 9
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 8
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 7
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 6
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 5
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 4
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 3
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 2
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}  // 1
   },
   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature / длительность впрыска на пуске */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature / обогащение при прогреве */
   {// -30        -20        -10          0        10          20         30        40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110       120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage / время открытия форсунки (динам. производ.)*/
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) / положение ШД РХХ при работе*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) / положение ШД РХХ на пуске*/
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //16
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //15
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //14
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //13
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //12
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //11
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //10
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 9
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 8
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 7
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 6
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 5
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 4
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 3
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 2
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 1
   },

   /**reserved bytes */
   {0}
  },

  {//Ignition maps
   {'Љ','«',' ','б','б','Ё','Є',' ',' ','1','.','6',' ',' ',' ',' '},
   {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //cranking map
   {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //idling map
   {
    {0x14,0x15,0x17,0x19,0x1B,0x1D,0x20,0x24,0x29,0x2D,0x2F,0x31,0x33,0x35,0x20,0x20}, //work map
    {0x16,0x18,0x1A,0x1C,0x1E,0x20,0x23,0x27,0x2C,0x31,0x34,0x36,0x38,0x3A,0x20,0x20},
    {0x18,0x1A,0x1C,0x1E,0x21,0x24,0x28,0x2D,0x33,0x38,0x3C,0x3E,0x3F,0x41,0x20,0x20},
    {0x1A,0x1C,0x1E,0x21,0x24,0x28,0x2D,0x33,0x39,0x3F,0x43,0x45,0x46,0x47,0x20,0x20},
    {0x1C,0x1E,0x20,0x23,0x26,0x2B,0x31,0x39,0x40,0x46,0x49,0x4A,0x4B,0x4C,0x20,0x20},
    {0x1E,0x20,0x22,0x25,0x29,0x30,0x36,0x3E,0x46,0x4D,0x51,0x52,0x52,0x52,0x20,0x20},
    {0x20,0x22,0x24,0x28,0x2C,0x34,0x3A,0x42,0x4A,0x51,0x55,0x57,0x58,0x59,0x20,0x20},
    {0x22,0x24,0x28,0x2C,0x30,0x38,0x3E,0x46,0x4E,0x54,0x59,0x5B,0x5D,0x5E,0x1C,0x1C},
    {0x24,0x28,0x2C,0x30,0x34,0x3D,0x42,0x49,0x51,0x57,0x5C,0x5E,0x5F,0x5F,0x18,0x18},
    {0x26,0x2A,0x2E,0x32,0x38,0x40,0x44,0x4B,0x53,0x59,0x5E,0x5F,0x5F,0x5F,0x14,0x14},
    {0x28,0x2C,0x2E,0x32,0x3A,0x42,0x46,0x4C,0x54,0x5A,0x5E,0x5F,0x5F,0x5F,0x10,0x10},
    {0x28,0x2C,0x2E,0x32,0x3A,0x42,0x48,0x4C,0x54,0x5A,0x5E,0x5F,0x5F,0x5F,0x10,0x10},
    {0x24,0x24,0x24,0x24,0x2C,0x42,0x48,0x4C,0x54,0x5A,0x5C,0x5E,0x5F,0x5F,0x10,0x10},
    {0x24,0x24,0x24,0x24,0x2C,0x42,0x48,0x4C,0x54,0x5A,0x5C,0x5E,0x5F,0x5F,0x10,0x10},
    {0x24,0x24,0x24,0x24,0x2C,0x40,0x46,0x48,0x50,0x54,0x56,0x58,0x58,0x58,0x10,0x10},
    {0x24,0x24,0x24,0x24,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x2C,0x10,0x10},
   },
   {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC}, //CLT correction map

   //Fuel injection maps
   /**Fill VE lookup table, value can be in range 0...1.99 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //16
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //15
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //14
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //13
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //12
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //11
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //10
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 9
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 8
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 7
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 6
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 5
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 4
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 3
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 2
    {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}  // 1
   },
   /**Fill AFR lookup table, value can be in range 8.1...22.0 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //16
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //15
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //14
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //13
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //12
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //11
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //10
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 9
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 8
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 7
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 6
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 5
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 4
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 3
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 2
    {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}  // 1
   },
   /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
   {// -30           -20        -10           0        10            20         30          40
    _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
    //  50            60         70          80        90           100        110         120
    _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
   },
   /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
   {// -30        -20        -10          0        10           20         30        40
    _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
    //  50         60         70         80        90          100        110        120
    _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
   },
   /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
   {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
    _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
    //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
    _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
    // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
    _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
    // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
    _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
   },
   /**Fill IAC/PWM open-loop position lookup table (run mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
    //  50            60         70          80        90           100        110         120
    _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
   },
   /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
   {// -30           -20        -10           0        10            20         30          40
    _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
    //  50            60         70          80        90           100        110         120
    _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
   },
   /**Fill values of the AE's TPS lookup table, range is -55...199% */
   {
    AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
   },
   /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
   {
    AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
   },
   /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
   {
    AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
   },
   /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
   {
    AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
   },
   /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
   {// -30           -20        -10           0        10            20         30          40
    _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
    //  50            60         70          80        90           100        110         120
    _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
   },

   /**Fill injection timing map, value can be in range -360...360 */
   {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //16
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //15
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //14
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //13
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //12
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //11
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //10
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 9
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 8
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 7
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 6
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 5
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 4
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 3
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 2
    {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 1
   },

   /**reserved bytes */
   {0}
  }
 },

 /**Size of this string must be equal to FW_SIGNATURE_INFO_SIZE!
  * Date in format Mmm dd yyyy.
  * Do not forget to write out same value of version into to the fw_version field of cd_data_t!
  */
 {"SECU-3 firmware v4.5. Build ["__DATE__"]       "},

 /**Version of this structure - 0.0*/
 0x00,

 /**Size of all data for checking */
 (sizeof(fw_data_t) - sizeof(cd_data_t)),

 /**Contains check sum for whole firmware */
 0x0000
};

/**Firmware information */
PGM_DECLARE(uint8_t fwinfo[60]) =
{ 0x20,0x53,0x45,0x43,0x55,0x2d,0x33,0x20,0x46,0x69,0x72,0x6d,0x77,0x61,0x72,0x65,
  0x20,0x28,0x43,0x29,0x20,0x32,0x30,0x30,0x37,0x20,0x41,0x2e,0x20,0x53,0x68,0x61,
  0x62,0x65,0x6c,0x6e,0x69,0x6b,0x6f,0x76,0x2c,0x20,0x68,0x74,0x74,0x70,0x3a,0x2f,
  0x2f,0x73,0x65,0x63,0x75,0x2d,0x33,0x2e,0x6f,0x72,0x67,0x20 };

#ifdef REALTIME_TABLES
/**Fill default data for tunable tables */
PGM_DECLARE(f_data_t tt_def_data) =
{
 {'T','u','n','a','b','l','e','_','s','e','t',' ',' ',' ',' ',' '},
 //Ignition maps
 {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x10,0x14,0x14,0x14},  //cranking map
 {0x14,0x14,0x14,0x14,0x14,0x14,0x17,0x1D,0x28,0x32,0x36,0x37,0x37,0x37,0x37,0x37},  //idling map
 {
  {0x04,0x05,0x06,0x07,0x0A,0x0C,0x10,0x15,0x1A,0x20,0x24,0x27,0x28,0x28,0x28,0x28}, //work map
  {0x04,0x05,0x06,0x08,0x0B,0x0D,0x10,0x16,0x1B,0x21,0x26,0x29,0x2A,0x2A,0x2A,0x2A},
  {0x04,0x05,0x08,0x08,0x0C,0x0E,0x12,0x18,0x1E,0x23,0x28,0x2A,0x2C,0x2C,0x2C,0x2C},
  {0x04,0x06,0x08,0x0A,0x0C,0x10,0x14,0x1B,0x21,0x26,0x28,0x2A,0x2C,0x2C,0x2C,0x2D},
  {0x06,0x07,0x0A,0x0C,0x0E,0x12,0x18,0x20,0x26,0x2A,0x2C,0x2D,0x2E,0x2E,0x2F,0x30},
  {0x08,0x08,0x0A,0x0E,0x12,0x16,0x1E,0x27,0x2D,0x2F,0x30,0x31,0x33,0x34,0x35,0x36},
  {0x0A,0x0B,0x0D,0x10,0x14,0x1B,0x24,0x2D,0x32,0x33,0x35,0x37,0x39,0x3A,0x3A,0x3B},
  {0x0C,0x10,0x12,0x15,0x1A,0x21,0x29,0x32,0x36,0x37,0x39,0x3C,0x3E,0x40,0x40,0x40},
  {0x10,0x16,0x18,0x1C,0x22,0x28,0x2E,0x36,0x3A,0x3B,0x3D,0x3F,0x41,0x44,0x44,0x44},
  {0x16,0x1C,0x1E,0x22,0x27,0x2E,0x34,0x3A,0x3D,0x3E,0x40,0x42,0x44,0x48,0x48,0x48},
  {0x1C,0x21,0x22,0x26,0x2A,0x31,0x38,0x3F,0x41,0x42,0x44,0x45,0x48,0x4A,0x4A,0x4B},
  {0x1E,0x22,0x24,0x27,0x2B,0x33,0x3B,0x42,0x45,0x45,0x47,0x48,0x4A,0x4C,0x4C,0x4C},
  {0x20,0x24,0x26,0x29,0x2E,0x36,0x3D,0x45,0x47,0x47,0x48,0x49,0x4A,0x4C,0x4C,0x4D},
  {0x20,0x24,0x27,0x2B,0x31,0x37,0x3F,0x45,0x47,0x48,0x49,0x49,0x4B,0x4D,0x4D,0x4D},
  {0x1E,0x22,0x26,0x2C,0x31,0x39,0x40,0x46,0x48,0x4A,0x4A,0x4B,0x4D,0x4E,0x4E,0x4E},
  {0x1E,0x21,0x25,0x29,0x2F,0x36,0x3F,0x45,0x49,0x4B,0x4C,0x4D,0x4F,0x4F,0x4F,0x4F}
 },
 {0x22,0x1C,0x19,0x16,0x13,0x0F,0x0C,0x0A,0x07,0x05,0x02,0x00,0x00,0xFD,0xF6,0xEC},  //CLT correction map

 //Fuel injection maps
 /**Fill VE lookup table, value can be in range 0...1.99  */
 {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //16
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //15
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //14
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //13
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //12
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //11
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, //10
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 9
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 8
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 7
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 6
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 5
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 4
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 3
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}, // 2
  {_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00),_VE(1.00)}  // 1
 },
 /**Fill AFR lookup table, value can be in range 8.1...22.0 */
 {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //16
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //15
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //14
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //13
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //12
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //11
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, //10
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 9
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 8
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 7
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 6
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 5
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 4
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 3
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}, // 2
  {_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7),_FR(14.7)}  // 1
 },
 /**Fill cranking pulse width lookup table, time in ms vs coolant temperature */
 {// -30           -20        -10           0        10            20         30          40
  _DLV(10.20), _DLV(9.30), _DLV(8.60), _DLV(8.00), _DLV(7.25), _DLV(6.65), _DLV(6.00), _DLV(5.4),
  //  50            60         70          80        90           100        110         120
  _DLV(4.80), _DLV(4.20), _DLV(3.75), _DLV(3.30), _DLV(2.90), _DLV(2.55), _DLV(2.30), _DLV(2.15)
 },
 /**Fill warmup enrichment lookup table, factor(0...199%) vs coolant temperature */
 {// -30         -20        -10         0        10           20         30        40
  _WLV(160), _WLV(156), _WLV(152), _WLV(147), _WLV(142), _WLV(137), _WLV(133), _WLV(129),
  //  50          60         70        80        90          100        110        120
  _WLV(126), _WLV(123), _WLV(120), _WLV(117), _WLV(114), _WLV(110), _WLV(105), _WLV(100)
 },
 /**Fill injector dead time lookup table (Siemens DEKA ZMZ6354), time in ms vs voltage */
 {//  5.4       5.8        6.2        6.6         7.0        7.4        7.8        8.2
  _DLV(5.80),_DLV(4.50),_DLV(3.80),_DLV(3.30),_DLV(3.00),_DLV(2.75),_DLV(2.50),_DLV(2.30),
  //  8.6       9.0        9.4        9.8        10.2       10.6       11.0       11.4
  _DLV(2.12),_DLV(2.00),_DLV(1.90),_DLV(1.82),_DLV(1.75),_DLV(1.67),_DLV(1.60),_DLV(1.50),
  // 11.8      12.2       12.6       13.0        13.4       13.8       14.2       14.6
  _DLV(1.45),_DLV(1.40),_DLV(1.34),_DLV(1.30),_DLV(1.26),_DLV(1.22),_DLV(1.18),_DLV(1.16),
  // 15.0      15.4       15.8        16.2       16.6        17.0      17.4       17.8
  _DLV(1.12),_DLV(1.10),_DLV(1.06),_DLV(1.04),_DLV(1.02),_DLV(0.99),_DLV(0.97),_DLV(0.96)
 },
 /**Fill IAC/PWM open-loop position lookup table (run mode) */
 {// -30           -20        -10           0        10            20         30          40
  _CLV(46.0), _CLV(41.0), _CLV(38.0), _CLV(35.0), _CLV(33.0), _CLV(31.5), _CLV(30.4), _CLV(29.0),
  //  50            60         70          80        90           100        110         120
  _CLV(28.0), _CLV(26.7), _CLV(25.5), _CLV(24.4), _CLV(23.4), _CLV(22.2), _CLV(21.0), _CLV(20.0)
 },
 /**Fill IAC/PWM open-loop position lookup table (cranking mode) */
 {// -30           -20        -10           0        10            20         30          40
  _CLV(65.0), _CLV(60.0), _CLV(55.0), _CLV(50.0), _CLV(46.0), _CLV(42.0), _CLV(38.2), _CLV(36.0),
  //  50            60         70          80        90           100        110         120
  _CLV(33.3), _CLV(31.5), _CLV(30.0), _CLV(28.5), _CLV(27.0), _CLV(25.7), _CLV(24.8), _CLV(24.5)
 },
 /**Fill values of the AE's TPS lookup table, range is -55...199% */
 {
  AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(0.0), AE_TPS_V(1.0), AE_TPS_V(20.0), AE_TPS_V(120.0), AE_TPS_V(160.0)
 },
 /**Fill bins of the AE's TPS lookup table, range is -1000...+1000% / 1s */
 {
  AE_TPS_B(-500.0), AE_TPS_B(-200.0), AE_TPS_B(-100.0), AE_TPS_B(-50.0), AE_TPS_B(50.0), AE_TPS_B(100.0), AE_TPS_B(200.0), AE_TPS_B(500.0)
 },
 /**Fill values of the AE's RPM lookup table, %, range is 0...199% */
 {
  AE_RPM_V(100.0), AE_RPM_V(70.0), AE_RPM_V(40.0), AE_RPM_V(30.0)
 },
 /**Fill bins of the AE's RPM lookup table, range is 0...25000min-1 */
 {
  AE_RPM_B(1000), AE_RPM_B(2000), AE_RPM_B(4000), AE_RPM_B(8000)
 },
 /**Fill afterstart enrichment lookup table, range is 0...199%, value indicates how many % will be added to fuel */
 {// -30           -20        -10           0        10            20         30          40
  _ASE(45.0), _ASE(41.0), _ASE(38.0), _ASE(35.0), _ASE(33.0), _ASE(31.5), _ASE(30.4), _ASE(29.0),
  //  50            60         70          80        90           100        110         120
  _ASE(28.0), _ASE(26.7), _ASE(25.5), _ASE(24.4), _ASE(23.4), _ASE(22.2), _ASE(21.0), _ASE(20.0)
 },

 /**Fill injection timing map, value can be in range -360...360 */
 {//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //16
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //15
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //14
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //13
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //12
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //11
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, //10
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 9
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 8
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 7
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 6
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 5
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 4
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 3
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 2
  {_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0),_IT(   0)}, // 1
 },

 /**reserved bytes */
 {0}
};
#endif
