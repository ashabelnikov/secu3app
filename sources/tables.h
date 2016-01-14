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

/** \file tables.h
 * \author Alexey A. Shabelnikov
 * Tables and datastructures stored in the frmware.
 */

/*Structure of program memory's allocation of the SECU-3 firmware
 *     _  ________________________
 *  c |  |                        |
 *  o |  |       код              |
 *  d |  |       code             |
 *  e |  |------------------------|<--- free space between code and data (usually FF)
 *    |  |                        |
 *  a |  |   free space           |
 *  r |  |                        |
 *  e |  |------------------------|<--- data which is strongly coupled with code (stored in code area)
 *  a |  |                        |
 *    |  |      code data         |
 *    |  |                        |
 *    |_ |------------------------|<--- data which is not strongly coupled with code
 *       |                        |
 *       |      firmware data     |
 *       |                        |
 *       |------------------------|
 *       |     CRC16              |<--- CRC of firmware without 2 bytes of this CRC and boot loader
 *       |________________________|
 *       |                        |
 *       |     boot loader        |
 *        ------------------------
 *
 *       See struct fw_data_t for more information about data layout
 */

#ifndef _TABLES_H_
#define _TABLES_H_

#include "port/pgmspace.h"
#include <stdint.h>
#include "bootldr.h"   //to know value of SECU3BOOTSTART, and only

//Define number of interpolation nodes for lookup tables and othe constants
#define F_WRK_POINTS_F                  16          //!< number of points on RPM axis - work map
#define F_WRK_POINTS_L                  16          //!< number of points on Pressure axis - work map
#define F_TMP_POINTS                    16          //!< number of points in temperature map
#define F_STR_POINTS                    16          //!< number of points in start map
#define F_IDL_POINTS                    16          //!< number of points in idle map
#define F_WRK_TOTAL (F_WRK_POINTS_L*F_WRK_POINTS_F) //!< total size of work map

#define F_NAME_SIZE                     16          //!< number of symbols in names of tables' sets

#define KC_ATTENUATOR_LOOKUP_TABLE_SIZE 128         //!< number of points in attenuator's lookup table
#define FW_SIGNATURE_INFO_SIZE          48          //!< number of bytes reserved for firmware's signature information
#define COIL_ON_TIME_LOOKUP_TABLE_SIZE  32          //!< number of points in lookup table used for dwell control
#define THERMISTOR_LOOKUP_TABLE_SIZE    16          //!< Size of lookup table for coolant temperature sensor
#define CHOKE_CLOSING_LOOKUP_TABLE_SIZE 16          //!< Size of lookup table defining choke closing versus coolant temperature
#define ATS_CORR_LOOKUP_TABLE_SIZE      16          //!< Air temperature sensor advance angle correction lookup table
#define RPM_GRID_SIZE                   16          //!< Number of points on the RPM axis in advance angle lookup tables
#define IBTN_KEYS_NUM                   2           //!< Number of iButton keys
#define IBTN_KEY_SIZE                   6           //!< Size of iButton key (except CRC8 and family code)

#define INJ_VE_POINTS_L                 16          //!< number of points on MAP axis in VE lookup table
#define INJ_VE_POINTS_F                 16          //!< number of points on RPM axis in VE lookup table
#define INJ_DT_LOOKUP_TABLE_SIZE        32          //!< number of points in the injector dead-time lookup table
#define INJ_CRANKING_LOOKUP_TABLE_SIZE  16          //!< number of points in the cranking lookup table
#define INJ_WARMUP_LOOKUP_TABLE_SIZE    16          //!< number of points in the warmup enrichment lookup table
#define INJ_IAC_POS_TABLE_SIZE          16          //!< number of points in the IAC/PWM position lookup table
#define INJ_AE_TPS_LOOKUP_TABLE_SIZE    8           //!< number of points in AE TPS (d%/dt) lookup table
#define INJ_AE_RPM_LOOKUP_TABLE_SIZE    4           //!< number of points in AE RPM lookup table size
#define INJ_AFTSTR_LOOKUP_TABLE_SIZE    16          //!< afterstart enrichment lookup table

#define UNI_OUTPUT_NUMBER               3           //!< number of universal programmable outputs

#define GASDOSE_POS_RPM_SIZE            16          //!< RPM axis size
#define GASDOSE_POS_TPS_SIZE            16          //!< TPS axis size


/**Number of sets of tables stored in the firmware */
#define TABLES_NUMBER_PGM               4

/**Total number of tables' sets (stored in program memory + stored in EEPROM with loading to RAM) */
#ifdef REALTIME_TABLES
 #define TABLES_NUMBER          (TABLES_NUMBER_PGM + 1)
#else
 #define TABLES_NUMBER           TABLES_NUMBER_PGM
#endif


//Bluetooth and security flags (bit numbers)
#define BTF_USE_BT                      0           //!< Bluetooth and security flags: specifies to use or not to use bluetooth
#define BTF_SET_BBR                     1           //!< Bluetooth and security flags: indicates that bluetooth baud rate has to be set during start up
#define BTF_USE_IMM                     2           //!< Bluetooth and security flags: specifies to use or not to use immobilizer

//Injection configuration constants
#define INJCFG_THROTTLEBODY             0           //!< Throttle-body or central injection
#define INJCFG_SIMULTANEOUS             1           //!< Simultaneous port injection
#define INJCFG_2BANK_ALTERN             2           //!< 2 banks alternating injection
#define INJCFG_SEMISEQUENTIAL           3           //!< Semi-sequential injection
#define INJCFG_FULLSEQUENTIAL           4           //!< Full-sequential injection

//Fuel pump flags
#define FPF_OFFONGAS                    0           //!< Turn off fuel pump when fuel type is gas

//Choke flags
#define CKF_OFFSTRTADDONGAS             0           //!< Turn off additional startup closing of choke when fuel type is gas
#define CKF_OFFRPMREGONGAS              1           //!< Turn off RPM regulator when fuel type is gas
#define CKF_USETHROTTLEPOS              2           //!< Use throttle limit switch in choke initialization

//Idling regulator flags
#define IRF_USE_REGULATOR               0           //!< Use regulator (keep selected idling RPM by alternating advance angle)
#define IRF_USE_REGONGAS                1           //!< Use regulator if fuel type is gas

/**Describes one set(family) of chracteristics (maps), discrete = 0.5 degr.*/
typedef struct f_data_t
{
  uint8_t name[F_NAME_SIZE];                        //!< associated name of tables' set, displayed in user interface
  //ignition maps
  int8_t f_str[F_STR_POINTS];                       //!< function of advance angle at start
  int8_t f_idl[F_IDL_POINTS];                       //!< function of advance angle at idling
  int8_t f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     //!< working function of advance angle
  int8_t f_tmp[F_TMP_POINTS];                       //!< coolant temper. correction of advance angle
  //fuel injection maps
  uint8_t inj_ve[INJ_VE_POINTS_L][INJ_VE_POINTS_F]; //!< Volumetric efficiency lookup table, value * 128
  uint8_t inj_afr[INJ_VE_POINTS_L][INJ_VE_POINTS_F];//!< Air-Fuel ratio lookup table, (1/value) * 2048, e.g. 1/14.7 * 2048 = 139
  uint16_t inj_cranking[INJ_CRANKING_LOOKUP_TABLE_SIZE];//!< Injector pulse width used when engine is starting up (cranking)
  uint8_t inj_warmup[INJ_WARMUP_LOOKUP_TABLE_SIZE]; //!< Warmup enrichment lookup table (factor), value * 128, e.g. 128 = 1.00
  uint16_t inj_dead_time[INJ_DT_LOOKUP_TABLE_SIZE]; //!< Injector dead-time lookup table, value in ticks of timer, 1 tick = 3.2uS
  /**Position of the IAC/PWM vs coolant temperature for run mode (used in open-loop idle control)
   * value in % * 2, e.g. 200 = 100.0% */
  uint8_t inj_iac_run_pos[INJ_IAC_POS_TABLE_SIZE];
  /**Position of the IAC/PWM vs coolant temperature for cranking mode (used by both in open and closed-loop idle control)
   * value in % * 2, e.g. 200 = 100.0% */
  uint8_t inj_iac_crank_pos[INJ_IAC_POS_TABLE_SIZE];
  //note! inj_ae_tps_enr must be followed by inj_ae_tps_bins, inj_ae_rpm_enr must be followed by inj_ae_rpm_bins
  uint8_t inj_ae_tps_enr[INJ_AE_TPS_LOOKUP_TABLE_SIZE];  //!< values of the AE's TPS lookup table (additive factor), value + 55, e.g. 155 = 1.00, this means AE = 100% (so PW will be increased by 100%))
  int8_t  inj_ae_tps_bins[INJ_AE_TPS_LOOKUP_TABLE_SIZE]; //!< bins of the AE's TPS lookup table (d%/dt, (signed value in %) / 100ms)
  uint8_t inj_ae_rpm_enr[INJ_AE_RPM_LOOKUP_TABLE_SIZE];  //!< values of the AE's RPM lookup table (factor), value * 128.0, e.g. 128 = 1.00, this means to use 100% of AE)
  uint8_t inj_ae_rpm_bins[INJ_AE_RPM_LOOKUP_TABLE_SIZE]; //!< bins of the AE's RPM lookup table (value / 100, e.g. value=25 means 2500min-1)

  uint8_t inj_aftstr[INJ_AFTSTR_LOOKUP_TABLE_SIZE];      //!< afterstart enrichment vs coolant temperature lookup table, value * 128.0, 128 = 1.00 and means 100% will be adde to fuel

  /* Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */
  uint8_t reserved[702];
}f_data_t;


/**Describes separate tables stored in the firmware
 */
typedef struct fw_ex_data_t
{
  /**Knock. table of attenuator's gain factors (contains codes of gains, gain depends on RPM) */
  uint8_t attenuator_table[KC_ATTENUATOR_LOOKUP_TABLE_SIZE];

  /**Table for dwell control. Accumulation(dwell) time depends on board voltage */
  uint16_t coil_on_time[COIL_ON_TIME_LOOKUP_TABLE_SIZE];

  /**Coolant temperature sensor lookup table */
  int16_t cts_curve[THERMISTOR_LOOKUP_TABLE_SIZE];
  /**Voltage corresponding to the beginning of axis*/
  uint16_t cts_vl_begin;
  /**Voltage corresponding to the end of axis*/
  uint16_t cts_vl_end;

  /**Choke closing versus coolant temperature */
  uint8_t choke_closing[CHOKE_CLOSING_LOOKUP_TABLE_SIZE];

  /**Air temperature sensor lookup table*/
  int16_t ats_curve[THERMISTOR_LOOKUP_TABLE_SIZE];
  /**Voltage corresponding to the beginning of axis*/
  uint16_t ats_vl_begin;
  /**Voltage corresponding to the end of axis*/
  uint16_t ats_vl_end;

  /**Air temperature correction of advance angle*/
  int8_t ats_corr[ATS_CORR_LOOKUP_TABLE_SIZE];

  /**Points of the RPM grid*/
  int16_t rpm_grid_points[RPM_GRID_SIZE];
  /**Sizes of cells in RPM grid (so, we don't need to calculate them at the runtime)*/
  int16_t rpm_grid_sizes[RPM_GRID_SIZE-1];

  /** Gas dose actuator position vs (TPS,RPM)*/
  uint8_t gasdose_pos[GASDOSE_POS_TPS_SIZE][GASDOSE_POS_RPM_SIZE];

  /**Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */
  uint8_t reserved[1792];
}fw_ex_data_t;

/**Describes a unirersal programmable output*/
typedef struct uni_output_t
{
 uint8_t flags;                          //!< MS Nibble - logic function, LS Nibble - flags (inversion)
 uint8_t condition1;                     //!< code of condition 1
 uint8_t condition2;                     //!< code of condition 2
 uint16_t on_thrd_1;                     //!< ON threshold (if value > on_thrd_1)
 uint16_t off_thrd_1;                    //!< OFF threshold (if value < off_thrd_1)
 uint16_t on_thrd_2;                     //!< same as on_thrd_1
 uint16_t off_thrd_2;                    //!< same as off_thrd_1
}uni_output_t;


/**Describes system's parameters. One instance of this structure stored in the EEPROM and one
 * in the FLASH (program memory) */
typedef struct params_t
{
  // Cranking
  uint16_t starter_off;                  //!< RPM when starter will be turned off
  uint16_t smap_abandon;                 //!< RPM when switching from start map(min-1)

  // MAP sensor
  uint16_t map_lower_pressure;           //!< lower value of MAP at the axis of table(work map) (kPa)
  int16_t  map_upper_pressure;           //!< upper value of MAP at the axis of table(work map) (kPa)
  int16_t  map_curve_offset;             //!< offset of curve in volts, can be negative
  int16_t  map_curve_gradient;           //!< gradient of curve in kPa/V, can be negative (inverse characteristic curve)

  // TPS sensor/limit switch
  uint8_t  carb_invers;                  //!< flag of inversion of carburetor's limit switch
  int16_t  tps_curve_offset;             //!< offset of curve in volts
  int16_t  tps_curve_gradient;           //!< gradient of curve in Percentage/V
  uint8_t  tps_threshold;                //!< TPS threshold used to switch work and idle modes (if 0 then input is treated as digital and simple switch is used)

  // Idle cut-off valve and power valve
  uint16_t ie_lot;                       //!< lower threshold for idle cut off valve(min-1) for gasoline
  uint16_t ie_hit;                       //!< upper threshold for idle cut off valve(min-1) for gasoline
  uint16_t ie_lot_g;                     //!< lower threshold for idle cut off valve(min-1) for gas
  uint16_t ie_hit_g;                     //!< upper threshold for idle cut off valve(min-1) for gas
  int16_t  fe_on_threshold;              //!< switch on threshold of power valve (FE)
  uint8_t  shutoff_delay;                //!< idle cut off valve's turn off delay
  uint16_t fuelcut_map_thrd;             //!< fuel cut off MAP threshold
  int16_t fuelcut_cts_thrd;              //!< fuel cut off CTS threshold

  // Advance angle control
  int16_t  angle_dec_speed;              //!< limitation of alternation speed of advance angle (when decreasing)
  int16_t  angle_inc_speed;              //!< limitation of alternation speed of advance angle (when increasing)
  int16_t  max_angle;                    //!< system's maximum advance angle limit
  int16_t  min_angle;                    //!< system's minimum advance angle limit
  int16_t  angle_corr;                   //!< octane correction of advance angle
  uint8_t  zero_adv_ang;                 //!< Zero advance angle flag

  uint8_t  fn_gasoline;                  //!< index of set of characteristics used for gasoline
  uint8_t  fn_gas;                       //!< index of set of characteristics used for gas

  // Idling regulator (via advance angle)
  uint8_t  idl_flags;                    //!< Idling regulator flags (see IRF_x constants for more information)
  uint16_t idling_rpm;                   //!< selected idling RPM regulated by using advance angle
  int16_t  ifac1;                        //!< Idling regulator's factor for positive error
  int16_t  ifac2;                        //!< Idling regulator's factor for negative error
  int16_t  MINEFR;                       //!< dead band of idling regulator (min-1)
  int16_t  idlreg_min_angle;             //!< minimum advance angle correction which can be produced by idling regulator
  int16_t  idlreg_max_angle;             //!< maximum advance angle correction which can be produced by idling regulator
  int16_t  idlreg_turn_on_temp;          //!< Idling regulator turn on temperature

  // Temperature and cooling fan control
  uint8_t  tmp_use;                      //!< flag of using coolant sensor
  int16_t  vent_on;                      //!< cooling fan's turn on temperature
  int16_t  vent_off;                     //!< cooling fan's turn off temperature
  uint8_t  vent_pwm;                     //!< flag - control cooling fan by using PWM
  uint8_t  cts_use_map;                  //!< Flag which indicates using of lookup table for coolant temperature sensor
  uint16_t vent_pwmfrq;                  //!< PWM frequency (value = 1/f * 524288), 10....5000Hz

  // ADC corrections/compensations
  int16_t  map_adc_factor;               //!< ADC error compensation factor for MAP
  int32_t  map_adc_correction;           //!< ADC error compensation correction for MAP
  int16_t  ubat_adc_factor;              //!< ADC error compensation factor for board voltage
  int32_t  ubat_adc_correction;          //!< ADC error compensation correction for board voltage
  int16_t  temp_adc_factor;              //!< ADC error compensation factor for CLT
  int32_t  temp_adc_correction;          //!< ADC error compensation correction for CLT
  int16_t  tps_adc_factor;               //!< ADC error compensation factor for TPS
  int32_t  tps_adc_correction;           //!< ADC error compensation correction for TPS
  int16_t  ai1_adc_factor;               //!< ADC error compensation factor for ADD_IO1 input
  int32_t  ai1_adc_correction;           //!< ADC error compensation correction for ADD_IO1 input
  int16_t  ai2_adc_factor;               //!< ADC error compensation factor for ADD_IO2 input
  int32_t  ai2_adc_correction;           //!< ADC error compensation correction for ADD_IO2 input

  // Synchronization
  uint8_t  ckps_edge_type;               //!< Edge type for interrupt from CKP sensor (rising or falling edge). Depends on polarity of sensor
  uint8_t  ckps_cogs_btdc;               //!< Teeth before TDC
  uint8_t  ckps_ignit_cogs;              //!< Duration of ignition driver's pulse countable in teeth of wheel
  uint8_t  ckps_engine_cyl;              //!< number of engine's cylinders
  uint8_t  ckps_cogs_num;                //!< number of crank wheel's teeth
  uint8_t  ckps_miss_num;                //!< number of missing crank wheel's teeth
  uint8_t  ref_s_edge_type;              //!< Edge type of REF_S input
  uint8_t  hall_flags;                   //!< Hall sensor related flags
  int16_t  hall_wnd_width;               //!< Hall sensor's shutter window width in degrees of crankshaft (advance value of distributor)

  // Ignition outputs control
  uint8_t  ign_cutoff;                   //!< Cutoff ignition when RPM reaches specified threshold
  uint16_t ign_cutoff_thrd;              //!< Cutoff threshold (RPM)
  uint8_t  merge_ign_outs;               //!< Merge ignition signals to single output flag

  // Hall emulation output
  int8_t   hop_start_cogs;               //!< Hall output: start of pulse in teeth relatively to TDC 
  uint8_t  hop_durat_cogs;               //!< Hall output: duration of pulse in teeth

  // Communication
  uint16_t uart_divisor;                 //!< divider which corresponds to selected baud rate
  uint8_t  uart_period_t_ms;             //!< transmition period of data packets which SECU-3 sends, one discrete = 10ms

  // Knock control
  uint8_t  knock_use_knock_channel;      //!< flag of using knock channel
  uint8_t  knock_bpf_frequency;          //!< Band pass filter frequency
  int16_t  knock_k_wnd_begin_angle;      //!< Opening angle of knock phase window
  int16_t  knock_k_wnd_end_angle;        //!< Closing angle of knock phase window
  uint8_t  knock_int_time_const;         //!< Integration time constant
  int16_t  knock_retard_step;            //!< Displacement(retard) step of angle
  int16_t  knock_advance_step;           //!< Recovery step of angle
  int16_t  knock_max_retard;             //!< Maximum displacement of angle
  uint16_t knock_threshold;              //!< detonation threshold - voltage
  uint8_t  knock_recovery_delay;         //!< Recovery delay of angle countable in engine's cycles

  // Choke control
  uint16_t sm_steps;                     //!< Number of steps of choke stepper motor
  uint16_t choke_rpm[2];                 //!< Values of RPM needed for RPM-based control of choke position
  uint8_t  choke_startup_corr;           //!< Startup correction value for choke (0...200)
  uint16_t choke_rpm_if;                 //!< Integral factor for RPM-based control of choke position (factor * 1024)
  uint16_t choke_corr_time;              //!< Time for startup correction will be applied
  int16_t  choke_corr_temp;              //!< Temperature threshold for startup correction

  // Bluetooth and security
  uint8_t  bt_flags;                     //!< Bluetooth and security related flags
  uint8_t  ibtn_keys[IBTN_KEYS_NUM][IBTN_KEY_SIZE]; //!< iButton keys for immobilizer

  uni_output_t uni_output[UNI_OUTPUT_NUMBER]; //!< parameters for versatile outputs
  uint8_t uniout_12lf;                   //!< logic function between 1st and 2nd outputs

  // Fuel injection
  uint8_t  inj_flags;                    //!< Fuel injection related flags
  uint8_t  inj_config;                   //!< Configuration of injection (7-4 bits: inj. config., 3-0 bits: num of squitrs), inj.config: INJCFG_x constants
  uint16_t inj_flow_rate;                //!< Injector flow rate (cc/min) * 64
  uint16_t inj_cyl_disp;                 //!< The displacement of one cylinder in liters * 16384
  uint32_t inj_sd_igl_const;             //!< Constant used in speed-density algorithm to calculate PW. Const = ((CYL_DISP * 3.482 * 18750000) / Ifr ) * ((Nbnk * Ncyl) / (Nsq * Ninj))

  uint16_t inj_prime_cold;               //!< Prime pulse PW at cold (CLT=-30°C)
  uint16_t inj_prime_hot;                //!< Prime pulse PW at hot (CLT=70°C)
  uint8_t  inj_prime_delay;              //!< Prime pulse delay in 0.1 sec units

  uint16_t inj_cranktorun_time;          //!< Time in seconds for going from the crank position to the run position (1 tick = 10ms)
  uint8_t  inj_aftstr_strokes;           //!< Number of engine strokes, during this time afterstart enrichment is applied

  uint8_t  inj_lambda_str_per_stp;       //!< Number of strokes per step for lambda control
  uint8_t  inj_lambda_step_size_p;       //!< "+" Step size, value * 512, max 0.49
  uint16_t inj_lambda_corr_limit_p;      //!< "+" limit, value * 512
  uint16_t inj_lambda_swt_point;         //!< lambda switch point in volts
  int16_t  inj_lambda_temp_thrd;         //!< Coolant temperature activation threshold
  uint16_t inj_lambda_rpm_thrd;          //!< RPM activation threshold
  uint8_t  inj_lambda_activ_delay;       //!< Lambda sensor activation delay

  uint8_t  inj_ae_tpsdot_thrd;           //!< TPS %/sec threshold, max rate is 255%/sec
  uint8_t  inj_ae_coldacc_mult;          //!< Cold acceleration multiplier (-30°C), (value - 1.0) * 128

  uint16_t gd_steps;                     //!< Number of steps of gas dosator stepper motor

  int16_t  inj_timing;                   //!< Injection timing in crank degrees * ANGLE_MULTIPLIER

  uint8_t  flpmp_flags;                  //!< fuel pump flags

  uint8_t  choke_flags;                  //!< choke related flags (see CKF_x constants for more information)

  uint16_t revlim_lot;                   //!< lower threshold for rev.limitting (fuel injection)
  uint16_t revlim_hit;                   //!< upper threshold for rev.limitting (fuel injection)

  int16_t  inj_timing_crk;               //!< Injection timing on cranking in crank degrees * ANGLE_MULTIPLIER

  uint8_t  gd_fc_closing;                //!< How much close (in %) gas doser in fuel cut mode (relatively to current position)

  uint8_t  inj_lambda_step_size_m;       //!<"-" Step size, value * 512, max 0.49

  uint16_t inj_lambda_corr_limit_m;      //!<"-" limit, value * 512

  /**Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */
  uint8_t  reserved[70];

  /**CRC of this structure (for checking correctness of data after loading from EEPROM) */
  uint16_t crc;
}params_t;

//Define data structures are related to code area data and IO remapping data
typedef uint16_t fnptr_t;                //!< Special type for function pointers
#define IOREM_SLOTS  37                  //!< Number of slots used for I/O remapping
#define IOREM_PLUGS  68                  //!< Number of plugs used in I/O remapping

/**Describes all data related to I/O remapping */
typedef struct iorem_slots_t
{
 fnptr_t i_slots[IOREM_SLOTS];           //!< initialization slots
 fnptr_t i_slotsi[IOREM_SLOTS];          //!< initialization slots (inverted)
 fnptr_t v_slots[IOREM_SLOTS];           //!< data slots
 fnptr_t v_slotsi[IOREM_SLOTS];          //!< data slots           (inverted)
 fnptr_t i_plugs[IOREM_PLUGS];           //!< initialization plugs
 fnptr_t v_plugs[IOREM_PLUGS];           //!< data plugs
 fnptr_t s_stub;                         //!< special pointer used as stub
 fnptr_t g_stub;                         //!< reserved
 uint8_t version;                        //!< version of this structure (used for compatibility checkings)
 uint16_t size;                          //!< size of this structure (used for compatibility checkings)
}iorem_slots_t;

/**Describes all the data residing directly in the code area.*/
typedef struct cd_data_t
{
 /** Arrays which are used for I/O remapping. Some arrays are "slots", some are "plugs" */
 iorem_slots_t iorem;

 /**Holds flags which give information about options were used to build firmware */
 uint32_t config;

 uint8_t reserved[3];                    //!< reserved bytes

 uint8_t fw_version;                     //!< version of the firmware

 uint16_t size;                          //!< size of this structure (used for compatibility checkings)
}cd_data_t;


/**Описывает все данные находящиеся в прошивке 
 * Describes all data residing in firmware */
typedef struct fw_data_t
{
 cd_data_t cddata;                       //!< All data which is strongly coupled with code

 //following fields are belong to data area, not to the code area:
 params_t def_param;                     //!< Reserve parameters (loaded when instance in EEPROM is broken)

 fw_ex_data_t exdata;                    //!< Additional data containing separate tables

 f_data_t tables[TABLES_NUMBER_PGM];     //!< Tables' sets for advance angle and fuel injection

 uint8_t fw_signature_info[FW_SIGNATURE_INFO_SIZE];//!< Signature information (contains information about firmware)

 uint8_t version;                        //!< version of this structure

 uint16_t fw_data_size;                  //!< Used for checking compatibility with mngmt software. Holds size of all data stored in the firmware.

 uint16_t code_crc;                      //!< Check sum of the whole firmware (except this check sum and boot loader)
}fw_data_t;

//================================================================================
/**Size of CRC variable of parameters in bytes (used in params_t structure) */
#define PAR_CRC_SIZE   sizeof(uint16_t)

/**Size of CRC variable of whole firmware in bytes */
#define CODE_CRC_SIZE  sizeof(uint16_t)

/**Size of application's section without taking into account its CRC */
#define CODE_SIZE (SECU3BOOTSTART-CODE_CRC_SIZE)

/*Define address of data in the firmware starting from the boot loader's address */
#define FIRMWARE_DATA_START (SECU3BOOTSTART-sizeof(fw_data_section_t))

//================================================================================
//Variables:

/**All firmware data */
PGM_FIXED_ADDR_OBJ(extern fw_data_t fw_data, ".firmware_data");

#ifdef REALTIME_TABLES
/**Default data for tunable tables stored in the EEPROM */
PGM_DECLARE(extern f_data_t tt_def_data);
#endif

#endif //_TABLES_H_
