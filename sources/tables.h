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
 *  o |  |       ÍÓ‰              |
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
#define ATS_CORR_LOOKUP_TABLE_SIZE      16          //!< Air temperature sensor advance angle correction lookup table
#define RPM_GRID_SIZE                   16          //!< Number of points on the RPM axis in advance angle lookup tables
#define CLT_GRID_SIZE                   16          //!< Number of points on the CLT axis in the lookup tables
#define LOAD_GRID_SIZE                  16          //!< Number of points on the load axis in 3D lookup tables
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
#define INJ_TARGET_RPM_TABLE_SIZE       16          //!< idling target RPM lookup table size
#define INJ_IDL_RIGIDITY_SIZE           8           //! size of the idling regulator's rigidity function lookup table
#define INJ_EGO_CURVE_SIZE              16          //!< oxygen sensor AFR curve
#define INJ_IAC_CORR_W_SIZE             16          //!< IAC correction weight lookup table size
#define INJ_IAC_CORR_SIZE               8           //!< IAC correction lookup table size
#define INJ_IATCLT_CORR_SIZE            8           //!< IAT/CLT correction lookup table size
#define BAROCORR_SIZE                   9           //!< Barometric correction map size
#define PA4_LOOKUP_TABLE_SIZE           16          //!< Manual ign.timing map size

#define UNI_OUTPUT_NUMBER               6           //!< number of universal programmable outputs

#define GASDOSE_POS_RPM_SIZE            16          //!< RPM axis size
#define GASDOSE_POS_TPS_SIZE            16          //!< TPS axis size
#define INJ_TPSSWT_SIZE                 16          //!< Size of the TPS switch point lookup table
#define INJ_TPSZON_SIZE                 16          //!< Size of the MAP/TPS load axis aloocation table
#define INJ_GTS_CORR_SIZE               16          //!< Size of gas temperature correction map
#define INJ_GPS_CORR_SIZE               17          //!< Size of gas pressure correction map
#define INJ_ATS_CORR_SIZE               16          //!< Size of air temperature correction map
#define CTS_CRKCORR_SIZE                16          //!< Size of cranking ign.timing correction vs CLT map

#define CRANK_THRD_SIZE                 16          //!< Size of the cranking threshold vs coolant temp. table
#define CRANK_TIME_SIZE                 16          //!< Size of the cranking time vs coolant temp. table
#define SMAPABAN_THRD_SIZE              16          //!< Size of the start map abandon threshold vs coolant temp. table

#define KNKZONE_TPS_SIZE                16          //! Number of points along tps axis in the knock zones look up table

#define INPUTNUM                        14          //!< number of ring buffers (analog inputs)

#define PWMIAC_UCOEF_SIZE               16          //!< size of PWM IAC duty coefficient vs board voltage map
#define AFTSTR_STRK_SIZE                16

#define FTLS_LOOKUP_TABLE_SIZE          17          //!< Size of "fuel tank level vs voltage" map
#define EGTS_LOOKUP_TABLE_SIZE          17          //!< Size of "exhaust gas temperature vs voltage" map
#define OPS_LOOKUP_TABLE_SIZE           17          //!< Size of "oil pressure vs voltage" map

#define MAF_FLOW_CURVE_SIZE             64          //!< Size of the MAF's flow curve lookup table

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
#define BTF_USE_RESPAR                  3           //!< Use reserve parameters instead of parameters stored in the EEPROM
#define BTF_CHK_FWCRC                   4           //!< Check firmware CRC (time consuming operation)
#define BTF_BT_TYPE0                    5           //!< Bluetooth chip type (2 bits): 0 - BC417, 1 - BK3231, 2 - BK3231S (JDY-31)
#define BTF_BT_TYPE1                    6           //!< See BTF_BT_TYPE0

//Injection configuration constants
#define INJCFG_THROTTLEBODY             0           //!< Throttle-body or central injection
#define INJCFG_SIMULTANEOUS             1           //!< Simultaneous port injection
#define INJCFG_2BANK_ALTERN             2           //!< 2 banks alternating injection
#define INJCFG_SEMISEQUENTIAL           3           //!< Semi-sequential injection
#define INJCFG_FULLSEQUENTIAL           4           //!< Full-sequential injection
#define INJCFG_SEMISEQSEPAR             5           //!< semi-sequential injection with separate channels

//Inj.pulse origin
#define INJANGLESPEC_BEGIN              0           //!< Beginning of pulse
#define INJANGLESPEC_MIDDLE             1           //!< Middle of pulse
#define INJANGLESPEC_END                2           //!< End of pulse

//Injection flags (see inj_flags variable)
#define INJFLG_USETIMINGMAP             0           //!< Use injection timing map instead of simple constant
#define INJFLG_USETIMINGMAP_G           1           //!< Use injection timing map instead of simple constant
#define INJFLG_USEADDCORRS              2           //!< Use additional corrections required for precise gas injection
#define INJFLG_USEAIRDEN                3           //!< Use iar density correction map
#define INJFLG_USEDIFFPRESS             4           //!< Use differential pressure for correction from GPS
#define INJFLG_SECINJROWSWT             5           //!< Switch to second inj. row when switching to second fuel type

//Fuel pump flags
#define FPF_OFFONGAS                    0           //!< Turn off fuel pump when fuel type is gas
#ifdef FUEL_INJECT
#define FPF_INJONGAS                    1           //!< Turn off injectors when fuel type is gas (second fuel)
#define FPF_INJONPET                    2           //!< Turn off injectors when fuel type is petrol (first fuel)
#endif

//Choke flags
#define CKF_USECLRPMREG                 0           //!< Use closed loop RPM regulator
#define CKF_OFFRPMREGONGAS              1           //!< Turn off RPM regulator when fuel type is gas
#define CKF_USETHROTTLEPOS              2           //!< Use throttle limit switch in choke initialization
#define CKF_MAXFREQINIT                 3           //!< Use maximum frequency at initialization

//Idling regulator flags
#define IRF_USE_REGULATOR               0           //!< Use regulator (keep selected idling RPM by alternating advance angle)
#define IRF_USE_REGONGAS                1           //!< Use regulator if fuel type is gas
#define IRF_USE_INJREG                  2           //!< Using of closed loop mode for IAC valve (fuel injection only)
#define IRF_PREG_MODE                   3           //!< Use P-regulator instead on I-regulator
#define IRF_USE_CLONGAS                 4           //!< Use closed loop on gas

//CKPS flags
#define CKPF_RISING_SPARK               0           //!< Generate rising edge of ignition pulse on spark
#define CKPF_USE_CAM_REF                1           //!< Use cam sensor as reference sensor
#define CKPF_CKPS_EDGE                  2           //!< Edge type for interrupt from CKP sensor (rising or falling edge). Depends on polarity of sensor
#define CKPF_REFS_EDGE                  3           //!< Edge type of REF_S input
#define CKPF_MERGE_OUTS                 4           //!< Merge ignition signals to single output flag

//Temperature tab flags
#define TMPF_CLT_USE                    0           //!< Flag of using coolant temperature sensor
#define TMPF_CLT_MAP                    1           //!< Flag which indicates using of lookup table for coolant temperature sensor
#define TMPF_VENT_PWM                   2           //!< Flag - control cooling fan by using PWM

//lambda flags
#define LAMFLG_HTGDET                   0           //!< Determine oxygen sensor's heating by monitoring of voltage from it
#define LAMFLG_IDLCORR                  1           //!< Use lambda correction on idling
#define LAMFLG_CRKHEAT                  2           //!< Use heating before cranking

//Functions tab flags
#define FUNC_LDAX_GRID                  0           //!< Use grid table for load axis (load grid is defined by two values (default) or load grid is defined by table if this flag is set)

//VE2 map functions
#define VE2MF_1ST                       0           //!< VE = VE1 (default)
#define VE2MF_MUL                       1           //!< VE = VE1 * VE2
#define VE2MF_ADD                       2           //!< VE = VE1 + VE2

#define INJPWCOEF_LUT_SIZE              17          //!< inj. PW coefficient look up table size

//Ignition timing flags
#define IGNTF_WRKMAP                    0           //!< 1 - always use work map for ignition timing (idle map will be not used), 0 - regular behaviour (work map for working mode, idle map for idling mode)
#define IGNTF_MANIDL                    1           //!< 0 - don't apply manual ignition timing correction on idling, 1 - apply manual ign. tim. correction on idling

//Cranking flags
#define STRTF_FLDCLRSTR                 0           //!< allow start of engine in flood clear mode (0 - not allowed, 1 - allowed)

/**Describes one set(family) of chracteristics (maps) */
typedef struct f_data_t
{
  uint8_t name[F_NAME_SIZE];                        //!< associated name of tables' set, displayed in user interface
  //ignition maps
  int8_t f_str[F_STR_POINTS];                       //!< function of advance angle at start, discrete = 0.5 degr.
  int8_t f_idl[F_IDL_POINTS];                       //!< function of advance angle at idling, discrete = 0.5 degr.
  int8_t f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     //!< working function of advance angle, discrete = 0.5 degr.
  int8_t f_tmp[F_TMP_POINTS];                       //!< coolant temper. correction of advance angle, discrete = 0.5 degr.
  //fuel injection maps
  uint8_t inj_ve[INJ_VE_POINTS_L][(INJ_VE_POINTS_F*3)/2]; //!< Volumetric efficiency lookup table, value * 2048 (12-bit)
  uint8_t inj_afr[INJ_VE_POINTS_L][INJ_VE_POINTS_F];//!< Air-Fuel ratio lookup table, (value - 8) * 16
  uint8_t inj_timing[INJ_VE_POINTS_L][(INJ_VE_POINTS_F*3)/2]; //!< injection timing in crankshaft degrees, value * 2 (12-bit), 0...1080 deg.

  uint16_t inj_cranking[INJ_CRANKING_LOOKUP_TABLE_SIZE];//!< Injector pulse width used when engine is starting up (cranking)
  uint8_t inj_warmup[INJ_WARMUP_LOOKUP_TABLE_SIZE]; //!< Warmup enrichment lookup table (factor), value * 128, e.g. 128 = 1.00
  int16_t inj_dead_time[INJ_DT_LOOKUP_TABLE_SIZE]; //!< Injector dead-time lookup table, value in ticks of timer, 1 tick = 3.2uS
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

  uint8_t inj_target_rpm[INJ_TARGET_RPM_TABLE_SIZE];  //!< target RPM on idling (value / 10)

  uint16_t inj_idl_rigidity[INJ_IDL_RIGIDITY_SIZE];   //!< table containing idling regulator's rigidity function (value * 128)

  //note! inj_ego_curve must be followed by ego_vl_begin and ego_vl_end values!
  uint16_t inj_ego_curve[INJ_EGO_CURVE_SIZE+2];       //!< Air-Fuel ratio lookup table, value * 128, the last two values are voltages corresponding to the beginning and to the end of axis (ADC discretes)

  uint8_t inj_iac_corr_w[INJ_IAC_CORR_W_SIZE+2];      //!< IAC correction weight lookup table (value * 256), the last two values are TPS corresponding to the beginning and to the end of axis

  int16_t inj_iac_corr[INJ_IAC_CORR_SIZE+2];          //!< IAC correction lookup table (value * 8192), the last two values are IAC positions corresponding to the beginning and to the end of axis

  uint16_t inj_iatclt_corr[INJ_IATCLT_CORR_SIZE+2];   //!< IAT/CLT correction lookup table (value * 8192), the last two values are air flows (load*rpm) corresponding to the beginning and to the end of axis

  uint8_t inj_tpsswt[INJ_TPSSWT_SIZE];                //!< Speed-density/Alpha-N switch point vs RPM

  uint8_t inj_gts_corr[INJ_GTS_CORR_SIZE];            //!< PW correction from gas temperature sensor, value * 128, max. 1.99

  uint8_t inj_gps_corr[INJ_GPS_CORR_SIZE+2];          //!< PW correction from gas pressure sensor, value * 128, max. 1.99, last two values are pressures (value in kPa / 2)

  uint8_t inj_ats_corr[INJ_ATS_CORR_SIZE];            //!< PW correction from air temperature sensor (air density correction), value * 128, max. 1.99

  uint8_t pwm_duty1[F_WRK_POINTS_L][F_WRK_POINTS_F];  //!< PWM1 duty lookup table, value 0...255. It is also used for split angle (firmware with SPLIT_ANGLE option), sign. value * 2
  uint8_t map_mode;

  uint8_t pwm_duty2[F_WRK_POINTS_L][F_WRK_POINTS_F];  //!< PWM2 duty lookup table, value 0...255

  int8_t f_tmp_idl[F_TMP_POINTS];                     //!< coolant temper. correction of advance angle, discrete = 0.5 degr.

  int8_t iac_mat_corr[INJ_ATS_CORR_SIZE];             //!< IAC position correction vs MAT, discrete = 0.25% (signed value * 4)

  uint8_t inj_ve2[INJ_VE_POINTS_L][(INJ_VE_POINTS_F*3)/2]; //!< Secondary volumetric efficiency lookup table, value * 2048 (12-bit)

  uint8_t inj_tpszon[INJ_TPSZON_SIZE];                //!< Speed-density/Alpha-N load axis allocation

  /* Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */
  uint8_t reserved[69];

  uint16_t checksum;                                  //!< CRC16 checksum of this structure (except these 16 bits)
}f_data_t;


/**Check Engine settings data*/
typedef struct ce_sett_t
{
 uint16_t map_v_min;                                //!< mininum correct value
 uint16_t map_v_max;                                //!< maximum correct value
 uint16_t map_v_em;                                 //!< emergency value, used in case of error
 uint8_t  map_v_flg;

 uint16_t vbat_v_min;
 uint16_t vbat_v_max;
 uint16_t vbat_v_em;
 uint8_t  vbat_v_flg;

 uint16_t cts_v_min;
 uint16_t cts_v_max;
 uint16_t cts_v_em;
 uint8_t  cts_v_flg;

 uint16_t ks_v_min;
 uint16_t ks_v_max;
 uint16_t ks_v_em;
 uint8_t  ks_v_flg;

 uint16_t tps_v_min;
 uint16_t tps_v_max;
 uint16_t tps_v_em;
 uint8_t  tps_v_flg;

 uint16_t add_i1_v_min;
 uint16_t add_i1_v_max;
 uint16_t add_i1_v_em;
 uint8_t  add_i1_v_flg;

 uint16_t add_i2_v_min;
 uint16_t add_i2_v_max;
 uint16_t add_i2_v_em;
 uint8_t  add_i2_v_flg;

 uint16_t add_i3_v_min;
 uint16_t add_i3_v_max;
 uint16_t add_i3_v_em;
 uint8_t  add_i3_v_flg;

 uint16_t add_i4_v_min;
 uint16_t add_i4_v_max;
 uint16_t add_i4_v_em;
 uint8_t  add_i4_v_flg;

 uint16_t add_i5_v_min;
 uint16_t add_i5_v_max;
 uint16_t add_i5_v_em;
 uint8_t  add_i5_v_flg;

 uint16_t add_i6_v_min;
 uint16_t add_i6_v_max;
 uint16_t add_i6_v_em;
 uint8_t  add_i6_v_flg;

 uint16_t add_i7_v_min;
 uint16_t add_i7_v_max;
 uint16_t add_i7_v_em;
 uint8_t  add_i7_v_flg;

 uint16_t add_i8_v_min;
 uint16_t add_i8_v_max;
 uint16_t add_i8_v_em;
 uint8_t  add_i8_v_flg;

 uint16_t oilpress_thrd;
 uint16_t oilpress_timer;
}ce_sett_t;

/**Describes separate tables stored in the firmware
 */
typedef struct fw_ex_data_t
{
  /**Knock. table of attenuator's gain factors (contains codes of gains, gain depends on RPM) */
  uint8_t attenuator_table[KC_ATTENUATOR_LOOKUP_TABLE_SIZE];

  /**Table for dwell control. Accumulation(dwell) time depends on board voltage */
  uint16_t coil_on_time[COIL_ON_TIME_LOOKUP_TABLE_SIZE];

  /**Coolant temperature sensor lookup table, last two values: voltages corresponding to the beginning and end of x-axis */
  int16_t cts_curve[THERMISTOR_LOOKUP_TABLE_SIZE+2];

  /**Air temperature sensor lookup table, last two values: voltages corresponding to the beginning and end of x-axis */
  int16_t ats_curve[THERMISTOR_LOOKUP_TABLE_SIZE+2];

  /**Air temperature correction of advance angle*/
  int8_t ats_corr[ATS_CORR_LOOKUP_TABLE_SIZE];

  /**Points of the RPM grid*/
  int16_t rpm_grid_points[RPM_GRID_SIZE];
  /**Sizes of cells in RPM grid (so, we don't need to calculate them at the runtime)*/
  int16_t rpm_grid_sizes[RPM_GRID_SIZE-1];

  /** Gas dose actuator position vs (TPS,RPM)*/
  uint8_t gasdose_pos[GASDOSE_POS_TPS_SIZE][GASDOSE_POS_RPM_SIZE];

  /**CE settings data*/
  ce_sett_t cesd;

  /**Barometric correction map (value * 4096), last two values are pressures (kPa*64) corresponding to the beginning and to the end of axis*/
  uint16_t barocorr[BAROCORR_SIZE+2];

  /**Ignition timing vs voltage. By default linear function with small dead band near to 2.5V */
  int8_t pa4_igntim_corr[PA4_LOOKUP_TABLE_SIZE];

  /**TEMP2 sensor's temperature vs voltage. 16 points of function, plus two values for setting of x-axis range*/
  int16_t tmp2_curve[THERMISTOR_LOOKUP_TABLE_SIZE+2];

  /**CLT temperature correction of advance angle on cranking*/
  int8_t cts_crkcorr[CTS_CRKCORR_SIZE];

  /**Value of pause in seconds (10ms units) vs board voltage*/
  uint8_t eh_pause[COIL_ON_TIME_LOOKUP_TABLE_SIZE];

  /**Value of RPM (in 10 min-1 units) vs coolant temperature*/
  uint8_t cranking_thrd[CRANK_THRD_SIZE];

  /**Number of strokes vs coolant temperature*/
  uint8_t cranking_time[CRANK_TIME_SIZE];

  /**Value of RPM (in 10 min-1 units) vs coolant temperature*/
  uint8_t smapaban_thrd[SMAPABAN_THRD_SIZE];

  /**Points of the RPM grid*/
  int16_t clt_grid_points[CLT_GRID_SIZE];
  /**Sizes of cells in RPM grid (so, we don't need to calculate them at the runtime)*/
  int16_t clt_grid_sizes[CLT_GRID_SIZE-1];

  /**Knock zones vs rpm, tps*/
  uint16_t knock_zones[KNKZONE_TPS_SIZE];

  /**GRTEMP sensor's temperature vs voltage. 16 points of function, plus two values for setting of x-axis range*/
  int16_t grts_curve[THERMISTOR_LOOKUP_TABLE_SIZE+2];

  /**Gas reducer's heating control duty, value in % * 2*/
  uint8_t grheat_duty[F_TMP_POINTS];

  /**PWM IAC duty coefficient vs board voltage map, value * 4096*/
  uint16_t pwmiac_ucoef[PWMIAC_UCOEF_SIZE];

  /**Points of the load grid*/
  int16_t load_grid_points[LOAD_GRID_SIZE];

  /**Sizes of cells in load grid (so, we don't need to calculate them at the runtime)*/
  int16_t load_grid_sizes[LOAD_GRID_SIZE-1];

  /**After start enrichment strokes vs coolant temperature - petrol*/
  uint16_t inj_aftstr_strk0[AFTSTR_STRK_SIZE];

  /**After start enrichment strokes vs coolant temperature - gas*/
  uint16_t inj_aftstr_strk1[AFTSTR_STRK_SIZE];

  /**Gas valve's opening delay vs gas reducer's temperature*/
  uint16_t grv_delay[F_TMP_POINTS];

 /**Fuel tank level vs voltage. 16 points of function, plus two values for setting of x-axis range*/
  int16_t ftls_curve[FTLS_LOOKUP_TABLE_SIZE+2];

 /**Exhaust gas temperature vs voltage. 16 points of function, plus two values for setting of x-axis range*/
  int16_t egts_curve[EGTS_LOOKUP_TABLE_SIZE+2];

 /**Oil pressure vs voltage. 16 points of function, plus two values for setting of x-axis range*/
  int16_t ops_curve[OPS_LOOKUP_TABLE_SIZE+2];

  /**Injection PW coefficient vs voltage, value * 4096, range: 0.5...1.5 */
  int16_t injpw_coef[INJPWCOEF_LUT_SIZE];

  /**MAF's flow curve lookup table. Value in g/sec  * 64. Last value - Y axis's range in g/sec */
  uint16_t maf_curve[MAF_FLOW_CURVE_SIZE+1+2];

  /**reserved*/
  uint8_t reserved1[1425];

  //---------------------------------------------------------------
  //Firmware constants - rare used parameters, fine tune parameters for experienced users...
  int16_t evap_clt;
  uint8_t evap_tps_lo;
  uint8_t evap_tps_hi;
  uint8_t fi_enter_strokes;
  uint8_t fi_leave_strokes;
  uint8_t iac_cond_add;
  int16_t  aircond_clt;
  uint8_t  aircond_tps;
  int16_t  idl_ve;
  uint16_t frap;
  int16_t  idl_ve_g;
  uint16_t  reserv_0;   //reserved
  int16_t  heating_t_off; //Heating off temperature
  uint8_t  heating_time;  //Input manifold heating time
  uint8_t  idltorun_stp_en;
  uint8_t  idltorun_stp_le;
  uint8_t  inpavnum[INPUTNUM];
  uint8_t  vent_delay;   //Ventilator's turn on delay
  uint8_t  vent_iacoff;  //Value to be added to IAC position when ventilator is being turned on
  uint8_t  epas_iacoff;  //Value to be added to IAC position when electronic power steering is being turned on
  uint8_t  vent_pwmsteps;
  uint8_t  vent_minband;
  uint8_t  an_tps_mul;
  uint8_t  hall_predict;  //prediction mode for hall (N=Ncyl) synchronization mode: 0 - last interval (default), 1 - 1st derivative
  uint16_t vtachom_mult;  //Event multiplier for VTACHO output, value * 8192, value in range 0.125...7.900
  uint16_t grheat_time;   //Maximum time for heating on stopped engine
  uint8_t  add_i1_sub;    // 1 - default (ADD_i1), 2 - ADD_i2, 3 - ADD_i3, 4 - ADD_i4, 5 - ADD_i5, 6 - ADD_i6, 7 - ADD_I7, 8 - ADD_I8
  uint8_t  add_i2_sub;    // 1 - ADD_i1, 2 - default (ADD_i2), 3 - ADD_i3, 4 - ADD_i4, 5 - ADD_i5, 6 - ADD_i6, 7 - ADD_I7, 8 - ADD_I8
  uint16_t idlreg_captrange; //capture range for ign. tim. idling regulator
  uint8_t  idlent_timval;
  uint16_t gasval_ontime; //time during which gas valve will be turned on when engine is stopped (10 ms units)
  uint16_t tdc_angle[8];  //Angle of TDC for each cylinder, value * ANGLE_MULTIPLIER, relatively to 0 tooth.
  uint16_t smp_angle;     //Angle for sampling of sensors, value * ANGLE_MULTIPLIER, relatively to TDC (BTDC)
  uint16_t dwl_dead_time; //Dwell dead time, 1 discrete = 3.2us
  uint8_t  sfc_tps_thrd;  //TPS threshold used for immediate leaving fuel cut mode
  uint16_t evap_map_thrd;
  uint16_t ckps_skip_trig; //Number of teeth to skip before searching for missing tooth
  uint8_t  maninjpw_idl;  //0 - don't apply manual inj.PW correction on idling, 1 - apply manual inj.PW correction on idling
  uint8_t  oilpress_cut;  //cut off ignition and injection of fuel if oil pressire falls to crytical value
  uint16_t tpsdot_mindt;  //minimum time delta used for calculation of d%/dt, 1 discrete = 3.2 us
  uint8_t  irr_k_load;    //coefficient of load used to calcultate argument in rigidity map, value * 32.0, max 6.0
  uint8_t  irr_k_rpm;     //coefficient of rpm used to calcultate argument in rigidity map, value * 32.0, max 6.0
  uint8_t  cold_eng_int;  //!Use only integral component on cold engine (1), use regular algorithm (0)
  uint8_t  iacreg_period; //!Period between calls of IAC's closed loop regulator, value in 0.01s units
  int16_t  iacreg_turn_on_temp;//!< IAC closed loop regulator turn on temperature
  uint8_t  vent_maxband;  //!< used with air conditioner
  uint16_t pwron_time;    //!< Keep power on during this time after switching ignition off. Value in 0.01 sec units
  uint16_t pwron_time1;   //!< Keep power on (including ignition and fuel injection) during this time after switching ignition off. Value in 0.01 sec units

  uint8_t  ltft_mode;      //!< 0 - LTFT is turned off, 1 - use only for petrol, 2 - use only for gas, 3 - use for both petrol and gas
  int16_t  ltft_learn_clt; //!< Temperature threshold for learning, value in 0.25∞C units
  uint8_t  ltft_cell_band; //!< cell band in %, Value 256 corresponds to 100%, e.g. 1 discrete correspons to 0,390625%
  uint8_t  ltft_stab_time; //!< Learn stability time, value in 0.01 second units
  uint8_t  ltft_learn_grad; //!< Learning gradient, 256 corresponds to 1.0, range 0...0.99

  uint8_t pwrrelay_uni;    //!< Selection of universal output used as condition for power managment, allowed values: 0,1,2,3,4,5,0x0F, 0x0F means disabled

  //---------------------------------------------------------------

  /**Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */
  uint8_t reserved[1996];
}fw_ex_data_t;

/**Describes a universal programmable output*/
typedef struct uni_output_t
{
 uint8_t flags;                          //!< MS Nibble - logic function, LS Nibble - flags (0,1 bits - inversion, 2nd bit - usage flag, 3rd bit - common inversion),
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
  uint16_t load_lower;                   //!< value of load corresponding to the beginning of load axis in lookup tables
  int16_t  load_upper;                   //!< value of load corresponding to the end of load axis in lookup tables
  int16_t  map_curve_offset;             //!< offset of curve in volts, can be negative
  int16_t  map_curve_gradient;           //!< gradient of curve in kPa/V, can be negative (inverse characteristic curve)
  int16_t  map2_curve_offset;            //!< offset of curve in volts, can be negative
  int16_t  map2_curve_gradient;          //!< gradient of curve in kPa/V, can be negative (inverse characteristic curve)

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
  uint16_t idling_rpm;                   //!< selected idling RPM regulated by using of advance angle
  int16_t  ifac1;                        //!< Idling regulator's factor for positive error (value * 256)
  int16_t  ifac2;                        //!< Idling regulator's factor for negative error (value * 256)
  int16_t  MINEFR;                       //!< dead band of idling regulator (min-1)
  int16_t  idlreg_min_angle;             //!< minimum advance angle correction which can be produced by idling regulator
  int16_t  idlreg_max_angle;             //!< maximum advance angle correction which can be produced by idling regulator
  int16_t  idlreg_turn_on_temp;          //!< Idling regulator turn on temperature

  // Temperature and cooling fan control
  uint8_t  tmp_flags;                    //!< flag of using coolant sensor (see TMPF_x defines)
  int16_t  vent_on;                      //!< cooling fan's turn on temperature
  int16_t  vent_off;                     //!< cooling fan's turn off temperature
  uint16_t vent_pwmfrq;                  //!< PWM frequency (value = 1/f * 524288), 10....5000Hz

  // ADC corrections/compensations
  uint16_t map_adc_factor;               //!< ADC error compensation factor for MAP
  int32_t  map_adc_correction;           //!< ADC error compensation correction for MAP
  uint16_t ubat_adc_factor;              //!< ADC error compensation factor for board voltage
  int32_t  ubat_adc_correction;          //!< ADC error compensation correction for board voltage
  uint16_t temp_adc_factor;              //!< ADC error compensation factor for CLT
  int32_t  temp_adc_correction;          //!< ADC error compensation correction for CLT
  uint16_t tps_adc_factor;               //!< ADC error compensation factor for TPS
  int32_t  tps_adc_correction;           //!< ADC error compensation correction for TPS
  uint16_t ai1_adc_factor;               //!< ADC error compensation factor for ADD_IO1 input
  int32_t  ai1_adc_correction;           //!< ADC error compensation correction for ADD_IO1 input
  uint16_t ai2_adc_factor;               //!< ADC error compensation factor for ADD_IO2 input
  int32_t  ai2_adc_correction;           //!< ADC error compensation correction for ADD_IO2 input

  // Synchronization
  uint8_t  ckps_cogs_btdc;               //!< Teeth before TDC
  uint8_t  ckps_ignit_cogs;              //!< Duration of ignition driver's pulse countable in teeth of wheel
  uint8_t  ckps_engine_cyl;              //!< number of engine's cylinders
  uint8_t  ckps_cogs_num;                //!< number of crank wheel's teeth
  uint8_t  ckps_miss_num;                //!< number of missing crank wheel's teeth
  uint8_t  hall_flags;                   //!< Hall sensor related flags (See CKPF_ defines)
  int16_t  hall_wnd_width;               //!< Hall sensor's shutter window width in degrees of crankshaft (advance value of distributor)

  // Ignition outputs control
  uint8_t  ign_cutoff;                   //!< Cutoff ignition when RPM reaches specified threshold
  uint16_t ign_cutoff_thrd;              //!< Cutoff threshold (RPM)

  // Hall emulation output
  int16_t   hop_start_ang;               //!< Hall output: start of pulse in degrees relatively to TDC, value * ANGLE_MULTIPLIER
  uint16_t  hop_durat_ang;               //!< Hall output: duration of pulse in degrees, value * ANGLE_MULTIPLIER

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
  uint16_t choke_rpm_if;                 //!< Integral factor for RPM-based control of choke position (factor * 1024)
  uint16_t choke_corr_time[2];           //!< Time for startup correction will be applied at -30∞C and 40∞C (2 points function)

  // Bluetooth and security
  uint8_t  bt_flags;                     //!< Bluetooth and security related flags
  uint8_t  ibtn_keys[IBTN_KEYS_NUM][IBTN_KEY_SIZE]; //!< iButton keys for immobilizer

  uni_output_t uni_output[UNI_OUTPUT_NUMBER]; //!< parameters for versatile outputs
  uint8_t uniout_12lf;                   //!< logic function between 1st and 2nd outputs

  // Fuel injection
  uint8_t  inj_flags;                    //!< Fuel injection related flags (see INJFLG_x bits def.)
  uint8_t  inj_config[2];                //!< Configuration of injection (7-4 bits: inj. config., 3-0 bits: num of squitrs), inj.config: INJCFG_x constants
  uint16_t inj_flow_rate[2];             //!< Injector flow rate (cc/min) * 64
  uint16_t inj_cyl_disp;                 //!< The displacement of one cylinder in liters * 16384
  uint32_t inj_sd_igl_const[2];          //!< Constant used in speed-density algorithm to calculate PW. Const = ((CYL_DISP * 3.482 * 18750000) / Ifr ) * ((Nbnk * Ncyl) / (Nsq * Ninj))

  uint16_t inj_prime_cold;               //!< Prime pulse PW at cold (CLT=-30∞C)
  uint16_t inj_prime_hot;                //!< Prime pulse PW at hot (CLT=70∞C)
  uint8_t  inj_prime_delay;              //!< Prime pulse delay in 0.1 sec units

  uint16_t inj_cranktorun_time;          //!< Time in seconds for going from the crank position to the run position (1 tick = 10ms)
  uint8_t  inj_aftstr_strokes;           //!< Number of engine strokes, during this time afterstart enrichment is applied (divided by 4)

  uint8_t  inj_lambda_str_per_stp;       //!< Number of strokes per step for lambda control
  uint8_t  inj_lambda_step_size_p;       //!< "+" Step size, value * 512, max 0.49
  uint16_t inj_lambda_corr_limit_p;      //!< "+" limit, value * 512
  uint16_t inj_lambda_swt_point;         //!< lambda switch point in volts
  int16_t  inj_lambda_temp_thrd;         //!< Coolant temperature activation threshold
  uint16_t inj_lambda_rpm_thrd;          //!< RPM activation threshold
  uint8_t  inj_lambda_activ_delay;       //!< Lambda sensor activation delay

  uint8_t  inj_ae_tpsdot_thrd;           //!< TPS %/sec threshold, max rate is 255%/sec
  uint16_t inj_ae_coldacc_mult;          //!< Cold acceleration multiplier (-30∞C), (value - 1.0) * 128

  uint16_t gd_steps;                     //!< Number of steps of gas dosator stepper motor

  int16_t  inj_timing[2];                //!< Injection timing in crank degrees * 16

  uint8_t  flpmp_flags;                  //!< fuel pump flags

  uint8_t  choke_flags;                  //!< choke related flags (see CKF_x constants for more information)

  uint16_t revlim_lot;                   //!< lower threshold for rev.limitting (fuel injection)
  uint16_t revlim_hit;                   //!< upper threshold for rev.limitting (fuel injection)

  int16_t  inj_timing_crk[2];            //!< Injection timing on cranking in crank degrees * 16

  uint8_t  gd_fc_closing;                //!< How much close (in %) gas doser in fuel cut mode (relatively to current position)

  uint8_t  inj_lambda_step_size_m;       //!<"-" Step size, value * 512, max 0.49

  uint16_t inj_lambda_corr_limit_m;      //!<"-" limit, value * 512

  uint16_t gd_lambda_corr_limit_p;       //!<"+" limit, used for gas doser (idling), value * 512
  uint16_t gd_lambda_corr_limit_m;       //!<"-" limit, used for gas doser (idling), value * 512

  uint16_t inj_lambda_dead_band;         //!< lambda switch point dead band

  uint8_t  load_src_cfg;                 //!< Engine load source selection (0 - MAP, 1 - MAP(baro), 2 - TPS, 3 - MAP+TPS, 4 - MAF)

  uint8_t  idl_to_run_add;               //!< Value (in %) added to IAC position when exiting from closed loop (value * 2)
  uint8_t  rpm_on_run_add;               //!< Value added to target RPM when vehicle starts to run (min-1, value / 10)
  uint16_t idl_reg_p[2];                 //!< IAC closeed loop proportional coefficient (value * 256, max 5.0), [0] - negative error, [1] - positive error
  uint16_t idl_reg_i[2];                 //!< IAC closed loop integral coefficient (value * 256, max 5.0), [0] - negative error, [1] - positive error
  uint8_t  idl_coef_thrd1;               //!< coefficient for calculating closed loop entering RPM threshold (value - 1.0) * 128, max 1.99)
  uint8_t  idl_coef_thrd2;               //!< coefficient for calculating closed loop leaving RPM threshold (value - 1.0) * 128, max 1.99)
  uint8_t  idl_intrpm_lim;               //!< RPM error limit for integrator (min-1, value / 10, max 1200)
  uint16_t idl_map_value;                //!< intake manifold pressure on idling (kPa * MAP_PHYSICAL_MAGNITUDE_MULTIPLIER)

  uint8_t  inj_lambda_senstype;          //!< EGO sensor type (0 - NBO, 1 - WBO)

  uint16_t gd_lambda_stoichval;          //!< Stoichiometric value of fuel used for second fuel (GAS_V=1), value * 128

  uint8_t  inj_lambda_ms_per_stp;        //!< Number of strokes per step for lambda control

  uint8_t  idl_iacminpos;                //!< restriction: IAC min pos
  uint8_t  idl_iacmaxpos;                //!< restriction: IAC max pos

  uint16_t  hall_degrees_btdc;           //!< Degrees BTDC, used for synchronization from a hall sensor (value * ANGLE_MULTIPLIER)

  uint16_t  vss_period_dist;             //!< VSS period distance im meters (value * 32768)

  uint8_t inj_anglespec;                 //!< Specifies how inj.timing coupled with inj.pulse (beginning, middle, end). 0-3 bits: first fuel, 4-7 bits: second fuel

  uint16_t evap_afbegin;                 //!< Air flow value when evap starts to open (PWM duty = 0%), value = (rpm * load) / 32, 32 - number of PWM steps
  uint16_t evap_afslope;                 //!< Slope value = (32 / (afend - afbegin)) * 1048576, user should see only afend = (32 / afslope) + afbegin

  uint8_t sm_freq;                       //!< Frequency of stepper motor's pulses (choke, stepper IAC). 0 - 300Hz, 1 - 150Hz, 2 - 100 Hz, 3 - 75Hz

  uint16_t ai3_adc_factor;               //!< ADC error compensation factor for ADD_I3 input
  int32_t  ai3_adc_correction;           //!< ADC error compensation correction for ADD_I3 input
  uint16_t ai4_adc_factor;               //!< ADC error compensation factor for ADD_I4 input
  int32_t  ai4_adc_correction;           //!< ADC error compensation correction for ADD_I4 input

  uint16_t cond_pvt_on;                  //!< Voltage threshold from pressure sensor when air conditioner clutch should be turned on (e.g. turn off if V < 1.6V) in case of pending request
  uint16_t cond_pvt_off;                 //!< Voltage threshold from pressure sensor when air conditioner clutch should be turned off (e.g. turn off if V > 2.5V)

  uint8_t inj_ae_decay_time;             //!< AE decay time in strokes

  uint16_t cond_min_rpm;                 //!< minimum RPM required for turning on of the air conditioner

  uint8_t inj_lambda_flags;              //!< lambda flags, see LAMFLG_ defines for more information

  uint8_t gd_freq;                       //!< Frequency of GD stepper motor's pulses (stepper gas valve). 0 - 300Hz, 1 - 150Hz, 2 - 100 Hz, 3 - 75Hz

  uint8_t gd_maxfreqinit;                //!< Flag, indicate using of maximum frequency for GD at initialization

  uint16_t fff_const;                    //!< Constant for calculating frequency value of fuel flow rate (value  = (kf/(1000*60))*65536, e.g kf = 16000)

  uint8_t mapsel_uni;                    //!< Selection of universal outputs used as conditions for selection set map sets (7-4 bits: for gas, 3-0 bits: for petrol). Allowed values: 0,1,2,3,4,5,0xFF, 0xFF means disabled

  uint8_t barocorr_type;                 //!< Values: 0 - not barocorrection, 1 - static corr. using primary MAP, 2 - dynamic corr. using primary MAP, 3 - dynamic corr. using additional MAP (SECU-3i only)

  uint8_t inj_floodclear_tps;            //!< TPS Threshold for entering fllod clear mode before cranking, 0 - means that flood clear is disabled

  uint16_t vent_tmr;                     //!< How long cooling fan will work on stopped engine, 1 discrete = 10ms

  uint8_t fp_timeout_strt;               //!< Fuel pump turn off timeout before cranking, value in 0.1 second units (max 25.5 seconds)

  uint8_t eh_heating_time[2];            //!< Heating time without PWM at low(0) and high(1) temperatures (see also temperature threshold)
  uint8_t eh_temper_thrd;                //!< Low/high temperature threshold, value in 1C derg. units
  uint8_t eh_heating_act;                //!< During this time heater is on in PWM mode (see also pause lookup table)
  uint16_t eh_aflow_thrd;                //!< Air flow threshold above which heater will be turned off to prevent overheating

  uint8_t inj_min_pw[2];                 //!< minimum injection PW, 1 discrete = 25.6us (3.2us * 8), max = 6.5ms

  uint16_t pwmfrq[2];                    //!< PWM frequency for PWM1/2 outputs (value = 1/f * 524288), 10....5000Hz

  uint16_t ai5_adc_factor;               //!< ADC error compensation factor for ADD_I5 input
  int32_t  ai5_adc_correction;           //!< ADC error compensation correction for ADD_I5 input
  uint16_t ai6_adc_factor;               //!< ADC error compensation factor for ADD_I6 input
  int32_t  ai6_adc_correction;           //!< ADC error compensation correction for ADD_I6 input
  uint16_t ai7_adc_factor;               //!< ADC error compensation factor for ADD_I7 input
  int32_t  ai7_adc_correction;           //!< ADC error compensation correction for ADD_I7 input
  uint16_t ai8_adc_factor;               //!< ADC error compensation factor for ADD_I8 input
  int32_t  ai8_adc_correction;           //!< ADC error compensation correction for ADD_I8 input

  uint8_t func_flags;

  uint8_t  inj_aftstr_strokes1;          //!< Number of engine strokes, during this time afterstart enrichment is applied (divided by 4) second fuel

  uint8_t  ve2_map_func;                 //!< Selected function of the secondary VE map (0 - just use 1st map, 1 - multiply by the main VE map, 2 - add to the main VE map).

  uint8_t  inj_ae_type;                  //!< Type of selected AE: 0 - acceleration pump, 1 - time based
  uint8_t  inj_ae_time;                  //!< time of AE in strokes, used by "Time based AE"

  uint8_t  knock_selch;                  //!< 1 bit per channel (cylinder). 0 - 1st KS, 1 - 2nd KS

  int16_t  knkclt_thrd;

  uint16_t  iac_reg_db;                  //!< IAC regulator's dead band (rpm).

  uint32_t inj_maf_const[2];             //!< Constant used to calculate inj. PW with MAF.  Const = (((120 * 18750000) / Ifr ) * (Nbnk / (Nsq * Ninj))) / 64
  uint32_t mafload_const;                //!< Constant used to calculate synthetic load based on MAF. Const = (((101.35 * 64 * 128) * 120) / (64 * 0.0012041)) / (CYL_DISP * Ncyl), where CYL_DISP is in cc units

  uint8_t adc_flags;

  uint8_t fuelcut_uni;                   //!< Selection of universal output used as condition for fuel cut, allowed values: 0,1,2,3,4,5,0x0F, 0x0F means disabled
  uint8_t igncut_uni;                    //!< Selection of universal output used as condition for ignition cut, allowed values: 0,1,2,3,4,5,0x0F, 0x0F means disabled

  uint8_t gas_v_uni;                     //!< Selection of universal output used as condition for emulation of GAS_V input, allowed values: 0,1,2,3,4,5,0x0F, 0x0F means disabled

  uint16_t inj_max_pw[2];                //!< Maximum injection PW, value in 3.2us units

  uint8_t igntim_flags;                  //!< Ignition timing flags, see IGNTF_* defines for more information
  int16_t shift_igntim;                  //!< Ignition timing for shifting

  uint8_t  stbl_str_cnt;
  uint8_t  strt_flags;                   //!< Cranking flags, see STRTF_* defines for more information

  /**Following reserved bytes required for keeping binary compatibility between
   * different versions of firmware. Useful when you add/remove members to/from
   * this structure. */

  uint8_t  reserved[148];

  /**CRC of this structure (for checking correctness of data after loading from EEPROM) */
  uint16_t crc;
}params_t;

//Define data structures are related to code area data and IO remapping data
typedef uint16_t fnptr_t;                //!< Special type for function pointers
#define IOREM_SLOTS  49                  //!< Number of slots used for I/O remapping
#define IOREM_PLUGS  100                 //!< Number of plugs used in I/O remapping

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


/** Describes all data residing in the firmware */
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

/*fwinfo*/
#define FWINFOSIZE 69
PGM_DECLARE(extern uint8_t fwinfo[FWINFOSIZE]);

#endif //_TABLES_H_
