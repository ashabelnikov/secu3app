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

/** \file ecudata.h
 * \author Alexey A. Shabelnikov
 * ECU data in RAM (global data structures and state variables).
 * This file contains main data structures used in the firmware.
 */

#ifndef _ECUDATA_H_
#define _ECUDATA_H_

#include "tables.h"

/** Start mode of engine (used by state machine) */
#define EM_START 0

/** Idle mode of engine (used by state machine) */
#define EM_IDLE  1

/** Work mode of engine (used by state machine) */
#define EM_WORK  2

#ifdef REALTIME_TABLES
typedef uint8_t  (*mm_func8_ptr_t)(uint16_t);
typedef uint16_t (*mm_func16_ptr_t)(uint16_t);
typedef uint16_t (*mm_func12_ptr_t)(uint16_t,uint8_t);

uint8_t mm_get_byte_ram(uint16_t offset);
uint8_t mm_get_byte_pgm(uint16_t offset);
uint16_t mm_get_word_ram(uint16_t offset);
uint16_t mm_get_w12_ram(uint16_t offset, uint8_t off);
uint16_t mm_get_word_pgm(uint16_t offset);
#endif
uint16_t mm_get_w12_pgm(uint16_t offset, uint8_t off);

#ifdef DIAGNOSTICS
/**Describes diagnostics inputs data */
typedef struct diagnost_inp_t
{
 uint8_t flags;                          //!< flags
 uint16_t voltage;                       //!< board voltage
 uint16_t map;                           //!< MAP sensor
 uint16_t temp;                          //!< coolant temperature
 uint16_t add_i1;                        //!< additional input 1 (analog)
 uint16_t add_i2;                        //!< additional input 2 (analog)
#ifndef SECU3T //SECU-3i
 uint16_t add_i3;                        //!< additional input 3 (analog)
#endif
#ifdef TPIC8101
 uint16_t add_i4;                        //!< additional input 4 (analog)
#endif
#if !defined(SECU3T) && defined(MCP3204)
 uint16_t add_i5;                        //!< additional input 5 (analog)
 uint16_t add_i6;                        //!< additional input 6 (analog)
 uint16_t add_i7;                        //!< additional input 7 (analog)
 uint16_t add_i8;                        //!< additional input 8 (analog)
#endif
 uint16_t carb;                          //!< carburetor switch, throttle position sensor (analog)
 uint16_t bits;                          //!< bits describing states of: gas valve, CKP sensor, VR type cam sensor, Hall-effect cam sensor, BL jmp, DE jmp, IGN_I, COND_I (plus EPAS_I for SECU-3i)
 uint16_t ks_1;                          //!< knock sensor 1
 uint16_t ks_2;                          //!< knock sensor 2
}diagnost_inp_t;
#endif

/**Describes all system's inputs - theirs derivative and integral magnitudes
 */
typedef struct sensors_t
{
 uint16_t map;                           //!< Intake Manifold Pressure (averaged)
 uint16_t voltage;                       //!< Board voltage (averaged)
 int16_t  temperat;                      //!< Coolant temperature (averaged)
 uint16_t rpm;                           //!< Averaged RPM (min-1)
 uint8_t  carb;                          //!< State of carburetor's limit switch (throttle limit switch)
 uint8_t  gas;                           //!< State of gas valve
 uint8_t  gas_raw;                       //!< State of gas valve (raw input value)
 uint16_t knock_k;                       //!< Knock signal level
 uint8_t  tps;                           //!< Throttle position sensor (0...100%, x2)
 uint16_t add_i1;                        //!< ADD_I1 input voltage
 uint16_t add_i2;                        //!< ADD_I2 input voltage

#if !defined(SECU3T)
 uint16_t add_i3;                        //!< ADD_I3 input voltage
#endif

#if defined(TPIC8101)
 uint16_t add_i4;                        //!< ADD_I4 input voltage
#endif

#if !defined(SECU3T) && defined(MCP3204)
 uint16_t add_i5;                        //!< inputs from SPI ADC
 uint16_t add_i6;
 uint16_t add_i7;
 uint16_t add_i8;
#endif

#ifdef SPEED_SENSOR
 uint32_t vss_int_dist;                  //!< total passed distance in m stored in EEPROM, value * 32
 uint32_t vss_dist;                      //!< total passed distance in m, value in 8m units. This value is sent to PC
 uint16_t vss_speed;                     //!< Speed in km/h, value * 32
#endif
#ifdef AIRTEMP_SENS
 int16_t air_temp;                       //!< Intake air temperature
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 int16_t tpsdot;                         //!< Speed of TPS movement (d%/dt = %/s), positive when acceleration, negative when deceleration
 int16_t mapdot;                         //!< Speed of MAP changing (dP/dt = kPa/s), positive when acceleration, negative when deceleration
#endif

#ifndef SECU3T //SECU-3i
 uint8_t oilpress_ok;                    //!< Flag.1 - oil pressure is OK, 0 - failure
 uint8_t generator_ok;                   //!< Flag.1 - dynamo generator is OK, 0 - failure
 uint8_t epas_i;
 uint8_t gpa4_i;
#endif
 uint8_t auto_i;
 uint8_t ign_i;
 uint8_t cond_i;
 uint8_t mapsel0;

 //raw values of sensors (ADC discretes with compensated errors)
 int16_t  map_raw;                       //!< raw ADC value from MAP sensor
 int16_t  voltage_raw;                   //!< raw ADC value from voltage
 int16_t  temperat_raw;                  //!< raw ADC value from coolant temperature sensor
 int16_t  tps_raw;                       //!< raw ADC value from TPS sensor
 int16_t  add_i1_raw;                    //!< raw ADC value from ADD_I1 input
 int16_t  add_i2_raw;                    //!< raw ADC value from ADD_I2 input

#if !defined(SECU3T)
 int16_t  add_i3_raw;                    //!< raw ADC value from ADD_I3 input
#endif

#ifdef TPIC8101
 int16_t  add_i4_raw;                    //!< raw ADC value from ADD_I4 input
#endif

#if !defined(SECU3T) && defined(MCP3204)
 int16_t add_i5_raw;                     //!< inputs from SPI ADC
 int16_t add_i6_raw;
 int16_t add_i7_raw;
 int16_t add_i8_raw;
#endif

 int16_t  knock_raw;                     //!< raw value of signal from KS chip

 uint16_t map2;                          //!< Secondary MAP sensor

#ifndef SECU3T //SECU-3i
 int16_t tmp2;                           //!< Secondary temperature sensor (gas temperature)
 int16_t gps;                            //!< Gas pressure sensor (LPG eq.)
#endif

#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 uint16_t afr[2];                        //!< AFR value calculated from lambda sensor #1 and #2, value * 128
 uint16_t lambda[2];                     //!< voltage from lambda sensor #1 and #2
 uint16_t lambda_mx;                     //!< mixed value from available sensors
#endif

#if !defined(SECU3T) && defined(MCP3204)
 int16_t grts;                           //!< Gas reducer temperature sensor
#endif

#if !defined(SECU3T)
 int16_t ftls;                           //!< fuel tank level sensor (L), value * 64
 int16_t egts;                           //!< Exhaust gas temperature sensor (°C), value * 4
 int16_t ops;                            //!< Oil pressure sensor (kg/cm2), value * 256
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint16_t rxlaf;                         //!< Calculated RxLxVE value
#endif

#if defined(FUEL_INJECT)
 uint8_t inj_duty;                      //!< Injector's duty in %, value * 2
#endif

 uint16_t baro_press;                    //!< Barometric pressure (measured before cranking or dynamicaly updated using additional pressure sensor)

 uint8_t input1;
 uint8_t input2;

 uint16_t maf;                           //! Value of air flow (g/sec) * 64
#ifndef SECU3T //SECU-3i
 int16_t fts;                            //!< Secondary temperature sensor (gas temperature)
#endif
}sensors_t;

typedef struct correct_t
{
 int16_t curr_angle;                     //!< Current advance angle of 1st spark plug
#ifdef SPLIT_ANGLE
 int16_t split_angle;                    //!< Current value of split angle
 int16_t curr_angle1;                    //!< Current advance angle of 2nd spark plug
#endif
 int16_t knock_retard;                   //!< Correction of advance angle from knock detector
 int16_t knkret_aac;                     //!< Correction of advance angle from knock detector
 int16_t idlreg_aac;                     //!< Idle regulator advance angle correction
 int16_t octan_aac;                      //!< Octane advance angle correction
 int16_t strt_aalt;                      //!< Advance angle from start map
 int16_t idle_aalt;                      //!< Advance angle from idle map
 int16_t work_aalt;                      //!< Advance angle from work map
 int16_t temp_aalt;                      //!< Advance angle from coolant temp. corr. map
 int16_t airt_aalt;                      //!< Advance angle from air temp. corr. map
#if defined(FUEL_INJECT) || defined(CARB_AFR) || defined(GD_CONTROL)
 int16_t lambda[2];                      //!< Current value of lambda (EGO) correction, can be negative
#endif
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint16_t afr;                           //!< Current value of air to fuel ratio (from AFR map), value*128
#endif
#ifdef FUEL_INJECT
 int16_t inj_timing;                     //!< Current injection timing
 uint8_t rigid_arg;                      //!< IAC regulator rigidity's argument, value * 256, value 255 is reserved
#endif
#ifdef PA4_INP_IGNTIM
 int16_t pa4_aac;                        //!< Ignition timing correction from PA4
#endif
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 int16_t tchrg;                          //!< Corrected value of MAT, temperature units
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint8_t idlve;                          //!< flag which indicates using of idling VE
#endif
}correct_t;

/**Describes system's data (main ECU data structure)
 */
typedef struct ecudata_t
{
 struct params_t  param;                 //!< --parameters
 struct sensors_t sens;                  //!< --sensors
 struct correct_t corr;                  //!< --calculated corrections and lookup tables' values

 uint16_t load;                          //!< value * 64

 uint8_t  ie_valve;                      //!< State of Idle cut off valve
 uint8_t  fe_valve;                      //!< State of Power valve
#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint8_t  fc_revlim;                     //!< Flag indicates fuel cut from rev. limitter
#endif
 uint8_t  cool_fan;                      //!< State of the cooling fan
 uint8_t  st_block;                      //!< State of the starter blocking output (starter relay)
 uint8_t  ce_state;                      //!< State of CE lamp
 uint8_t  airflow;                       //!< Air flow (number of curve on the load axis in 3D tables)
 uint8_t  choke_pos;                     //!< Choke position in % * 2
 uint8_t  gasdose_pos;       /*GD*/      //!< Gas dosator position in % * 2

#ifdef REALTIME_TABLES
 f_data_t tables_ram;                    //!< set of tables in RAM
 mm_func8_ptr_t mm_ptr8;                 //!< callback, returns 8 bit result, unsigned
 mm_func16_ptr_t mm_ptr16;               //!< callback, return 16 bit result, unsigned
 mm_func12_ptr_t mm_ptr12;               //!< callback, return 12 bit result, unsigned
#endif
 f_data_t _PGM *fn_dat;                  //!< Pointer to the set of tables

 uint16_t op_comp_code;                  //!< Contains code of operation for packet being sent - OP_COMP_NC packet
 uint16_t op_actn_code;                  //!< Contains code of operation for packet being received - OP_COMP_NC packet
 uint32_t ecuerrors_for_transfer;        //!< Buffering of error codes being sent via UART in real time
 uint32_t ecuerrors_saved_transfer;      //!< Buffering of error codes for read/write from/to EEPROM which is being sent/received
 uint8_t  use_knock_channel_prev;        //!< Previous state of knock channel's usage flag
 uint8_t  chmode_data;                   //!< Data of the CHANGEMODE packet

 uint8_t engine_mode;                    //!< Current engine mode(start, idle, work)

#ifdef DIAGNOSTICS
 diagnost_inp_t diag_inp;                //!< diagnostic mode: values of inputs
 uint32_t       diag_out;                //!< diagnostic mode: values of outputs

 uint16_t       diag_frq;                //!< PWM frequency, value = 1/f * 524288 (2.5...5000 Hz)
 uint8_t        diag_duty;               //!< duty 0...255 (0...100%)
 uint8_t        diag_chan;               //!< index of selected output for testing (=0 means no selection)
#endif

 uint8_t choke_testing;                  //!< Used to indcate that choke testing is on/off (so it is applicable only if SM_CONTROL compilation option is used)
 int8_t choke_manpos_d;                  //!< Muanual position setting delta value used for choke control
 uint8_t choke_rpm_reg;                  //!< Used to indicate that at the moment system regulates RPM by means of choke position

 uint8_t gasdose_testing;    /*GD*/      //!< Used to indcate that gas dosator testing is on/off (so it is applicable only if GD_CONTROL compilation option is used)
 int8_t gasdose_manpos_d;    /*GD*/      //!< Muanual position setting delta value used for gasdose control

 uint8_t bt_name[9];                     //!< received name for Bluetooth (8 chars max), zero element is size
 uint8_t bt_pass[7];                     //!< received password for Bluetooth (6 chars max), zero element is size
 uint8_t sys_locked;                     //!< used by immobilizer to indicate that system is locked

#ifdef FUEL_INJECT
 uint16_t inj_pw;                        //!< current value of injector pulse width
 volatile uint16_t inj_pwns[2][8];       //!< normal and shrinked injection PW for each channel
 int16_t inj_dt;                         //!< current value of injector's dead time
 uint16_t inj_fff;                       //!< Instant fuel flow as frequency (Hz), 16000 pulses per 1L of burnt fuel (value * 256)
 uint8_t  eng_running;                   //!< flag, indicates that engine is revolving now (even very slow revolving is taken into account)
 uint32_t cons_fuel_int;                 //!< Quantity of fuel totally consumed by vehicle (L), value * 2^18. This value is stored in EEPROM
 uint32_t cons_fuel_imm;                 //!< Quantity of fuel consumed by vehicle for a short period of time (L), value * 2^27
 uint32_t cons_fuel;                     //!< Quantity of fuel consumed by vehicle shown for user (L), value * 1024. This valie is send to PC
#endif

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint8_t acceleration;                   //!< acceleration/decelaration flag
#endif

#ifdef AIRCONDIT
 uint16_t cond_req_rpm;                  //!< Minimum required RPM for turning on of air conditioner clutch (air cond. control algorithm will increase it smoothly)
 uint8_t  cond_req_fan;                  //!< Flag, indicates request from air conditioner to turn on cooling fan
#endif

 uint8_t floodclear;                    //!< Flood clear mode flag

 uint8_t cond_state;                    //!< Current state of the air conditioner (0 - off, 1 - on)
 uint8_t vent_req_on;                   //!< Ventilator's turn on request

#if defined(FUEL_INJECT) || defined(GD_CONTROL)
 uint8_t aftstr_enr;                    //!< Flag which indicates that after start enrichment is active
#endif

#ifndef SECU3T
 uint8_t gasval_on;                     //!< State of the GASVAL_O output
 uint8_t gasval_res;
#endif

#ifdef FUEL_INJECT
 uint8_t iac_closed_loop;               //!< Flag. Indicates closed loop mode for IAC
 uint8_t iac_in_deadband;               //!< Flag. Indicates that IAC regulator is in dead band of error.
#endif

 uint8_t vent_duty;                     //!< PWM duty of coolong fan (value in % * 2)

#ifdef UNI_OUTPUT
 uint8_t uniout[UNI_OUTPUT_NUMBER];     //!< states of universal outputs
#endif

#ifdef FUEL_INJECT
 int8_t inj_ltft1[INJ_VE_POINTS_L][INJ_VE_POINTS_F]; //!< Long Term Fuel Trim map 1, value * 512 (range -0.247...+0.247)
 uint16_t inj_ltft1_crc;                //!< used when saving to EEPROM, must follow inj_ltft1 array!
 int8_t inj_ltft2[INJ_VE_POINTS_L][INJ_VE_POINTS_F]; //!< Long Term Fuel Trim map 2, value * 512 (range -0.247...+0.247)
 uint16_t inj_ltft2_crc;                //!< used when saving to EEPROM, must follow inj_ltft2 array!
#endif
}ecudata_t;


extern struct ecudata_t d;               //!< ECU data structure. Contains all related data and state information
extern struct params_t eeprom_parameters_cache;//!< Cache for buffering EEPROM parameters
extern struct fw_ex_tabs_t ram_extabs;   //!< Separate tables loaded from firmware
#endif //_ECUDATA_H_
