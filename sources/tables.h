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

/* ��������� ������������� ������ �������� ����� ���������� SECU-3
 * Structure of program memory's allocation of SECU-3 firmware
 *        ________________________
 *       |                        |
 *       |       ���              |
 *       |       code             |
 *       |------------------------|
 *       | ��������� ������������ |
 *       |   free space           |
 *       |------------------------|<--- ����� ������ � ��������� ��������� � ����� ������
 *       |                        |     place new data from here going to code
 *       | �������������� �����.  |
 *       |additional data (param.)|
 *       |------------------------|
 *       |  ���������  ���������  |<--- loaded when instace in the EEPROM is broken
 *       |   reserve parameters   |
 *       |------------------------|
 *       |   ������ ������        |
 *       |   array of tables      |
 *       |                        |
 *       |------------------------|     ����������� ����� �������� ��� ����� ���� ����
 *       |     CRC16              |<--- ����������� ����� � ������ ����������.
 *       |________________________|     (CRC of firmware without 2 bytes of this CRC and boot loader)
 *       |                        |
 *       |  boot loader           |
 *        ------------------------
 */



#ifndef _TABLES_H_
#define _TABLES_H_

#include <stdint.h>
#include "bootldr.h"   //to know value of SECU3BOOTSTART, and only

//���������� ���������� ����� ������������ ��� ������ �������
//define number of interpolation nodes for lookup tables
#define F_WRK_POINTS_F         16
#define F_WRK_POINTS_L         16
#define F_TMP_POINTS           16
#define F_STR_POINTS           16
#define F_IDL_POINTS           16

#define F_NAME_SIZE            16

#define KC_ATTENUATOR_LOOKUP_TABLE_SIZE 128
#define FW_SIGNATURE_INFO_SIZE 48
#define COIL_ON_TIME_LOOKUP_TABLE_SIZE 32

//��������� ���� ��������� �������������, �������� ��� = 0.5 ����.
//Describes one set(family) of chracteristics (maps), descrete = 0.5 degr.
typedef struct f_data_t
{
  int8_t f_str[F_STR_POINTS];                       // ������� ��� �� ������ (function of advance angle at start)
  int8_t f_idl[F_IDL_POINTS];                       // ������� ��� ��� �� (function of advance angle at idling)
  int8_t f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     // �������� ������� ��� (3D) (working function of advance angle)
  int8_t f_tmp[F_TMP_POINTS];                       // ������� �������. ��� �� ����������� (coolant temper. correction of advance angle)
  uint8_t name[F_NAME_SIZE];                        // ��������������� ��� (��� ���������) (assosiated name, displayed in user interface)
}f_data_t;


//��������� �������������� ������ �������� � ��������
//Describes additional data stored in the firmware
typedef struct firmware_data_t
{
  uint8_t fw_signature_info[FW_SIGNATURE_INFO_SIZE];

  //������� �������� ����������� (����������� �� ��������).
  //Knock. table of attenuator's gain factors (contains codes of gains, gain depends on RPM)
  uint8_t attenuator_table[KC_ATTENUATOR_LOOKUP_TABLE_SIZE];

  //������� ������� ���������� ������� � �������� ��������� (����������� �� ����������)
  //Table for coil's regulation. Regulation depends on board voltage
  uint16_t coil_on_time[COIL_ON_TIME_LOOKUP_TABLE_SIZE];

  //used for checking compatibility with management software. Holds size of all data stored in the firmware.
  uint16_t fw_data_size;

  //holds flags which give information about options were used to build firmware
  //(������ ����� ������ ���������� � ��� � ������ ������� ���� �������������� ��������)
  uint32_t config;

  //��� ����������������� ����� ���������� ��� ���������� �������� �������������
  //����� ������ �������� � ����� ������� ��������. ��� ���������� ����� ������
  //� ���������, ���������� ����������� ��� �����.
  //Following reserved bytes required for keeping binary compatibility between
  //different versions of firmware. Useful when you add/remove members to/from
  //this structure.
  uint8_t reserved[58];
}firmware_data_t;

//��������� ��������� �������
//describes system's parameters. One instance of this structure stored in the EEPROM and one
//in the FLASH (program memory)
typedef struct params_t
{
  uint8_t  tmp_use;                      //������� ������������ ����-�� (flag of using coolant sensor)
  uint8_t  carb_invers;                  //�������� ��������� �� ����������� (flag of inversion of carburetor's limit switch)
  uint8_t  idl_regul;                    //������������ �������� ������� �� �������������� ��� (keep selected idling RPM by alternating advance angle)
  uint8_t  fn_benzin;                    //����� ������ ������������� ������������ ��� ������� (index of set of characteristics used for gasoline)
  uint8_t  fn_gas;                       //����� ������ ������������� ������������ ��� ���� (index of set of characteristics used for gas)
  uint16_t map_lower_pressure;           //������ ������� ��� �� ��� ������� (���) (lower value of MAP at the axis of table(work map) (kPa))
  uint16_t ie_lot;                       //������ ����� ���� (���-1) (lower threshold for idle economizer valve(min-1) for gasiline)
  uint16_t ie_hit;                       //������� ����� ���� (���-1) (upper threshold for idle economizer valve(min-1) for gasoline)
  uint16_t starter_off;                  //����� ���������� �������� (���-1) (RPM when starter will be turned off)
  int16_t  map_upper_pressure;           //������� �������� ��� �� ��� ������� (���) (upper value of MAP at the axis of table(work map) (kPa))
  uint16_t smap_abandon;                 //������� �������� � �������� ����� �� �������  (���-1) (RPM when switching from start map(min-1))
  int16_t  max_angle;                    //����������� ������������� ��� (system's maximum advance angle limit)
  int16_t  min_angle;                    //����������� ������������ ��� (system's minimum advance angle limit)
  int16_t  angle_corr;                   //�����-��������� ��� (octane correction of advance angle)
  uint16_t idling_rpm;                   //�������� ������� �� ��� ����������� �������������� ��� (selected idling RPM regulated by using advance angle)
  int16_t  ifac1;                        //������������ ���������� �������� ��, ��� ������������� �
  int16_t  ifac2;                        //������������� ������ ��������������. (Idling regulator's factors for positive and negative errors correspondingly)
  int16_t  MINEFR;                       //���� ������������������ ���������� (�������) (dead band of idling regulator (min-1))
  int16_t  vent_on;                      //����������� ��������� ����������� (cooling fan's turn on temperature)
  int16_t  vent_off;                     //����������� ���������� ����������� (cooling fan's turn off temperature)

  int16_t  map_adc_factor;               // �������� ��� ��������� ������������ ���
  int32_t  map_adc_correction;           // (correction values (factors and additions) for ADC) - error compensations
  int16_t  ubat_adc_factor;              // --
  int32_t  ubat_adc_correction;          // --
  int16_t  temp_adc_factor;              // --
  int32_t  temp_adc_correction;          // --

  uint8_t  ckps_edge_type;               //Edge type for interrupt from CKP sensor (rising or falling edge). Depends on polarity of sensor
  uint8_t  ckps_cogs_btdc;               //Teeth before TDC
  uint8_t  ckps_ignit_cogs;              //Duration of ignition driver's pulse countable in teeth of wheel

  int16_t  angle_dec_spead;              //limitation of alternation speed of advance angle (when decreasing)
  int16_t  angle_inc_spead;              //limitation of alternation speed of advance angle (when increasing)
  int16_t  idlreg_min_angle;             //minimum advance angle correction which can be produced by idling regulator
  int16_t  idlreg_max_angle;             //maximum advance angle correction which can be produced by idling regulator
  uint16_t map_curve_offset;             //offset of curve in volts
  uint16_t map_curve_gradient;           //gradient of curve in kPa/V

  int16_t  fe_on_threshold;              //����� ��������� ������������ ���������� ������� (switch on threshold of FE)

  uint16_t ie_lot_g;                     //������ ����� ���� (���) (lower threshold for idle economizer valve(min-1) for gas)
  uint16_t ie_hit_g;                     //������� ����� ���� (���) (upper threshold for idle economizer valve(min-1) for gas)
  uint8_t  shutoff_delay;                //�������� ���������� ������� (idle economizer valve's turn off delay)

  uint16_t uart_divisor;                 //�������� ��� ��������������� �������� UART-a (divider which corresponds to selected baud rate)
  uint8_t  uart_period_t_ms;             //������ ������� ������� � �������� ����������� (transmition period of data packets which SECU-3 sends, one discrete = 10ms)

  uint8_t ckps_engine_cyl;               //���-�� ��������� ��������� (number of engine's cylinders)

  //--knock
  uint8_t  knock_use_knock_channel;      //������� ������������� ������ ��������� (flag of using knock channel)
  uint8_t  knock_bpf_frequency;          //����������� ������� ���������� ������� (Band pass filter frequency)
  int16_t  knock_k_wnd_begin_angle;      //������ �������������� ���� (�������) (Opening angle of knock phase window)
  int16_t  knock_k_wnd_end_angle;        //����� �������������� ���� (�������)  (Closing angle of knock phase window)
  uint8_t  knock_int_time_const;         //���������� ������� �������������� (���) (Integration time constant)
  //--
  int16_t knock_retard_step;             //��� �������� ��� ��� ��������� (Displacement step of angle)
  int16_t knock_advance_step;            //��� �������������� ��� (Recovery step of angle)
  int16_t knock_max_retard;              //������������ �������� ��� (Maximum displacement of angle)
  uint16_t knock_threshold;              //����� ��������� - ���������� (detonation threshold - voltage)
  uint8_t knock_recovery_delay;          //�������� �������������� ��� � ������� ������ ��������� (Recovery delay of angle countable in engine's cycles)
  //--/knock

  uint8_t vent_pwm;                      //flag - control cooling fan by using PWM
  //��� ����������������� ����� ���������� ��� ���������� �������� �������������
  //����� ������ �������� � ����� ������� ��������. ��� ���������� ����� ������
  //� ���������, ���������� ����������� ��� �����.
  //Following reserved bytes required for keeping binary compatibility between
  //different versions of firmware. Useful when you add/remove members to/from
  //this structure.
  uint8_t  reserved[9];

  uint16_t crc;                         //����������� ����� ������ ���� ��������� (��� �������� ������������ ������ ����� ���������� �� EEPROM)
                                        //CRC of data of this structure (for checking correctness of data after loading from EEPROM)
}params_t;

//================================================================================
//���������� ������ ������ � �������� ������������ �� ����������
//Define addresses of tables in the firmware starting from boot loader's address

//������ ���������� ����������� ����� ���������� � ������
//size of variable of CRC of parameters in bytes (used in params_t structure)
#define PAR_CRC_SIZE   sizeof(uint16_t)

//������ ���������� ����������� ����� �������� � ������
//size of variable of CRC of whole firmware in bytes
#define CODE_CRC_SIZE   sizeof(uint16_t)

//������ ������ ���������� ��� ����� ����������� �����
//Size of application's section without taking into account its CRC
#define CODE_SIZE (SECU3BOOTSTART-CODE_CRC_SIZE)

//���������� ������� ������������� �������� � ������ ��������
//Number of sets(families) of characteristics stored in the firmware
#define TABLES_NUMBER  8

//����� ����������� ����� � ��������
//Address of CRC of whole firmware
#define CODE_CRC_ADDR (SECU3BOOTSTART-CODE_CRC_SIZE)

//����� ������� ������ - �������� �������������
//Address of array of tables (array of sets of characteristics)
#define TABLES_START (CODE_CRC_ADDR-(sizeof(f_data_t)*TABLES_NUMBER))

//����� ��������� ���������� ���������� (���������� EEPROM �� ���������)
//Address of structure containing default parameters (loaded when parameters from EEPROM are broken)
#define DEFPARAM_START (TABLES_START-sizeof(params_t))

//����� �������������� ����������
//Address of additional parameters (extended). Add new data into this structure
#define FIRMWARE_DATA_START (DEFPARAM_START-sizeof(firmware_data_t))

//================================================================================
//Variables:

//�������������� ������
//Additional data in the firmware
#pragma object_attribute=__root
extern firmware_data_t __flash fwdata;

//������� ���
//Array of tables of advance angle
#pragma object_attribute=__root
extern f_data_t __flash tables[TABLES_NUMBER];

//��������� ���������
//Reserve parameters (loaded when instance in EEPROM is broken)
#pragma object_attribute=__root
extern params_t __flash def_param;

//Check sum of whole firmware (except this check sum and boot loader)
#pragma object_attribute=__root
extern uint16_t __flash code_crc;

#endif //_TABLES_H_