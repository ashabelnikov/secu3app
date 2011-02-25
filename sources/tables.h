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

/* Структура распределения памяти программ блока управления SECU-3
 * Structure of program memory's allocation of SECU-3 firmware
 *        ________________________
 *       |                        |
 *       |       код              |   
 *       |       code             |
 *       |------------------------|
 *       | свободное пространство |
 *       |   free space           |
 *       |------------------------|<--- новые данные и структуры добавлять с этого адреса
 *       |                        |     place new data from here going to code
 *       | дополнительные парам.  |
 *       |additional data (param.)|
 *       |------------------------| 
 *       |  резервные  параметры  |<--- loaded when instace in the EEPROM is broken
 *       |   reserve parameters   |
 *       |------------------------|
 *       |   массив таблиц        |                        
 *       |   array of tables      |
 *       |                        |
 *       |------------------------|     контрольная сумма прошивки без учета байт этой  
 *       |     CRC16              |<--- контрольной суммы и байтов бутлоадера.
 *       |________________________|     (CRC of firmware without 2 bytes of this CRC and boot loader)
 *       |                        |
 *       |  boot loader           |
 *        ------------------------
 */



#ifndef _TABLES_H_
#define _TABLES_H_

#include <stdint.h>
#include "bootldr.h"   //to know value of SECU3BOOTSTART, and only

//определяем количество узлов интерполяции для каждой функции
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

//Описывает одно семейство характеристик, дискрета УОЗ = 0.5 град.
//Describes one set(family) of chracteristics (maps), descrete = 0.5 degr.
typedef struct f_data_t
{
  int8_t f_str[F_STR_POINTS];                       // функция УОЗ на старте (function of advance angle at start)
  int8_t f_idl[F_IDL_POINTS];                       // функция УОЗ для ХХ (function of advance angle at idling)
  int8_t f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     // основная функция УОЗ (3D) (working function of advance angle)
  int8_t f_tmp[F_TMP_POINTS];                       // функция коррект. УОЗ по температуре (coolant temper. correction of advance angle)
  uint8_t name[F_NAME_SIZE];                        // ассоциированное имя (имя семейства) (assosiated name, displayed in user interface)
}f_data_t;


//описывает дополнительные данные хранимые в прошивке
//Describes additional data stored in the firmware
typedef struct firmware_data_t
{
  uint8_t fw_signature_info[FW_SIGNATURE_INFO_SIZE];
  
  //таблица усиления аттенюатора (зависимость от оборотов).
  //Knock. table of attenuator's gain factors (contains codes of gains, gain depends on RPM)
  uint8_t attenuator_table[KC_ATTENUATOR_LOOKUP_TABLE_SIZE]; 
  
  //таблица времени накопления энергии в катушках зажигания (зависимость от напряжения)
  //Table for coil's regulation. Regulation depends on board voltage
  uint16_t coil_on_time[COIL_ON_TIME_LOOKUP_TABLE_SIZE];
  
  //used for checking compatibility with management software. Holds size of all data stored in the firmware.
  uint16_t fw_data_size; 
  
  //holds flags which give information about options were used to build firmware
  //(хранит флаги дающие информацию о том с какими опциями была скомпилирована прошивка)
  uint32_t config;
  
  //Эти зарезервированные байты необходимы для сохранения бинарной совместимости
  //новых версий прошивок с более старыми версиями. При добавлении новых данных
  //в структуру, необходимо расходовать эти байты.
  //Following reserved bytes required for keeping binary compatibility between 
  //different versions of firmware. Useful when you add/remove members to/from 
  //this structure.
  uint8_t reserved[58];  
}firmware_data_t;

//описывает параметры системы
//describes system's parameters. One instance of this structure stored in the EEPROM and one
//in the FLASH (program memory)
typedef struct params_t
{
  uint8_t  tmp_use;                      //признак комплектации ДТОЖ-ом (flag of using coolant sensor)
  uint8_t  carb_invers;                  //инверсия концевика на карбюраторе (flag of inversion of carburetor's limit switch)
  uint8_t  idl_regul;                    //поддерживать заданные обороты ХХ регулированием УОЗ (keep selected idling RPM by alternating advance angle)
  uint8_t  fn_benzin;                    //номер набора характеристик используемый для бензина (index of set of characteristics used for gasoline)
  uint8_t  fn_gas;                       //номер набора характеристик используемый для газа (index of set of characteristics used for gas)
  uint16_t map_lower_pressure;           //нижнее значене ДАД по оси таблицы (кПа) (lower value of MAP at the axis of table(work map) (kPa))
  uint16_t ephh_lot;                     //нижний порог ЭПХХ (мин-1) (lower threshold for forced idle valve(min-1) for gasiline)
  uint16_t ephh_hit;                     //верхний порог ЭПХХ (мин-1) (upper threshold for forced idle valve(min-1) for gasoline)
  uint16_t starter_off;                  //порог выключения стартера (мин-1) (RPM when starter will be turned off)
  int16_t  map_upper_pressure;           //верхнее значение ДАД по оси таблицы (кПа) (upper value of MAP at the axis of table(work map) (kPa))
  uint16_t smap_abandon;                 //обороты перехода с пусковой карты на рабочую  (мин-1) (RPM when switching from start map(min-1))
  int16_t  max_angle;                    //ограничение максимального УОЗ (system's maximum advance angle limit)
  int16_t  min_angle;                    //ограничение минимального УОЗ (system's minimum advance angle limit)
  int16_t  angle_corr;                   //октан-коррекция УОЗ (octane correction of advance angle)   
  uint16_t idling_rpm;                   //заданные обороты ХХ для поддержания регулмрованием УОЗ (selected idling RPM regulated by using advance angle)  
  int16_t  ifac1;                        //коэффициенты регулятора оборотов ХХ, для положительной и
  int16_t  ifac2;                        //отрицательной ошибок соответственно. (Idling regulator's factors for positive and negative errors correspondingly)
  int16_t  MINEFR;                       //зона нечувствительности регулятора (обороты) (dead band of idling regulator (min-1))
  int16_t  vent_on;                      //температура включения вентилятора (cooling fan's turn on temperature)
  int16_t  vent_off;                     //температура выключения вентилятора (cooling fan's turn off temperature) 

  int16_t  map_adc_factor;               // Поправки для коррекции погрешностей АЦП
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
  
  int16_t  epm_on_threshold;             //порог включения экономайзера мощностных режимов (switch on threshold of EPM)
  
  uint16_t ephh_lot_g;                   //нижний порог ЭПХХ (газ) (lower threshold for forced idle valve(min-1) for gas)
  uint16_t ephh_hit_g;                   //верхний порог ЭПХХ (газ) (upper threshold for forced idle valve(min-1) for gas)
  uint8_t  shutoff_delay;                //задержка выключения клапана (forced idle valve's turn off delay)
 
  uint16_t uart_divisor;                 //делитель для соответствующей скорости UART-a (divider which corresponds to selected baud rate)
  uint8_t  uart_period_t_ms;             //период посылки пакетов в десятках миллисекунд (transmition period of data packets which SECU-3 sends, one discrete = 10ms)
  
  uint8_t ckps_engine_cyl;               //кол-во цилиндров двигателя (number of engine's cylinders)
  
  //--knock 
  uint8_t  knock_use_knock_channel;      //признак использования канала детенации (flag of using knock channel)
  uint8_t  knock_bpf_frequency;          //центральная частота полосового фильтра (Band pass filter frequency)
  int16_t  knock_k_wnd_begin_angle;      //начало детонационного окна (градусы) (Opening angle of knock phase window)
  int16_t  knock_k_wnd_end_angle;        //конец детонационного окна (градусы)  (Closing angle of knock phase window)
  uint8_t  knock_int_time_const;         //постоянная времени интегрирования (код) (Integration time constant)
  //--
  int16_t knock_retard_step;             //шаг смещения УОЗ при детонации (Displacement step of angle)
  int16_t knock_advance_step;            //шаг восстановления УОЗ (Recovery step of angle)
  int16_t knock_max_retard;              //максимальное смещение УОЗ (Maximum displacement of angle)
  uint16_t knock_threshold;              //порог детонации - напряжение (detonation threshold - voltage)
  uint8_t knock_recovery_delay;          //задержка восстановления УОЗ в рабочих циклах двигателя (Recovery delay of angle countable in engine's cycles)
  //--/knock
  
  uint8_t vent_pwm;                      //flag - control cooling fan by using PWM
  //Эти зарезервированные байты необходимы для сохранения бинарной совместимости
  //новых версий прошивок с более старыми версиями. При добавлении новых данных
  //в структуру, необходимо расходовать эти байты.
  //Following reserved bytes required for keeping binary compatibility between 
  //different versions of firmware. Useful when you add/remove members to/from 
  //this structure.
  uint8_t  reserved[9];
  
  uint16_t crc;                         //контрольная сумма данных этой структуры (для проверки корректности данных после считывания из EEPROM)
                                        //CRC of data of this structure (for checking correctness of data after loading from EEPROM)  
}params_t;

//================================================================================
//определяем адреса таблиц в прошивке отталкиваясь от бутлоадера
//Define addresses of tables in the firmware starting from boot loader's address

//размер переменной контрольной суммы параметров в байтах
//size of variable of CRC of parameters in bytes (used in params_t structure)
#define PAR_CRC_SIZE   sizeof(uint16_t) 

//размер переменной контрольной суммы прошивки в байтах
//size of variable of CRC of whole firmware in bytes
#define CODE_CRC_SIZE   sizeof(uint16_t) 

//размер секции приложения без учета контрольной суммы
//Size of application's section without taking into account its CRC
#define CODE_SIZE (SECU3BOOTSTART-CODE_CRC_SIZE)

//количество наборов характеристик хранимых в памяти программ
//Number of sets(families) of characteristics stored in the firmware
#define TABLES_NUMBER  8   

//адрес контрольной суммы в прошивке
//Address of CRC of whole firmware
#define CODE_CRC_ADDR (SECU3BOOTSTART-CODE_CRC_SIZE)

//адрес массива таблиц - семейств характеристик
//Address of array of tables (array of sets of characteristics)
#define TABLES_START (CODE_CRC_ADDR-(sizeof(f_data_t)*TABLES_NUMBER))

//адрес структуры дефаултных параметров (параметров EEPROM по умолчанию)
//Address of structure containing default parameters (loaded when parameters from EEPROM are broken)
#define DEFPARAM_START (TABLES_START-sizeof(params_t))

//адрес дополнительных параметров
//Address of additional parameters (extended). Add new data into this structure
#define FIRMWARE_DATA_START (DEFPARAM_START-sizeof(firmware_data_t))

//================================================================================
//Variables:

//дополнительные данные
//Additional data in the firmware
#pragma object_attribute=__root
extern firmware_data_t __flash fwdata;

//таблицы УОЗ
//Array of tables of advance angle
#pragma object_attribute=__root
extern f_data_t __flash tables[TABLES_NUMBER];

//резервные параметры
//Reserve parameters (loaded when instance in EEPROM is broken)
#pragma object_attribute=__root
extern params_t __flash def_param;

//Check sum of whole firmware (except this check sum and boot loader)
#pragma object_attribute=__root
extern uint16_t __flash code_crc;

#endif //_TABLES_H_
