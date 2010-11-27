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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

/* Структура распределения памяти программ блока управления SECU-3
 *        ________________________
 *       |                        |
 *       |       код              |   
 *       |                        |
 *       |------------------------|
 *       | свободное пространство | 
 *       |------------------------|<--- новые данные и структуры добавлять с этого адреса
 *       | дополнительные парам.  |
 *       |------------------------| 
 *       |  резервные  параметры  |
 *       |------------------------|
 *       |   массив таблиц        |
 *       |                        |
 *       |------------------------|
 *       |     CRC16              |  - контрольная сумма прошивки без учета байт этой  
 *       |________________________|    контрольной суммы и байтов бутлоадера
 *       |                        |
 *       |  boot loader           |
 *        ------------------------
 */



#ifndef _TABLES_H_
#define _TABLES_H_

#include <stdint.h>
#include "bootldr.h"   //для того чтобы знать значение SECU3BOOTSTART, и только

//определяем количество узлов интерполяции для каждой функции
#define F_WRK_POINTS_F         16  
#define F_WRK_POINTS_L         16  
#define F_TMP_POINTS           16
#define F_STR_POINTS           16                            
#define F_IDL_POINTS           16     

#define F_NAME_SIZE            16

#define KC_ATTENUATOR_LOOKUP_TABLE_SIZE 128
#define FW_SIGNATURE_INFO_SIZE 48
#define COIL_ON_TIME_LOOKUP_TABLE_SIZE 16

//Описывает одно семейство характеристик, дискрета УОЗ = 0.5 град.
typedef struct f_data_t
{
  int8_t f_str[F_STR_POINTS];                       // функция УОЗ на старте
  int8_t f_idl[F_IDL_POINTS];                       // функция УОЗ для ХХ
  int8_t f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     // основная функция УОЗ (3D)
  int8_t f_tmp[F_TMP_POINTS];                       // функция коррект. УОЗ по температуре
  uint8_t name[F_NAME_SIZE];                        // ассоциированное имя (имя семейства)
}f_data_t;


//описывает дополнительные данные хранимые в прошивке
typedef struct firmware_data_t
{
  uint8_t fw_signature_info[FW_SIGNATURE_INFO_SIZE];
  
  //таблица усиления аттенюатора (зависимость от оборотов).
  uint8_t attenuator_table[KC_ATTENUATOR_LOOKUP_TABLE_SIZE]; 
  
  //таблица времени накопления энергии в катушках зажигания (зависимость от напряжения)
  uint16_t coil_on_time[COIL_ON_TIME_LOOKUP_TABLE_SIZE];
  
  //used for checking compatibility with management software. Holds size of all data stored in the firmware.
  uint16_t fw_data_size; 
  
  //Эти зарезервированные байты необходимы для сохранения бинарной совместимости
  //новых версий прошивок с более старыми версиями. При добавлении новых данных
  //в структуру, необходимо расходовать эти байты.
  uint8_t reserved[94];  
}firmware_data_t;

//описывает параметры системы
typedef struct params_t
{
  uint8_t  tmp_use;                      //признак комплектации ДТОЖ-ом
  uint8_t  carb_invers;                  //инверсия концевика на карбюраторе
  uint8_t  idl_regul;                    //поддерживать заданные обороты ХХ регулмрованием УОЗ
  uint8_t  fn_benzin;                    //номер набора характеристик используемый для бензина
  uint8_t  fn_gas;                       //номер набора характеристик используемый для газа
  uint16_t map_lower_pressure;           //нижнее значене ДАД по оси таблицы (кПа)
  uint16_t ephh_lot;                     //нижний порог ЭПХХ (мин-1)
  uint16_t ephh_hit;                     //верхний порог ЭПХХ (мин-1)
  uint16_t starter_off;                  //порог выключения стартера (мин-1)
  int16_t  map_upper_pressure;           //верхнее значение ДАД по оси таблицы (кПа)
  uint16_t smap_abandon;                 //обороты перехода с пусковой карты на рабочую  (мин-1) 
  int16_t  max_angle;                    //ограничение максимального УОЗ
  int16_t  min_angle;                    //ограничение минимального УОЗ
  int16_t  angle_corr;                   //октан-коррекция УОЗ    
  uint16_t idling_rpm;                   //заданные обороты ХХ для поддержания регулмрованием УОЗ   
  int16_t  ifac1;                        //коэффициенты регулятора оборотов ХХ, для положительной и
  int16_t  ifac2;                        //отрицательной ошибок соответственно.
  int16_t  MINEFR;                       //зона нечувствительности регулятора (обороты)
  int16_t  vent_on;                      //температура включения вентилятора
  int16_t  vent_off;                     //температура выключения вентилятора  

  int16_t  map_adc_factor;               // Поправки для коррекции погрешностей АЦП
  int32_t  map_adc_correction;           //
  int16_t  ubat_adc_factor;              //
  int32_t  ubat_adc_correction;          //
  int16_t  temp_adc_factor;              //
  int32_t  temp_adc_correction;          //
  
  uint8_t  ckps_edge_type;                
  uint8_t  ckps_cogs_btdc;
  uint8_t  ckps_ignit_cogs;
  
  int16_t  angle_dec_spead;
  int16_t  angle_inc_spead;  
  int16_t  idlreg_min_angle;
  int16_t  idlreg_max_angle;
  uint16_t map_curve_offset;
  uint16_t map_curve_gradient;
  
  int16_t  epm_on_threshold;             //порог включения экономайзера мощностных режимов
  
  uint16_t ephh_lot_g;                   //нижний порог ЭПХХ (газ)
  uint16_t ephh_hit_g;                   //верхний порог ЭПХХ (газ)
  uint8_t  shutoff_delay;                //задержка выключения клапана
 
  uint16_t uart_divisor;                 //делитель для соответствующей скорости UART-a
  uint8_t  uart_period_t_ms;             //период посылки пакетов в десятках миллисекунд
  
  uint8_t ckps_engine_cyl;               //кол-во цилиндров двигателя 
  
  //--knock 
  uint8_t  knock_use_knock_channel;      //признак использования канала детенации
  uint8_t  knock_bpf_frequency;          //центральная частота полосового фильтра
  int16_t  knock_k_wnd_begin_angle;      //начало детонационного окна (градусы)
  int16_t  knock_k_wnd_end_angle;        //конец детонационного окна (градусы)
  uint8_t  knock_int_time_const;         //постоянная времени интегрирования (код)
  //--
  int16_t knock_retard_step;             //шаг смещения УОЗ при детонации 
  int16_t knock_advance_step;            //шаг восстановления УОЗ 
  int16_t knock_max_retard;              //максимальное смещение УОЗ
  uint16_t knock_threshold;              //порог детонации - напряжение
  uint8_t knock_recovery_delay;          //задержка восстановления УОЗ в рабочих циклах двигателя
  //--/knock
  
  uint8_t vent_pwm;                      //flag - control ventilator by using PWM
  //Эти зарезервированные байты необходимы для сохранения бинарной совместимости
  //новых версий прошивок с более старыми версиями. При добавлении новых данных
  //в структуру, необходимо расходовать эти байты.
  uint8_t  reserved[9];
  
  uint16_t crc;                         //контрольная сумма данных этой структуры (для проверки корректности данных после считывания из EEPROM)  
}params_t;

//================================================================================
//определяем адреса таблиц в прошивке отталкиваясь от бутлоадера

//размер переменной контрольной суммы параметров в байтах
#define PAR_CRC_SIZE   sizeof(uint16_t) 

//размер переменной контрольной суммы прошивки в байтах
#define CODE_CRC_SIZE   sizeof(uint16_t) 

//размер кода программы без учета контрольной суммы
#define CODE_SIZE (SECU3BOOTSTART-CODE_CRC_SIZE)

//количество наборов характеристик хранимых в памяти программ
#define TABLES_NUMBER  8   

//адрес контрольной суммы в прошивке
#define CODE_CRC_ADDR (SECU3BOOTSTART-CODE_CRC_SIZE)

//адрес массива таблиц - семейств характеристик
#define TABLES_START (CODE_CRC_ADDR-(sizeof(f_data_t)*TABLES_NUMBER))

//адрес структуры дефаултных параметров (параметров EEPROM по умолчанию)
#define DEFPARAM_START (TABLES_START-sizeof(params_t))

//адрес дополнительных параметров
#define FIRMWARE_DATA_START (DEFPARAM_START-sizeof(firmware_data_t))

//================================================================================

//дополнительные данные по умолчанию
#pragma object_attribute=__root
extern firmware_data_t __flash fwdata;

//данные в таблицах по умолчанию
#pragma object_attribute=__root
extern f_data_t __flash tables[TABLES_NUMBER];

//резервные параметры
#pragma object_attribute=__root
extern params_t __flash def_param;

#pragma object_attribute=__root
extern uint16_t __flash code_crc;

#endif //_TABLES_H_
