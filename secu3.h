
#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SAVE_PARAM_TIMEOUT_VALUE      3000
#define FORCE_MEASURE_TIMEOUT_VALUE   50
#define CE_CONTROL_STATE_TIME_VALUE   50
#define ENGINE_ROTATION_TIMEOUT_VALUE 15
#define IDLE_PERIOD_TIME_VALUE        50

//описывает все входы системы - их производные и интегральные величины
typedef struct
{
 uint16_t map;                           //давление во впускном коллекторе (усредненное)
 uint16_t voltage;                       //напряжение бортовой сети (усредненное)
 int16_t  temperat;                      //температура охлаждающей жидкости (усредненная)
 uint16_t frequen;                       //частота вращения коленвала (усредненная)
 uint16_t inst_frq;                      //мгновенная частота вращения
 uint8_t  carb;                          //состояние концевика карбюратора 
 uint8_t  gas;                           //состояние газового клапана 
 uint16_t frequen4;                      //частота усредненная всего по 4-м выборкам 
 uint16_t knock_k;                       //уровень сигнала детонации 

 //сырые значения датчиков (дискреты АЦП с компенсированными погрешностями)
 int16_t  map_raw;
 int16_t  voltage_raw;
 int16_t  temperat_raw;

}sensors;


//описывает данные системы, обеспечивает единый интерфейс данных
typedef struct
{
 params   param;                        //--параметры
 sensors  sens;                         //--сенсоры
 
 uint8_t  ephh_valve;                   //состояние клапана ЭПХХ
 int16_t  atmos_press;                  //атмосферное давление
 uint8_t  airflow;                      //расход воздуха
 int16_t  curr_angle;                   //текущий угол опережения
 
 __flash F_data*  fn_dat;               //указатель на набор характеристик

 uint8_t  op_comp_code;                 //содержит код который посылается через UART (пакет OP_COMP_NC)
 uint8_t  op_actn_code;                 //содержит код который принимается через UART (пакет OP_COMP_NC)
 uint16_t ecuerrors_for_transfer;       //буферизирует коды ошибок передаваемые через UART в реальном времени.
 uint16_t ecuerrors_saved_transfer;     //буферизирует коды ошибок для чтения/записи в EEPROM, передаваемые/принимаемые через UART.  
 uint8_t  use_knock_channel_prev;       //предыдущее состояние признака использования канала детонации
}ecudata;                                     


#endif  //_SECU3_H_
