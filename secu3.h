
#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SEND_PACKET_INTERVAL_VALUE    8
#define SAVE_PARAM_TIMEOUT_VALUE      3000
#define FORCE_MEASURE_TIMEOUT_VALUE   50
#define CE_CONTROL_STATE_TIME_VALUE   50
#define ENGINE_ROTATION_TIMEOUT_VALUE 15
#define IDLE_PERIOD_TIME_VALUE        50

//описывает все входы системы - их производные и интегральные величины
typedef struct
{
 unsigned int map;                             //давление во впускном коллекторе (усредненное)
 unsigned int voltage;                         //напряжение бортовой сети (усредненное)
 signed int temperat;                          //температура охлаждающей жидкости (усредненная)
 unsigned int frequen;                         //частота вращения коленвала (усредненная)
 unsigned int inst_frq;                        //мгновенная частота вращения
 unsigned char carb;                           //состояние концевика карбюратора 
 unsigned char gas;                            //состояние газового клапана 
 unsigned int frequen4;                        //частота усредненная всего по 4-м выборкам 
 unsigned int knock_k;                         //уровень сигнала детонации 

 //сырые значения датчиков (дискреты АЦП с компенсированными погрешностями)
 signed int map_raw;
 signed int voltage_raw;
 signed int temperat_raw;

}sensors;


//описывает данные системы, обеспечивает единый интерфейс данных
typedef struct
{
 params           param;                        //--параметры
 sensors          sens;                         //--сенсоры
 
 unsigned char    ephh_valve;                   //состояние клапана ЭПХХ
 int              atmos_press;                  //атмосферное давление
 unsigned char    airflow;                      //расход воздуха
 signed int       curr_angle;                   //текущий угол опережения
 
 __flash F_data*  fn_dat;                       //указатель на набор характеристик

 char             op_comp_code;                 //содержит код который посылается через UART (пакет OP_COMP_NC)
 char             op_actn_code;                 //содержит код который принимается через UART (пакет OP_COMP_NC)
 unsigned int     ecuerrors_for_transfer;       //буферизирует коды ошибок передаваемые через UART в реальном времени.
 unsigned int     ecuerrors_saved_transfer;     //буферизирует коды ошибок для чтения/записи в EEPROM, передаваемые/принимаемые через UART.  
 unsigned char    use_knock_channel_prev;       //предыдущее состояние признака использования канала детонации
}ecudata;                                     


#endif  //_SECU3_H_

