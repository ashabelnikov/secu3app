
#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SEND_PACKET_INTERVAL_VALUE   8
#define SAVE_PARAM_TIMEOUT_VALUE     254
#define FORCE_MEASURE_TIMEOUT_VALUE  8
#define ENGINE_STOP_TIMEOUT_VALUE    25
#define CE_CONTROL_STATE_TIME_VALUE  50

//описывает все входы системы - их производные и интегральные величины
typedef struct
{
 unsigned int map;                                          //давление во впускном коллекторе (усредненное)
 unsigned int voltage;                                      //напр€жение бортовой сети (усредненное)
 unsigned int temperat;                                     //температура охлаждающей жидкости (усредненна€)
 unsigned int frequen;                                      //частота вращени€ коленвала (усредненна€)
 unsigned int inst_frq;                                     //мгновенна€ частота вращени€
 unsigned char carb;                                        //состо€ние концевика карбюратора 
 unsigned char gas;                                         //состо€ние газового клапана 
 unsigned int frequen4;                                     //частота усредненна€ всего по 4-м выборкам 
}sensors;


//описывает данные системы, обеспечивает единый интерфейс данных
typedef struct
{
 params           param;                                      //--параметры
 sensors          sens;                                       //--сенсоры
 unsigned char    ephh_valve;                                 //состо€ние клапана Ёѕ’’
 int              atmos_press;                                //атмосферное давление
 unsigned char    airflow;                                    //расход воздуха
 signed int       curr_angle;                                 //текущий угол опережени€
 __flash F_data*  fn_dat;                                     //указатель на набор характеристик
}ecudata;                                     


#endif  //_SECU3_H_

