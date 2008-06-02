
#ifndef _SECU3_H_
#define _SECU3_H_

#include "tables.h"

#define SEND_PACKET_INTERVAL_VALUE   4
#define SAVE_PARAM_TIMEOUT_VALUE     254
#define FORCE_MEASURE_TIMEOUT_VALUE  8
#define ENGINE_STOP_TIMEOUT_VALUE    25
#define CE_CONTROL_STATE_TIME_VALUE  50

//описывает все входы системы - их производные и интегральные величины
typedef struct
{
 unsigned int map;                                          //давление во впускном коллекторе (усредненное)
 unsigned int voltage;                                      //напряжение бортовой сети (усредненное)
 unsigned int temperat;                                     //температура охлаждающей жидкости (усредненная)
 unsigned int frequen;                                      //частота вращения коленвала (усредненная)
 unsigned int inst_frq;                                     //мгновенная частота вращения
 unsigned char carb;                                        //состояние концевика карбюратора 
 unsigned char gas;                                         //состояние газового клапана 
}sensors;


//описывает данные системы, обеспечивает единый интерфейс данных
typedef struct
{
 params           param;                                      //--параметры
 sensors          sens;                                       //--сенсоры
 unsigned char    ephh_valve;                                 //состояние клапана ЭПХХ
 int              atmos_press;                                //атмосферное давление
 unsigned char    airflow;                                    //расход воздуха
 signed int       curr_angle;                                 //текущий угол опережения
 __flash F_data*  fn_dat;                                     //указатель на набор характеристик
}ecudata;                                     


typedef struct
{
 unsigned char  dpkv_new_engine_cycle_happen:1;               //флаг синхронизации с вращением
 unsigned char  adc_sensors_ready:1;                          //датчики обработаны и значения готовы к считыванию           
 unsigned char  dpkv_returned_to_gap_search:1;                //признак того что были отсчитаны все зубья и КА вновь был переведен в режим поиска синхрометки
 unsigned char  dpkv_error_flag:1;                            //признак ошибки ДПКВ, устанавливается в прерывании от ДПКВ, сбрасывается после обработки
}bitfield1;


//наиболее критические флаги используемые в прерываниях размещаем в свободных регистрах ввода/вывода
__no_init volatile bitfield1 f1@0x22;

#endif  //_SECU3_H_

