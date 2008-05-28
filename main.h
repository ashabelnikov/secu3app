
#ifndef _MAIN_H_
#define _MAIN_H_

#include "tables.h"

#define EEPROM_PARAM_START           0x002                      //адрес структуры параметров в EEPROM 

#define SND_TIMECONST                4
#define PAR_SAVE_COUNTER             254

#define FORCE_MEASURE_TIMEOUT_VALUE  8
#define ENGINE_STOP_TIMEOUT_VALUE    25


//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то 
//напротив середины сердечника ДПКВ должен находиться зуб диска синхронизации определенный ниже (считаем против 
//направления вращения от места выреза).
#define DPKV_COGS_BEFORE_TDC         20                          //количество зубьев после вызеза до в.м.т (18...22)
#define DPKV_DEGREES_PER_COG         6                           //количество градусов приходящееся на один зуб диска
//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в большую сторону 
//возможен выход коммутатора из строя. 
#define DPKV_IGNITION_PULSE_COGS     10                          //длительность импульса зажигания (в зубьях шкива)

#define ANGLE_MULTIPLAYER            40                          //коэффициент масштабирования углов поворота коленвала  

#define FRQ_AVERAGING                16                          //кол-во значений для усреднения частоты вращения к.в.
                                      
#define TIMER2_RELOAD_VALUE          100

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
 unsigned char  new_engine_cycle_happen:1;                    //флаг синхронизации с вращением
 unsigned char  uart_send_busy:1;                             //флаг озабоченности передатчика (занят)
 unsigned char  uart_recv_busy:1;                             //флаг озабоченности приемника (занят)
 unsigned char  adc_sensors_ready:1;                          //датчики обработаны и значения готовы к считыванию           
}bitfield1;


//наиболее критические флаги используемые в прерываниях размещаем в свободных регистрах ввода/вывода
__no_init volatile bitfield1 f1@0x22;

#endif  //_MAIN_H_

