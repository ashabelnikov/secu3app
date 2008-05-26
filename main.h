
#ifndef _MAIN_H_
#define _MAIN_H_


#include "tables.h"
#include "bootldr.h"

#define ADC_DISCRETE            0.0025       //одна дискрета АЦП в вольтах

#define TSENS_SLOPP             0.01        //наклон прямой датчика температуры вольт/градус
#define TSENS_ZERO_POINT        2.73        //напряжение на выходе датчика температуры при 0 градусов цельсия

#define EEPROM_PARAM_START      0x002       //адрес структуры параметров в EEPROM 

#define SND_TIMECONST           4
#define PAR_SAVE_COUNTER        254

#define FORCE_MEASURE_TIMEOUT_VALUE 8


//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке поршня первого цилиндра, то 
//напротив середины сердечника ДПКВ должен находиться зуб диска синхронизации определенный ниже (считаем против 
//направления вращения от места выреза).
#define TEETH_BEFORE_UP         20                                      //количество зубьев после вызеза до в.м.т (18...22)
#define ANGLE_MULTIPLAYER       40                                      //коэффициент масштабирования углов поворота коленвала  
#define DEGREES_PER_TEETH       6                                       //количество градусов приходящееся на один зуб диска

//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в большую сторону 
//возможен выход коммутатора из строя. 
#define IGNITION_TEETH          10                                      //длительность импульса зажигания (в зубьях шкива)


#define IGNITION_PULSE_14       0x10                                    //маска для старта импульса зажигания на 1-4 цилиндры
#define IGNITION_PULSE_23       0x20                                    //маска для старта импульса зажигания на 2-3 цилиндры
#define IGNITION_PULSE_OFF      0xCF                                    //сброс импульса старта зажигания для двух пар цилиндров одновременно

//кол-во значений для усреднения измеряемых величин 
#define FRQ_AVERAGING           16                                          
#define T2_RELOAD_VALUE         100

//переводит температуру из градусов Цельсия в дискреты АЦП
#define T_TO_DADC(Tc) ((unsigned int)((TSENS_ZERO_POINT + (Tc*TSENS_SLOPP))/ADC_DISCRETE)) 


#define TSCALE_LO_VALUE     T_TO_DADC(-16)                      //-16 градусов самая нижняя точка шкалы температуры (в градусах цельсия)
#define TSCALE_STEP      ((unsigned int)((11.0*TSENS_SLOPP)/ADC_DISCRETE)) // 11 градусов между узлами интерполяции по горизонтальной оси (в дискретах АЦП)

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
 unsigned char  timer1_overflow_happen:1;                     //флаг переполнения таймера 1
 unsigned char  new_engine_cycle_happen:1;                    //флаг синхронизации с вращением
 unsigned char  uart_send_busy:1;                             //флаг озабоченности передатчика (занят)
 unsigned char  uart_recv_busy:1;                             //флаг озабоченности приемника (занят)
 unsigned char  adc_sensors_ready:1;                          //датчики обработаны и значения готовы к считыванию           
}bitfield1;


//наиболее критические флаги используемые в прерываниях размещаем в свободных регистрах ввода/вывода
__no_init volatile bitfield1 f1@0x22;

#endif  //_MAIN_H_

