
#define SETBIT(x,y)   (x |= (1<<y))    /* установка бита y в байте x*/
#define CLEARBIT(x,y) (x &= (~(1<<y))) /* сброс бита y в байте x*/
#define CHECKBIT(x,y) (x & (1<<y))     /* проверка бита y в байте x*/

#define ADC_DISCRETE       0.0025       //одна дискрета АЦП в вольтах

#define TSENS_SLOPP        0.01        //наклон прямой датчика температуры вольт/градус
#define TSENS_ZERO_POINT   2.73        //напряжение на выходе датчика температуры при 0 градусов цельсия

#define EEPROM_PARAM_START 0x002       //адрес структуры параметров в EEPROM 

#define LOW_ENDIAN_DATA_FORMAT        //Intel data format

#define SND_TIMECONST     250
#define PAR_SAVE_COUNTER  254


#define PAR_CRC_SIZE   sizeof(unsigned short) //размер переменной контрольной суммы параметров в байтах
#define CODE_CRC_SIZE   sizeof(unsigned short) //размер переменной контрольной суммы прошивки в байтах

//размер кода программы без учета контрольной суммы
#define CODE_SIZE (BOOT_START-CODE_CRC_SIZE)


//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке поршня первого цилиндра, то 
//напротив середины сердечника ДПКВ должен находиться зуб диска синхронизации определенный ниже (считаем против 
//направления вращения от места выреза).
#define TEETH_BEFORE_UP         20                                      //количество зубьев после вызеза до в.м.т (18...22)
#define SPEED_MEASURE_TEETH     10                                       //кол-во зубьев для измерения скорости вращения коленвала
#define ANGLE_MULTIPLAYER       40                                      //коэффициент масштабирования углов поворота коленвала  
#define DEGREES_PER_TEETH       6                                       //количество градусов приходящееся на один зуб диска
#define TEETH_BACK_SYNC         55                                      //зуб после которого КА вновь переходит в состояние поиска синхрометки
//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в большую сторону 
//возможен выход коммутатора из строя. 
#define IGNITION_TEETH          10                                      //длительность импульса зажигания (в зубьях шкива)


#define IGNITION_PULSE_14       0x10                                    //маска для старта импульса зажигания на 1-4 цилиндры
#define IGNITION_PULSE_23       0x20                                    //маска для старта импульса зажигания на 2-3 цилиндры
#define IGNITION_PULSE_OFF      0xCF                                    //сброс импульса старта зажигания для двух пар цилиндров одновременно


#define ADC_VREF_TYPE           0xC0
//номера используемых каналов АЦП
#define ADCI_MAP                2
#define ADCI_UBAT               1         
#define ADCI_TEMP               0
//кол-во значений для усреднения измеряемых величин 
#define FRQ_AVERAGING           16                                          
#define MAP_AVERAGING           4   
#define BAT_AVERAGING           6   
#define TMP_AVERAGING           8  

#define T2_RELOAD_VALUE         100


//определяем количество узлов интерполяции для каждой функции
#define F_WRK_POINTS_F         16
#define F_WRK_POINTS_L         16
#define F_TMP_POINTS_T         12
#define F_TMP_POINTS_L         8
#define F_STR_POINTS           16                            
#define F_IDL_POINTS           16     

#define F_NAME_SIZE            16

//переводит температуру из градусов Цельсия в дискреты АЦП
#define T_TO_DADC(Tc) ((unsigned int)((TSENS_ZERO_POINT + (Tc*TSENS_SLOPP))/ADC_DISCRETE)) 


#define TSCALE_LO_VALUE     T_TO_DADC(-16)                      //-16 градусов самая нижняя точка шкалы температуры (в градусах цельсия)
#define TSCALE_STEP      ((unsigned int)((11.0*TSENS_SLOPP)/ADC_DISCRETE)) // 11 градусов между узлами интерполяции по горизонтальной оси (в дискретах АЦП)

#define TABLES_NUMBER          8                                      //количество наборов характеристик хранимых в памяти программ


#ifndef _MAIN_
#define  _MAIN_

//Описывает одно семейство характеристик, дискрета УОЗ = 0.5 град 
typedef struct 
{
  signed   char f_str[F_STR_POINTS];                       // функция УОЗ на старте
  signed   char f_idl[F_IDL_POINTS];                       // функция УОЗ для ХХ
  signed   char f_wrk[F_WRK_POINTS_L][F_WRK_POINTS_F];     // основная функция УОЗ
  signed   char f_tmp[F_TMP_POINTS_L][F_TMP_POINTS_T];     // функция коррект. УОЗ по температуре
  unsigned char name[F_NAME_SIZE];                         // ассоциированное имя (имя семейства)
}F_data;


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

//описывает параметры системы
typedef struct
{
  unsigned char tmp_use;                                    //признак комплектации ДТОЖ-ом
  unsigned char carb_invers;                                //инверсия концевика на карбюраторе
  unsigned char idl_regul;                                  //поддерживать заданные обороты ХХ регулмрованием УОЗ
  unsigned char fn_benzin;                                  //номер набора характеристик используемый для бензина
  unsigned char fn_gas;                                     //номер набора характеристик используемый для газа
  unsigned char map_grad;                                   //наклон шкалы расхода воздуха
  unsigned int  ephh_lot;                                   //нижний порог ЭПХХ
  unsigned int  ephh_hit;                                   //верхний порог ЭПХХ
  unsigned int  starter_off;                                //порог выключения стартера (обороты)
  signed   int  press_swing;                                //перепад давления при полностью открытом дросселе   
  unsigned int  smap_abandon;                               //обороты перехода с пусковой карты на рабочую   
  signed   int  max_angle;                                  //ограничение максимального УОЗ
  signed   int  min_angle;                                  //ограничение минимального УОЗ
  signed   int  angle_corr;                                 //октан-коррекция УОЗ    
  unsigned int  idl_turns;                                  //заданные обороты ХХ для поддержания регулмрованием УОЗ   
  signed   int  ifac1;                                      //коэффициенты П-регулятора оборотов ХХ, для положительной и
  signed   int  ifac2;                                      //отрицательной ошибке соответственно, 1...100 
  signed   int  MINEFR;                                     //зона нечувствительности регулятора (обороты)
  signed   int  vent_on;                                    //температура включения вентилятора
  signed   int  vent_off;                                   //температура выключения вентилятора  
  unsigned short crc;                                       //контрольная сумма данных этой структуры (для проверки корректности данных после считывания из EEPROM)  
}params;

//для деcкриптора 't' (действительна только в домене UART-a)
typedef struct
{
  unsigned char tmp_use;
  signed   int  vent_on;                                    
  signed   int  vent_off;                                   
}ud_t;

//для деcкриптора 'c' (действительна только в домене UART-a)
typedef struct
{
  unsigned int  ephh_lot;                                   
  unsigned int  ephh_hit;                                   
  unsigned char carb_invers;                                
}ud_c;


//для деcкриптора 'r' (действительна только в домене UART-a)
typedef struct
{
  unsigned char idl_regul;                                  
  signed   int  ifac1;                                      
  signed   int  ifac2;                                      
  signed   int  MINEFR;      //зачем signed??? (возможно надо сделать unsigned)                              
  unsigned int  idl_turns;                                  
}ud_r;


//для деcкриптора 'a' (действительна только в домене UART-a)
typedef struct
{
  signed   int  max_angle;                                  
  signed   int  min_angle;                                  
  signed   int  angle_corr;                                 
}ud_a;

//для деcкриптора 'm' (действительна только в домене UART-a)
typedef struct
{
  unsigned char fn_benzin;                
  unsigned char fn_gas;            
  unsigned char map_grad;         
  signed   int  press_swing;   //зачем signed??? (возможно надо сделать unsigned)
}ud_m;

//для деcкриптора 'p' (действительна только в домене UART-a)
typedef struct
{
   unsigned int  starter_off;       
   unsigned int  smap_abandon;             
}ud_p;

//для деcкриптора 'n' (действительна только в домене UART-a)
typedef struct
{
   char snd_mode;
}ud_n;


//для деcкриптора 'f' (действительна только в домене UART-a)
typedef struct
{
  unsigned char tables_num;                                  //количество семейств характеристик
  unsigned char index;                                       //номер набора
  char          name[F_NAME_SIZE];                           //ассоциированное имя (имя семейства)  
}ud_f;

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



//для деcкриптора 's' (действительна только в домене UART-a)
typedef struct 
{
  sensors sens;
  unsigned char    ephh_valve;                                 
  unsigned char    airflow;                                    
  signed int       curr_angle;                                 
}ud_s;



#ifdef LOW_ENDIAN_DATA_FORMAT  //low endian data store format (Intel)
#define GETBYTE(src,rel) *(((unsigned char*)&src+rel))
#define SETBYTE(des,rel) *(((unsigned char*)&des+rel))
#else                          //big endian data store format (Motorola) 
#define GETBYTE(src,rel) *(((unsigned char*)&src+sizeof(src)-1-rel))
#define SETBYTE(des,rel) *(((unsigned char*)&des+sizeof(des)-1-rel))
#endif  

typedef struct
{
 unsigned char   t0mode:1;                                              //0 - счетчик, 1 - таймер
 unsigned char released:1;                                              //флаг реализации старта импульса запуска зажигания
 unsigned char    t1ovf:1;                                              //флаг переполнения таймера 1
 unsigned char  rotsync:1;                                              //флаг синхронизации с вращением
 unsigned char  snd_busy:1;                                             //флаг озабоченности передатчика (занят)
 unsigned char  rcv_busy:1;                                             //флаг озабоченности приемника (занят)
 unsigned char  sens_ready:1;                                           //датчики обработаны и значения готовы к считыванию           
}bitfield1;

#endif

//наиболее критические переменные и флаги размещаем  в регистрах и свободных регистрах ввода/вывода
__no_init volatile bitfield1 f1@0x22;
__no_init __regvar unsigned char TCNT0_H@15;                            //для дополнения таймера/счетчика 0 до 16 разрядов

