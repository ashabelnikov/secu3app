 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#pragma language=extended  // enable use of extended keywords

#include <inavr.h>
#include <iom16.h>
#include <pgmspace.h>

#include "funconv.h"
#include "secu3.h"
#include "uart.h"
#include "tables.h"
#include "bootldr.h"
#include "ufcodes.h"
#include "crc16.h"
#include "eeprom.h"
#include "bitmask.h"
#include "adc.h"
#include "ckps.h"

#define FRQ_AVERAGING                16                          //кол-во значений для усреднения частоты вращения к.в.
#define FRQ4_AVERAGING               4

#define TIMER2_RELOAD_VALUE          100                         //для 10 мс
#define EEPROM_PARAM_START           0x002                       //адрес структуры параметров в EEPROM 
#define EEPROM_ECUERRORS_START       (EEPROM_PARAM_START+(sizeof(params)))

//включает/выключает лампу Check Engine  
#define SET_CE_STATE(s)  {PORTB_Bit2 = s;}

//включает/выключает вентилятор
#define SET_VENTILATOR_STATE(s) {PORTB_Bit1 = s;}

//блокирует/разблокирывает стартер
#define SET_STARTER_BLOCKING_STATE(s) {PORTD_Bit7 = s;}

//открывает/закрывает клапан ЭПХХ
#define SET_EPHH_VALVE_STATE(s) {PORTB_Bit0 = s;}

//считывает состояние газового клапана
#define GET_GAS_VALVE_STATE(s) (PINC_Bit6)

//считывает состояние дроссельной заслонки (только значение, без инверсии)
#define GET_THROTTLE_GATE_STATE(s) (PINC_Bit5)

#define GET_DEFEEPROM_JUMPER_STATE() (PINC_Bit2)

#define disable_comparator() {ACSR=(1<<ACD);}

 
//--глобальные переменные     
unsigned char send_packet_interval_counter = 0;
unsigned char force_measure_timeout_counter = 0;
unsigned char engine_stop_timeout_counter = 0;
unsigned char save_param_timeout_counter = 0;
unsigned char ce_control_time_counter = 0;

unsigned int freq_average_buf[FRQ_AVERAGING];                     //буфер усреднения частоты вращения коленвала
unsigned int freq4_average_buf[FRQ4_AVERAGING];

unsigned char eeprom_parameters_cache[64];

//-------------------------------------------------------------
unsigned int ecuerrors;

//определяем биты ошибок
#define ECUERROR_CKPS_MALFUNCTION     0
#define ECUERROR_EEPROM_PARAM_BROKEN  1
#define ECUERROR_PROGRAM_CODE_BROKEN  2

//операции над ошибками
#define SET_ECUERROR(error)   SETBIT(ecuerrors,error)
#define CLEAR_ECUERROR(error) CLEARBIT(ecuerrors,error)
//-------------------------------------------------------------


//При возникновении любой ошибки, СЕ загорается на фиксированное время. Если ошибка не исчезает (например испорчен код программы),
//то CE будет гореть непрерывно. При запуске программы СЕ загорается на 0.5 сек. для индицирования работоспособности. 
void check_engine(void)
{
  unsigned int temp_errors;
  static unsigned int merged_errors = 0; //кеширует ошибки для сбережения ресурса EEPROM
  static unsigned int write_errors;      //ф. eeprom_start_wr_data() запускает фоновый процесс!
  static unsigned char need_to_save = 0; 

  //если была ошибка ДПКВ то устанавливаем бит соответствующей ошибки
  if (ckps_is_error())
  {
    SET_ECUERROR(ECUERROR_CKPS_MALFUNCTION);
    ckps_reset_error();        
  }
  else
  {
    CLEAR_ECUERROR(ECUERROR_CKPS_MALFUNCTION);  
  }

  //если есть хотя бы одна ошибка - зажигаем СЕ
  if (ecuerrors!=0)
  {
   ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE;
   SET_CE_STATE(1);  
  }

  temp_errors = (merged_errors | ecuerrors);
  if (temp_errors!=merged_errors) //появилась ли ошибка которой нет в merged_errors?
  {
   //так как на момент возникновения новой ошибки EEPROM может быть занято (например сохранением параметров),
   //то необходимо установить только признак, который будет оставаться установленным до тех пор пака EEPROM
   //не освободится и ошибки не будут сохранены. 
   need_to_save = 1;
  }

  merged_errors = temp_errors;

  //Если EEPROM не занято и есть новая ошибка. То необходимо сохранить новую ошибку.
  //Для сбережения ресурса EEPROM cохранение ошибки произойдет только в том случае, если 
  //она еще не была сохранена. Для этого производится чтение и сравнение.  
  if (eeprom_is_idle() && need_to_save)
  {
   eeprom_read(&temp_errors,EEPROM_ECUERRORS_START,sizeof(unsigned int));
   write_errors = temp_errors | merged_errors; 
   if (write_errors!=temp_errors)    
    eeprom_start_wr_data(EEPROM_ECUERRORS_START,(unsigned char*)&write_errors,sizeof(unsigned int));      
   need_to_save = 0;
  }
}



//прерывание по переполению Т/С 2 - для отсчета временных интервалов в системе (для общего использования). 
//Вызывается каждые 10мс
#pragma vector=TIMER2_OVF_vect
__interrupt void timer2_ovf_isr(void)
{ 
  TCNT2 = TIMER2_RELOAD_VALUE; 

  if (force_measure_timeout_counter > 0)
    force_measure_timeout_counter--;
  else
  {
    force_measure_timeout_counter = FORCE_MEASURE_TIMEOUT_VALUE;
    adc_begin_measure();
  }      
  __enable_interrupt();     
    
  if (save_param_timeout_counter > 0)
    save_param_timeout_counter--; 

  if (send_packet_interval_counter > 0)
    send_packet_interval_counter--;

  if (engine_stop_timeout_counter > 0)
    engine_stop_timeout_counter--;

  if (ckps_is_rotation_cutover_r())
  {
    engine_stop_timeout_counter = ENGINE_STOP_TIMEOUT_VALUE;   
    force_measure_timeout_counter = FORCE_MEASURE_TIMEOUT_VALUE;

    // индицирование этих ошибок можно прекращать при начале вращения двигателя или N-оборотов...
    //CLEAR_ECUERROR(ECUERROR_EEPROM_PARAM_BROKEN);  
    //CLEAR_ECUERROR(ECUERROR_PROGRAM_CODE_BROKEN);  
  }

  //-----------------Check engine---------------------------
  if (ce_control_time_counter > 0) 
    ce_control_time_counter--;
  else 
    SET_CE_STATE(0);       
 //--------------------------------------------------------
}

//обновление буфера усреднения для частоты вращения
void update_buffer_freq(ecudata* d)
{
  static unsigned char frq_ai = FRQ_AVERAGING-1;
  static unsigned char frq4_ai = FRQ4_AVERAGING-1;

    //обновляем содержимое буфера усреднения  и значение его индекса
  if ((engine_stop_timeout_counter == 0)||(ckps_is_cycle_cutover_r()))
  {
    freq_average_buf[frq_ai] = d->sens.inst_frq;      
    (frq_ai==0) ? (frq_ai = FRQ_AVERAGING - 1): frq_ai--; 
        
    freq4_average_buf[frq4_ai] = d->sens.inst_frq;      
    (frq4_ai==0) ? (frq4_ai = FRQ4_AVERAGING - 1): frq4_ai--; 

    engine_stop_timeout_counter = ENGINE_STOP_TIMEOUT_VALUE;   
  }
}

//управление отдельными узлами двигателя и обновление данных о состоянии 
//концевика карбюратора, газового клапана, клапана ЭПХХ
void control_engine_units(ecudata *d)
{
  //реализация функции ЭПХХ. Если заслонка карбюратора закрыта и frq > [верх.порог] или
  //заслонка карбюратора закрыта и frq > [ниж.порог] но клапан уже закрыт, то производится
  //выключение подачи топлива путем прекращения подачи напряжения на обмотку эл.клапана. Иначе - подача топлива.
  //--инверсия концевика карбюратора если необходимо, включение/выключение клапана ЭПХХ
  d->sens.carb=d->param.carb_invers^GET_THROTTLE_GATE_STATE(); //результат: 0 - дроссель закрыт, 1 - открыт
  d->ephh_valve = ((!d->sens.carb)&&(((d->sens.frequen > d->param.ephh_lot)&&(!d->ephh_valve))||
                             (d->sens.frequen > d->param.ephh_hit)))?0:1;
  SET_EPHH_VALVE_STATE(d->ephh_valve);
 
#ifndef VPSEM_STARTER_BLOCKING
  //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов)
  if (d->sens.frequen4 > d->param.starter_off)
    SET_STARTER_BLOCKING_STATE(1);
#else
  //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов)
  //и управление индикацией состояния клапана ЭПХХ (используется выход блокировки стартера) 
  SET_STARTER_BLOCKING_STATE( (d->sens.frequen4 > d->param.starter_off)&&(d->ephh_valve) ? 1 : 0);
#endif
 
  if (d->param.tmp_use)
  {
    //управление электро вентилятором охлаждения двигателя
    if (d->sens.temperat >= d->param.vent_on)
       SET_VENTILATOR_STATE(1);
    if (d->sens.temperat <= d->param.vent_off)   
       SET_VENTILATOR_STATE(0); 
  }  
  
  //считываем и сохраняем состояние газового клапана
  d->sens.gas = GET_GAS_VALVE_STATE();    
}


//усреднение измеряемых величин используя текущие значения буферов усреднения, компенсация погрешностей
void average_measured_values(ecudata* d)
{     
  unsigned char i;unsigned long sum;       
  ADCSRA&=~((1<<ADIF)|(1<<ADIE));             //запрещаем прерывание от АЦП не сбрасывая флаг прерывания
            
  for (sum=0,i = 0; i < MAP_AVERAGING; i++)
   sum+=adc_get_map_value(i);       
  d->sens.map_raw = adc_compensate((sum/MAP_AVERAGING)*2,d->param.map_adc_factor,d->param.map_adc_correction); 
  d->sens.map = map_adc_to_kpa(d->sens.map_raw);
          
  for (sum=0,i = 0; i < BAT_AVERAGING; i++)   //усредняем напряжение бортовой сети
   sum+=adc_get_ubat_value(i);      
  d->sens.voltage_raw = adc_compensate((sum/BAT_AVERAGING)*6,d->param.ubat_adc_factor,d->param.ubat_adc_correction);; 
  d->sens.voltage = ubat_adc_to_v(d->sens.voltage_raw);  
     
  if (d->param.tmp_use) 
  {       
   for (sum=0,i = 0; i < TMP_AVERAGING; i++) //усредняем температуру (ДТОЖ)
    sum+=adc_get_temp_value(i);      
   d->sens.temperat_raw = adc_compensate((5*(sum/TMP_AVERAGING))/3,d->param.temp_adc_factor,d->param.temp_adc_correction); 
   d->sens.temperat = temp_adc_to_c(d->sens.temperat_raw);
  }  
  else             //ДТОЖ не используется
   d->sens.temperat=0;
    
  ADCSRA=(ADCSRA&(~(1<<ADIF)))|(1<<ADIE);    //разрешаем прерывание от АЦП не сбрасывая флаг прерывания
           
  for (sum=0,i = 0; i < FRQ_AVERAGING; i++)    //усредняем частоту вращения коленвала
   sum+=freq_average_buf[i];      
  d->sens.frequen=(sum/FRQ_AVERAGING);           

  for (sum=0,i = 0; i < FRQ4_AVERAGING; i++)    //усредняем частоту вращения коленвала
   sum+=freq4_average_buf[i];      
  d->sens.frequen4=(sum/FRQ4_AVERAGING);           

}


//обрабатывает передаваемые/принимаемые фреймы UART-a
void process_uart_interface(ecudata* d)
{ 
 unsigned char descriptor;

 if (uart_is_packet_received())//приняли новый фрейм ?
 { 
  descriptor = uart_recept_packet(d);
  switch(descriptor)
  {
    case TEMPER_PAR:            
    case CARBUR_PAR:   
    case IDLREG_PAR:   
    case ANGLES_PAR:   
    case FUNSET_PAR:   
    case STARTR_PAR:   
      //если были изменены параметры то сбрасываем счетчик времени
      save_param_timeout_counter = SAVE_PARAM_TIMEOUT_VALUE;
      break;  
  }

  //мы обработали принятые данные - приемник ничем теперь не озабочен
  uart_notify_processed();         
 }

 //периодически передаем фреймы с данными
 if (send_packet_interval_counter==0)
 {
  if (!uart_is_sender_busy())
  {                
   uart_send_packet(d);    //теперь передатчик озабочен передачей данных
   send_packet_interval_counter = SEND_PACKET_INTERVAL_VALUE;
  }
 }
}


//Предварительное измерение перед пуском двигателя
void InitialMeasure(ecudata* d)
{ 
  unsigned char i=16;
  __enable_interrupt();
  do
  {
    adc_begin_measure();                                                     
    while(!adc_is_measure_ready()); 
  }while(--i);  
  __disable_interrupt();
  average_measured_values(d);  
  d->atmos_press = d->sens.map;      //сохраняем атмосферное давление в кПа!
}

//Запись данных в EEPROM - процесс очень медленный. Он будет проходить параллельно с выполнением программы,
//а для обеспечения атомарности копируем данные в отдельный буфер и из него их потом пишем в EEPROM.
//Сохранение данных в EEPROM произойдет только если за заданное время не произошло ни одной операции приема параметров
//из UART-a и сохраненные параметры отличаются от текущих.        
void save_param_if_need(ecudata* d)
{
  //параметры не изменились за заданное время
  if (save_param_timeout_counter==0) 
  {
    //текущие и сохраненные параметры отличаются?
    if (memcmp(eeprom_parameters_cache,&d->param,sizeof(params)-PAR_CRC_SIZE)) 
    {
    //мы не можем начать сохранение параметров, так как EEPROM на данный момент занято - сохранение 
    //откладывается и будет осуществлено когда EEPROM освободится и будет вновь вызвана эта функция.
    if (!eeprom_is_idle())
      return;

     memcpy(eeprom_parameters_cache,&d->param,sizeof(params));  
     ((params*)eeprom_parameters_cache)->crc=crc16(eeprom_parameters_cache,sizeof(params)-PAR_CRC_SIZE); //считаем контролбную сумму
     eeprom_start_wr_data(EEPROM_PARAM_START,eeprom_parameters_cache,sizeof(params));
     
     //если была соответствующая ошибка, то она теряет смысл после того как в EEPROM будут
     //записаны новые параметры с корректной контрольной суммой 
     CLEAR_ECUERROR(ECUERROR_EEPROM_PARAM_BROKEN); 
    }
    save_param_timeout_counter = SAVE_PARAM_TIMEOUT_VALUE;
  }
}

//загружает параметры из EEPROM, проверяет целостность данных и если они испорчены то
//берет резервную копию из FLASH.
void load_eeprom_params(ecudata* d)
{
 if (GET_DEFEEPROM_JUMPER_STATE())
 { 
   //Загружаем параметры из EEPROM, а затем проверяем целостность.
   //При подсчете контрольной суммы не учитываем байты самой контрольной суммы
   //если контрольные суммы не совпадают - загружаем резервные параметры из FLASH
   eeprom_read(&d->param,EEPROM_PARAM_START,sizeof(params));  
   
   if (crc16((unsigned char*)&d->param,(sizeof(params)-PAR_CRC_SIZE))!=d->param.crc)
   {
     memcpy_P(&d->param,&def_param,sizeof(params));
     SET_ECUERROR(ECUERROR_EEPROM_PARAM_BROKEN);
   }
   
   //инициализируем кеш параметров, иначе после старта программы произойдет ненужное 
   //их сохранение. 
   memcpy(eeprom_parameters_cache,&d->param,sizeof(params));         
 }
 else
 { //перемычка закрыта - загружаем дефаултные параметры, которые позже будут сохранены    
   memcpy_P(&d->param,&def_param,sizeof(params)); 
 }
} 
   

void init_system_timer(void)
{
  TCCR2 = (1<<CS22)|(1<<CS21)|(1<<CS20);      //clock = 15.625kHz  
  TIMSK|= (1<<TOIE2); //разрешаем прерывание по переполнению таймера 2                          
}

void init_io_ports(void)
{
  //конфигурируем порты ввода/вывода
  PORTA  = 0;   
  DDRA   = 0;       
  PORTB  = (1<<PB2)|(1<<PB4)|(1<<PB3)|(1<<PB0);             //CE горит(для проверки), клапан ЭПХХ включен, интерфейс с HIP выключен (CS=1, TEST=1)
  DDRB   = (1<<DDB4)|(1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);   
  PORTC  = (1<<PC3)|(1<<PC2);
  DDRC   = 0;  
  PORTD  = (1<<PD6)|(1<<PD3)|(1<<PD7);                      //стартер заблокирован, режим интегрирования для HIP
  DDRD   = (1<<DDD7)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3);
}


//---------------[TEST]------------------------
const float  K1=0.008654,       K2=0.004688,   K3=0.0;
const float  B1=-5.19,          B2=7.49,       B3=30;
const int    N1=600,  N2=3200,  N3=4800;

float func(int it,float tkorr)
{
    if (it < N1)   
        return 0.0;  
    if (it < N2)
        return (K1 * it + B1) + tkorr;
    if (it < N3)
        return (K2 * it + B2) + tkorr;
    else
        return (K3 * it + B3) + tkorr;
}
//--------------[/TEST]-------------------------


      
__C_task void main(void)
{
  unsigned char mode=0;
  ecudata edat; 
    
  init_io_ports();

  if (crc16f(0,CODE_SIZE)!=code_crc)
  { //код программы испорчен - зажигаем СЕ
    SET_ECUERROR(ECUERROR_PROGRAM_CODE_BROKEN); 
  }

  adc_init();

  //запрещаем компаратор - он нам не нужен  
  disable_comparator();             

  //проводим несколько циклов измерения датчиков для инициализации данных
  InitialMeasure(&edat);   
   
  //снимаем блокировку стартера
  SET_STARTER_BLOCKING_STATE(0); 
     
  //читаем параметры
  load_eeprom_params(&edat);

  init_system_timer();
  
  //инициализируем UART
  uart_init(CBR_9600);
  
  //инициализируем модуль ДПКВ             
  ckps_init_state();  
  ckps_set_edge_type(0);
  ckps_set_ignition_cogs(CKPS_IGNITION_PULSE_COGS);
  
  //разрешаем глобально прерывания            
  __enable_interrupt();    

  //------------------------------------------------------------------------     
  while(1)
  {
    //управление фиксированием и индицированием возникающих ошибок
    check_engine();

    //обработка приходящих/уходящих данных последовательного порта
    process_uart_interface(&edat);  
   
    //управление сохранением настроек
    save_param_if_need(&edat);                        
    
    edat.sens.inst_frq = ckps_calculate_instant_freq();                           

    update_buffer_freq(&edat);

    average_measured_values(&edat);        

    control_engine_units(&edat);
       
    //в зависимости от текущего типа топлива выбираем соответствующий набор таблиц             
    if (edat.sens.gas)
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_gas];    //на газе
    else  
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_benzin];//на бензине
    
    /*
    //КА состояний системы
    switch(mode)
    {
      case 0: //режим пуска
        if (edat.sens.inst_frq > edat.param.smap_abandon)
        {                   
         mode=1;        
        }      
        edat.curr_angle=str_func(&edat);                //базовый УОЗ - функция для пуска
        edat.airflow=0;
        break;            
      case 1: //режим холостого хода
       if (edat.sens.carb)//педаль газа нажали - в рабочий режим
       {
        mode=2;
       }      
        edat.curr_angle=idl_func(&edat);               //базовый УОЗ - функция для ХХ 
        edat.curr_angle+=tmp_func(&edat);              //добавляем к УОЗ температурную коррекцию
        edat.curr_angle+=idl_pregul(&edat);            //добавляем регулировку
        edat.airflow=0;
        break;            
      case 2: //рабочий режим 
       if (edat.sens.carb)//педаль газа отпустили - в режим ХХ
       {
        mode=1;
       }
       edat.curr_angle=wrk_func(&edat);                //базовый УОЗ - функция рабочего режима
       edat.curr_angle+=tmp_func(&edat);               //добавляем к УОЗ температурную коррекцию
        break;     
      default:  //непонятная ситуация - угол в ноль       
        edat.curr_angle=0;
        break;     
    }
      
    */
    
    //--------------[TEST]-----------------------
    edat.curr_angle = func(edat.sens.inst_frq, 0.0 ) * ANGLE_MULTIPLAYER;    
    //--------------[/TEST]----------------------

    //добавляем к УОЗ октан-коррекцию
    edat.curr_angle+=edat.param.angle_corr;
      
/*    //ограничиваем получившийся УОЗ установленными пределами
    if (edat.curr_angle > edat.param.max_angle)
               edat.curr_angle = edat.param.max_angle;  
    if (edat.curr_angle < edat.param.min_angle)
               edat.curr_angle = edat.param.min_angle; */


    //сохраняем УОЗ для реализации в ближайшем по времени цикле зажигания       
    ckps_set_dwell_angle(edat.curr_angle);  
  }
  //------------------------------------------------------------------------     
}
