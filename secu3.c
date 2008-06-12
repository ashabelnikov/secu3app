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

  //-----------------Check engine---------------------------
  if (ckps_is_error())
  {
    ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE;
    SET_CE_STATE(1);  
    ckps_reset_error();        
  }

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

  if ((engine_stop_timeout_counter == 0)||(ckps_is_cycle_cutover_r()))
  {
    //обновляем содержимое буфера усреднения  и значение его индекса
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


//усреднение измеряемых величин используя текущие значения буферов усреднения
void average_measured_values(ecudata* d)
{     
  unsigned char i;unsigned long sum;       
  ADCSRA&=~((1<<ADIF)|(1<<ADIE));             //запрещаем прерывание от АЦП не сбрасывая флаг прерывания
            
  for (sum=0,i = 0; i < MAP_AVERAGING; i++)
   sum+=adc_get_map_value(i);      
  d->sens.map=(sum/MAP_AVERAGING)*2; 
          
  for (sum=0,i = 0; i < BAT_AVERAGING; i++)   //усредняем напряжение бортовой сети
   sum+=adc_get_ubat_value(i);      
  d->sens.voltage=(sum/BAT_AVERAGING)*6; 
       
  if (d->param.tmp_use) 
  {       
   for (sum=0,i = 0; i < TMP_AVERAGING; i++) //усредняем температуру (ДТОЖ)
    sum+=adc_get_temp_value(i);      
   d->sens.temperat=((sum/TMP_AVERAGING)*5)/3; 
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
  d->atmos_press = d->sens.map;      //сохраняем атмосферное давление
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
     memcpy(eeprom_parameters_cache,&d->param,sizeof(params));  
     ((params*)eeprom_parameters_cache)->crc=crc16(eeprom_parameters_cache,sizeof(params)-PAR_CRC_SIZE); //считаем контролбную сумму
     eeprom_start_wr_data(EEPROM_PARAM_START,eeprom_parameters_cache,sizeof(params));
    }
   save_param_timeout_counter = SAVE_PARAM_TIMEOUT_VALUE;
  }
}


//загружает параметры из EEPROM, проверяет целостность данных и если они испорчены то
//берет резервную копию из FLASH.
void load_eeprom_params(ecudata* d)
{
 unsigned char* e = (unsigned char*)&d->param;
 int count=sizeof(params);
 int adr=EEPROM_PARAM_START;
 
 if (GET_DEFEEPROM_JUMPER_STATE())
 { 
   //загружаем параметры из EEPROM, а затем проверяем целостность
   do
   {
     __EEGET(*e,adr);
     adr++;
     e++;
   }while(--count); 
 
   EEAR=0x000;      
 
   //при подсчете контрольной суммы не учитываем байты самой контрольной суммы
   //если контрольные суммы не совпадают - загружаем резервные параметры из FLASH
   if (crc16((unsigned char*)&d->param,(sizeof(params)-PAR_CRC_SIZE))!=d->param.crc)
   {
     memcpy_P(&d->param,&def_param,sizeof(params));
   }
 }
 else
 { //перемычка закрыта - загружаем дефаултные параметры
   memcpy_P(&d->param,&def_param,sizeof(params)); 
 }

 //инициализируем кеш параметров, иначе после страта программы произойдет ненужное 
 //их сохранение. 
 memcpy(eeprom_parameters_cache,&d->param,sizeof(params));       
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
  PORTB  = (1<<PB4)|(1<<PB3)|(1<<PB0);                      //клапан ЭПХХ включен, интерфейс с HIP выключен (CS=1, TEST=1)
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
     SET_CE_STATE(1);                                       //код программы испорчен - зажигаем СЕ

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
               
  ckps_init_state();  
  ckps_set_edge_type(0);
  ckps_set_ignition_cogs(10);
  
  //разрешаем глобально прерывания            
  __enable_interrupt();    
     
  while(1)
  {
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
      
    //добавляем к УОЗ октан-коррекцию
    edat.curr_angle+=edat.param.angle_corr;
      
    //ограничиваем получившийся УОЗ установленными пределами
    if (edat.curr_angle > edat.param.max_angle)
               edat.curr_angle = edat.param.max_angle;  
    if (edat.curr_angle < edat.param.min_angle)
               edat.curr_angle = edat.param.min_angle; 
    */
    
    //--------------[TEST]-----------------------
    edat.curr_angle = func(edat.sens.inst_frq, 4.0 ) * ANGLE_MULTIPLAYER;    
    //--------------[/TEST]----------------------

    //сохраняем УОЗ для реализации в ближайшем по времени цикле зажигания       
    ckps_set_dwell_angle(edat.curr_angle);  
  }
}
