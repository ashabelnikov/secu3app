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
#include "vstimer.h"
#include "magnitude.h"

#define OPCODE_EEPROM_PARAM_SAVE 1

//режимы двигателя
#define EM_START 0   
#define EM_IDLE  1
#define EM_WORK  2


//кол-во значений для усреднения частоты вращения к.в.
#define FRQ_AVERAGING           16                          
#define FRQ4_AVERAGING          4

//размер буферов усреднения по каждому аналоговому датчику
#define MAP_AVERAGING           4   
#define BAT_AVERAGING           4   
#define TMP_AVERAGING           8  

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
 
//-----------глобальные переменные----------------------------
s_timer8  send_packet_interval_counter = 0;
s_timer8  force_measure_timeout_counter = 0;
s_timer16 save_param_timeout_counter = 0;
s_timer8  ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE;
s_timer8  engine_rotation_timeout_counter = 0;
s_timer8  epxx_delay_time_counter = 0;
s_timer8  idle_period_time_counter = 0;

unsigned int freq_circular_buffer[FRQ_AVERAGING];     //буфер усреднения частоты вращения коленвала
unsigned int freq4_circular_buffer[FRQ4_AVERAGING];
unsigned int map_circular_buffer[MAP_AVERAGING];      //буфер усреднения абсолютного давления
unsigned int ubat_circular_buffer[BAT_AVERAGING];     //буфер усреднения напряжения бортовой сети
unsigned int temp_circular_buffer[TMP_AVERAGING];     //буфер усреднения температуры охлаждающей жидкости

unsigned char eeprom_parameters_cache[sizeof(params) + 1];

//-------------------------------------------------------------
unsigned int ecuerrors;    //максимум 16 кодов ошибок

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

  //если таймер отсчитал время, то гасим СЕ
  if (s_timer_is_action(ce_control_time_counter))
    SET_CE_STATE(0);       

  //если есть хотя бы одна ошибка - зажигаем СЕ и запускаем таймер 
  if (ecuerrors!=0)
  {
   s_timer_set(ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
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
    eeprom_start_wr_data(0,EEPROM_ECUERRORS_START,(unsigned char*)&write_errors,sizeof(unsigned int));      
   need_to_save = 0;
  }
}

//прерывание по переполению Т/С 2 - для отсчета временных интервалов в системе (для общего использования). 
//Вызывается каждые 10мс
#pragma vector=TIMER2_OVF_vect
__interrupt void timer2_ovf_isr(void)
{ 
  TCNT2 = TIMER2_RELOAD_VALUE; 
  __enable_interrupt();     
    
  s_timer_update(force_measure_timeout_counter);
  s_timer_update(save_param_timeout_counter);
  s_timer_update(send_packet_interval_counter);  
  s_timer_update(ce_control_time_counter);
  s_timer_update(engine_rotation_timeout_counter);   
  s_timer_update(epxx_delay_time_counter);
  s_timer_update(idle_period_time_counter);  
}

//управление отдельными узлами двигателя и обновление данных о состоянии 
//концевика карбюратора, газового клапана, клапана ЭПХХ
void control_engine_units(ecudata *d)
{
  //--инверсия концевика карбюратора если необходимо, включение/выключение клапана ЭПХХ
  d->sens.carb=d->param.carb_invers^GET_THROTTLE_GATE_STATE(); //результат: 0 - дроссель закрыт, 1 - открыт

  //считываем и сохраняем состояние газового клапана
  d->sens.gas = GET_GAS_VALVE_STATE();      

#ifndef VPSEM /* простой алгоритм ЭПХХ без использования таймера и индикации */
  //реализация функции ЭПХХ. Если заслонка карбюратора закрыта и frq > [верх.порог] или
  //заслонка карбюратора закрыта и frq > [ниж.порог] но клапан уже закрыт, то производится
  //выключение подачи топлива путем прекращения подачи напряжения на обмотку эл.клапана. Иначе - подача топлива.
  d->ephh_valve = ((!d->sens.carb)&&(((d->sens.frequen > d->param.ephh_lot)&&(!d->ephh_valve))||
                             (d->sens.frequen > d->param.ephh_hit)))?0:1;
  SET_EPHH_VALVE_STATE(d->ephh_valve);
  
  //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов, но обратно не включается!)
  if (d->sens.frequen4 > d->param.starter_off)
    SET_STARTER_BLOCKING_STATE(1);
  
#else /* сложный алгоритм ЭПХХ с использованием таймера на отключение, с индикацией состояния ЭПХХ и с учетом вида топлива.
    использование параметров пороговых значений ЭПХХ: d->param.ephh_lot - верхний порог для газа
    d->param.ephh_hit - верхний порог для бензина. Нижние пороги на 50 единиц меньше сответственно. */
  if (d->sens.carb) //если дроссель открыт, то открываем клапан, заряжаем таймер и выходим из условия.
  {d->ephh_valve = 1; s_timer_set(epxx_delay_time_counter, EPXX_DELAY_TIME_VALUE);}
  else //если дроссель закрыт, то состояние клапана зависит от оборотов, предыдущего состояния клапана, таймера и вида топлива.
    if (d->sens.gas) // если газовое топливо, то используем параметры d->param.ephh_lot и d->param.ephh_lot-50
      d->ephh_valve = ((s_timer_is_action(epxx_delay_time_counter))
      &&(((d->sens.frequen > d->param.ephh_lot-50)&&(!d->ephh_valve))||(d->sens.frequen > d->param.ephh_lot)))?0:1;
    else // если бензин, то используем параметры d->param.ephh_hit и  d->param.ephh_hit-50
      d->ephh_valve = ((s_timer_is_action(epxx_delay_time_counter))
      &&(((d->sens.frequen > d->param.ephh_hit-50)&&(!d->ephh_valve))||(d->sens.frequen > d->param.ephh_hit)))?0:1;     
  SET_EPHH_VALVE_STATE(d->ephh_valve);
  //управление блокировкой стартера (стартер блокируется при оборотах больше пороговых)
  //и индикация состояния клапана ЭПХХ (используется выход блокировки стартера) 
  SET_STARTER_BLOCKING_STATE( (d->sens.frequen4 > d->param.starter_off)&&(d->ephh_valve) ? 1 : 0);
#endif

  //управление электро вентилятором охлаждения двигателя, при условии что ДТОЖ присутствует в системе 
  if (d->param.tmp_use)
  {
    if (d->sens.temperat >= d->param.vent_on)
       SET_VENTILATOR_STATE(1);
    if (d->sens.temperat <= d->param.vent_off)   
       SET_VENTILATOR_STATE(0); 
  }  
  
}


//обновление буферов усреднения (частота вращения, датчики...)
void update_values_buffers(ecudata* d)
{
  static unsigned char  map_ai  = MAP_AVERAGING-1;
  static unsigned char  bat_ai  = BAT_AVERAGING-1;
  static unsigned char  tmp_ai  = TMP_AVERAGING-1;      
  static unsigned char  frq_ai  = FRQ_AVERAGING-1;
  static unsigned char  frq4_ai = FRQ4_AVERAGING-1;  

  map_circular_buffer[map_ai] = adc_get_map_value();      
  (map_ai==0) ? (map_ai = MAP_AVERAGING - 1): map_ai--;            

  ubat_circular_buffer[bat_ai] = adc_get_ubat_value();      
  (bat_ai==0) ? (bat_ai = BAT_AVERAGING - 1): bat_ai--;            

  temp_circular_buffer[tmp_ai] = adc_get_temp_value();      
  (tmp_ai==0) ? (tmp_ai = TMP_AVERAGING - 1): tmp_ai--;               

  freq_circular_buffer[frq_ai] = d->sens.inst_frq;      
  (frq_ai==0) ? (frq_ai = FRQ_AVERAGING - 1): frq_ai--; 
        
  freq4_circular_buffer[frq4_ai] = d->sens.inst_frq;      
  (frq4_ai==0) ? (frq4_ai = FRQ4_AVERAGING - 1): frq4_ai--;   
}


//усреднение измеряемых величин используя текущие значения кольцевых буферов усреднения, компенсация 
//погрешностей АЦП, перевод измеренных значений в физические величины.
void average_measured_values(ecudata* d)
{     
  unsigned char i;  unsigned long sum;       
            
  for (sum=0,i = 0; i < MAP_AVERAGING; i++)  //усредняем значение с датчика абсолютного давления
   sum+=map_circular_buffer[i];       
  d->sens.map_raw = adc_compensate((sum/MAP_AVERAGING)*2,d->param.map_adc_factor,d->param.map_adc_correction); 
  d->sens.map = map_adc_to_kpa(d->sens.map_raw);
          
  for (sum=0,i = 0; i < BAT_AVERAGING; i++)   //усредняем напряжение бортовой сети
   sum+=ubat_circular_buffer[i];      
  d->sens.voltage_raw = adc_compensate((sum/BAT_AVERAGING)*6,d->param.ubat_adc_factor,d->param.ubat_adc_correction);
  d->sens.voltage = ubat_adc_to_v(d->sens.voltage_raw);  
     
  if (d->param.tmp_use) 
  {       
   for (sum=0,i = 0; i < TMP_AVERAGING; i++) //усредняем температуру (ДТОЖ)
    sum+=temp_circular_buffer[i];      
   d->sens.temperat_raw = adc_compensate((5*(sum/TMP_AVERAGING))/3,d->param.temp_adc_factor,d->param.temp_adc_correction); 
   d->sens.temperat = temp_adc_to_c(d->sens.temperat_raw);
  }  
  else                                       //ДТОЖ не используется
   d->sens.temperat = 0;
               
  for (sum=0,i = 0; i < FRQ_AVERAGING; i++)  //усредняем частоту вращения коленвала
   sum+=freq_circular_buffer[i];      
  d->sens.frequen=(sum/FRQ_AVERAGING);           

  for (sum=0,i = 0; i < FRQ4_AVERAGING; i++) //усредняем частоту вращения коленвала
   sum+=freq4_circular_buffer[i];      
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
    case ADCCOR_PAR: 
    case CKPS_PAR:        
      //если были изменены параметры то сбрасываем счетчик времени
      s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
      break;  
  }
  
  //если были изменены параметры ДПКВ, то немедленно применяем их на работающем двигателе
  if (descriptor == CKPS_PAR)
  {
    ckps_set_edge_type(d->param.ckps_edge_type);
    ckps_set_cogs_btdc(d->param.ckps_cogs_btdc);
    ckps_set_ignition_cogs(d->param.ckps_ignit_cogs);
  }

  //мы обработали принятые данные - приемник ничем теперь не озабочен
  uart_notify_processed();         
 }

 //периодически передаем фреймы с данными
 if (s_timer_is_action(send_packet_interval_counter))
 {
  if (!uart_is_sender_busy())
  {                
   uart_send_packet(d,0);    //теперь передатчик озабочен передачей данных
   s_timer_set(send_packet_interval_counter,SEND_PACKET_INTERVAL_VALUE);
  }
 }

 //передаем нотификационный код завершения последней операции 
 if ((0!=d->op_comp_code)&&(!uart_is_sender_busy()))
  {                
   uart_send_packet(d,OP_COMP_NC);    //теперь передатчик озабочен передачей данных
   d->op_comp_code = 0;
  } 
 
}


//Предварительное измерение перед пуском двигателя
void InitialMeasure(ecudata* d)
{ 
  unsigned char i = 16;
  __enable_interrupt();
  do
  {
    adc_begin_measure();                                                     
    while(!adc_is_measure_ready()); 
    update_values_buffers(d);
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
  char opcode;
  
  if (d->op_actn_code == OPCODE_EEPROM_PARAM_SAVE)
    goto force_parameters_save; //goto - это зло.
    
  //параметры не изменились за заданное время?
  if (s_timer16_is_action(save_param_timeout_counter)) 
  {
    //текущие и сохраненные параметры отличаются?
    if (memcmp(eeprom_parameters_cache,&d->param,sizeof(params)-PAR_CRC_SIZE)) 
    {
force_parameters_save:    
    //мы не можем начать сохранение параметров, так как EEPROM на данный момент занято - сохранение 
    //откладывается и будет осуществлено когда EEPROM освободится и будет вновь вызвана эта функция.
    if (!eeprom_is_idle())
      return;

     memcpy(eeprom_parameters_cache,&d->param,sizeof(params));  
     ((params*)eeprom_parameters_cache)->crc=crc16(eeprom_parameters_cache,sizeof(params)-PAR_CRC_SIZE); //считаем контролбную сумму
     eeprom_start_wr_data(OPCODE_EEPROM_PARAM_SAVE,EEPROM_PARAM_START,eeprom_parameters_cache,sizeof(params));
     
     //если была соответствующая ошибка, то она теряет смысл после того как в EEPROM будут
     //записаны новые параметры с корректной контрольной суммой 
     CLEAR_ECUERROR(ECUERROR_EEPROM_PARAM_BROKEN);
     d->op_actn_code = 0; //обработали       
    }
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
  }
  
  //если есть завершенная операция то сохраняем ее код для отправки нотификации
  opcode = eeprom_take_completed_opcode();
  if (opcode)
   d->op_comp_code = opcode;   
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
  DDRD   = (1<<DDD7)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3)|(1<<DDD1); //вых. PD1 пока UART не проинициализировал TxD 
}
      
__C_task void main(void)
{
  unsigned char mode = EM_START;   
  unsigned char turnout_low_priority_errors_counter = 100;
  signed int advance_angle_inhibitor_state = 0;
  char engine_cycle_occured = 0;
  ecudata edat; 
  
  edat.op_comp_code = 0;
  edat.op_actn_code = 0;
  edat.sens.inst_frq = 0;
    
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
  ckps_set_edge_type(edat.param.ckps_edge_type);
  ckps_set_cogs_btdc(edat.param.ckps_cogs_btdc);
  ckps_set_ignition_cogs(edat.param.ckps_ignit_cogs);
  
  //разрешаем глобально прерывания            
  __enable_interrupt();    

  //------------------------------------------------------------------------     
  while(1)
  {    
    if (ckps_is_cog_changed())
    {
     s_timer_set(engine_rotation_timeout_counter,ENGINE_ROTATION_TIMEOUT_VALUE);    
    }
     
    if (s_timer_is_action(engine_rotation_timeout_counter))
    { //двигатель остановился (его обороты ниже критических)
     ckps_init_state_variables();
     advance_angle_inhibitor_state = 0;
    }
      
    //запускаем измерения АЦП, через равные промежутки времени. При обнаружении каждого рабочего
    //цикла этот таймер переинициализируется. Таким образом, когда частота вращения двигателя превысит
    //определенную величину, это условие выполнятся перестанет.
    if (s_timer_is_action(force_measure_timeout_counter))
    {
     __disable_interrupt();
     adc_begin_measure();
     __enable_interrupt();
     
     s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
     update_values_buffers(&edat);
    }      
  
    //выполняем операции которые необходимо выполнять строго для каждого рабочего цикла.      
    if (ckps_is_cycle_cutover_r())
    {
     update_values_buffers(&edat);       
     s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
        
     // индицирование этих ошибок прекращаем при начале вращения двигателя 
     //(при прошествии N-го количества циклов)
     if (turnout_low_priority_errors_counter == 1)
     {    
      CLEAR_ECUERROR(ECUERROR_EEPROM_PARAM_BROKEN);  
      CLEAR_ECUERROR(ECUERROR_PROGRAM_CODE_BROKEN);  
     }
     if (turnout_low_priority_errors_counter > 0)
      turnout_low_priority_errors_counter--; 
      
      engine_cycle_occured = 1;
    }
   
    //управление фиксированием и индицированием возникающих ошибок
    check_engine();

    //обработка приходящих/уходящих данных последовательного порта
    process_uart_interface(&edat);  
   
    //управление сохранением настроек
    save_param_if_need(&edat);                        
   
    //расчет мгновенной частоты вращения коленвала
    edat.sens.inst_frq = ckps_calculate_instant_freq();                           
    
    //усреднение физических величин хранящихся в кольцевых буферах
    average_measured_values(&edat);        

    //управление периферией
    control_engine_units(&edat);
       
    //в зависимости от текущего типа топлива выбираем соответствующий набор таблиц             
    if (edat.sens.gas)
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_gas];    //на газе
    else  
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_benzin];//на бензине
    
    
    //----------КА состояний системы (диспетчер режимов)--------------
    switch(mode)
    {
      case EM_START: //режим пуска
       if (edat.sens.inst_frq > edat.param.smap_abandon)
       {                   
        mode = EM_IDLE;    
        idling_regulator_init();    
       }      
       edat.curr_angle=start_function(&edat);         //базовый УОЗ - функция для пуска
       edat.airflow = 0;
       break;     
              
      case EM_IDLE: //режим холостого хода
       if (edat.sens.carb)//педаль газа нажали - в рабочий режим
       {
        mode = EM_WORK;
       }      
       edat.curr_angle = idling_function(&edat);      //базовый УОЗ - функция для ХХ 
       edat.curr_angle+=coolant_function(&edat);      //добавляем к УОЗ температурную коррекцию
       edat.curr_angle+=idling_pregulator(&edat,&idle_period_time_counter);//добавляем регулировку
       /*edat.airflow = 0;*/
       break;            
                                             
      case EM_WORK: //рабочий режим 
       if (!edat.sens.carb)//педаль газа отпустили - в переходной режим ХХ
       {
        mode = EM_IDLE;
        idling_regulator_init();    
       }
       edat.curr_angle=work_function(&edat);           //базовый УОЗ - функция рабочего режима
       edat.curr_angle+=coolant_function(&edat);       //добавляем к УОЗ температурную коррекцию
       break;     
       
      default:  //непонятная ситуация - угол в ноль       
       edat.curr_angle = 0;
       break;     
    }
    //-----------------------------------------------------------------------
             
    //добавляем к УОЗ октан-коррекцию
    edat.curr_angle+=edat.param.angle_corr;
      
    //ограничиваем получившийся УОЗ установленными пределами
    restrict_value_to(&edat.curr_angle, edat.param.min_angle, edat.param.max_angle);
        
    //Ограничиваем быстрые изменения УОЗ. Проверка срабатывает один раз за один рабочий чикл. 
    if (engine_cycle_occured)
    {
     edat.curr_angle = advance_angle_inhibitor(edat.curr_angle, &advance_angle_inhibitor_state, ANGLE_MAGNITUDE(3), ANGLE_MAGNITUDE(3));
     engine_cycle_occured = 0;
    } 
    else
    {
     edat.curr_angle = advance_angle_inhibitor_state;
    }

    //сохраняем УОЗ для реализации в ближайшем по времени цикле зажигания       
    ckps_set_dwell_angle(edat.curr_angle);  
  }
  //------------------------------------------------------------------------     
}
