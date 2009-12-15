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
#include "ce_errors.h"
#include "knock.h"
#include "suspendop.h"
#include "measure.h"
#include "knklogic.h"

//режимы двигателя
#define EM_START 0   
#define EM_IDLE  1
#define EM_WORK  2

//включает/выключает вентилятор
#define SET_VENTILATOR_STATE(s) {PORTB_Bit1 = s;}

//блокирует/разблокирывает стартер
#define SET_STARTER_BLOCKING_STATE(s) {PORTD_Bit7 = s;}

//открывает/закрывает клапан ЭПХХ
#define SET_EPHH_VALVE_STATE(s) {PORTB_Bit0 = s;}

//открывает/закрывает клапан ЭМР
#define SET_EPM_VALVE_STATE(s) {PORTC_Bit7 = s;}

#define GET_DEFEEPROM_JUMPER_STATE() (PINC_Bit2)

#define disable_comparator() {ACSR=(1<<ACD);}
 
 
uint8_t eeprom_parameters_cache[sizeof(params) + 1];

ecudata edat;

//управление отдельными узлами двигателя и обновление данных о состоянии 
//концевика карбюратора, газового клапана, клапана ЭПХХ
void control_engine_units(ecudata *d)
{
  int16_t discharge;

  //реализация функции ЭПХХ. Если заслонка карбюратора закрыта и frq > [верх.порог] или
  //заслонка карбюратора закрыта и frq > [ниж.порог] но клапан уже закрыт, то производится
  //выключение подачи топлива путем прекращения подачи напряжения на обмотку эл.клапана. Иначе - подача топлива.  
  if (d->sens.carb) //если дроссель открыт, то открываем клапан, заряжаем таймер и выходим из условия.
  {
   d->ephh_valve = 1; 
   s_timer_set(epxx_delay_time_counter, d->param.shutoff_delay);
  }
  else //если дроссель закрыт, то состояние клапана зависит от оборотов, предыдущего состояния клапана, таймера и вида топлива.
    if (d->sens.gas) //газовое топливо
      d->ephh_valve = ((s_timer_is_action(epxx_delay_time_counter))
      &&(((d->sens.frequen > d->param.ephh_lot_g)&&(!d->ephh_valve))||(d->sens.frequen > d->param.ephh_hit_g)))?0:1;
    else //бензин
      d->ephh_valve = ((s_timer_is_action(epxx_delay_time_counter))
      &&(((d->sens.frequen > d->param.ephh_lot)&&(!d->ephh_valve))||(d->sens.frequen > d->param.ephh_hit)))?0:1;     
  SET_EPHH_VALVE_STATE(d->ephh_valve);

#ifndef VPSEM   
  //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов, но обратно не включается!)
  if (d->sens.frequen4 > d->param.starter_off)
    SET_STARTER_BLOCKING_STATE(1);  
#else 
  //управление блокировкой стартера (стартер блокируется при оборотах больше пороговых)
  //и индикация состояния клапана ЭПХХ (используется выход блокировки стартера) 
  SET_STARTER_BLOCKING_STATE( (d->sens.frequen4 > d->param.starter_off)&&(d->ephh_valve) ? 1 : 0);
  //если расход воздуха максимальный - зажигаем СЕ и запускаем таймер 
    if (d->airflow > 15)
     {
      s_timer_set(ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
      SET_CE_STATE(1);  
     }
#endif

  //управление электро вентилятором охлаждения двигателя, при условии что ДТОЖ присутствует в системе 
  if (d->param.tmp_use)
  {
    if (d->sens.temperat >= d->param.vent_on)
       SET_VENTILATOR_STATE(1);
    if (d->sens.temperat <= d->param.vent_off)   
       SET_VENTILATOR_STATE(0); 
  }  
  
  //Управление ЭМР (экономайзер мощностных режимов)
  discharge = (d->param.map_upper_pressure - d->sens.map);
  if (discharge < 0) discharge = 0;    
  d->epm_valve = discharge < d->param.epm_on_threshold;
  SET_EPM_VALVE_STATE(d->epm_valve);
}

//обрабатывает передаваемые/принимаемые фреймы UART-a
void process_uart_interface(ecudata* d)
{ 
 uint8_t descriptor;

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
    case KNOCK_PAR:  
    case MISCEL_PAR:     
      //если были изменены параметры то сбрасываем счетчик времени
      s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
      break;        
    case OP_COMP_NC: 
      if (d->op_actn_code == OPCODE_EEPROM_PARAM_SAVE) //приняли команду сохранения параметров
      {
       sop_set_operation(SOP_SAVE_PARAMETERS);     
       d->op_actn_code = 0; //обработали 
      }
      if (d->op_actn_code == OPCODE_CE_SAVE_ERRORS) //приняли команду чтения сохраненных кодов ошибок  
      {
       sop_set_operation(SOP_READ_CE_ERRORS);     
       d->op_actn_code = 0; //обработали 
      }
      if (d->op_actn_code == OPCODE_READ_FW_SIG_INFO) //приняли команду чтения и передачи информации о прошивке
      {
       sop_set_operation(SOP_SEND_FW_SIG_INFO);
       d->op_actn_code = 0; //обработали        
      }
      break;    
      
    case CE_SAVED_ERR:
      sop_set_operation(SOP_SAVE_CE_ERRORS);
      break;       
  }
  
  //если были изменены параметры ДПКВ, то немедленно применяем их на работающем двигателе
  if (descriptor == CKPS_PAR)
  {
    ckps_set_edge_type(d->param.ckps_edge_type);
    ckps_set_cogs_btdc(d->param.ckps_cogs_btdc);
    ckps_set_ignition_cogs(d->param.ckps_ignit_cogs);
  }
  
  //аналогично для контороля детонации, обязательно после CKPS_PAR!
  if (descriptor == KNOCK_PAR)
  {
    //инициализируем процессор детонации в случае если он не использовался, а теперь поступила коменда его использовать.
    if (!d->use_knock_channel_prev && d->param.knock_use_knock_channel)
     if (!knock_module_initialize())
     {//чип сигнального процессора детонации неисправен - зажигаем СЕ
      ce_set_error(ECUERROR_KSP_CHIP_FAILED);   
     }    

    ckps_set_knock_window(d->param.knock_k_wnd_begin_angle, d->param.knock_k_wnd_end_angle);  
    knock_set_band_pass(edat.param.knock_bpf_frequency);
    ckps_use_knock_channel(d->param.knock_use_knock_channel);   
    
    //запоминаем состояние флага для того чтобы потом можно было опрежелить нужно инициализировать
    //процессор детонации или нет.   
    d->use_knock_channel_prev = d->param.knock_use_knock_channel;  
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
   s_timer_set(send_packet_interval_counter, d->param.uart_period_t_ms);
   
   //после передачи очищаем кеш ошибок
   d->ecuerrors_for_transfer = 0;
  }
 }
}

//Запись данных в EEPROM - процесс очень медленный. Он будет проходить параллельно с выполнением программы.
//Сохранение данных в EEPROM произойдет только если за заданное время не произошло ни одной операции приема параметров
//из UART-a и сохраненные параметры отличаются от текущих.        
void save_param_if_need(ecudata* d)
{   
 //параметры не изменились за заданное время?
 if (s_timer16_is_action(save_param_timeout_counter)) 
 {
  //текущие и сохраненные параметры отличаются?
  if (memcmp(eeprom_parameters_cache,&d->param,sizeof(params)-PAR_CRC_SIZE))   
   sop_set_operation(SOP_SAVE_PARAMETERS);       
  s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
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
   
   if (crc16((uint8_t*)&d->param,(sizeof(params)-PAR_CRC_SIZE))!=d->param.crc)
   {
     memcpy_P(&d->param,&def_param,sizeof(params));
     ce_set_error(ECUERROR_EEPROM_PARAM_BROKEN);
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
   
void init_io_ports(void)
{
  //конфигурируем порты ввода/вывода
  PORTA  = 0;   
  DDRA   = 0;       
  PORTB  = (1<<PB2)|(1<<PB4)|(1<<PB3)|(1<<PB0);             //CE горит(для проверки), клапан ЭПХХ включен, интерфейс с HIP выключен (CS=1, TEST=1)
  DDRB   = (1<<DDB4)|(1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);   
  PORTC  = (1<<PC3)|(1<<PC2)/*|(1<<PC7)*/; //ЭМР выключен
  DDRC   = (1<<DDC7);  //выход для управления клапаном ЭМР
  PORTD  = (1<<PD6)|(1<<PD3)|(1<<PD7);                      //стартер заблокирован, режим интегрирования для HIP
  DDRD   = (1<<DDD7)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3)|(1<<DDD1); //вых. PD1 пока UART не проинициализировал TxD 
}

void advance_angle_state_machine(int16_t* padvance_angle_inhibitor_state, ecudata* d)
{
 switch(d->engine_mode)
 {
  case EM_START: //режим пуска
   if (d->sens.inst_frq > d->param.smap_abandon)
   {                   
    d->engine_mode = EM_IDLE;    
    idling_regulator_init();    
   }      
   d->curr_angle=start_function(d);               //базовый УОЗ - функция для пуска
   d->airflow = 0;                                //в режиме пуска нет расхода
   *padvance_angle_inhibitor_state = d->curr_angle;//в режиме пуска фильтр отключен
   break;     
              
  case EM_IDLE: //режим холостого хода
   if (d->sens.carb)//педаль газа нажали - в рабочий режим
   {
    d->engine_mode = EM_WORK;
   }             
   work_function(d, 1);                           //обновляем значение расхода воздуха 
   d->curr_angle = idling_function(d);            //базовый УОЗ - функция для ХХ 
   d->curr_angle+=coolant_function(d);            //добавляем к УОЗ температурную коррекцию
   d->curr_angle+=idling_pregulator(d,&idle_period_time_counter);//добавляем регулировку
   break;            
                                             
  case EM_WORK: //рабочий режим 
   if (!d->sens.carb)//педаль газа отпустили - в переходной режим ХХ
   {
    d->engine_mode = EM_IDLE;
    idling_regulator_init();    
   }
   d->curr_angle=work_function(d, 0);           //базовый УОЗ - функция рабочего режима
   d->curr_angle+=coolant_function(d);          //добавляем к УОЗ температурную коррекцию
   break;     
       
  default:  //непонятная ситуация - угол в ноль       
   d->curr_angle = 0;
   break;     
 }
}
      
void init_ecu_data(ecudata* d)
{
 edat.op_comp_code = 0;
 edat.op_actn_code = 0;
 edat.sens.inst_frq = 0;
 edat.curr_angle = 0;
 edat.knock_retard = 0;
 edat.ecuerrors_for_transfer = 0;
 edat.eeprom_parameters_cache = &eeprom_parameters_cache[0];
 edat.engine_mode = EM_START;   
}      
            
__C_task void main(void)
{
  uint8_t turnout_low_priority_errors_counter = 255;
  int16_t advance_angle_inhibitor_state = 0;
  retard_state_t retard_state;     
  
  //подготовка структуры данных переменных состояния системы
  init_ecu_data(&edat);
  knklogic_init(&retard_state);
    
  init_io_ports();
  
  //если код программы испорчен - зажигаем СЕ
  if (crc16f(0,CODE_SIZE)!=code_crc)
   ce_set_error(ECUERROR_PROGRAM_CODE_BROKEN); 

  adc_init();

  //запрещаем компаратор - он нам не нужен  
  disable_comparator();             

  //проводим несколько циклов измерения датчиков для инициализации данных
  meas_initial_measure(&edat);   
   
  //снимаем блокировку стартера
  SET_STARTER_BLOCKING_STATE(0); 
     
  //читаем параметры
  load_eeprom_params(&edat);
   
  s_timer_init();
  
  //инициализируем UART
  uart_init(edat.param.uart_divisor);
  
  //инициализируем модуль ДПКВ             
  ckps_init_state();  
  ckps_set_edge_type(edat.param.ckps_edge_type);
  ckps_set_cogs_btdc(edat.param.ckps_cogs_btdc);
  ckps_set_ignition_cogs(edat.param.ckps_ignit_cogs);
  ckps_set_knock_window(edat.param.knock_k_wnd_begin_angle,edat.param.knock_k_wnd_end_angle);  
  ckps_use_knock_channel(edat.param.knock_use_knock_channel);
    
  //разрешаем глобально прерывания            
  __enable_interrupt();    

  //предварительная инициализация параметров сигнального процессора детонации
  knock_set_band_pass(edat.param.knock_bpf_frequency);
  knock_set_gain(fwdata.attenuator_table[0]);
  knock_set_int_time_constant(edat.param.knock_int_time_const);

  if (edat.param.knock_use_knock_channel)
   if (!knock_module_initialize())
   {//чип сигнального процессора детонации неисправен - зажигаем СЕ
    ce_set_error(ECUERROR_KSP_CHIP_FAILED);   
   }
  edat.use_knock_channel_prev = edat.param.knock_use_knock_channel;  

  sop_init_operations();
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
     edat.engine_mode = EM_START; //режим пуска 	      
     SET_STARTER_BLOCKING_STATE(0); //снимаем блокировку стартера
     
     if (edat.param.knock_use_knock_channel)
      knock_start_settings_latching();     
    }
      
    //запускаем измерения АЦП, через равные промежутки времени. При обнаружении каждого рабочего
    //цикла этот таймер переинициализируется. Таким образом, когда частота вращения двигателя превысит
    //определенную величину, это условие выполнятся перестанет.
    if (s_timer_is_action(force_measure_timeout_counter))
    {
     if (!edat.param.knock_use_knock_channel)
     {
      __disable_interrupt();
       adc_begin_measure();            
      __enable_interrupt();     
     }
     else
     {     
      //если сейчас происходит загрузка настроек в HIP, то нужно дождаться ее завершения.
      while(!knock_is_latching_idle());
      __disable_interrupt();
      //включаем режим интегрирования и ждем около 20мкс, пока интегратор начнет интегрировать (напряжение
      //на его выходе упадет до минимума). В данном случае нет ничего страшного в том, что мы держим прерывания
      //запрещенными 20-25мкс, так как это проискодит на очень маленьких оборотах.  
      knock_set_integration_mode(KNOCK_INTMODE_INT);
      __delay_cycles(350);     
      knock_set_integration_mode(KNOCK_INTMODE_HOLD);    
      adc_begin_measure_all(); //измеряем сигнал с ДД тоже            
      __enable_interrupt();     
     }
          
     s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
     meas_update_values_buffers(&edat);          
    }      
  
   //----------непрерывное выполнение-----------------------------------------
    //выполнение отложенных операций
    sop_execute_operations(&edat);
    //управление фиксированием и индицированием возникающих ошибок
    ce_check_engine(&edat, &ce_control_time_counter);
    //обработка приходящих/уходящих данных последовательного порта
    process_uart_interface(&edat);  
    //управление сохранением настроек
    save_param_if_need(&edat);    
    //расчет мгновенной частоты вращения коленвала
    edat.sens.inst_frq = ckps_calculate_instant_freq();                           
    //усреднение физических величин хранящихся в кольцевых буферах
    meas_average_measured_values(&edat);        
    //cчитываем дискретные входы системы и переключаем тип топлива
    meas_take_discrete_inputs(&edat);
    //управление периферией
    control_engine_units(&edat);      
    //КА состояний системы (диспетчер режимов - сердце основного цикла)
    advance_angle_state_machine(&advance_angle_inhibitor_state,&edat);
    //добавляем к УОЗ октан-коррекцию
    edat.curr_angle+=edat.param.angle_corr;       
    
    //---------------------------------------------- 
    //отнимаем поправку регулятора по детонации     
    edat.curr_angle-=edat.knock_retard;     
    //---------------------------------------------- 
    
    //ограничиваем получившийся УОЗ установленными пределами
    restrict_value_to(&edat.curr_angle, edat.param.min_angle, edat.param.max_angle);  
    //------------------------------------------------------------------------
    
    //выполняем операции которые необходимо выполнять строго для каждого рабочего цикла.      
    if (ckps_is_cycle_cutover_r())
    {
     meas_update_values_buffers(&edat);       
     s_timer_set(force_measure_timeout_counter, FORCE_MEASURE_TIMEOUT_VALUE);
    
     //Ограничиваем быстрые изменения УОЗ, он не может изменится больше чем на определенную величину
     //за один рабочий цикл. 
     edat.curr_angle = advance_angle_inhibitor(edat.curr_angle, &advance_angle_inhibitor_state, edat.param.angle_inc_spead, edat.param.angle_dec_spead);         
         
     //---------------------------------------------- 
     if (edat.param.knock_use_knock_channel)
     {
      knklogic_detect(&edat, &retard_state);
      knklogic_retard(&edat, &retard_state);
     }
     else     
      edat.knock_retard = 0;     
     //----------------------------------------------  
     
     //сохраняем УОЗ для реализации в ближайшем по времени цикле зажигания       
     ckps_set_dwell_angle(edat.curr_angle);        
    
     //управляем усилением аттенюатора в зависимости от оборотов
     if (edat.param.knock_use_knock_channel)
      knock_set_gain(knock_attenuator_function(&edat));
    
     // индицирование этих ошибок прекращаем при начале вращения двигателя 
     //(при прошествии N-го количества циклов)
     if (turnout_low_priority_errors_counter == 1)
     {    
      ce_clear_error(ECUERROR_EEPROM_PARAM_BROKEN);  
      ce_clear_error(ECUERROR_PROGRAM_CODE_BROKEN);        
     }
     if (turnout_low_priority_errors_counter > 0)
      turnout_low_priority_errors_counter--;      
    }   
     
   }//main loop
  //------------------------------------------------------------------------     
}
