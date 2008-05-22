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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "funconv.h"
#include "main.h"
#include "uart.h"
#include "tables.h"
#include "boot.h"
#include "ufcodes.h"
#include "crc_lib.h"


#define BEGIN_MEASURE() { f1.sens_ready=0; ADMUX = ADCI_MAP|ADC_VREF_TYPE; SETBIT(ADCSRA,ADSC);} //запускает измерение значений с датчиков
#define RESET_TCNT0()     TCNT0 = TCNT0_H = 0                           //сброс счетчика
#define GET_TCNT0()      (((TCNT0_H<<8)&0xFF00)|TCNT0)                  //получение значения счетчика
#define SET_TCNT0(v)     {TCNT0_H = v >> 8,TCNT0 = 255-(v & 0xFF);}     //инициализация таймера указанным значением
#define EE_START_WR_BYTE()  {EECR|= (1<<EEMWE);  EECR|= (1<<EEWE);}     //инициирует процесс записи байта в EEPROM
#define EE_START_WR_DATA(e_a,r_a,cnt)  {eewd.eews=0,eewd.ee_addr=e_a,eewd.sram_addr=r_a,eewd.count=cnt;EECR|=0x08;}//запускает процесс записи в EEPROM указанного блока данных 

   
//--глобальные переменные     
unsigned int  time_nt;                                                  //хранит последнее измерение времени прохождения n зубьев  
signed   int  goal_angle;                                               //требуемый УОЗ * ANGLE_MULTIPLAYER
unsigned char ignmask;                                                  //текущая маска для запуска текущего канала зажигания
unsigned char ign_teeth;                                                //для отсчета длительности импульса запуска зажигания по зубьям
unsigned char stop_counter=0;
unsigned char pars_counter=0;

unsigned int map_abuf[MAP_AVERAGING];                                   //буфер усреднения абсолютного давления
unsigned int bat_abuf[BAT_AVERAGING];                                   //буфер усреднения напряжения бортовой сети
unsigned int tmp_abuf[TMP_AVERAGING];                                   //буфер усреднения температуры охлаждающей жидкости
unsigned int frq_abuf[FRQ_AVERAGING];                                   //буфер усреднения частоты вращения коленвала

unsigned char eeprom_buf[64];

//Описывает информацию необходимую для сохранения данных в EEPROM
typedef struct 
{
  unsigned int ee_addr;                                                  //адрес для записи в EEPROM
  unsigned char* sram_addr;                                              //адрес данных в ОЗУ 
  unsigned char count;                                                   //количество байтов
  unsigned char eews;                                                    //состояние процесса записи
}ee_wrdesc;

ee_wrdesc eewd;


//Обработчик прерывания от EEPROM
//при завершении работы автомата всегда заносим в регистр адреса адрес нулевой ячейки
#pragma vector=EE_RDY_vect
__interrupt void ee_ready_isr(void)
{ 
  switch(eewd.eews)
  {
    case 0:
      EEAR = eewd.ee_addr;       
      EEDR = *eewd.sram_addr;
      EE_START_WR_BYTE();                          
      eewd.sram_addr++;
      eewd.ee_addr++;
      if (--eewd.count==0)
       eewd.eews = 1;   //последний байт запущен на запись.
      else      
       eewd.eews = 0;   
      break;    
    case 1:
      EEAR=0x000;      
      CLEARBIT(EECR,EERIE); //запрещаем прерывание от EEPROM        
      break;      
  }//switch  
}



//прерывание по переполнению Т/C 0. Т/С 0 дополняем до 16-ти разрядов и используем для измерения
//временных интервалов между зубьями (режим счетчика), а также для отсчета остаточного времени при реализации УОЗ(режим таймера)
#pragma vector=TIMER0_OVF_vect
__interrupt void timer0_ovf_isr(void)
{
  if (f1.t0mode==0)  
  {//режим 16-битного счетчика
   if (TCNT0_H < 255) TCNT0_H++;
  }
  else
  {//режим 16-битного таймера        
      if (TCNT0_H!=0)  //старший байт не исчерпан ?
      {
        TCNT0 = 0;
        TCNT0_H--;         
      }  
      else  
      {//запуск зажигания на очередную пару цилиндров    
       f1.t0mode=0;
       PORTD|=ignmask;  
       ign_teeth=0;
      }
  }
}


//прерывание по захвату таймера 1 (вызывается при прохождении очередного зуба)
//измерение частоты вращения коленвала начинается после прохождения синхрометки(для 1-4) и
//после 30-го зуба (для 2-3) и происходит независимо от реализации УОЗ. 
#pragma vector=TIMER1_CAPT_vect
__interrupt void timer1_capt_isr(void)
{  
  unsigned int t0; int diff;
  static unsigned char teeth=0;                                        //номер текущего зуба (1 оборот коленвала)
  static signed int  curr_angle;                                       //текущий УОЗ * ANGLE_MULTIPLAYER
  static unsigned int pptm=0x7FFF;                                     //хранит значение предыдущего замера времени между зубьями
  static unsigned char measure_state=0;                                //текущее состояние конечного автомата (КА) 
          
  //считываем измеренное время между зубьями и сбрасываем счетчик если мы находимся в режиме счетчика  
  t0 = GET_TCNT0();    
  if (f1.t0mode==0) {  RESET_TCNT0();  }
                      
  //конечный автомат для синхронизации, измерения скорости вращения коленвала, запуска зажигания в нужное время  
  switch(measure_state)
  {
   case 0:              //состояние синхронизации (поиск синхрометки)
       TCNT1 = 0;
       if (pptm < t0)
       {                //синхрометка найдена
        teeth = 0;                                                      //начинаем отсчет зубьев
        curr_angle = ANGLE_MULTIPLAYER * DEGREES_PER_TEETH * (TEETH_BEFORE_UP-1);
        f1.released=0;                                                  //в начале ноаого цикла зажигания необходимо сбросить флаг реализации зажигания
        ignmask=IGNITION_PULSE_14;                                      //будем запускать зажигание на 1-4 цилиндрах
        ign_teeth+=2;                                                   //учитываем два пропущеных зуба        
        f1.t1ovf=0;                                                     //сбрасываем флаг переполнения перед новым замером
        f1.rotsync = 1;                                                 //устанавливаем событие цикловой синхронизации 
        BEGIN_MEASURE();                                                //запуск процесса измерения значений аналоговых входов
        measure_state = 1;
       }    
      goto inxt;
   case 1:               //измерение скорости 1-4 (измеряем время прохождения зубьев)
      if (teeth==SPEED_MEASURE_TEETH)
      {                      
        time_nt=(f1.t1ovf)?0xFFFF:ICR1;                                 //если было переполнение то устанавливаем максимально возможное время      
        measure_state = 2;   
      } 
      goto crel;
   case 2:              //ожидаем зуба с которого начнем измерять скорость 2-3 
      if (teeth==30)
      {
       TCNT1 = 0;
       f1.t0mode=0;                                                     //опять будем измерять время между зубьями    
       curr_angle = ANGLE_MULTIPLAYER * DEGREES_PER_TEETH * (TEETH_BEFORE_UP-1);
       f1.released=0;
       ignmask=IGNITION_PULSE_23;                     
       f1.t1ovf=0;
       f1.rotsync = 1;
       BEGIN_MEASURE();                                                //запуск процесса измерения значений аналоговых входов       
       measure_state = 3;   
      }
      goto crel;
   case 3:              //измерение скорости 2-3 (измеряем время прохождения зубьев)
      if (teeth==(30+SPEED_MEASURE_TEETH))
      {
        time_nt=(f1.t1ovf)?0xFFFF:ICR1;                                 //если было переполнение то устанавливаем максимально возможное время                         
        measure_state = 4;                                              //на ожидание момента перехода в начальное состояние (поиск синхрометки)
      }           
      goto crel;
   case 4:              //ожидаем зуба с которого КА перейдет в состояние поиска синхрометки   
      if (teeth!=TEETH_BACK_SYNC)
          goto crel;
      f1.t0mode=0;    
      measure_state=0;                                                  //опять переходим в состояние синхронизации        
      goto inxt;          
  }
crel:                   //нужно быть готовым отсчитывать УОЗ
   if (!f1.released)    //только один раз
   {
     diff = curr_angle-goal_angle;
     if (diff <= (ANGLE_MULTIPLAYER * DEGREES_PER_TEETH))
     { //осталось отсчитать меньше одного зуба. Необходимо отсчитать время соответствующее оставшемуся углу
      f1.t0mode = 1;
      f1.released=1;
      t0 = ((long)diff * t0)/(ANGLE_MULTIPLAYER * DEGREES_PER_TEETH);      
      SET_TCNT0(t0);                 
    }
   } 
inxt:                   //проверка длительности импульса запуска зажигания и обновление счетчиков зубьев и текущего угла
  if (ign_teeth >= (IGNITION_TEETH-1))
     PORTD&=IGNITION_PULSE_OFF;

   teeth++;  ign_teeth++;                                          
   curr_angle=curr_angle-(ANGLE_MULTIPLAYER * DEGREES_PER_TEETH);        //прошел зуб - угол до в.м.т. уменьшился на 6 град.        
   pptm = t0*2;                                                          //двухкратный барьер для селекции синхрометки 
}


//прерывание по переполнению Т/С 1. Используется в случаях очень небольшой частоты вращения коленвала
//для фиксирования факта переполнения. Переполнение необходимо учитывать при измерении частоты вращения,
//иначе при медленной прокрутке двигателя возможно неправильное измерение, и проскакивание на дисплее БК
//больших частот.
#pragma vector=TIMER1_OVF_vect
__interrupt void timer1_ovf_isr(void)
{ 
 f1.t1ovf=1;
}
 
  
//прерывание по завершению преобразования АЦП. Измерение значений всех аналоговых датчиков. После запуска
//измерения это прерывание будет вызыватся для каждого входа, до тех пор пока все входы не будут обработаны.
#pragma vector=ADC_vect
__interrupt void ADC_isr(void)
{
 static unsigned char  map_ai=MAP_AVERAGING-1;
 static unsigned char  bat_ai=BAT_AVERAGING-1;
 static unsigned char  tmp_ai=TMP_AVERAGING-1;;      
 __enable_interrupt(); 

 switch(ADMUX&0x07)
 {
   case ADCI_MAP: //закончено измерение абсолютного давления
      map_abuf[map_ai] = ADC;      
      //обновляем значение индекса буфера усреднения
      (map_ai==0) ? (map_ai = MAP_AVERAGING - 1): map_ai--;            
      ADMUX = ADCI_UBAT|ADC_VREF_TYPE;   
      SETBIT(ADCSRA,ADSC);
      break;
   case ADCI_UBAT://закончено измерение напряжения бортовой сети
      bat_abuf[bat_ai] = ADC;      
      //обновляем значение индекса буфера усреднения
      (bat_ai==0) ? (bat_ai = BAT_AVERAGING - 1): bat_ai--;            
      ADMUX = ADCI_TEMP|ADC_VREF_TYPE;   
      SETBIT(ADCSRA,ADSC);
      break;
   case ADCI_TEMP://закончено измерение температуры охлаждающей жидкости
      tmp_abuf[tmp_ai] = ADC;      
      //обновляем  значение индекса буфера усреднения
      (tmp_ai==0) ? (tmp_ai = TMP_AVERAGING - 1): tmp_ai--;               
      ADMUX = ADCI_MAP|ADC_VREF_TYPE;    
      f1.sens_ready = 1;                
      break; 
 } 
}


//прерывание по переполению Т/С 2 - для отсчета временных интервалов в системе. Вызывается каждые 10мс
#pragma vector=TIMER2_OVF_vect
__interrupt void timer2_ovf_isr(void)
{ 
 TCNT2=T2_RELOAD_VALUE; 
 if (stop_counter > 12)
  {//периодически устанавливаем признак цикловой синхронизации и запускаем измерение
    f1.rotsync=1;
    BEGIN_MEASURE();
    stop_counter=0;
  }
   else
    stop_counter++;
    
  __enable_interrupt();       
  if (pars_counter < PAR_SAVE_COUNTER)
    pars_counter++; 
}
  
//высчитывание частоты вращения коленвала по измеренному времени прохождения SPEED_MEASURE_TEETH зубьев шкива.
//time_nt - время в дискретах таймера (одна дискрета = 4мкс), в одной минуте 60 сек, диск содержит 60 зубьев,
//в одной секунде 1000000 мкс, значит:
//   N(min-1) = 60/((time*(60/SPEED_MEASURE_TEETH)*4)/1000000)
void rotation_frq(ecudata* d)
{
  unsigned int time;
  __disable_interrupt();
   time=time_nt;           //обеспечиваем атомарный доступ к переменной
  __enable_interrupt();      
  d->sens.inst_frq=((15000000L*SPEED_MEASURE_TEETH)/60)/(time);
}

//обновление буфера усреднения для частоты вращения
void fillfrq(ecudata* d)
{
  static unsigned char frq_ai=FRQ_AVERAGING-1;
  //обновляем содержимое буфера усреднения  и значение его индекса
  frq_abuf[frq_ai] = d->sens.inst_frq;      
  (frq_ai==0) ? (frq_ai = FRQ_AVERAGING - 1): frq_ai--;     
}

//управление отдельными узлами двигателя и обновление данных о состоянии 
//концевика карбюратора, газового клапана, клапана ЭПХХ
void units_control(ecudata *d)
{
  //реализация функции ЭПХХ. Если заслонка карбюратора закрыта и frq > [верх.порог] или
  //заслонка карбюратора закрыта и frq > [ниж.порог] но клапан уже закрыт, то производится
  //выключение подачи топлива путем прекращения подачи напряжения на обмотку эл.клапана. Иначе - подача топлива.
  //--инверсия концевика карбюратора если необходимо, включение/выключение клапана ЭПХХ
  d->sens.carb=d->param.carb_invers^PINC_Bit5; //результат: 0 - дроссель закрыт, 1 - открыт
  d->ephh_valve=PORTB_Bit0 = ((!d->sens.carb)&&(((d->sens.frequen > d->param.ephh_lot)&&(!d->ephh_valve))||
                             (d->sens.frequen > d->param.ephh_hit)))?0:1;
 
  //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов)
  if (d->sens.inst_frq > d->param.starter_off)
    SETBIT(PORTD,PD7);
 
  if (d->param.tmp_use)
  {
    //управление электро вентилятором охлаждения двигателя
    if (d->sens.temperat >= d->param.vent_on)
       PORTB_Bit1 = 1;
    if (d->sens.temperat <= d->param.vent_off)   
       PORTB_Bit1 = 0; 
  }  
  
  //считываем и сохраняем состояние газового клапана
  d->sens.gas = PINC_Bit6;    
}


//усреднение измеряемых величин используя текущие значения буферов усреднения
void average_values(ecudata* d)
{     
      unsigned char i;unsigned long sum;unsigned long s;       
      ADCSRA&=0xE7;                                //запрещаем прерывание от АЦП
            
      for (sum=0,i = 0; i < MAP_AVERAGING; i++)
       sum+=map_abuf[i];      
      d->sens.map=(sum/MAP_AVERAGING)*2; 
          
      for (sum=0,i = 0; i < BAT_AVERAGING; i++)   //усредняем напряжение бортовой сети
       sum+=bat_abuf[i];      
      d->sens.voltage=(sum/BAT_AVERAGING)*6; 
       
      if (d->param.tmp_use) 
      {       
       for (sum=0,i = 0; i < TMP_AVERAGING; i++) //усредняем температуру (ДТОЖ)
        sum+=tmp_abuf[i];      
       d->sens.temperat=((sum/TMP_AVERAGING)*5)/3; 
      }  
      else             //ДТОЖ не используется
       d->sens.temperat=0;
    
      ADCSRA=(ADCSRA&0xEF)|(1<<ADIE);            //разрешаем прерывание от АЦП не сбрасывая флаг прерывания
           
      for (s=0,i = 0; i < FRQ_AVERAGING; i++)    //усредняем частоту вращения коленвала
       s+=frq_abuf[i];      
      d->sens.frequen=(s/FRQ_AVERAGING);           
}


//перекачивает данные между системным доменом и доменом UART-a
void swap_domains(ecudata* edat)
{ 
 static unsigned char uscount=0;
 static unsigned char index=0;
 unsigned char i;

 if (f1.rcv_busy)//приняли новый фрейм ?
 { 
  switch(rcv_mode)  //интерпретируем данные принятого фрейма в зависимости от дескриптора
  {
    case CHANGEMODE:       
        if (f1.snd_busy)    //передатчик занят. необходимо подождать его освобождения и только потом менять дескриптор потока 
            goto remitted;  //применение команды откладывается
        snd_mode = ((ud_n*)rcv_data)->snd_mode;
        goto completed;     
    case BOOTLOADER:
        if (f1.snd_busy)    //передатчик занят. необходимо подождать его освобождения и только потом запускать бутлоадер
            goto remitted;  //применение команды откладывается
        __disable_interrupt(); //если в бутлоадере есть команда "cli", то эту строчку можно убрать
        BOOT_JMP();         //прыгаем на бутлоадер минуя проверку перемычки            
        goto completed;     
    case TEMPER_PAR:            
       edat->param.tmp_use   = ((ud_t*)rcv_data)->tmp_use;
       edat->param.vent_on   = ((ud_t*)rcv_data)->vent_on;
       edat->param.vent_off  = ((ud_t*)rcv_data)->vent_off; 
       goto param_changed;     
    case CARBUR_PAR:   
       edat->param.ephh_lot  = ((ud_c*)rcv_data)->ephh_lot;
       edat->param.ephh_hit  = ((ud_c*)rcv_data)->ephh_hit;
       edat->param.carb_invers=((ud_c*)rcv_data)->carb_invers;
       goto param_changed;     
    case IDLREG_PAR:   
       edat->param.idl_regul = ((ud_r*)rcv_data)->idl_regul;
       edat->param.ifac1     = ((ud_r*)rcv_data)->ifac1;        
       edat->param.ifac2     = ((ud_r*)rcv_data)->ifac2;       
       edat->param.MINEFR    = ((ud_r*)rcv_data)->MINEFR;       
       edat->param.idl_turns = ((ud_r*)rcv_data)->idl_turns;    
       goto param_changed;     
    case ANGLES_PAR:   
       edat->param.max_angle = ((ud_a*)rcv_data)->max_angle;    
       edat->param.min_angle = ((ud_a*)rcv_data)->min_angle;    
       edat->param.angle_corr= ((ud_a*)rcv_data)->angle_corr;   
       goto param_changed;     
    case FUNSET_PAR:   
       if (((ud_m*)rcv_data)->fn_benzin < TABLES_NUMBER)
          edat->param.fn_benzin = ((ud_m*)rcv_data)->fn_benzin;    
       if (((ud_m*)rcv_data)->fn_gas < TABLES_NUMBER)    
          edat->param.fn_gas    = ((ud_m*)rcv_data)->fn_gas;              
       edat->param.map_grad  = ((ud_m*)rcv_data)->map_grad;     
       edat->param.press_swing=((ud_m*)rcv_data)->press_swing;  
       goto param_changed;     
    case STARTR_PAR:   
       edat->param.starter_off=((ud_p*)rcv_data)->starter_off;  
       edat->param.smap_abandon=((ud_p*)rcv_data)->smap_abandon;
       goto param_changed;     
  }//switch     
param_changed:                   //если были изменены параметры то сбрасываем счетчик времени
  pars_counter=0;
completed:    
  f1.rcv_busy=0;   //мы сохранили принятые данные - приемник ничем теперь не озабочен       
 }

 //периодически передаем фреймы с данными
 if (uscount >= SND_TIMECONST)
 {
  if (!f1.snd_busy)
  {                
  //передатчик ничем не озабочен - теперь можно перекачать данные 
  //в зависимости от текущего дескриптора посылаемых фреймов перекачиваем соответствующие данные
  switch(snd_mode)
  {
    case TEMPER_PAR:   
       ((ud_t*)snd_data)->tmp_use     = edat->param.tmp_use;
       ((ud_t*)snd_data)->vent_on     = edat->param.vent_on;
       ((ud_t*)snd_data)->vent_off    = edat->param.vent_off;
       break;
    case CARBUR_PAR:   
       ((ud_c*)snd_data)->ephh_lot    = edat->param.ephh_lot;
       ((ud_c*)snd_data)->ephh_hit    = edat->param.ephh_hit;
       ((ud_c*)snd_data)->carb_invers = edat->param.carb_invers;
       break;
    case IDLREG_PAR:   
       ((ud_r*)snd_data)->idl_regul   = edat->param.idl_regul;
       ((ud_r*)snd_data)->ifac1       = edat->param.ifac1;
       ((ud_r*)snd_data)->ifac2       = edat->param.ifac2;
       ((ud_r*)snd_data)->MINEFR      = edat->param.MINEFR;
       ((ud_r*)snd_data)->idl_turns   = edat->param.idl_turns;
       break;
    case ANGLES_PAR:   
       ((ud_a*)snd_data)->max_angle   = edat->param.max_angle;
       ((ud_a*)snd_data)->min_angle   = edat->param.min_angle;
       ((ud_a*)snd_data)->angle_corr  = edat->param.angle_corr;
       break;
   case FUNSET_PAR:   
       ((ud_m*)snd_data)->fn_benzin   = edat->param.fn_benzin;
       ((ud_m*)snd_data)->fn_gas      = edat->param.fn_gas;
       ((ud_m*)snd_data)->map_grad    = edat->param.map_grad;
       ((ud_m*)snd_data)->press_swing = edat->param.press_swing;
       break;
   case STARTR_PAR:   
       ((ud_p*)snd_data)->starter_off = edat->param.starter_off;
       ((ud_p*)snd_data)->smap_abandon = edat->param.smap_abandon;
       break;
    case FNNAME_DAT:
       for(i = 0; i < F_NAME_SIZE; i++ )
          ((ud_f*)snd_data)->name[i]=tables[index].name[i];
       ((ud_f*)snd_data)->tables_num = TABLES_NUMBER;
       ((ud_f*)snd_data)->index      = index++;       
       if (index>=TABLES_NUMBER) index=0;              
       break;
    case SENSOR_DAT:
       memcpy(&((ud_s*)snd_data)->sens,&edat->sens,sizeof(sensors));
       ((ud_s*)snd_data)->ephh_valve  = edat->ephh_valve;
       ((ud_s*)snd_data)->airflow     = edat->airflow;
       ((ud_s*)snd_data)->curr_angle  = edat->curr_angle;       
       break;
  }//switch
  UDR='@';                       //начинаем передачу новой посылки только если закончена передача предыдущей
  f1.snd_busy=1;                 //теперь передатчик озабочен передачей данных
  uscount=0;
  }
 }
remitted: 
  if (uscount < SND_TIMECONST) uscount++; 
}


//Предварительное измерение перед пуском двигателя
void InitialMeasure(ecudata* e)
{ 
  unsigned char i=16;
  __enable_interrupt();
  do
  {
      BEGIN_MEASURE();                                                     
      while(!f1.sens_ready); 
  }while(--i);  
  __disable_interrupt();
  average_values(e);  
  e->atmos_press = e->sens.map;      //сохраняем атмосферное давление
}

//копирует указанный блок данных из flash в SRAM
void memcpy_f(unsigned char* sram,unsigned char* fl,int size)
{
   int count;
   for(count = 0; count < size; count++)
    sram[count] = ((__flash unsigned char*)fl)[count];
}


//Запись данных в EEPROM - процесс очень медленный. Он будет проходить параллельно с выполнением программы,
//а для обеспечения атомарности копируем данные в отдельный буфер и из него их потом пишем в EEPROM.
//Сохранение данных в EEPROM произойдет только если за заданное время не произошло ни одной операции приема параметров
//из UART-a и сохраненные параметры отличаются от текущих.        
void save_param_if_need(ecudata* pd)
{
  if (pars_counter==PAR_SAVE_COUNTER) //параметры не изменились за заданное время
 {
  if (memcmp(eeprom_buf,&pd->param,sizeof(params)-PAR_CRC_SIZE)) //текущие и сохраненные параметры отличаются?
  {
    memcpy(eeprom_buf,&pd->param,sizeof(params));  
    ((params*)eeprom_buf)->crc=crc16(eeprom_buf,sizeof(params)-PAR_CRC_SIZE); //считаем контролбную сумму
    EE_START_WR_DATA(EEPROM_PARAM_START,eeprom_buf,sizeof(params));
  }
   pars_counter=0;
 }
}


//загружает параметры из EEPROM, проверяет целостность данных и если они испорчены то
//берет резервную копию из FLASH.
void load_eeprom_params(ecudata* d)
{
 unsigned char* e = (unsigned char*)&d->param;
 int count=sizeof(params);
 int adr=EEPROM_PARAM_START;
 
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
  memcpy_f((unsigned char*)&d->param,(unsigned char*)&def_param,sizeof(params));
 }    
} 
   
      
__C_task void main(void)
{
  unsigned char mode=0;
  ecudata edat; 
  
  //конфигурируем порты ввода/вывода
  PORTA  = 0;   
  DDRA   = 0;       
  PORTB  = (1<<PB4)|(1<<PB3)|(1<<PB0);                      //клапан ЭПХХ включен, интерфейс с HIP выключен (CS=1, TEST=1)
  DDRB   = (1<<DDB4)|(1<<DDB3)|(1<<DDB2)|(1<<DDB1)|(1<<DDB0);   
  PORTC  = (1<<PC3)|(1<<PC2);
  DDRC   = 0;  
  PORTD  = (1<<PD6)|(1<<PD3)|(1<<PD7);                      //стартер заблокирован, режим интегрирования для HIP
  DDRD   = (1<<DDD7)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3);

  
  if (crc16f(0,CODE_SIZE)!=code_crc)
     SETBIT(PORTB,PB2);                                    //код программы испорчен - зажигаем СЕ

  //инициализация АЦП, параметры: f = 125.000 kHz, внутренний источник опорного напряжения - 2.56V, прерывание разрешено 
  ADMUX=ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);     

  //запрещаем компаратор - он нам не нужен  
  ACSR=(1<ACD);             

  InitialMeasure(&edat);   //проводим несколько циклов измерения датчиков для инициализации данных

  PORTD_Bit7 = 0;     //снимаем блокировку стартера
  
  //конфигурируем таймеры T0 и T1
  TCCR0  = (1<<CS01)|(1<<CS00);                             //clock = 250kHz
  TCCR2  = (1<<CS22)|(1<<CS21)|(1<<CS20);                   //clock = 15.625kHz
  TCCR1B = (1<<ICNC1)|(1<<ICES1)|(1<<CS11)|(1<<CS10);       //подавление шума, передний фронт захвата, clock = 250kHz
  TIMSK  = (1<<TICIE1)|(1<<TOIE0)|(1<<TOIE1)|(1<<TOIE2);    //разрешаем прерывание по захвату и переполнению Т/C 1, переполнению T/C 0, переполнению T/C 2
    
  //инициализируем UART
   USART_Init(CBR_9600);

  //устанавливаем режим поиска синхрометки и инициализируем переменные                                       
  f1.t0mode=0;   
  time_nt=0xFFFF;           
  goal_angle=0;       
    
  //читаем параметры
  load_eeprom_params(&edat);
  
  //разрешаем глобально прерывания            
  __enable_interrupt();    
     
  while(1)
  {
    rotation_frq(&edat);                      
    if (f1.rotsync)
    {//пришло время выполнить синхронизированные операции 
      fillfrq(&edat);    
      f1.rotsync=0;
      stop_counter=0;
    }    
      
    average_values(&edat);        
    units_control(&edat);
    
    //в зависимости от текущего типа топлива выбираем соответствующий набор таблиц             
    if (edat.sens.gas)
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_gas];    //на газе
    else  
      edat.fn_dat = (__flash F_data*)&tables[edat.param.fn_benzin];//на бензине
    
    
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
    
    //сохраняем УОЗ для реализации в ближайшем по времени цикле зажигания        
    __disable_interrupt();
     goal_angle = edat.curr_angle;
    __enable_interrupt();                
  
   swap_domains(&edat);  //обмен данными между доменами  
   
   save_param_if_need(&edat);    //управление сохранением настроек                    
  }
}
