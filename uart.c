 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <ina90.h>
#include <iom16.h>
#include "uart.h"
#include "ufcodes.h"
#include "bitmask.h"

//Дополнительно этот модуль использует глобальные переменные-флажки:
//  f1.uart_send_busy и f1.uart_recv_busy


char snd_mode = SENSOR_DAT;
char rcv_mode = 0;

UART_recv_buf uart_recv_buf;
UART_send_buf uart_send_buf;


const __flash char hdig[] = "0123456789ABCDEF";


#define HTOD(h) ((h<0x3A)?h-'0':h-'A'+10)


void uart_send_packet(void)
{
  UDR = '@';                       
  f1.uart_send_busy = 1;
}

void uart_notify_processed(void)
{
  f1.uart_recv_busy = 0;
}

unsigned char uart_is_sender_busy(void)
{
  return f1.uart_send_busy;
}

unsigned char uart_is_packet_received(void)
{
  return f1.uart_recv_busy;
}

char uart_get_send_mode(void)
{
 return snd_mode;
}

char uart_get_recv_mode(void)
{
 return rcv_mode;
}

char uart_set_send_mode(char send_mode)
{
 return snd_mode = send_mode;
}

void uart_init(unsigned int baud)
{
  // Set baud rate 
  UBRRH = (unsigned char)(baud>>8);
  UBRRL = (unsigned char)baud;
  UCSRA = 0;                                                  //удвоение не используем 
  UCSRB=(1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN);            //приемник,передатчик и их прерывания разрешены
  UCSRC=(1<<URSEL)/*|(1<<USBS)*/|(1<<UCSZ1)|(1<<UCSZ0);       //8 бит, 1 стоп, нет контроля четности
  f1.uart_send_busy = 0;                                      //передатчик ни чем не озабочен
  f1.uart_recv_busy = 0;
}


unsigned char send_i8h(unsigned char i)
{
  static unsigned char state=0;
  static unsigned char ii;    
  switch(state)
  {
    case 0:
      ii=i;
      UDR = hdig[ii/16];    //передаем старший байт HEX числа
      state=1;
      return state;
    case 1:  
      UDR = hdig[ii%16];    //передаем младший байт HEX числа
      state=0;              //КА в исходное состояние
      return state;      
  }
  return 0;
}

unsigned char send_i16h(unsigned int i)
{
  static unsigned char state=0;
  static unsigned char il,ih;  
  switch(state)
  {
    case 0:
      il=GETBYTE(i,0);
      ih=GETBYTE(i,1);
      UDR = hdig[ih/16];    //передаем старший байт HEX числа (старший байт)
      state=1;
      return state;
    case 1:  
      UDR = hdig[ih%16];    //передаем младший байт HEX числа (старший байт)
      state=2;
      return state;      
    case 2:  
      UDR = hdig[il/16];    //передаем старший байт HEX числа (младший байт)
      state=3;
      return state;      
    case 3:  
      UDR = hdig[il%16];    //передаем младший байт HEX числа (младший байт)
      state=0;              //КА в исходное состояние
      return state;      
  }
  return 0;
}

unsigned char recv_i8h(unsigned char *s)
{
  static unsigned char state=0;
  static unsigned char ii;    
  unsigned char u=UDR;             //из UDR можно прочитать только один раз
  switch(state)
  {
    case 0:       
      ii=HTOD(u)<<4;    
      state++;
      return 0;
    case 1:  
      ii|=HTOD(u);          
      state=0;              //КА в исходное состояние
      (*s)++;
      return ii;      
  }
  return 0;
}

unsigned int recv_i16h(unsigned char *s)
{
  static unsigned char state=0;
  static unsigned int ii;    
  unsigned char u=UDR;
  switch(state)
  {
    case 0:      
      SETBYTE(ii,1) = (HTOD(u))<<4;          
      state++;
      return 0;
    case 1:  
      SETBYTE(ii,1)|=(HTOD(u));          
      state++;              
      return 0;      
    case 2:   
      SETBYTE(ii,0)=(HTOD(u))<<4;    
      state++;
      return 0;
    case 3:  
      SETBYTE(ii,0)|=(HTOD(u));          
      state=0;              //КА в исходное состояние
      (*s)++;
      return ii;      
  }
  return 0;
}



//Обработчик прерывания по передаче байтов через UART
#pragma vector=USART_TXC_vect
__interrupt void usart_tx_isr(void)
{       
static unsigned char state=0;
static unsigned char i;
 __enable_interrupt();
 switch(snd_mode)
 {     
   case TEMPER_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        UDR = uart_send_buf.tmp_use+0x30;        
          state++;
        break;  
      case 2:
        if (!send_i16h(uart_send_buf.vent_on))       
          state++;
        break;  
      case 3:
        if (!send_i16h(uart_send_buf.vent_off))       
          state++;
        break;  
      case 4: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 5: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break; 
     
  case CARBUR_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        if (!send_i16h(uart_send_buf.ephh_lot))       
          state++;
        break;  
      case 2:
        if (!send_i16h(uart_send_buf.ephh_hit))       
          state++;
        break;  
      case 3:
        UDR = uart_send_buf.carb_invers+0x30;        
          state++;
        break;  
      case 4: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 5: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break;      
     
  case IDLREG_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        UDR = uart_send_buf.idl_regul+0x30;         
        state++;
        break;  
      case 2:
        if (!send_i16h(uart_send_buf.ifac1))       
          state++;
        break;  
      case 3:
        if (!send_i16h(uart_send_buf.ifac2))       
          state++;
        break;  
      case 4:
        if (!send_i16h(uart_send_buf.MINEFR))       
          state++;
        break;  
      case 5:
        if (!send_i16h(uart_send_buf.idl_turns))       
          state++;
        break;  
      case 6: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 7: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break;      

    case ANGLES_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        if (!send_i16h(uart_send_buf.max_angle))       
          state++;
        break;  
      case 2:
        if (!send_i16h(uart_send_buf.min_angle))       
          state++;
        break;  
      case 3:
        if (!send_i16h(uart_send_buf.angle_corr))       
          state++;
        break;  
      case 4: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 5: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break;      

   case FUNSET_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:        
        if (!send_i8h(uart_send_buf.fn_benzin))       
          state++;
        break;  
      case 2:
        if (!send_i8h(uart_send_buf.fn_gas))       
          state++;
        break;  
      case 3:
        if (!send_i8h(uart_send_buf.map_grad))       
          state++;
        break;  
      case 4:
        if (!send_i16h(uart_send_buf.press_swing))       
          state++;
        break;  
      case 5: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 6: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break;      
     
     
  case STARTR_PAR:  
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        if (!send_i16h(uart_send_buf.starter_off))       
          state++;
        break;  
      case 2:
        if (!send_i16h(uart_send_buf.smap_abandon))       
          state++;
        break;  
      case 3: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 4: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }
     break;         
     
     
  case FNNAME_DAT:
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:
        if (!send_i8h(uart_send_buf.tables_num))       
          state++;
        break;  
      case 2:
        if (!send_i8h(uart_send_buf.index))       
          state++;
        i=0;  
        break;  
      case 3: //передаем символы имени семейства
          UDR=uart_send_buf.name[i++];
          if (i>=F_NAME_SIZE)
             state++;             
        break;          
      case 4: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 5: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;            
     }     
     break;     
     
     
  case SENSOR_DAT:
     switch(state)
     {
      case 0:
        UDR = snd_mode;   //передаем дескриптор посылки
        state++;
        break;
      case 1:  
        if (!send_i16h(uart_send_buf.sens.frequen))   //частота вращения коленвала
          state++;
        break;    
      case 2:  
        if (!send_i16h(uart_send_buf.sens.map))       //абсолютное давление во впускном коллекторе
          state++;
        break;    
      case 3:  
        if (!send_i16h(uart_send_buf.sens.voltage))   //напряжение в бортовой сети
          state++;
        break;    
      case 4:  
        if (!send_i16h(uart_send_buf.sens.temperat)) //температура охлаждающей жидкости
          state++;
        break;    
      case 5:  
        if (!send_i16h(uart_send_buf.curr_angle))   //текущий УОЗ
          state++;
        break;            
      case 6:  
        if (!send_i8h(uart_send_buf.airflow))       //два символа - расход воздуха
          state++;
        break;            
      case 7:  
        UDR = uart_send_buf.ephh_valve+0x30;       //один символ - состояние клапана ЭПХХ
        state++;
        break;            
      case 8:  
        UDR = uart_send_buf.sens.carb+0x30;        //один символ - состояние дроссельной заслонки
        state++;
        break;    
      case 9:  
        UDR = uart_send_buf.sens.gas+0x30;         //состояние газового клапана
        state++;
        break;                        
      case 10: 
        UDR = '\r';         //передаем символ - признак конца данных
        state++;
        break;
      case 11: 
        state=0;          //возвращаем КА в исходное состояние 
        f1.uart_send_busy=0;   //передатчик готов к передаче новых данных
        break;    
     }       
     break;      
 }
}


//Обраюотчик прерывания по приему байтов через UART
#pragma vector=USART_RXC_vect
__interrupt void usart_rx_isr()
{
  static unsigned char cstate=0,state=0;  

  switch(cstate)
  {
    case 0:            //принимаем (ожидаем символ начала посылки)
      if (UDR=='!')      
      {
         cstate++;               
      }
      break;
    case 1:           //принимаем дескриптор посылки
      rcv_mode=UDR;
      cstate++;
      state=0;      
      break;  
    case 2:           //прием данных посылки
      switch(rcv_mode)
      {
       case CHANGEMODE:
         switch(state)
         {
           case 0:     //приняли значение нового дескриптора
             uart_recv_buf.snd_mode = UDR;
             cstate=3;
             break;         
         }       
         break;   
      
      
       case BOOTLOADER:     //запуск бутлоадера
         switch(state)
         {
           case 0:     
             if (UDR=='l')             
               cstate=3;
             else
               cstate=0;    //ошибка                                        
             break;         
         }       
         break;   
                          
       case TEMPER_PAR:
         switch(state)
         {
           case 0:
             uart_recv_buf.tmp_use = UDR - 0x30;                
             state++;
             break;      
           case 1:  
             uart_recv_buf.vent_on=recv_i16h(&state);                
             break;     
           case 2:  
             uart_recv_buf.vent_off=recv_i16h(&cstate);  
             break;                                                                                                
         }       
         break;   
                  
                  
       case CARBUR_PAR:  
         switch(state)
         {
           case 0:  
             uart_recv_buf.ephh_lot=recv_i16h(&state);                
             break;     
           case 1:  
             uart_recv_buf.ephh_hit=recv_i16h(&state);  
             break;                                                                                                
           case 2:
             uart_recv_buf.carb_invers = UDR - 0x30;                
             cstate=3;              
             break;      
         }            
         break;               


       case IDLREG_PAR:  
         switch(state)
         {
           case 0:
             uart_recv_buf.idl_regul = UDR - 0x30;                
             state++;
             break;      
           case 1:  
             uart_recv_buf.ifac1=recv_i16h(&state);                
             break;     
           case 2:  
             uart_recv_buf.ifac2=recv_i16h(&state);  
             break;                                                                                                
           case 3:  
             uart_recv_buf.MINEFR=recv_i16h(&state);  
             break;                                                                                                
           case 4:  
             uart_recv_buf.idl_turns=recv_i16h(&cstate);  
             break;                                                                                                
         }         
         break;               


       case ANGLES_PAR: 
         switch(state)
         {
            case 0:  
             uart_recv_buf.max_angle=recv_i16h(&state);                
             break;     
           case 1:  
             uart_recv_buf.min_angle=recv_i16h(&state);  
             break;                                                                                                
           case 2:  
             uart_recv_buf.angle_corr=recv_i16h(&cstate);  
             break;                                                                                                
         }        
         break;               

         
       case FUNSET_PAR:  
         switch(state)
         {
           case 0:
             uart_recv_buf.fn_benzin=recv_i8h(&state);                             
             break;      
           case 1:  
             uart_recv_buf.fn_gas=recv_i8h(&state);                
             break;     
           case 2:  
             uart_recv_buf.map_grad=recv_i8h(&state);  
             break;                                                                                                
           case 3:  
             uart_recv_buf.press_swing=recv_i16h(&cstate);  
             break;                                                                                                
         }         
         break;               


       case STARTR_PAR:  
         switch(state)
         {
           case 0:  
             uart_recv_buf.starter_off=recv_i16h(&state);                
             break;     
           case 1:  
             uart_recv_buf.smap_abandon=recv_i16h(&cstate);  
             break;                                                                                                
         }               
         break;                        
         
      }   
      break;   
      
    case 3:  
       if (UDR=='\r')
       {             
         cstate=0;       //КА в исходное состояние      
         f1.uart_recv_busy=1;  //установили признак готовности данных
       }    
      break;                   
  }
    
}
