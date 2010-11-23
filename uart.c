/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   contacts:
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#include <ina90.h>
#include <ioavr.h>
#include <pgmspace.h>
#include "secu3.h"
#include "uart.h"
#include "ufcodes.h"
#include "bitmask.h"

//Mega64 compatibility
#ifdef __ATmega64__
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#define RXEN  RXEN0
#define TXEN  TXEN0
#define UDR   UDR0
#define UDRE  UDRE0
#define RXC   RXC0
#define RXCIE RXCIE0
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UDRIE UDRIE0
#define UCSZ0 UCSZ00
#define UCSZ1 UCSZ01
#define USART_UDRE_vect USART0_UDRE_vect
#define USART_RXC_vect USART0_RXC_vect
#endif

typedef struct
{
 uint8_t send_mode;
 uint8_t recv_buf[UART_RECV_BUFF_SIZE];
 uint8_t send_buf[UART_SEND_BUFF_SIZE];
 volatile uint8_t send_size;
 uint8_t send_index;
 volatile uint8_t recv_size;
 uint8_t recv_index;
}uartstate_t;

uartstate_t uart;

const __flash uint8_t hdig[] = "0123456789ABCDEF";


#define HTOD(h) (((h)<0x3A) ? ((h)-'0') : ((h)-'A'+10))

//--------вспомогательные функции для построения пакетов-------------
#define build_fb(src, size) \
{ \
 memcpy_P(&uart.send_buf[uart.send_size],(src),(size)); \
 uart.send_size+=(size); \
}

#define build_i4h(i) {uart.send_buf[uart.send_size++] = ((i)+0x30);}

void build_i8h(uint8_t i)
{
 uart.send_buf[uart.send_size++] = hdig[i/16];    //старший байт HEX числа
 uart.send_buf[uart.send_size++] = hdig[i%16];    //младший байт HEX числа
}

void build_i16h(uint16_t i)
{
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,1)/16];    //старший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,1)%16];    //младший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,0)/16];    //старший байт HEX числа (младший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,0)%16];    //младший байт HEX числа (младший байт)
}


void build_i32h(uint32_t i)
{
 build_i16h(i>>16);
 build_i16h(i);
}

//----------вспомагательные функции для распознавания пакетов---------
#define recept_i4h() (uart.recv_buf[uart.recv_index++] - 0x30)

uint8_t recept_i8h(void)
{
 uint8_t i8;    
 i8 = HTOD(uart.recv_buf[uart.recv_index])<<4;
 ++uart.recv_index;
 i8|= HTOD(uart.recv_buf[uart.recv_index]);
 ++uart.recv_index;
 return i8;
}

uint16_t recept_i16h(void)
{
 uint16_t i16;    
 SETBYTE(i16,1) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;          
 ++uart.recv_index;
 SETBYTE(i16,1)|= (HTOD(uart.recv_buf[uart.recv_index]));          
 ++uart.recv_index;
 SETBYTE(i16,0) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;    
 ++uart.recv_index;
 SETBYTE(i16,0)|= (HTOD(uart.recv_buf[uart.recv_index]));          
 ++uart.recv_index;
 return i16;
}


uint32_t recept_i32h(void)
{
 uint32_t i = 0;
 i = recept_i16h();
 i = i << 16;
 i|=recept_i16h();
 return i;
}

//--------------------------------------------------------------------


void uart_begin_send(void)
{
 uart.send_index = 0;
 UCSRB |= (1<<UDRIE); /* enable UDRE interrupt */ 
}

//строит пакет взависимости от текущего дескриптора и запускает его на передачу. Функция не проверяет
//занят передатчик или нет, это должно быть сделано до вызова функции
void uart_send_packet(struct ecudata_t* d, uint8_t send_mode)  
{
 static uint8_t index = 0;

 //служит индексом во время сборки пакетов, а после сборки будет содержать размер пакета
 uart.send_size = 0; 
 
 if (send_mode==0) //используем текущий дескриптор
   send_mode = uart.send_mode;
 
 //общая часть для всех пакетов
 uart.send_buf[uart.send_size++] = '@';
 uart.send_buf[uart.send_size++] = send_mode; 
   
  switch(send_mode)
  {
    case TEMPER_PAR:   
       build_i4h(d->param.tmp_use);
       build_i4h(d->param.vent_pwm);
       build_i16h(d->param.vent_on);
       build_i16h(d->param.vent_off);
       break;
    case CARBUR_PAR:   
       build_i16h(d->param.ephh_lot);
       build_i16h(d->param.ephh_hit);
       build_i4h(d->param.carb_invers);
       build_i16h(d->param.epm_on_threshold);
       build_i16h(d->param.ephh_lot_g);
       build_i16h(d->param.ephh_hit_g);
       build_i8h(d->param.shutoff_delay);
       break;
    case IDLREG_PAR:   
       build_i4h(d->param.idl_regul);
       build_i16h(d->param.ifac1);
       build_i16h(d->param.ifac2);
       build_i16h(d->param.MINEFR);
       build_i16h(d->param.idling_rpm);
       build_i16h(d->param.idlreg_min_angle);
       build_i16h(d->param.idlreg_max_angle);
       break;
    case ANGLES_PAR:   
       build_i16h(d->param.max_angle);
       build_i16h(d->param.min_angle);
       build_i16h(d->param.angle_corr);
       build_i16h(d->param.angle_dec_spead);
       build_i16h(d->param.angle_inc_spead);
       break;
   case FUNSET_PAR:   
       build_i8h(d->param.fn_benzin);
       build_i8h(d->param.fn_gas);
       build_i16h(d->param.map_lower_pressure);
       build_i16h(d->param.map_upper_pressure);
       build_i16h(d->param.map_curve_offset);
       build_i16h(d->param.map_curve_gradient);       
       break;
   case STARTR_PAR:   
       build_i16h(d->param.starter_off);
       build_i16h(d->param.smap_abandon);
       break;
    case FNNAME_DAT: 
       build_i8h(TABLES_NUMBER);
       build_i8h(index);     
       build_fb(tables[index].name,F_NAME_SIZE);  
       index++;
       if (index>=TABLES_NUMBER) index=0;              
       break;
    case SENSOR_DAT:
       build_i16h(d->sens.frequen);   
       build_i16h(d->sens.map);       
       build_i16h(d->sens.voltage);   
       build_i16h(d->sens.temperat);  
       build_i16h(d->curr_angle);     
       build_i16h(d->sens.knock_k);  // <-- knock value
       build_i16h(d->knock_retard);  // <-- knock retard       
       build_i8h(d->airflow);              
       //boolean values              
       build_i8h((d->ephh_valve  << 0) | 
                 (d->sens.carb   << 1) | 
                 (d->sens.gas    << 2) | 
                 (d->epm_valve   << 3) |
                 (d->ce_state    << 4));
       break;
   case ADCCOR_PAR:   
       build_i16h(d->param.map_adc_factor);
       build_i32h(d->param.map_adc_correction);
       build_i16h(d->param.ubat_adc_factor);
       build_i32h(d->param.ubat_adc_correction);
       build_i16h(d->param.temp_adc_factor);
       build_i32h(d->param.temp_adc_correction);
       break;
   case ADCRAW_DAT:
       build_i16h(d->sens.map_raw);       
       build_i16h(d->sens.voltage_raw);   
       build_i16h(d->sens.temperat_raw);  
       build_i16h(d->sens.knock_k);   //<-- knock signal level
       break;
   case CKPS_PAR:
       build_i4h(d->param.ckps_edge_type);       
       build_i8h(d->param.ckps_cogs_btdc);   
       build_i8h(d->param.ckps_ignit_cogs);
       build_i8h(d->param.ckps_engine_cyl);  
       break;
   case OP_COMP_NC:    
       build_i4h(d->op_comp_code);              
       break;   
   case CE_ERR_CODES:
       build_i16h(d->ecuerrors_for_transfer);
       break;     
   case KNOCK_PAR:    
       build_i4h(d->param.knock_use_knock_channel);   
       build_i8h(d->param.knock_bpf_frequency);  
       build_i16h(d->param.knock_k_wnd_begin_angle);
       build_i16h(d->param.knock_k_wnd_end_angle);
       build_i8h(d->param.knock_int_time_const);
              
       build_i16h(d->param.knock_retard_step);
       build_i16h(d->param.knock_advance_step);
       build_i16h(d->param.knock_max_retard);
       build_i16h(d->param.knock_threshold);
       build_i8h(d->param.knock_recovery_delay);      
       break;     
   case CE_SAVED_ERR:
       build_i16h(d->ecuerrors_saved_transfer);
       break;   
       
   case FWINFO_DAT:
       //проверка на то, чтобы мы не вылезли за пределы буфера. 3 символа - заголовок и конец пакета.
#if ((UART_SEND_BUFF_SIZE - 3) < FW_SIGNATURE_INFO_SIZE)
 #error "Out of buffer!"
#endif       
       build_fb(fwdata.fw_signature_info, FW_SIGNATURE_INFO_SIZE);
       break;     
       
   case MISCEL_PAR:
       build_i16h(d->param.uart_divisor);
       build_i8h(d->param.uart_period_t_ms); 
       break;                          
  }//switch

  //общая часть для всех пакетов
  uart.send_buf[uart.send_size++] = '\r';

  //буфер передатчика содержит полностью готовый пакет - начинаем передачу
  uart_begin_send();
}

//эта функция не проверяет, был или не был принят фрейм, проверка должна быть произведена до вызова функции.
//Возвращает дескриптор обработанного фрейма
uint8_t uart_recept_packet(struct ecudata_t* d)
{
 //буфер приемника содержит дескриптор пакета и данные
 uint8_t temp; 
 uint8_t descriptor; 
   
 uart.recv_index = 0;
   
 descriptor = uart.recv_buf[uart.recv_index++];

// TODO: сделать проверку uart_recv_size для каждого типа пакета.
// Проверять байты пакетов на принадлежность к шестнадцатерчным символам     

 //интерпретируем данные принятого фрейма в зависимости от дескриптора
  switch(descriptor)  
  {
    case CHANGEMODE:       
       uart_set_send_mode(uart.recv_buf[uart.recv_index++]);
       break;     

    case BOOTLOADER:       
       //передатчик занят. необходимо подождать его освобождения и только потом запускать бутлоадер
       while (uart_is_sender_busy());                             
       //если в бутлоадере есть команда "cli", то эту строчку можно убрать
       __disable_interrupt();              
       //прыгаем на бутлоадер минуя проверку перемычки
       boot_loader_start();                     
       break;

    case TEMPER_PAR:                   
       d->param.tmp_use   = recept_i4h();
       d->param.vent_pwm  = recept_i4h();
       d->param.vent_on   = recept_i16h();
       d->param.vent_off  = recept_i16h();
       break;

    case CARBUR_PAR:   
       d->param.ephh_lot  = recept_i16h();
       d->param.ephh_hit  = recept_i16h();
       d->param.carb_invers= recept_i4h();
       d->param.epm_on_threshold= recept_i16h();
       d->param.ephh_lot_g = recept_i16h();
       d->param.ephh_hit_g = recept_i16h();
       d->param.shutoff_delay = recept_i8h();
       break;

    case IDLREG_PAR:   
       d->param.idl_regul = recept_i4h();
       d->param.ifac1     = recept_i16h();        
       d->param.ifac2     = recept_i16h();       
       d->param.MINEFR    = recept_i16h();       
       d->param.idling_rpm = recept_i16h(); 
       d->param.idlreg_min_angle = recept_i16h();
       d->param.idlreg_max_angle = recept_i16h();   
       break;

    case ANGLES_PAR:   
       d->param.max_angle = recept_i16h();    
       d->param.min_angle = recept_i16h();    
       d->param.angle_corr= recept_i16h();   
       d->param.angle_dec_spead = recept_i16h();
       d->param.angle_inc_spead = recept_i16h();
       break;

    case FUNSET_PAR:   
       temp = recept_i8h();
       if (temp < TABLES_NUMBER)
          d->param.fn_benzin = temp;    

       temp = recept_i8h();
       if (temp < TABLES_NUMBER)    
          d->param.fn_gas = temp;
              
       d->param.map_lower_pressure = recept_i16h();     
       d->param.map_upper_pressure = recept_i16h();  
       d->param.map_curve_offset = recept_i16h();
       d->param.map_curve_gradient = recept_i16h();
       break;

    case STARTR_PAR:   
       d->param.starter_off = recept_i16h();  
       d->param.smap_abandon= recept_i16h();
       break;

    case ADCCOR_PAR:
       d->param.map_adc_factor     = recept_i16h();
       d->param.map_adc_correction = recept_i32h();
       d->param.ubat_adc_factor    = recept_i16h();
       d->param.ubat_adc_correction= recept_i32h();
       d->param.temp_adc_factor    = recept_i16h();
       d->param.temp_adc_correction= recept_i32h();     
       break;
       
    case CKPS_PAR:
       d->param.ckps_edge_type = recept_i4h();       
       d->param.ckps_cogs_btdc  = recept_i8h();  
       d->param.ckps_ignit_cogs = recept_i8h();
       d->param.ckps_engine_cyl = recept_i8h();  
       break;       
       
    case OP_COMP_NC: 
       d->op_actn_code = recept_i4h(); 
       break;   
       
    case KNOCK_PAR:  
       d->param.knock_use_knock_channel = recept_i4h();
       d->param.knock_bpf_frequency   = recept_i8h();
       d->param.knock_k_wnd_begin_angle = recept_i16h();
       d->param.knock_k_wnd_end_angle = recept_i16h(); 
       d->param.knock_int_time_const = recept_i8h();    
       
       d->param.knock_retard_step = recept_i16h();
       d->param.knock_advance_step = recept_i16h();
       d->param.knock_max_retard = recept_i16h();
       d->param.knock_threshold = recept_i16h();
       d->param.knock_recovery_delay = recept_i8h();             
       break;   
       
    case CE_SAVED_ERR:
       d->ecuerrors_saved_transfer = recept_i16h();
       break;   
       
    case MISCEL_PAR: 
       d->param.uart_divisor = recept_i16h();
       d->param.uart_period_t_ms = recept_i8h();  
       break;   
       
  }//switch     

 return descriptor;
}


void uart_notify_processed(void)
{
 uart.recv_size = 0;
}

uint8_t uart_is_sender_busy(void)
{
 return (uart.send_size > 0);
}

uint8_t uart_is_packet_received(void)
{
 return (uart.recv_size > 0);
}

uint8_t uart_get_send_mode(void)
{
 return uart.send_mode;
}

uint8_t uart_set_send_mode(uint8_t descriptor)
{
 return uart.send_mode = descriptor;
}

void uart_init(uint16_t baud)
{
 // Set baud rate 
 UBRRH = (uint8_t)(baud>>8);
 UBRRL = (uint8_t)baud;
 UCSRA = 0;                                                  //удвоение не используем 
 UCSRB=(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);                       //приемник,прерывание по приему и передатчик разрешены
#ifdef URSEL 
 UCSRC=(1<<URSEL)/*|(1<<USBS)*/|(1<<UCSZ1)|(1<<UCSZ0);       //8 бит, 1 стоп, нет контроля четности                                      
#else
 UCSRC=/*|(1<<USBS)*/(1<<UCSZ1)|(1<<UCSZ0);                  //8 бит, 1 стоп, нет контроля четности                                       
#endif

 uart.send_size = 0;                                         //передатчик ни чем не озабочен
 uart.recv_size = 0;                                         //нет принятых данных
 uart.send_mode = SENSOR_DAT;
}


//Обработчик прерывания по передаче байтов через UART (регистр данных передатчика пуст)
#pragma vector=USART_UDRE_vect
__interrupt void usart_udre_isr(void)
{       
 //__enable_interrupt();

 if (uart.send_size > 0)
 {
  UDR = uart.send_buf[uart.send_index];
  --uart.send_size;
  ++uart.send_index;
 }
 else
 {//все данные переданы
  UCSRB &= ~(1<<UDRIE); // disable UDRE interrupt 
 }
}


#pragma vector=USART_RXC_vect
__interrupt void usart_rx_isr()
{
  static uint8_t state=0;
  uint8_t chr;

  //__enable_interrupt();
  chr = UDR; 
  switch(state)
  {
    case 0:            //принимаем (ожидаем символ начала посылки)
      if (uart.recv_size!=0) //предыдущий принятый фрейм еще не обработан, а нам уже прислали новый.
       break;       

      if (chr=='!')   //начало пакета?   
      {        
       state = 1;
       uart.recv_index = 0;               
      }
      break;

    case 1:           //прием данных посылки               
      if (chr=='\r')
      {             
       state = 0;       //КА в исходное состояние      
       uart.recv_size = uart.recv_index; //данные готовы, сохраняем их размер
      }
      else
      {
       if (uart.recv_index >= UART_RECV_BUFF_SIZE)
       {
       //Ошибка: переполнение! - КА в исходное состояние, фрейм нельзя считать принятым!
       state = 0;                    
       }
       else
        uart.recv_buf[uart.recv_index++] = chr;
      }    
      break;                   
  }  
}
