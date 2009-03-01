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
#include <pgmspace.h>
#include "uart.h"
#include "ufcodes.h"
#include "bitmask.h"

typedef struct
{
 char send_mode;
 unsigned char recv_buf[UART_RECV_BUFF_SIZE];
 unsigned char send_buf[UART_SEND_BUFF_SIZE];
 unsigned char send_size;
 unsigned char send_index;
 unsigned char recv_size;
 unsigned char recv_index;
}UARTSTATE;

UARTSTATE uart;

const __flash char hdig[] = "0123456789ABCDEF";


#define HTOD(h) (((h)<0x3A) ? ((h)-'0') : ((h)-'A'+10))

//--------вспомогательные функции для построения пакетов-------------
#define build_fb(src, size) \
{ \
 memcpy_P(&uart.send_buf[uart.send_size],(src),(size)); \
 uart.send_size+=(size); \
}

#define build_i4h(i) {uart.send_buf[uart.send_size++] = ((i)+0x30);}

void build_i8h(unsigned char i)
{
 uart.send_buf[uart.send_size++] = hdig[i/16];    //старший байт HEX числа
 uart.send_buf[uart.send_size++] = hdig[i%16];    //младший байт HEX числа
}

void build_i16h(unsigned int i)
{
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,1)/16];    //старший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,1)%16];    //младший байт HEX числа (старший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,0)/16];    //старший байт HEX числа (младший байт)
 uart.send_buf[uart.send_size++] = hdig[GETBYTE(i,0)%16];    //младший байт HEX числа (младший байт)
}


void build_i32h(unsigned long i)
{
 build_i16h(i>>16);
 build_i16h(i);
}

//----------вспомагательные функции для распознавания пакетов---------
#define recept_i4h() (uart.recv_buf[uart.recv_index++] - 0x30)

unsigned char recept_i8h(void)
{
 unsigned char i8;    
 i8 = HTOD(uart.recv_buf[uart.recv_index])<<4;
 uart.recv_index++;
 i8|= HTOD(uart.recv_buf[uart.recv_index]);
 uart.recv_index++;
 return i8;
}

unsigned int recept_i16h(void)
{
 unsigned int i16;    
 SETBYTE(i16,1) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;          
 uart.recv_index++;
 SETBYTE(i16,1)|= (HTOD(uart.recv_buf[uart.recv_index]));          
 uart.recv_index++;
 SETBYTE(i16,0) = (HTOD(uart.recv_buf[uart.recv_index]))<<4;    
 uart.recv_index++;
 SETBYTE(i16,0)|= (HTOD(uart.recv_buf[uart.recv_index]));          
 uart.recv_index++;
 return i16;
}


unsigned long recept_i32h(void)
{
 unsigned long i = 0;
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
void uart_send_packet(ecudata* d, char send_mode)  
{
 static unsigned char index = 0;

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
       build_i8h(d->airflow);         
       build_i4h(d->ephh_valve);     
       build_i4h(d->sens.carb);      
       build_i4h(d->sens.gas); 
       build_i16h(d->sens.knock_k);  // <-- knock value      
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
  }//switch

  //общая часть для всех пакетов
  uart.send_buf[uart.send_size++] = '\r';

  //буфер передатчика содержит полностью готовый пакет - начинаем передачу
  uart_begin_send();
}

//эта функция не проверяет, был или не был принят фрейм, проверка должна быть произведена до вызова функции.
//Возвращает дескриптор обработанного фрейма
unsigned char uart_recept_packet(ecudata* d)
{
 //буфер приемника содержит дескриптор пакета и данные
 unsigned char temp; 
 unsigned char descriptor; 
   
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
       break;       
       
    case OP_COMP_NC: 
       d->op_actn_code = recept_i4h(); 
       break;   
       
    case KNOCK_PAR:  
       d->param.knock_use_knock_channel = recept_i4h();
       d->param.knock_bpf_frequency   = recept_i8h();
       d->param.knock_k_wnd_begin_angle = recept_i16h();
       d->param.knock_k_wnd_end_angle = recept_i16h();     
       break;   
       
    case CE_SAVED_ERR:
       d->ecuerrors_saved_transfer = recept_i16h();
       break;   
       
  }//switch     

 return descriptor;
}


void uart_notify_processed(void)
{
  uart.recv_size = 0;
}

unsigned char uart_is_sender_busy(void)
{
  return (uart.send_size > 0);
}

unsigned char uart_is_packet_received(void)
{
  return (uart.recv_size > 0);
}

char uart_get_send_mode(void)
{
 return uart.send_mode;
}

char uart_set_send_mode(char descriptor)
{
 return uart.send_mode = descriptor;
}

void uart_init(unsigned int baud)
{
  // Set baud rate 
  UBRRH = (unsigned char)(baud>>8);
  UBRRL = (unsigned char)baud;
  UCSRA = 0;                                                  //удвоение не используем 
  UCSRB=(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);                       //приемник,прерывание по приему и передатчик разрешены
  UCSRC=(1<<URSEL)/*|(1<<USBS)*/|(1<<UCSZ1)|(1<<UCSZ0);       //8 бит, 1 стоп, нет контроля четности                                      
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
  uart.send_size--;
  uart.send_index++;
 }
 else
 {//все данные переданы
  UCSRB &= ~(1<<UDRIE); // disable UDRE interrupt 
 }
}


#pragma vector=USART_RXC_vect
__interrupt void usart_rx_isr()
{
  static unsigned char state=0;
  unsigned char chr;

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
