
#ifndef  _UART_H_
#define  _UART_H__

#include "main.h"
#include "tables.h"

#define  CBR_4800                0x00CF
#define  CBR_9600                0x0067
#define  CBR_14400               0x0044
#define  CBR_19200               0x0033
#define  CBR_38400               0x0019

#define  UART_RECV_BUFF_SIZE     32
#define  UART_SEND_BUFF_SIZE     32

//дл€ деcкриптора TEMPER_PAR (действительна только в домене UART-a)
typedef struct
{
  unsigned char tmp_use;
  signed   int  vent_on;                                    
  signed   int  vent_off;                                   
}uart_temper_par;

//дл€ деcкриптора CARBUR_PAR (действительна только в домене UART-a)
typedef struct
{
  unsigned int  ephh_lot;                                   
  unsigned int  ephh_hit;                                   
  unsigned char carb_invers;                                
}uart_carbur_par;


//дл€ деcкриптора IDLREG_PAR (действительна только в домене UART-a)
typedef struct
{
  unsigned char idl_regul;                                  
  signed   int  ifac1;                                      
  signed   int  ifac2;                                      
  signed   int  MINEFR;      //зачем signed??? (возможно надо сделать unsigned)                              
  unsigned int  idl_turns;                                  
}uart_idlreg_par;


//дл€ деcкриптора ANGLES_PAR (действительна только в домене UART-a)
typedef struct
{
  signed   int  max_angle;                                  
  signed   int  min_angle;                                  
  signed   int  angle_corr;                                 
}uart_angles_par;

//дл€ деcкриптора FUNSET_PAR (действительна только в домене UART-a)
typedef struct
{
  unsigned char fn_benzin;                
  unsigned char fn_gas;            
  unsigned char map_grad;         
  signed   int  press_swing;   //зачем signed??? (возможно надо сделать unsigned)
}uart_funset_par;

//дл€ деcкриптора STARTR_PAR (действительна только в домене UART-a)
typedef struct
{
  unsigned int  starter_off;       
  unsigned int  smap_abandon;             
}uart_startr_par;


//дл€ деcкриптора FNNAME_DAT (действительна только в домене UART-a)
typedef struct
{
  unsigned char tables_num;                                  //количество семейств характеристик
  unsigned char index;                                       //номер набора
  char          name[F_NAME_SIZE];                           //ассоциированное им€ (им€ семейства)  
}uart_fnname_dat;


//дл€ деcкриптора SENSOR_DAT (действительна только в домене UART-a)
typedef struct 
{
  sensors sens;
  unsigned char    ephh_valve;                                 
  unsigned char    airflow;                                    
  signed int       curr_angle;                                 
}uart_sensor_dat;


//следующие две union используют аномимное объ€вление структур

typedef union  //домен UART-a - данные принимаемых фреймов
{
  uart_temper_par;
  uart_carbur_par;
  uart_idlreg_par;
  uart_angles_par;
  uart_funset_par;
  uart_startr_par;
  char snd_mode;
  unsigned char data[UART_RECV_BUFF_SIZE];
}UART_recv_buf;

typedef union //домен UART-a - данные передаваемых фреймов
{
  uart_temper_par;
  uart_carbur_par;
  uart_idlreg_par;
  uart_angles_par;
  uart_funset_par;
  uart_startr_par;
  uart_fnname_dat;
  uart_sensor_dat;
  unsigned char data[UART_SEND_BUFF_SIZE];
}UART_send_buf;

//==============интерфейс модул€=======================
 void uart_send_packet(void);
 void uart_notify_processed(void);
 unsigned char uart_is_sender_busy(void);
 unsigned char uart_is_packet_received(void);
 char uart_get_send_mode(void);
 char uart_get_recv_mode(void);
 char uart_set_send_mode(char send_mode);
 void uart_init(unsigned int baud);
 extern UART_recv_buf uart_recv_buf;
 extern UART_send_buf uart_send_buf;
//=====================================================

#endif //_UART_H_
