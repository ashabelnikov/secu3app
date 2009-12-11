
#ifndef  _UART_H_
#define  _UART_H_

#include <stdint.h>

#define  CBR_2400                0x01A0
#define  CBR_4800                0x00CF
#define  CBR_9600                0x0067
#define  CBR_14400               0x0044
#define  CBR_19200               0x0033
#define  CBR_38400               0x0019
#define  CBR_57600               0x0010

#define  UART_RECV_BUFF_SIZE     64
#define  UART_SEND_BUFF_SIZE     64

//==============интерфейс модуля=======================
 struct ecudata;
 void uart_send_packet(struct ecudata* d, uint8_t send_mode);
 uint8_t uart_recept_packet(struct ecudata* d);
 void uart_notify_processed(void);
 uint8_t uart_is_sender_busy(void);
 uint8_t uart_is_packet_received(void);
 uint8_t uart_get_send_mode(void);
 uint8_t uart_set_send_mode(uint8_t descriptor);
 void uart_init(uint16_t baud);

//=====================================================

#endif //_UART_H_
