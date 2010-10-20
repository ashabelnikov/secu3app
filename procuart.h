
#ifndef _PROCUART_H_
#define _PROCUART_H_

struct ecudata_t;

//обрабатывает передаваемые/принимаемые фреймы UART-a
void process_uart_interface(struct ecudata_t* d);

#endif //_PROCUART_H_
