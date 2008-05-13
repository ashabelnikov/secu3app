
#include "main.h"

#define CBR_9600   0x0067


extern char snd_mode;
extern char rcv_mode;
extern unsigned char *rcv_data;
extern unsigned char *snd_data;

void USART_Init(unsigned int baud);

