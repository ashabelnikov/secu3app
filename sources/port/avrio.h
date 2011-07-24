#ifndef _SECU3_AVRIO_H_
#define _SECU3_AVRIO_H_

#ifdef __ICCAVR__
 #include <ioavr.h>     //device IO

#else //AVR GCC
 #include <avr/io.h>    //device IO
 #include "portbits.h"  //gives PORTx_Bitx ability

 //Who can tell me why I must do this?
 #ifdef __AVR_ATmega64__
  #ifndef EE_RDY_vect
   #define EE_RDY_vect EE_READY_vect
  #endif
 #endif

 //Again...
 #ifdef __AVR_ATmega64__
  #ifndef USART0_RXC_vect
   #define USART0_RXC_vect USART0_RX_vect
  #endif
 #endif

#endif

#endif //_SECU3_AVRIO_H_
