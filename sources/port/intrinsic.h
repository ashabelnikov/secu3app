#ifndef _SECU3_INTRINSIC_H_
#define _SECU3_INTRINSIC_H_

#ifdef __ICCAVR__
 #include <inavr.h> //IAR's intrinsics

 //abstracting intrinsics
 #define _ENABLE_INTERRUPT() __enable_interrupt()
 #define _DISABLE_INTERRUPT() __disable_interrupt()
 #define _SAVE_INTERRUPT() __save_interrupt()
 #define _RESTORE_INTERRUPT(s) __restore_interrupt(s)
 #define _NO_OPERATION() __no_operation()
 #define _DELAY_CYCLES(cycles) __delay_cycles(cycles)

#else //AVR GCC
 #include <avr/eeprom.h>       //__EEGET(), __EEPUT() etc
 #include <util/delay_basic.h> //for _DELAY_CYCLES()

 //abstracting intrinsics
 #define _ENABLE_INTERRUPT() sei()
 #define _DISABLE_INTERRUPT() cli()
 #define _SAVE_INTERRUPT() SREG
 #define _RESTORE_INTERRUPT(s) SREG = (s)
 #define _NO_OPERATION() __asm__ __volatile__ ("nop" ::)
 #define _DELAY_CYCLES(cycles) _delay_loop_2(cycles / 4)

#endif

#endif //_SECU3_INTRINSIC_H_
