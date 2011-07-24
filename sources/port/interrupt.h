#ifndef _SECU3_INTERRUPT_H_
#define _SECU3_INTERRUPT_H_

#ifdef __ICCAVR__
 //Use C99 _Pragma, which can be used inside macro. Next, define macro ISR
 //to counterfeit GCC
 #define PRAGMA_QUOTE(x) _Pragma(#x)
 #define ISR(vec) PRAGMA_QUOTE(vector=vec) __interrupt void isr_##vec(void) 

#else //GCC
 #include <avr/interrupt.h>

#endif

//For using instead of __monitor
#define _BEGIN_ATOMIC_BLOCK()\
uint8_t _t = _SAVE_INTERRUPT();\
_DISABLE_INTERRUPT()

#define _END_ATOMIC_BLOCK() _RESTORE_INTERRUPT(_t)

#endif //_SECU3_INTERRUPT_H_
