#ifndef _SECU3_PORT_H_
#define _SECU3_PORT_H_

#ifdef __ICCAVR__
 #define MAIN() __C_task void main(void) 

 //convert compiler-specific symbols to common symbols
 #ifdef __ATmega16__
  #define _PLATFORM_M16_
 #elif  __ATmega32__
  #define _PLATFORM_M32_
 #elif  __ATmega64__
  #define _PLATFORM_M64_
 #else
  #error "avrio.h: Wrong platform identifier!"
 #endif

 #define INLINE _Pragma("inline")

#elif defined(__GNUC__) // GNU Compiler
 //main() can be void if -ffreestanding compiler option specified.
 #define MAIN() __attribute__ ((OS_main)) void main(void)

 //convert compiler-specific symbols to common symbols
 #if defined (__AVR_ATmega16__)
  #define _PLATFORM_M16_
 #elif defined (__AVR_ATmega32__)
  #define _PLATFORM_M32_
 #elif defined (__AVR_ATmega64__)
  #define _PLATFORM_M64_
 #else
  #error "avrio.h: Wrong platform identifier!"
 #endif

 #define INLINE inline
#else //Unknown compiler!
 #error "port.h: Unknown C compiler!"
#endif

#endif //_SECU3_PORT_H_
