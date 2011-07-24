#ifndef _SECU3_PGMSPACE_H_
#define _SECU3_PGMSPACE_H_

#ifdef __ICCAVR__
 #include <pgmspace.h>
 #include <stdint.h>

 typedef uint8_t __flash prog_uint8_t;

 //Declare variable in FLASH at fixed address
 #define PGM_FIXED_ADDR_OBJ(variable, sect_name) _Pragma("object_attribute=__root") \
__flash variable@sect_name

 //Declare variable in FLASH
 #define PGM_DECLARE(x) __flash x

 #define PGM_GET_BYTE(addr) *(addr)
 #define PGM_GET_WORD(addr) *(addr)
 #define PGM_GET_DWORD(addr) *(addr)

 #define _PGM __flash

#else //AVR GCC
 #include <avr/pgmspace.h>

 //Declare variable in FLASH at fixed address
 #define PGM_FIXED_ADDR_OBJ(variable, sect_name) variable __attribute__((section (sect_name)))

 //Declare variable in FLASH
 #define PGM_DECLARE(x) const x __attribute__((__progmem__))

 #define PGM_GET_BYTE(addr) pgm_read_byte(addr)
 #define PGM_GET_WORD(addr) pgm_read_word(addr)
 #define PGM_GET_DWORD(addr) pgm_read_dword(addr)

 //GCC doesn't support Harvard's architerture, so use const at least to indicate
 //that variable will be read-only
 #define _PGM const

#endif

#endif //_SECU3_PGMSPACE_H_
