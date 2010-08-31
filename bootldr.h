 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#ifndef _BOOTLDR_H_
#define _BOOTLDR_H_

#include <stdint.h>

//Определяем размер секции бутлоадера в зависимости от выбранной платформы
//Везде используется размер загрузчика соответствующий значению SECONDBOOTSTART
#ifdef __ATmega16__
 #define BOOT_LOADER_SIZE  512    
#elif __ATmega32__
 #define BOOT_LOADER_SIZE  1024   
#elif __ATmega64__
 #define BOOT_LOADER_SIZE  2048
#else
 #error "Not supported platform!"  
#endif 

//определяем стартовый адрес бутлоадера в прошивке (в байтах)
//FLASHEND определено в ioavr.h
#define SECU3BOOTSTART ((((unsigned int)FLASHEND) + 1) - BOOT_LOADER_SIZE)

//точка входа в бутлоадер из программы (минуя проверку перемычки)
#define boot_loader_start() ((void (*)())(SECU3BOOTSTART+0xA))()

#endif //_BOOTLDR_H_
