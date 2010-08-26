 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <ioavr.h>
#include "jumper.h"

#define GET_DEFEEPROM_JUMPER_STATE() (PINC_Bit2)

void jumper_init_ports(void)
{
 DDRC &= ~((1<<DDC3)|(1<<DDC2)); //входы
 PORTC|= (1<<PC3)|(1<<PC2);
}

uint8_t jumper_get_defeeprom_state(void)
{
 return GET_DEFEEPROM_JUMPER_STATE();
}
