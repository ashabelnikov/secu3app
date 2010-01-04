
#ifndef _JUMPER_H_
#define _JUMPER_H_

#include <stdint.h>

//инициализация используемых портов
void jumper_init_ports(void);

//возвращает состояние перемычки "Default EEPROM"
uint8_t jumper_get_defeeprom_state(void);

#endif //_JUMPER_H_
