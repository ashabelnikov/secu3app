
#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>

//==================интерфейс модуля===============================
//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeprom_addr, uint8_t* sram_addr, uint8_t count);  

//возвращает не 0 если в текущий момент никакая операция не выполняется
uint8_t eeprom_is_idle(void);

//читает указанный блок данных из EEPROM
void eeprom_read(void* sram_dest, int16_t eeaddr, uint16_t size);

uint8_t eeprom_take_completed_opcode(void);  
//=================================================================


#endif //_EEPROM_H_
