
#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>

//адрес структуры параметров в EEPROM
#define EEPROM_PARAM_START     0x002

//адрес массива ошибок (Check Engine) в EEPROM
#define EEPROM_ECUERRORS_START (EEPROM_PARAM_START+(sizeof(params_t)))


//==================интерфейс модуля===============================
//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(uint8_t opcode, uint16_t eeprom_addr, uint8_t* sram_addr, uint8_t count);  

//возвращает не 0 если в текущий момент никакая операция не выполняется
uint8_t eeprom_is_idle(void);

//читает указанный блок данных из EEPROM (без использования прерываний)
void eeprom_read(void* sram_dest, int16_t eeaddr, uint16_t size);

//записывает указанный блок данных в EEPROM (без использования прерываний)
void eeprom_write(const void* sram_src, int16_t eeaddr, uint16_t size);

//возвращает код выполненной операции (код переданный в функцию eeprom_start_wr_data())
uint8_t eeprom_take_completed_opcode(void);  
//=================================================================


#endif //_EEPROM_H_
