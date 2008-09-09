
#ifndef _EEPROM_H_
#define _EEPROM_H_


//==================интерфейс модуля===============================
//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(char opcode, unsigned int eeprom_addr, unsigned char* sram_addr, unsigned char count);  

//возвращает не 0 если в текущий момент никакая операция не выполняется
unsigned char eeprom_is_idle(void);

//читает указанный блок данных из EEPROM
void eeprom_read(void* sram_dest, int eeaddr, unsigned int size);

char eeprom_take_completed_opcode(void);  
//=================================================================


#endif //_EEPROM_H_
