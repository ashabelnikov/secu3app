 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include "eeprom.h"
#include "bitmask.h"
#include <iom16.h>


//Описывает информацию необходимую для сохранения данных в EEPROM
typedef struct 
{
  unsigned int ee_addr;               //адрес для записи в EEPROM
  unsigned char* sram_addr;           //адрес данных в ОЗУ 
  unsigned char count;                //количество байтов
  unsigned char eews;                 //состояние процесса записи
}eeprom_wr_desc;


eeprom_wr_desc eewd;


//инициирует процесс записи байта в EEPROM
#define EE_START_WR_BYTE()  {EECR|= (1<<EEMWE);  EECR|= (1<<EEWE);}     


//запускает процесс записи в EEPROM указанного блока данных
void eeprom_start_wr_data(unsigned int eeprom_addr, unsigned char* sram_addr, unsigned char count)  
{
  eewd.eews = 1;
  eewd.ee_addr = eeprom_addr;
  eewd.sram_addr = sram_addr;
  eewd.count = count;
  SETBIT(EECR,EERIE);
}

//возвращает не 0 если в текущий момент никакая операция не выполняется
unsigned char eeprom_is_idle(void)
{
 return (eewd.eews) ? 0 : 1;
}


//Обработчик прерывания от EEPROM
//при завершении работы автомата всегда заносим в регистр адреса - адрес нулевой ячейки
#pragma vector=EE_RDY_vect
__interrupt void ee_ready_isr(void)
{ 
  switch(eewd.eews)
  {
    case 0:   //КА остановлен
      break;

    case 1:   //КА в процессе записи
      EEAR = eewd.ee_addr;       
      EEDR = *eewd.sram_addr;
      EE_START_WR_BYTE();                          
      eewd.sram_addr++;
      eewd.ee_addr++;
      if (--eewd.count==0)
       eewd.eews = 2;   //последний байт запущен на запись.
      else      
       eewd.eews = 1;   
      break;    

    case 2:   //последний байт записан
      EEAR=0x000;      
      CLEARBIT(EECR,EERIE); //запрещаем прерывание от EEPROM        
      eewd.eews = 0;
      break;      
  }//switch  
}
