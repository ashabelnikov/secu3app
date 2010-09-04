
#ifndef _SUSPOP_H_
#define _SUSPOP_H_

#include <stdint.h>

#define SOP_NA                       255
#define SOP_SAVE_PARAMETERS          0 
#define SOP_SAVE_CE_MERGED_ERRORS    1 
#define SOP_SEND_NC_PARAMETERS_SAVED 2
#define SOP_SAVE_CE_ERRORS           3
#define SOP_SEND_NC_CE_ERRORS_SAVED  4
#define SOP_READ_CE_ERRORS           5
#define SOP_TRANSMIT_CE_ERRORS       6
#define SOP_SEND_FW_SIG_INFO         7 

//Эти константы не должны быть равны 0
#define OPCODE_EEPROM_PARAM_SAVE     1
#define OPCODE_CE_SAVE_ERRORS        2
#define OPCODE_READ_FW_SIG_INFO      3 

struct ecudata_t;

//установка указанной рперации в очередь на выполнение 
void sop_set_operation(uint8_t opcode); 

//проверка - ждет выполнения или выполняется операция
uint8_t sop_is_operation_active(uint8_t opcode);

//инициализация модуля
void sop_init_operations(void);

//обработка очереди отложенных операций
void sop_execute_operations(struct ecudata_t* d);

#endif //#define _SUSPOP_H_
