
#ifndef _CE_ERRORS_H_
#define _CE_ERRORS_H_

#include <stdint.h>
#include "vstimer.h"

//определяем биты (номера битов) ошибок (Check Engine)
#define ECUERROR_CKPS_MALFUNCTION       0
#define ECUERROR_EEPROM_PARAM_BROKEN    1
#define ECUERROR_PROGRAM_CODE_BROKEN    2
#define ECUERROR_KSP_CHIP_FAILED        3

struct ecudata_t;

//производит проверку наличия ошибок и управляет лампой CE.
void ce_check_engine(struct ecudata_t* d, volatile s_timer8_t* ce_control_time_counter);

//Установка/всброс указанной ошибки (номер бита)
void ce_set_error(uint8_t error);  
void ce_clear_error(uint8_t error);

//Производит сохранение всех накопленных во временной памяти ошибок в EEPROM. 
//Вызывать только если EEPROM готово!
void ce_save_merged_errors(void);

//очищает ошибки сохраненные в EEPROM
void ce_clear_errors(void);

//инициализация используемых портов ввода/вывода
void ce_init_ports(void);

//включает/выключает лампу Check Engine  
#define ce_set_state(s)  {PORTB_Bit2 = s;}
#define ce_get_state() (PORTB_Bit2)

#endif //_CE_ERRORS_H_
