
#ifndef _CE_ERRORS_H_
#define _CE_ERRORS_H_

#include <stdint.h>
#include "vstimer.h"

//определ€ем биты (номера битов) ошибок (Check Engine)
#define ECUERROR_CKPS_MALFUNCTION       0
#define ECUERROR_EEPROM_PARAM_BROKEN    1
#define ECUERROR_PROGRAM_CODE_BROKEN    2
#define ECUERROR_KSP_CHIP_FAILED        3

struct ecudata;

//производит проверку наличи€ ошибок и управл€ет лампой CE.
void ce_check_engine(struct ecudata* d, s_timer8* ce_control_time_counter);

//”становка/всброс указанной ошибки (номер бита)
void ce_set_error(uint8_t error);  
void ce_clear_error(uint8_t error);

//ѕроизводит сохранение всех накопленных во временной пам€ти ошибок в EEPROM. 
//¬ызывать только если EEPROM готово!
void ce_save_merged_errors(void);

//инициализаци€ используемых портов ввода/вывода
void ce_init_ports(void);


#endif //_CE_ERRORS_H_
