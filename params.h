#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <stdint.h>

struct ecudata_t;

//Запись данных в EEPROM - процесс очень медленный. Он будет проходить параллельно с выполнением программы.
//Сохранение данных в EEPROM произойдет только если за заданное время не произошло ни одной операции приема параметров
//из UART-a и сохраненные параметры отличаются от текущих.        
void save_param_if_need(struct ecudata_t* d);

//загружает параметры из EEPROM, проверяет целостность данных и если они испорчены то
//берет резервную копию из FLASH.
void load_eeprom_params(struct ecudata_t* d);

extern uint8_t eeprom_parameters_cache[];

#endif //_PARAMS_H_
