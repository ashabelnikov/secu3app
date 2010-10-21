/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Gorlovka

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   contacts:
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

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
