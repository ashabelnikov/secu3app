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

#ifndef _CE_ERRORS_H_
#define _CE_ERRORS_H_

#include <stdint.h>
#include "vstimer.h"

//определяем биты (номера битов) ошибок (Check Engine)
#define ECUERROR_CKPS_MALFUNCTION       0
#define ECUERROR_EEPROM_PARAM_BROKEN    1  // СE погаснет через несколько секунд после запуска двигателя
#define ECUERROR_PROGRAM_CODE_BROKEN    2  // СЕ погаснет через несколько секунд после запуска двигателя
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
