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
              http://secu-3.org
              email: shabelnikov@secu-3.org
*/

/** \file params.c
 * Implementation of functionality for work with parameters (save/restore/check)
 * (–еализаци€ функциональности дл€ работы с параметрами (сохранение/восстановление/проверка)).
 */

#include <inavr.h>
#include <pgmspace.h>

#include "crc16.h"
#include "params.h"
#include "secu3.h"
#include "suspendop.h"
#include "vstimer.h"
#include "jumper.h"
#include "eeprom.h"
#include "ce_errors.h"

uint8_t eeprom_parameters_cache[sizeof(params_t) + 1];

void save_param_if_need(struct ecudata_t* d)
{
 //параметры не изменились за заданное врем€?
 if (s_timer16_is_action(save_param_timeout_counter))
 {
  //текущие и сохраненные параметры отличаютс€?
  if (memcmp(eeprom_parameters_cache,&d->param,sizeof(params_t)-PAR_CRC_SIZE))
   sop_set_operation(SOP_SAVE_PARAMETERS);
  s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
 }
}

void load_eeprom_params(struct ecudata_t* d)
{
 if (jumper_get_defeeprom_state())
 {
  //«агружаем параметры из EEPROM, а затем провер€ем целостность.
  //ѕри подсчете контрольной суммы не учитываем байты самой контрольной суммы
  //если контрольные суммы не совпадают - загружаем резервные параметры из FLASH
  eeprom_read(&d->param,EEPROM_PARAM_START,sizeof(params_t));

  if (crc16((uint8_t*)&d->param,(sizeof(params_t)-PAR_CRC_SIZE))!=d->param.crc)
  {
   memcpy_P(&d->param,&def_param,sizeof(params_t));
   ce_set_error(ECUERROR_EEPROM_PARAM_BROKEN);
  }

  //инициализируем кеш параметров, иначе после старта программы произойдет ненужное
  //их сохранение.
  memcpy(eeprom_parameters_cache,&d->param,sizeof(params_t));
 }
 else
 { //перемычка закрыта - загружаем дефаултные параметры, которые позже будут сохранены
  memcpy_P(&d->param,&def_param,sizeof(params_t));
  ce_clear_errors(); //сбрасываем сохраненные ошибки
 }
}
