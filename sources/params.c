/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

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

#include "port/avrio.h"
#include "port/pgmspace.h"
#include "port/port.h"

#include <string.h>
#include "ce_errors.h"
#include "crc16.h"
#include "eeprom.h"
#include "ioconfig.h"
#include "jumper.h"
#include "params.h"
#include "secu3.h"
#include "starter.h"
#include "suspendop.h"
#include "ventilator.h"
#include "vstimer.h"
#include "wdt.h"

uint8_t eeprom_parameters_cache[sizeof(params_t) + 1];

void save_param_if_need(struct ecudata_t* d)
{
 //параметры не изменились за заданное врем€?
 if (s_timer16_is_action(save_param_timeout_counter))
 {
  //текущие и сохраненные параметры отличаютс€?
  if (memcmp(eeprom_parameters_cache, &d->param, sizeof(params_t)-PAR_CRC_SIZE))
   sop_set_operation(SOP_SAVE_PARAMETERS);
  s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
 }
}

void ckps_enable_ignition(uint8_t);
void ckps_init_ports(void);

void reset_eeprom_params(struct ecudata_t* d)
{
 uint16_t crc;
 uint16_t i = 5000; //5 seconds max

 ckps_enable_ignition(0);        //turn off ignition
 ckps_init_ports();
 vent_turnoff(d);                //turn off ventilator
 starter_set_blocking_state(1);  //block starter
 IOCFG_INIT(IOP_FL_PUMP, 0);     //turn off fuel pump
 IOCFG_INIT(IOP_IE, 0);          //turn off IE valve solenoid
 IOCFG_INIT(IOP_FE, 0);          //turn off power valve solenoid

 while(!eeprom_is_idle() && --i)
 {
  wdt_reset_timer();
  _DELAY_US(1000);      //1ms
 }
 _DISABLE_INTERRUPT();

 wdt_reset_timer();
 //1. calculate CRC; 2. write all except 2 bytes of CRC; 3. write CRC
 crc = crc16f((uint8_t _PGM*)&fw_data.def_param, sizeof(params_t)-PAR_CRC_SIZE);
 eeprom_write_P(&fw_data.def_param, EEPROM_PARAM_START, sizeof(params_t)-PAR_CRC_SIZE);
 eeprom_write(&crc, EEPROM_PARAM_START+(sizeof(params_t)-PAR_CRC_SIZE), PAR_CRC_SIZE);

 wdt_reset_timer();
 ce_clear_errors(); //сбрасываем сохраненные ошибки
 wdt_reset_timer();
#ifdef REALTIME_TABLES
 eeprom_write_P(&tt_def_data[0], EEPROM_REALTIME_TABLES_START, sizeof(f_data_t) * TUNABLE_TABLES_NUMBER);
#endif
 wdt_reset_device(); //reboot!
}

void load_eeprom_params(struct ecudata_t* d)
{
 if (jumper_get_defeeprom_state())
 {
  //«агружаем параметры из EEPROM, а затем провер€ем целостность.
  //ѕри подсчете контрольной суммы не учитываем байты самой контрольной суммы
  //если контрольные суммы не совпадают - загружаем резервные параметры из FLASH
  eeprom_read(&d->param,EEPROM_PARAM_START,sizeof(params_t));

  if (crc16((uint8_t*)&d->param, (sizeof(params_t)-PAR_CRC_SIZE))!=d->param.crc)
  {
   memcpy_P(&d->param, &fw_data.def_param, sizeof(params_t));
   ce_set_error(ECUERROR_EEPROM_PARAM_BROKEN);
  }

  //инициализируем кеш параметров, иначе после старта программы произойдет ненужное
  //их сохранение.
  memcpy(eeprom_parameters_cache, &d->param, sizeof(params_t));
 }
 else
 {//перемычка закрыта - загружаем дефаултные параметры, которые позже будут сохранены, а также
  //загружаем в EEPROM данные по умолчанию дл€ редактируемых наборов таблиц
  memcpy_P(&d->param, &fw_data.def_param, sizeof(params_t));
  ce_clear_errors(); //сбрасываем сохраненные ошибки
#ifdef REALTIME_TABLES
  eeprom_write_P(&tt_def_data[0], EEPROM_REALTIME_TABLES_START, sizeof(f_data_t) * TUNABLE_TABLES_NUMBER);
#endif
  //write 4 bytes of magic number identifying platform
  eeprom_write_P((void _PGM*)(FLASHEND-3), EEPROM_MAGIC_START, 4);
 }
}

#ifdef REALTIME_TABLES
void load_selected_tables_into_ram(struct ecudata_t* d)
{
 if (d->fn_gasoline_prev != d->param.fn_gasoline)
 {
  //load gasoline tables
  load_specified_tables_into_ram(d, 0, d->param.fn_gasoline);
  d->fn_gasoline_prev = d->param.fn_gasoline;
 }

 if (d->fn_gas_prev != d->param.fn_gas)
 {
  //load gas tables
  load_specified_tables_into_ram(d, 1, d->param.fn_gas);
  d->fn_gas_prev = d->param.fn_gas;
 }
}

void load_specified_tables_into_ram(struct ecudata_t* d, uint8_t fuel_type, uint8_t index)
{
 //load tables depending on type of fuel
 if (index < TABLES_NUMBER)
  memcpy_P(&d->tables_ram[fuel_type], &fw_data.tables[index], sizeof(f_data_t));
 else
  eeprom_read(&d->tables_ram[fuel_type], EEPROM_REALTIME_TABLES_START+(sizeof(f_data_t)*(index-TABLES_NUMBER)), sizeof(f_data_t));

 //будет послано уведомление о том, что загружен новый набор таблиц
 //notification will be sent about that new set of tables has been loaded
 sop_set_operation(SOP_SEND_NC_TABLSET_LOADED);
}

#endif
