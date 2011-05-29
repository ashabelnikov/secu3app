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

/** \file procuart.c
 * Implementation of functionality for processing of pending data which is sent/received via serial interface (UART)
 * (Реализация обработки поступающих данных для приема/передачи через последовательный интерфейс (UART)).
 */

#include <inavr.h>
#include <pgmspace.h>
#include <stdint.h>
#include "ce_errors.h"
#include "ckps.h"
#include "eeprom.h"
#include "knock.h"
#include "procuart.h"
#include "secu3.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "vstimer.h"


#ifdef REALTIME_TABLES
void load_selected_tables_into_ram(struct ecudata_t* d)
{
 if (d->fn_gas_prev != d->param.fn_gas)
 {
  //load gas tables
  if (d->param.fn_gas < TABLES_NUMBER)
   memcpy_P(&d->tables_ram[1], &tables[d->param.fn_gas], sizeof(f_data_t));
  else
   eeprom_read(&d->tables_ram[1], EEPROM_REALTIME_TABLES+(sizeof(f_data_t)*(d->param.fn_gas-TABLES_NUMBER)), sizeof(f_data_t));
  d->fn_gas_prev = d->param.fn_gas;
 }

 if (d->fn_gasoline_prev != d->param.fn_gasoline)
 {
  //load gasoline tables
  if (d->param.fn_gasoline < TABLES_NUMBER)
   memcpy_P(&d->tables_ram[0], &tables[d->param.fn_gasoline], sizeof(f_data_t));
  else
   eeprom_read(&d->tables_ram[0], EEPROM_REALTIME_TABLES+(sizeof(f_data_t)*(d->param.fn_gasoline-TABLES_NUMBER)), sizeof(f_data_t));  
  d->fn_gasoline_prev = d->param.fn_gasoline;
 }
}
#endif


void process_uart_interface(struct ecudata_t* d)
{
 uint8_t descriptor;

 if (uart_is_packet_received())//приняли новый фрейм ?
 {
  descriptor = uart_recept_packet(d);
  switch(descriptor)
  {
   case TEMPER_PAR:
   case CARBUR_PAR:
   case IDLREG_PAR:
   case ANGLES_PAR:
   case STARTR_PAR:
   case ADCCOR_PAR:
   case MISCEL_PAR:
    //если были изменены параметры то сбрасываем счетчик времени
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case FUNSET_PAR:
#ifdef REALTIME_TABLES
    load_selected_tables_into_ram(d);
#endif
    //если были изменены параметры то сбрасываем счетчик времени
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case OP_COMP_NC:
    if (d->op_actn_code == OPCODE_EEPROM_PARAM_SAVE) //приняли команду сохранения параметров
    {
     sop_set_operation(SOP_SAVE_PARAMETERS);
     d->op_actn_code = 0; //обработали
    }
    if (d->op_actn_code == OPCODE_CE_SAVE_ERRORS) //приняли команду чтения сохраненных кодов ошибок
    {
     sop_set_operation(SOP_READ_CE_ERRORS);
     d->op_actn_code = 0; //обработали
    }
    if (d->op_actn_code == OPCODE_READ_FW_SIG_INFO) //приняли команду чтения и передачи информации о прошивке
    {
     sop_set_operation(SOP_SEND_FW_SIG_INFO);
     d->op_actn_code = 0; //обработали
    }
    break;

   case CE_SAVED_ERR:
    sop_set_operation(SOP_SAVE_CE_ERRORS);
    break;

   case CKPS_PAR:
    //если были изменены параметры ДПКВ, то немедленно применяем их на работающем двигателе и сбрасываем счетчик времени
    ckps_set_cyl_number(d->param.ckps_engine_cyl);  //<--обязательно в первую очередь!
    ckps_set_edge_type(d->param.ckps_edge_type);
    ckps_set_cogs_btdc(d->param.ckps_cogs_btdc);
#ifndef COIL_REGULATION
    ckps_set_ignition_cogs(d->param.ckps_ignit_cogs);
#endif
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case KNOCK_PAR:
    //аналогично для контороля детонации, обязательно после CKPS_PAR!
    //инициализируем процессор детонации в случае если он не использовался, а теперь поступила команда его использовать.
    if (!d->use_knock_channel_prev && d->param.knock_use_knock_channel)
     if (!knock_module_initialize())
     {//чип сигнального процессора детонации неисправен - зажигаем СЕ
      ce_set_error(ECUERROR_KSP_CHIP_FAILED);
     }

    knock_set_band_pass(d->param.knock_bpf_frequency);
    //gain устанавливается в каждом рабочем цикле
    knock_set_int_time_constant(d->param.knock_int_time_const);
    ckps_set_knock_window(d->param.knock_k_wnd_begin_angle, d->param.knock_k_wnd_end_angle);
    ckps_use_knock_channel(d->param.knock_use_knock_channel);

    //запоминаем состояние флага для того чтобы потом можно было определить нужно инициализировать
    //процессор детонации или нет.
    d->use_knock_channel_prev = d->param.knock_use_knock_channel;

    //если были изменены параметры то сбрасываем счетчик времени
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;
  }

  //мы обработали принятые данные - приемник ничем теперь не озабочен
  uart_notify_processed();
 }

 //периодически передаем фреймы с данными
 if (s_timer_is_action(send_packet_interval_counter))
 {
  if (!uart_is_sender_busy())
  {
   uart_send_packet(d, 0);    //теперь передатчик озабочен передачей данных
   s_timer_set(send_packet_interval_counter, d->param.uart_period_t_ms);

   //после передачи очищаем кеш ошибок
   d->ecuerrors_for_transfer = 0;
  }
 }
}
