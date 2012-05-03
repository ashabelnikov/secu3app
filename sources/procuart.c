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

#include "port/pgmspace.h"
#include "port/intrinsic.h"
#include "port/port.h"
#include <stdint.h>
#include "bitmask.h"
#include "ce_errors.h"
#include "ckps.h"
#include "diagnost.h"
#include "knock.h"
#include "procuart.h"
#include "secu3.h"
#include "suspendop.h"
#include "uart.h"
#include "ufcodes.h"
#include "vstimer.h"

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
    //если были изменены параметры то сбрасываем счетчик времени
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case MISCEL_PAR:
#ifdef HALL_OUTPUT
    ckps_set_hall_pulse(d->param.hop_start_cogs, d->param.hop_durat_cogs);
#endif
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case FUNSET_PAR:
#ifdef REALTIME_TABLES
    sop_set_operation(SOP_SELECT_TABLSET);
#endif
    //если были изменены параметры то сбрасываем счетчик времени
    s_timer16_set(save_param_timeout_counter, SAVE_PARAM_TIMEOUT_VALUE);
    break;

   case OP_COMP_NC:
    if (_AB(d->op_actn_code, 0) == OPCODE_EEPROM_PARAM_SAVE) //приняли команду сохранения параметров
    {
     sop_set_operation(SOP_SAVE_PARAMETERS);
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
    if (_AB(d->op_actn_code, 0) == OPCODE_CE_SAVE_ERRORS) //приняли команду чтения сохраненных кодов ошибок
    {
     sop_set_operation(SOP_READ_CE_ERRORS);
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
    if (_AB(d->op_actn_code, 0) == OPCODE_READ_FW_SIG_INFO) //приняли команду чтения и передачи информации о прошивке
    {
     sop_set_operation(SOP_SEND_FW_SIG_INFO);
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
#ifdef REALTIME_TABLES
    if (_AB(d->op_actn_code, 0) == OPCODE_LOAD_TABLSET) //приняли команду выбора нового набора таблиц
    {
     sop_set_operation(SOP_LOAD_TABLSET);
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
    if (_AB(d->op_actn_code, 0) == OPCODE_SAVE_TABLSET) //приняли команду сохранения сохранения набора таблиц для указанного типа топлива
    {
     sop_set_operation(SOP_SAVE_TABLSET);
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
#endif
#ifdef DIAGNOSTICS
    if (_AB(d->op_actn_code, 0) == OPCODE_DIAGNOST_ENTER) //"enter diagnostic mode" command has been received
    {
     //this function will send confirmation answer and start diagnostic mode (it will has its own separate loop)     
     diagnost_start();
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
    if (_AB(d->op_actn_code, 0) == OPCODE_DIAGNOST_LEAVE) //"leave diagnostic mode" command has been received
    {
     //this function will send confirmation answer and reset device
     diagnost_stop();
     _AB(d->op_actn_code, 0) = 0; //обработали
    }
#endif
    break;

   case CE_SAVED_ERR:
    sop_set_operation(SOP_SAVE_CE_ERRORS);
    break;

   case CKPS_PAR:
    //если были изменены параметры ДПКВ, то немедленно применяем их на работающем двигателе и сбрасываем счетчик времени
    ckps_set_cyl_number(d->param.ckps_engine_cyl);  //<--обязательно в первую очередь!
    ckps_set_edge_type(d->param.ckps_edge_type);
    ckps_set_cogs_btdc(d->param.ckps_cogs_btdc);
    ckps_set_merge_outs(d->param.merge_ign_outs);

#ifndef DWELL_CONTROL
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
   uart_send_packet(d, 0);                  //теперь передатчик озабочен передачей данных
#ifdef DEBUG_VARIABLES
   {
   uint8_t desc = uart_get_send_mode();
   if (SENSOR_DAT==desc || ADCRAW_DAT==desc || CE_ERR_CODES==desc || DIAGINP_DAT==desc)
    sop_set_operation(SOP_DBGVAR_SENDING); //additionally we will send packet with debug information
   }
#endif

   s_timer_set(send_packet_interval_counter, d->param.uart_period_t_ms);

   //после передачи очищаем кеш ошибок
   d->ecuerrors_for_transfer = 0;
  }
 }
}
