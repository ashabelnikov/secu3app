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

#include <ioavr.h>
#include "secu3.h"
#include "starter.h"
#include "vstimer.h"
#include "ce_errors.h"

//блокирует/разблокирывает стартер
#define SET_STARTER_BLOCKING_STATE(s) {PORTD_Bit7 = s;}

void starter_set_blocking_state(uint8_t i_state)
{
 SET_STARTER_BLOCKING_STATE(i_state);
}

void starter_init_ports(void)
{
 PORTD|= (1<<PD7);    //стартер заблокирован
 DDRD |= (1<<DDD7);   //выход для стартера
}

void starter_control(struct ecudata_t* d)
{
#ifndef VPSEM   
 //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов, но обратно не включается!)
 if (d->sens.frequen4 > d->param.starter_off)
  SET_STARTER_BLOCKING_STATE(1);  
#else 
 //управление блокировкой стартера (стартер блокируется при оборотах больше пороговых)
 //и индикация состояния клапана ЭПХХ (используется выход блокировки стартера) 
 SET_STARTER_BLOCKING_STATE( (d->sens.frequen4 > d->param.starter_off)&&(d->ephh_valve) ? 1 : 0);
 //если расход воздуха максимальный - зажигаем СЕ и запускаем таймер 
 if (d->airflow > 15)
 {
  s_timer_set(ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
  ce_set_state(1);  
 }
#endif
}
