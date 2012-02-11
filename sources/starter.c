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

/** \file starter.c
 * Implementation of working with starter
 * (Реализация работы со стартером).
 */

#include "port/avrio.h"
#include "port/port.h"
#include "bitmask.h"
#include "ce_errors.h"
#include "secu3.h"
#include "starter.h"
#include "vstimer.h"

/** Blocks/unblocks starter (блокирует/разблокирывает стартер) */
#ifdef SECU3T  /*SECU-3T*/
 #define SET_STARTER_BLOCKING_STATE(s) {PORTB_Bit1 = s;}
#else          /*SECU-3*/
 #define SET_STARTER_BLOCKING_STATE(s) {PORTD_Bit7 = s;}
#endif

void starter_set_blocking_state(uint8_t i_state)
{
 SET_STARTER_BLOCKING_STATE(i_state);
}

void starter_init_ports(void)
{
#ifdef SECU3T /*SECU-3T*/
 PORTB|= _BV(PB1);    //starter is blocked (стартер заблокирован)
 DDRB |= _BV(DDB1);   //Output for starter (выход для стартера)
#else         /*SECU-3*/
 PORTD|= _BV(PD7);
 DDRD |= _BV(DDD7);
#endif
}

void starter_control(struct ecudata_t* d)
{
#ifndef VPSEM
 //control of starter's blocking (starter is blocked after reaching the specified RPM, but will not turn back!)
 //управление блокировкой стартера (стартер блокируется после достижения указанных оборотов, но обратно не включается!)
 if (d->sens.frequen4 > d->param.starter_off)
  SET_STARTER_BLOCKING_STATE(1);
#else
 //control of starter's blocking (starter is blocked at speeds greater than the threshold)
 //and status indication of idle economizer valve (output of starter's blocking is used)
 //(управление блокировкой стартера (стартер блокируется при оборотах больше пороговых)
 //и индикация состояния клапана ЭПХХ (используется выход блокировки стартера))
 SET_STARTER_BLOCKING_STATE( (d->sens.frequen4 > d->param.starter_off)&&(d->ie_valve) ? 1 : 0);

 //if air flow is maximum - turn on CE and start timer
 //(если расход воздуха максимальный - зажигаем СЕ и запускаем таймер)
 if (d->airflow > 15)
 {
  s_timer_set(ce_control_time_counter, CE_CONTROL_STATE_TIME_VALUE);
  ce_set_state(CE_STATE_ON);
 }
#endif
 if (d->sens.frequen4 < 30)
  SET_STARTER_BLOCKING_STATE(0); //unblock starter (снимаем блокировку стартера)
}
