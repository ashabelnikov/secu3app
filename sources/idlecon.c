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

/** \file idlecon.c
 * Implementation of controling of Idle Econimizer (IE) valve.
 * (Реализация управления клапаном принудительного холостого хода(ЭПХХ)).
 */

#include <ioavr.h>
#include "secu3.h"
#include "vstimer.h"
#include "idlecon.h"

/** Opens/closes IE valve (открывает/закрывает клапан ЭПХХ) */
#define SET_IE_VALVE_STATE(s) {PORTB_Bit0 = s;}

void idlecon_init_ports(void)
{
 PORTB|= (1<<PB0); //valve is turned on (клапан ЭПХХ включен)
 DDRB |= (1<<DDB0);
}

//Implementation of IE functionality. If throttle gate is closed AND frq > [up.threshold] OR
//throttle gate is closed AND frq > [lo.threshold] BUT valve is already closed, THEN turn off
//fuel supply by stopping to apply voltage to valve's coil. ELSE - fuel supply is turned on.

//реализация функции ЭПХХ. Если заслонка карбюратора закрыта и frq > [верх.порог] или
//заслонка карбюратора закрыта и frq > [ниж.порог] но клапан уже закрыт, то производится
//выключение подачи топлива путем прекращения подачи напряжения на обмотку эл.клапана. Иначе - подача топлива.
void idlecon_control(struct ecudata_t* d)
{
 //if throttle gate is opened, then onen valve,reload timer and exit from condition
 //(если дроссель открыт, то открываем клапан, заряжаем таймер и выходим из условия).
 if (d->sens.carb)
 {
  d->ie_valve = 1;
  s_timer_set(epxx_delay_time_counter, d->param.shutoff_delay);
 }
 //if throttle gate is closed, then state of valve depends on RPM,previous state of valve,timer and type of fuel
 //(если дроссель закрыт, то состояние клапана зависит от оборотов, предыдущего состояния клапана, таймера и вида топлива).
 else
  if (d->sens.gas) //gas (газовое топливо)
   d->ie_valve = ((s_timer_is_action(epxx_delay_time_counter))
   &&(((d->sens.frequen > d->param.ie_lot_g)&&(!d->ie_valve))||(d->sens.frequen > d->param.ie_hit_g)))?0:1;
  else //gasoline (бензин)
   d->ie_valve = ((s_timer_is_action(epxx_delay_time_counter))
   &&(((d->sens.frequen > d->param.ie_lot)&&(!d->ie_valve))||(d->sens.frequen > d->param.ie_hit)))?0:1;
 SET_IE_VALVE_STATE(d->ie_valve);
}
