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

/** \file fuelecon.c
 * Implementation of controlling algorithms for Fuel Economizer (FE)
 * (Реализация алгоритмов управления Экономайзером Мощностных Режимов (ЭМР)).
 */

#include <ioavr.h>
#include "secu3.h"
#include "fuelecon.h"

/** Open/Close FE valve (открывает/закрывает клапан ЭМР) */
#define SET_FE_VALVE_STATE(s) {PORTC_Bit7 = s;}

void fuelecon_init_ports(void)
{
 PORTC&= ~(1<<PC7); //FE valve is off (ЭМР выключен)
 DDRC|= (1<<DDC7);  //Output for control FE valve (выход для управления клапаном ЭМР)
}

void fuelecon_control(struct ecudata_t* d)
{
 int16_t discharge;
 
 discharge = (d->param.map_upper_pressure - d->sens.map);
 if (discharge < 0) 
  discharge = 0;    
 d->fe_valve = discharge < d->param.fe_on_threshold;
 SET_FE_VALVE_STATE(d->fe_valve);
}
