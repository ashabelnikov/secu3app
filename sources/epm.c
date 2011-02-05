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

/** \file epm.c
 * Implementation of controlling algorithms for power's modes economizer
 * (–еализаци€ алгоритмов управлени€ экономайзером мощностных режимов).
 */

#include <ioavr.h>
#include "secu3.h"
#include "epm.h"

/** Open/Close EPM valve (открывает/закрывает клапан Ёћ–) */
#define SET_EPM_VALVE_STATE(s) {PORTC_Bit7 = s;}

void epm_init_ports(void)
{
 PORTC&= ~(1<<PC7); //EPM is off (Ёћ– выключен)
 DDRC|= (1<<DDC7);  //Output for control EPM valve (выход дл€ управлени€ клапаном Ёћ–)
}

void epm_control(struct ecudata_t* d)
{
 int16_t discharge;
 
 discharge = (d->param.map_upper_pressure - d->sens.map);
 if (discharge < 0) 
  discharge = 0;    
 d->epm_valve = discharge < d->param.epm_on_threshold;
 SET_EPM_VALVE_STATE(d->epm_valve);
}
