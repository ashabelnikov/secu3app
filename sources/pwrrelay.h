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

/** \file pwrrelay.h
 * Power management using external relay, allows SECU-3 to be turned on some time
 * after ignition is off. So, for instance electric colling fan can work even ignition is off
 * (Управление питанием используя внешнее реле, позволяет SECU-3 оставаться включенным еще
 * некоторое время после выключения зажигания).
 */

#ifndef _PWRRELAY_H_
#define _PWRRELAY_H_

struct ecudata_t;

/** Initialization of used I/O ports (инициализация используемых портов) */
void pwrrelay_init_ports(void);

/** Initialization of the module */
void pwrrelay_init(void);

/** Control of power relay (управление реле питания)
 * \param d pointer to ECU data structure
 */
void pwrrelay_control(struct ecudata_t* d);

#endif //_PWRRELAY_H_
