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

/** \file ufcodes.h
 * Codes of descriptors of packets used in communication via serial interface
 * ( оды дескрипторов пакетов используемых при передаче данных через последовательный интерфейс).
 */

#ifndef _UFCODES_H_
#define _UFCODES_H_

#define   CHANGEMODE   'h'
#define   BOOTLOADER   'i'

#define   TEMPER_PAR   'j'
#define   CARBUR_PAR   'k'
#define   IDLREG_PAR   'l'
#define   ANGLES_PAR   'm'
#define   FUNSET_PAR   'n'
#define   STARTR_PAR   'o'

#define   FNNAME_DAT   'p'
#define   SENSOR_DAT   'q'

#define   ADCCOR_PAR   'r'
#define   ADCRAW_DAT   's'

#define   CKPS_PAR     't'
#define   OP_COMP_NC   'u'

#define   CE_ERR_CODES 'v'

#define   KNOCK_PAR    'w'

#define   CE_SAVED_ERR 'x'
#define   FWINFO_DAT   'y'
#define   MISCEL_PAR   'z'

#endif //_UFCODES_H_
