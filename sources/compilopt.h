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

/** \file compilopt.h
 * Definitions of cimpile-time options (preprocessor directives).
 */

#ifndef _COMPILOPT_H_
#define _COMPILOPT_H_
           
#ifdef __ATmega16__
 #define COPT_ATMEGA16 1
#else
 #define COPT_ATMEGA16 0
#endif

#ifdef __ATmega32__
 #define COPT_ATMEGA32 1
#else
 #define COPT_ATMEGA32 0
#endif

#ifdef __ATmega64__
 #define COPT_ATMEGA64 1
#else
 #define COPT_ATMEGA64 0
#endif

#ifdef __ATmega128__
 #define COPT_ATMEGA128 1
#else
 #define COPT_ATMEGA128 0
#endif

#ifdef VPSEM
 #define COPT_VPSEM 1
#else
 #define COPT_VPSEM 0
#endif

#ifdef WHEEL_36_1
 #define COPT_WHEEL_36_1 1
#else
 #define COPT_WHEEL_36_1 0
#endif

#ifdef INVERSE_IGN_OUTPUTS
 #define COPT_INVERSE_IGN_OUTPUTS 1
#else
 #define COPT_INVERSE_IGN_OUTPUTS 0
#endif

#ifdef COIL_REGULATION
 #define COPT_COIL_REGULATION 1
#else
 #define COPT_COIL_REGULATION 0
#endif

#ifdef COOLINGFAN_PWM
 #define COPT_COOLINGFAN_PWM 1
#else
 #define COPT_COOLINGFAN_PWM 0
#endif

#ifdef REALTIME_TABLES
 #define COPT_REALTIME_TABLES 1
#else
 #define COPT_REALTIME_TABLES 0
#endif

#endif //_COMPILOPT_H_
