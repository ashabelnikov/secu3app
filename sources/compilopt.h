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

#include "port/port.h"

/**ATmega16 target */
#ifdef _PLATFORM_M16_
 #define COPT_ATMEGA16 1
#else
 #define COPT_ATMEGA16 0
#endif

/**ATmega32 target */
#ifdef _PLATFORM_M32_
 #define COPT_ATMEGA32 1
#else
 #define COPT_ATMEGA32 0
#endif

/**ATmega64 target */
#ifdef _PLATFORM_M64_
 #define COPT_ATMEGA64 1
#else
 #define COPT_ATMEGA64 0
#endif

/**ATmega128 target */
#ifdef _PLATFORM_M128_
 #define COPT_ATMEGA128 1
#else
 #define COPT_ATMEGA128 0
#endif

/**Use VPSEM mode */
#ifdef VPSEM
 #define COPT_VPSEM 1
#else
 #define COPT_VPSEM 0
#endif

/** Use 36-1 crank */
#ifdef WHEEL_36_1
 #define COPT_WHEEL_36_1 1
#else
 #define COPT_WHEEL_36_1 0
#endif

/** Inverse ignition outputs */
#ifdef INVERSE_IGN_OUTPUTS
 #define COPT_INVERSE_IGN_OUTPUTS 1
#else
 #define COPT_INVERSE_IGN_OUTPUTS 0
#endif

/** Use dwell control */
#ifdef DWELL_CONTROL
 #define COPT_DWELL_CONTROL 1
#else
 #define COPT_DWELL_CONTROL 0
#endif

/** Use PWM for cooling fan*/
#ifdef COOLINGFAN_PWM
 #define COPT_COOLINGFAN_PWM 1
#else
 #define COPT_COOLINGFAN_PWM 0
#endif

/** Allow realtime editing of tables */
#ifdef REALTIME_TABLES
 #define COPT_REALTIME_TABLES 1
#else
 #define COPT_REALTIME_TABLES 0
#endif

/** Compiler used to build project (IAR) */
#ifdef __ICCAVR__
 #define COPT_ICCAVR_COMPILER 1
#else
 #define COPT_ICCAVR_COMPILER 0
#endif

/** Compiler used to build project (GCC) */
#ifdef __GNUC__
 #define COPT_AVRGCC_COMPILER 1
#else
 #define COPT_AVRGCC_COMPILER 0
#endif

/** Debug mode. Allows to watch variables and edit them remotely */
#ifdef DEBUG_VARIABLES
 #define COPT_DEBUG_VARIABLES 1
#else
 #define COPT_DEBUG_VARIABLES 0
#endif

/** Use of phase sensor */
#ifdef PHASE_SENSOR
 #define COPT_PHASE_SENSOR 1
#else
 #define COPT_PHASE_SENSOR 0
#endif

/** Use of phased ignition (coil on each cylinder)*/
#ifdef PHASED_IGNITION
 #define COPT_PHASED_IGNITION 1
#else
 #define COPT_PHASED_IGNITION 0
#endif

/** Enable controlling of electric fuel pump */
#ifdef FUEL_PUMP
 #define COPT_FUEL_PUMP 1
#else
 #define COPT_FUEL_PUMP 0
#endif

/** Coolant sensor is thermistor */
#ifdef THERMISTOR_CS
 #define COPT_THERMISTOR_CS 1
#else
 #define COPT_THERMISTOR_CS 0
#endif

#endif //_COMPILOPT_H_
