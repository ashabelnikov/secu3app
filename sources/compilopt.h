/* SECU-3  - An open source, free engine control unit
   Copyright (C) 2007 Alexey A. Shabelnikov. Ukraine, Kiev

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
 * Definitions of compile-time options (preprocessor directives).
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

/**ATmega644 target */
#ifdef _PLATFORM_M644_
 #define COPT_ATMEGA644 1
#else
 #define COPT_ATMEGA644 0
#endif

/**Use VPSEM mode */
#ifdef VPSEM
 #define COPT_VPSEM 1
#else
 #define COPT_VPSEM 0
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

/** For SECU3-T */
#ifdef SECU3T
 #define COPT_SECU3T 1
#else
 #define COPT_SECU3T 0
#endif

/** For SECU3-T rev.9 board */
#ifdef REV9_BOARD
 #define COPT_REV9_BOARD 1
#else
 #define COPT_REV9_BOARD 0
#endif

/** Diagnostics */
#ifdef DIAGNOSTICS
 #define COPT_DIAGNOSTICS 1
#else
 #define COPT_DIAGNOSTICS 0
#endif

/** Emulation of Hall sensor output */
#ifdef HALL_OUTPUT
 #define COPT_HALL_OUTPUT 1
#else
 #define COPT_HALL_OUTPUT 0
#endif

/** Stroboscope functionality */
#ifdef STROBOSCOPE
 #define COPT_STROBOSCOPE 1
#else
 #define COPT_STROBOSCOPE 0
#endif

/** Stepper motor control and related functionality */
#ifdef SM_CONTROL
 #define COPT_SM_CONTROL 1
#else
 #define COPT_SM_CONTROL 0
#endif

/** Use 5V ADC reference voltage instead of internal 2.56V source*/
#ifdef VREF_5V
 #define COPT_VREF_5V 1
#else
 #define COPT_VREF_5V 0
#endif

/** Use synchronization from Hall sensor instead of CKP sensor*/
#ifdef HALL_SYNC
 #define COPT_HALL_SYNC 1
#else
 #define COPT_HALL_SYNC 0
#endif

/** Use binary mode for UART instead of default ASCII */
#ifdef UART_BINARY
 #define COPT_UART_BINARY 1
#else
 #define COPT_UART_BINARY 0
#endif

/**Build firmware for using 2 channel igniters (driven by both edges)*/
#ifdef CKPS_2CHIGN
 #define COPT_CKPS_2CHIGN 1
#else
 #define COPT_CKPS_2CHIGN 0
#endif

/** Include support of fuel injection */
#ifdef FUEL_INJECT
 #define COPT_FUEL_INJECT 1
#else
 #define COPT_FUEL_INJECT 0
#endif

#endif //_COMPILOPT_H_
