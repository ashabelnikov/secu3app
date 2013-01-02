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

/** \file magnitude.h
 * Helpful macros for working with constants when fixed point representation of fractional numbers is used
 * (Вспомогательные макросы для работы с константными значениями при использовании целочисленного
 * представления дробных чисел).
 */

#ifndef _MAGNITUDE_H_
#define _MAGNITUDE_H_

/**Used for rounding-up when transforming from floating point value into integer.
 * Note: it is intended for use with constants
 * (необходим для округления при преобразовании из числа с плавающей точкой
 * в целое число).
 */
#define ROUND(x) ((int16_t)( (x) + 0.5 - ((x) < 0) ))

/* Following macros are necessary when transforming floating point constant-values into integers.
 * Values of phisical magnitudes stored in integers
 * (данные макросы необходимы для преобразования числел-констант с плавающей запятой
 * в целые числа. Значения физических величин хранятся в целых числах).
 */

/** Transforms floating point value of advance angle to fixed point value */
#define ANGLE_MAGNITUDE(a) ROUND ((a) * ANGLE_MULTIPLAYER)

/** Transforms floating point value of temperature to fixed point value */
#define TEMPERATURE_MAGNITUDE(t) ROUND ((t) * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER)

/** Transforms floating point value of voltage to fixed point value */
#define VOLTAGE_MAGNITUDE(t) ROUND ((t) * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER)

/** Transforms floating point value of pressure(MAP) to fixed point value */
#define PRESSURE_MAGNITUDE(t) ROUND ((t) * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER)

/** Transforms floating point value of percentage of opening (TPS) to fixed point value */
#define TPS_MAGNITUDE(t) ROUND ((t) * TPS_PHYSICAL_MAGNITUDE_MULTIPLAYER)

#endif //_MAGNITUDE_H_
