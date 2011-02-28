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

#ifndef _MAGNITUDE_H_
#define _MAGNITUDE_H_

//необходим для округления при преобразовании из числа с плавающей точкой
//в целое число
//Used for rounding-up when transforming from floating point value into integer.
//Note: it is intended for use with constants
#define ROUND(x) ((int16_t)( (x) + 0.5 - ((x) < 0) ))

//данные макросы необходимы для преобразования числел-констант с плавающей запятой
//в целые числа. Значения физических величин хранятся в целых числах.
//Given macros are necessary when transforming floating point constant-values into integers.
//Values of phisical magnitudes stored in integers.
#define ANGLE_MAGNITUDE(a) ROUND ((a) * ANGLE_MULTIPLAYER)
#define TEMPERATURE_MAGNITUDE(t) ROUND ((t) * TEMP_PHYSICAL_MAGNITUDE_MULTIPLAYER)
#define VOLTAGE_MAGNITUDE(t) ROUND ((t) * UBAT_PHYSICAL_MAGNITUDE_MULTIPLAYER)
#define PRESSURE_MAGNITUDE(t) ROUND ((t) * MAP_PHYSICAL_MAGNITUDE_MULTIPLAYER)

#endif //_MAGNITUDE_H_
