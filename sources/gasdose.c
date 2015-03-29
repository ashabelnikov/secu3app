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

/** \file gasdose.c
 * \author Alexey A. Shabelnikov
 * Implementation of gas dose controller (stepper motor).
 */

#include "port/port.h"
#include "port/pgmspace.h"
#include "funconv.h"
#include "magnitude.h"

#define GASDOSE_POS_RPM_SIZE 16   //!< RPM axis size
#define GASDOSE_POS_TPS_SIZE 16   //!< TPS axis size
#define _GD(v) ROUND((v)*2.0)     //!< For encoding of gas dose actuator position value

/** Gas dose actuator position vs (TPS,RPM)
 */
PGM_DECLARE(uint8_t gasdose_pos[GASDOSE_POS_TPS_SIZE][GASDOSE_POS_RPM_SIZE]) =
{//  600       720        840       990      1170      1380     1650      1950      2310      2730       3210      3840      4530      5370      6360      7500 (min-1)
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //100%
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}, //
 {_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0),_GD(50.0)}  //0%
};

