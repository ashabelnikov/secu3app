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

/** \file uni_out.h
 * \author Alexey A. Shabelnikov
 * A universal programmable output.
 */

#ifndef _UNI_OUT_H_
#define _UNI_OUT_H_

#ifdef UNI_OUTPUT

#include <stdint.h>

struct ecudata_t;

/**Initialization of used I/O lines*/
void uniout_init_ports(void);

/**Initialization of the module*/
void uniout_init(void);

/** Does control of universal programmable output
 * \param d Pointer to ECU data structure
 */
void uniout_control(struct ecudata_t *d);

#endif //UNI_OUTPUT

#endif //_UNI_OUT_H_
