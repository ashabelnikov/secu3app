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

/** \file uni_out.c
 * Implementation of a universal programmable output.
 */

#ifdef UNI_OUTPUT

#include "port/port.h"
#include "ioconfig.h"
#include "secu3.h"

void uniout_init_ports(void)
{
 IOCFG_INIT(IOP_UNI_OUT, 0);
}

void uniout_init(void)
{
}

void uniout_control(struct ecudata_t *d)
{
 if (!IOCFG_CHECK(IOP_UNI_OUT))
  return;
}

#endif //UNI_OUTPUT
