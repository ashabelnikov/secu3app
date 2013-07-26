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

/** \file immobiliz.c
 * Implementation of immobilizer logic
 * (Реализация логики иммобилайзера).
 */

#include "port/port.h"
#include <string.h>
#include "crc16.h"
#include "onewire.h"
#include "secu3.h"

uint8_t ibtn_key[6] = {0,0,0,0,0,0}; //<--write out your 48-bit key here

void immob_check_state(struct ecudata_t* d)
{
 uint8_t i = 0, crc = 0;
 uint8_t key[8];
 if (!(d->param.bt_flags & _BV(BTF_USE_IMM)))
  return; //immibilizer was not activated

 onewire_save_io_registers();

 if (!onewire_reset())
  goto lock_system;    //not device present, lock the system!

 //Read 64-bit key
 onewire_write_byte(OWCMD_READ_ROM);
 for(; i < 8; ++i) key[i] = onewire_read_byte();

 //validate CRC8
 for(i = 0; i < 7; ++i) crc = update_crc8(key[i], crc);
 if (crc != key[7])
  goto lock_system;    //crc doesn't match, lock the system!

 //validate read key, skip family code and CRC8 bytes
 if (memcmp(key+1, ibtn_key, 6))
  goto lock_system;    //read and stored keys don't match, lock the system!

 onewire_restore_io_registers();
 return; //ok, system is unlocked

lock_system:
 onewire_restore_io_registers();
 d->sys_locked = 1;    //set locking flag
}
