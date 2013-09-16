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
#include "bitmask.h"
#include "crc16.h"
#include "onewire.h"
#include "secu3.h"

#define IBTN_KEYS_NUM     2   //!< Number of iButton keys
#define IBTN_KEY_SIZE     6   //!< Size of iButton key (except CRC8 and family code)

/**iButton keys database. Write out your own 48-bit key here */
uint8_t ibtn_keys[IBTN_KEYS_NUM][IBTN_KEY_SIZE] = {{0xCB,0x60,0x82,0x16, 0, 0}, {0xB7, 0x83, 0xA8, 0, 0, 0}};

/** Key validation function
 * \param key Pointer to an array containing key
 * \param size Size of key in array
 * \return 1 - specified key is valid, 0 - not valid
 */
static uint8_t validate_key(uint8_t* key, uint8_t size)
{
 uint8_t i = 0;
 for(; i < IBTN_KEYS_NUM; ++i)
 {
  if (!memcmp(key, ibtn_keys[i], size))
   return 1; //valid key
 }
 return 0; //key is not valid
}


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

 //validate CRC8, all bytes except CRC8 byte
 for(i = 0; i < 7; ++i) crc = update_crc8(key[i], crc);

 if (crc != key[7])
  goto lock_system;    //crc doesn't match, lock the system!

 //validate read key, skip family code and CRC8 bytes
 if (!validate_key(key+1, IBTN_KEY_SIZE))
  goto lock_system;    //read and stored keys don't match, lock the system!

 onewire_restore_io_registers();
 return; //ok, system is unlocked

lock_system:
 onewire_restore_io_registers();
 d->sys_locked = 1;    //set locking flag
}
