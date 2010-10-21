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
              http://secu-3.narod.ru
              email: secu-3@yandex.ru
*/

#define SETBIT(x,y)   ((x) |= (1<<(y)))    /* установка бита y в байте x*/
#define CLEARBIT(x,y) ((x) &= (~(1<<(y)))) /* сброс бита y в байте x*/
#define CHECKBIT(x,y) ((x) & (1<<(y)))     /* проверка бита y в байте x*/

#ifdef LITTLE_ENDIAN_DATA_FORMAT //little-endian data store format (Intel)
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+(rel)))
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+(rel)))
#else                           //big-endian data store format (Motorola) 
  #define GETBYTE(src,rel) *(((unsigned char*)&(src)+sizeof((src))-1-(rel)))
  #define SETBYTE(des,rel) *(((unsigned char*)&(des)+sizeof((des))-1-(rel)))
#endif  

