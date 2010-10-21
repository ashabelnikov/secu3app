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

#ifndef _KNKLOGIC_H_
#define _KNKLOGIC_H_

#include <stdint.h>

//Модуль содержащий всю логику регулирования УОЗ по детонации

typedef struct retard_state_t
{
 uint8_t delay_counter; 
 uint8_t knock_flag;
}retard_state_t;

struct ecudata_t;

//Возвращает: 0 - нет детонации, 1 - есть 
uint8_t knklogic_detect(struct ecudata_t* d, retard_state_t* p_rs);

//инициализация переменных состояния
void knklogic_init(retard_state_t* p_rs);

//вызывается в каждом рабочем цикле
void knklogic_retard(struct ecudata_t* d, retard_state_t* p_rs);

#endif //_KNKLOGIC_H_
