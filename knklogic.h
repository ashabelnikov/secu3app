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
