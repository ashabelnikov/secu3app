/****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Kiev 2008.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/
#ifndef _VSTIMER_H_
#define _VSTIMER_H_

#include <stdint.h>

//инструментарий для реализации виртуальных таймерв

//Типы объектов таймеров. 8-ми разрядный тамйер может отсчитывать периоды
//до 2.56 сек. 16-ти разрядный таймер может отсчитывать периоды до 655 сек.
typedef uint8_t   s_timer8;
typedef uint16_t  s_timer16;

//обновление состояния указанного таймера
#define s_timer_update(T)    { if ((T) > 0) (T)--; }

//инициализация состояния указанного таймера. Один тик таймера равен 10 мс
#define s_timer_set(T, V)    { (T) = (V); }

//Проверяет сработал ли указанный таймер
#define s_timer_is_action(T) ((T)==0)

//Ниже, варианты функций для 16-ти разрядных виртуальных таймеров. 
//Так как для этих таймеров используется не атомарный тип данных, то 
//необходимо запрещать прерывания.

#define s_timer16_set(T, V)  \
{                            \
 __disable_interrupt();      \
 (T) = (V);                  \
 __enable_interrupt();       \
}
 
#pragma inline  //а в обычном "С" такого нет ;-), спасибо разработчикам компилятора.
__monitor uint8_t s_timer16_is_action(s_timer16 i_timer) 
{
 return (i_timer==0);
}

void s_timer_init(void);

//////////////////////////////////////////////////////////////////
extern volatile s_timer8  send_packet_interval_counter;
extern volatile s_timer8  force_measure_timeout_counter;
extern volatile s_timer8  ce_control_time_counter;
extern volatile s_timer8  engine_rotation_timeout_counter;
extern volatile s_timer8  epxx_delay_time_counter;
extern volatile s_timer8 idle_period_time_counter;
extern volatile s_timer16 save_param_timeout_counter;
//////////////////////////////////////////////////////////////////

#endif //_VSTIMER_H_
