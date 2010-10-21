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

#include <ioavr.h>
#include <inavr.h>
#include "vstimer.h"
#include "secu3.h"

#define TIMER2_RELOAD_VALUE          100                         //для 10 мс

volatile s_timer8_t  send_packet_interval_counter = 0;
volatile s_timer8_t  force_measure_timeout_counter = 0;
volatile s_timer8_t  ce_control_time_counter = CE_CONTROL_STATE_TIME_VALUE;
volatile s_timer8_t  engine_rotation_timeout_counter = 0;
volatile s_timer8_t  epxx_delay_time_counter = 0;
volatile s_timer8_t  idle_period_time_counter = 0;
volatile s_timer16_t save_param_timeout_counter = 0;

//прерывание по переполению Т/С 2 - для отсчета временных интервалов в системе (для общего использования). 
//Вызывается каждые 10мс
#pragma vector=TIMER2_OVF_vect
__interrupt void timer2_ovf_isr(void)
{ 
 TCNT2 = TIMER2_RELOAD_VALUE; 
 __enable_interrupt();     
    
 s_timer_update(force_measure_timeout_counter);
 s_timer_update(save_param_timeout_counter);
 s_timer_update(send_packet_interval_counter);  
 s_timer_update(ce_control_time_counter);
 s_timer_update(engine_rotation_timeout_counter);   
 s_timer_update(epxx_delay_time_counter);
 s_timer_update(idle_period_time_counter);  
}

void s_timer_init(void)
{
 TCCR2|= (1<<CS22)|(1<<CS21)|(1<<CS20);      //clock = 15.625kHz  
 TIMSK|= (1<<TOIE2); //разрешаем прерывание по переполнению таймера 2                          
}
