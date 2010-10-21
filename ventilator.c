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
#include "ventilator.h"
#include "secu3.h"

//включает/выключает вентилятор
#define SET_VENTILATOR_STATE(s) {PORTB_Bit1 = s;}


/*
//number of PWM discretes
#define PWM_STEPS 10

//MUST be same as in vstimer!
#define TIMER2_RELOAD_VALUE 100

volatile uint8_t pwm_state; //0 - passive, 1 - active
volatile uint8_t pwm_duty;
*/

void vent_init_ports(void)
{
 //конфигурируем порты ввода/вывода       
 PORTB&= ~(1<<PB1);
 DDRB |= (1<<DDB1);   
}

void vent_init_state(void)
{ 
/* pwm_state = 0;
 pwm_duty = 0; // 0%
 OCR2 = TIMER2_RELOAD_VALUE + 5; 
 TIMSK|=(1 << OCIE2);*/
}

/*
void vent_set_duty(uint8_t duty)
{
 pwm_duty = duty;
 
 //We don't need interrupts if duty is 0 or 100%
// if (duty == 0)
// {
//  TIMSK&=~(1 << OCIE2);
//  SET_VENTILATOR_STATE(0);
// }
// else if (duty == PWM_STEPS)
// {
//  TIMSK&=~(1 << OCIE2);
//  SET_VENTILATOR_STATE(1);
// }
// else
//  TIMSK|=(1 << OCIE2);  
}

//прерывание по сравненю Т/С 2 - для генерации ШИМ
#pragma vector=TIMER2_COMP_vect
__interrupt void timer2_comp_isr(void)
{ 
  uint8_t r = OCR2;
  if (0==pwm_state)
  { //start active part
   SET_VENTILATOR_STATE(1);
   r+=pwm_duty;
   ++pwm_state;
  }
  else
  { //start passive part
   SET_VENTILATOR_STATE(0);
   r+=PWM_STEPS-pwm_duty;
   --pwm_state;   
  } 
  
  if (r < TIMER2_RELOAD_VALUE)
    r+=TIMER2_RELOAD_VALUE;
    
  OCR2 = r;
}
*/

void vent_control(struct ecudata_t *d)
{
 //управление электро вентилятором охлаждения двигателя, при условии что ДТОЖ присутствует в системе 
 if (d->param.tmp_use)
 {
  if (d->sens.temperat >= d->param.vent_on)
   SET_VENTILATOR_STATE(1);
  if (d->sens.temperat <= d->param.vent_off)   
   SET_VENTILATOR_STATE(0); 
  }

 /* vent_set_duty(5);    */
}
