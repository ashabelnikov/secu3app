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

//Turns on/off ventilator
#define SET_VENTILATOR_STATE(s) {PORTB_Bit1 = s;}

//Warning must be the same as another definition in vstimer.h!
#define TIMER2_RELOAD_VALUE  6
#define COMPADD 5

//number of PWM discretes
#define PWM_STEPS 25

volatile uint8_t pwm_state; //0 - passive, 1 - active
volatile uint8_t pwm_duty;

void vent_init_ports(void)
{
 //configure used I/O ports
 PORTB&= ~(1<<PB1);
 DDRB |= (1<<DDB1);
}

void vent_init_state(void)
{
 pwm_state = 0;  //begin from active level
 pwm_duty = 0;   // 0%
 OCR2 = TIMER2_RELOAD_VALUE + COMPADD;
}

void vent_set_duty(uint8_t duty)
{
 //TODO: Maybe we need double buffering?
 pwm_duty = duty;

 //We don't need interrupts if duty is 0 or 100%
 if (duty == 0)
 {
  TIMSK&=~(1 << OCIE2);
  SET_VENTILATOR_STATE(0);
 }
 else if (duty == PWM_STEPS)
 {
  TIMSK&=~(1 << OCIE2);
  SET_VENTILATOR_STATE(1);
 }
 else
  TIMSK|=(1 << OCIE2);
}

//T/C 2 Compare interrupt for renerating PWM (ventilator control)
#pragma vector=TIMER2_COMP_vect
__interrupt void timer2_comp_isr(void)
{ 
 if (0 == pwm_state)
 { //start active part
  SET_VENTILATOR_STATE(1);
  OCR2+= pwm_duty;
  ++pwm_state;
 }
 else
 { //start passive part
  SET_VENTILATOR_STATE(0);
  OCR2+= (PWM_STEPS - pwm_duty);
  --pwm_state;   
 }        
}

//Control electro ventilator (engine cooling), only in case if coolant temperature 
//sensor is present in system
void vent_control(struct ecudata_t *d)
{
 if (!d->param.tmp_use)
  return;
 
#ifndef VENTILATOR_PWM
 if (d->sens.temperat >= d->param.vent_on)
  SET_VENTILATOR_STATE(1);
 if (d->sens.temperat <= d->param.vent_off)
  SET_VENTILATOR_STATE(0);
#else //PWM mode
//  if (!d->param.use_ventilator_pwm)
//  {
//     if (d->sens.temperat >= d->param.vent_on)
//      vent_set_duty(PWM_STEPS);
//     if (d->sens.temperat <= d->param.vent_off)
//      vent_set_duty(0);
//  }
//  else
//  {
//  }

  vent_set_duty((d->param.starter_off - 500)/10);
  
#endif
}
