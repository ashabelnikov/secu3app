 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <inavr.h>
#include <iom16.h>
#include "ventilator.h"
#include "secu3.h"

//включает/выключает вентилятор
#define SET_VENTILATOR_STATE(s) PORTB_Bit1 = (s)

//number of PWM discretes
#define PWM_STEPS 10

//MUST be same as in vstimer!
#define TIMER2_RELOAD_VALUE 100

volatile uint8_t pwm_state; //0 - passive, 1 - active
volatile uint8_t pwm_duty;

void vent_init_ports(void)
{
 //конфигурируем порты ввода/вывода       
 DDRB |= (1<<DDB1);   
 PORTB&= ~(1<<PB1);
}

void vent_init_state(void)
{ 
 pwm_state = 0;
 pwm_duty = 0; // 0%
 OCR2 = TIMER2_RELOAD_VALUE + 5; 
 TIMSK|=(1 << OCIE2);
}

void vent_set_duty(uint8_t duty)
{
 pwm_duty = duty;
 
 //We don't need interrupts if duty is 0 or 100%
 /*if (duty == 0)
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
  TIMSK|=(1 << OCIE2);  */
}

//прерывание по сравненю Т/С 2 - для генерации ШИМ
#pragma vector=TIMER2_COMP_vect
__interrupt void timer2_comp_isr(void)
{ 
  if (0==pwm_state)
  { //start active part
   SET_VENTILATOR_STATE(1);
   OCR2+=pwm_duty;
   ++pwm_state;
  }
  else
  { //start passive part
   SET_VENTILATOR_STATE(0);
   OCR2+=PWM_STEPS-pwm_duty;
   --pwm_state;   
  } 
  
  if (OCR2 < TIMER2_RELOAD_VALUE)
    OCR2+=TIMER2_RELOAD_VALUE;
}

void vent_control(ecudata *d)
{
 //управление электро вентилятором охлаждения двигателя, при условии что ДТОЖ присутствует в системе 
 /*if (d->param.tmp_use)
 {
  if (d->sens.temperat >= d->param.vent_on)
   SET_VENTILATOR_STATE(1);
  if (d->sens.temperat <= d->param.vent_off)   
   SET_VENTILATOR_STATE(0); 
  }*/
  
  vent_set_duty(5);  
}
