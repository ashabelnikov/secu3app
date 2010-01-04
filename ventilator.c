 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <iom16.h>
#include "ventilator.h"
#include "secu3.h"

//включает/выключает вентилятор
#define SET_VENTILATOR_STATE(s) {PORTB_Bit1 = s;}

void vent_init_ports(void)
{
 //конфигурируем порты ввода/вывода       
 DDRB |= (1<<DDB1);   
 PORTB&= ~(1<<PB1);
}

void vent_control(ecudata *d)
{
 //управление электро вентилятором охлаждения двигателя, при условии что ДТОЖ присутствует в системе 
 if (d->param.tmp_use)
 {
  if (d->sens.temperat >= d->param.vent_on)
   SET_VENTILATOR_STATE(1);
  if (d->sens.temperat <= d->param.vent_off)   
   SET_VENTILATOR_STATE(0); 
  }  
}
