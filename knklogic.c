/****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/
#include <inavr.h>
#include <ioavr.h>
#include "knklogic.h"
#include "secu3.h"
#include "funconv.h"

uint8_t knklogic_detect(ecudata* d, retard_state_t* p_rs)
{
 p_rs->knock_flag = (d->sens.knock_k > d->param.knock_threshold);
 return p_rs->knock_flag;
}

void knklogic_init(retard_state_t* p_rs)
{
 p_rs->delay_counter = 0;
 p_rs->knock_flag = 0; 
}

void knklogic_retard(ecudata* d, retard_state_t* p_rs)
{
 if (p_rs->delay_counter != 0)     
  p_rs->delay_counter--;     
 else
 {          
  if (p_rs->knock_flag)
  { //есть детонация         
   d->knock_retard+= d->param.knock_retard_step;//retard
   p_rs->knock_flag = 0;
  }
  else
  {//нет детонации
   d->knock_retard-= d->param.knock_advance_step;//advance
  } 
  restrict_value_to(&d->knock_retard, 0, d->param.knock_max_retard);
  
  p_rs->delay_counter = d->param.knock_recovery_delay;   
  if (0!=(p_rs->delay_counter)) 
    --(p_rs->delay_counter);
 }     
}
