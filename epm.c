 /****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/

#include <ioavr.h>
#include "secu3.h"
#include "epm.h"

//открывает/закрывает клапан Ёћ–
#define SET_EPM_VALVE_STATE(s) {PORTC_Bit7 = s;}

void epm_init_ports(void)
{
 PORTC&= ~(1<<PC7); //Ёћ– выключен
 DDRC|= (1<<DDC7);  //выход дл€ управлени€ клапаном Ёћ–
}

void epm_control(struct ecudata_t* d)
{
 int16_t discharge;
 
 discharge = (d->param.map_upper_pressure - d->sens.map);
 if (discharge < 0) 
  discharge = 0;    
 d->epm_valve = discharge < d->param.epm_on_threshold;
 SET_EPM_VALVE_STATE(d->epm_valve);
}
