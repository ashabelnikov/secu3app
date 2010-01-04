
#ifndef _VENTILATOR_H_
#define _VENTILATOR_H_

struct ecudata;

//инициализация используемых портов
void vent_init_ports(void);

//управление вентилятором охлаждения двигателя
void vent_control(struct ecudata *d);

#endif //_VENTILATOR_H_
