
#ifndef _EPHH_H_
#define _EPHH_H_

struct ecudata;

//инициализация используемых портов
void ephh_init_ports(void);

void ephh_control(ecudata* d);

#endif //_EPHH_H_
