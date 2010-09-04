
#ifndef _EPHH_H_
#define _EPHH_H_

struct ecudata_t;

//инициализация используемых портов
void ephh_init_ports(void);

void ephh_control(struct ecudata_t* d);

#endif //_EPHH_H_
