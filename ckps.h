#ifndef _CKPS_H_
#define _CKPS_H_

//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то 
//напротив середины сердечника ДПКВ должен находиться зуб диска синхронизации определенный ниже (считаем против 
//направления вращения от места выреза).
#define CKPS_COGS_BEFORE_TDC         20                          //количество зубьев после вызеза до в.м.т (18...22)
#define CKPS_DEGREES_PER_COG         6                           //количество градусов приходящееся на один зуб диска
//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в меньшую сторону 
//возможен выход коммутатора из строя.  Если соединять два выхода вместе для одного коммутатора, то необходимо ставить
//значение 10, если двухканальный режим то 40.
#define CKPS_IGNITION_PULSE_COGS     10                          //длительность импульса зажигания (в зубьях шкива)

//количество зубов которое будет пропускатся при старте перед синхронизацией
#define CKPS_ON_START_SKIP_COGS      30

//коэффициент масштабирования углов поворота коленвала, фигурирует в вычислениях и операциях деления
//поэтому он должен быть кратен степени 2
#define ANGLE_MULTIPLAYER            32                           


void ckps_init_state(void);
void ckps_set_edge_type(unsigned char edge_type);
void ckps_set_ignition_cogs(unsigned char cogs);

void ckps_set_dwell_angle(signed int angle);
unsigned int ckps_calculate_instant_freq(void);

unsigned char ckps_is_error(void);
void ckps_reset_error(void);
unsigned char ckps_is_cycle_cutover_r(void);
unsigned char ckps_is_rotation_cutover_r(void);

#endif //_CKPS_H_
