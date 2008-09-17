#ifndef _CKPS_H_
#define _CKPS_H_

//количество градусов приходящееся на один зуб диска
#define CKPS_DEGREES_PER_COG         6                           

//количество зубов которое будет пропускатся при старте перед синхронизацией
#define CKPS_ON_START_SKIP_COGS      30

//коэффициент масштабирования углов поворота коленвала, фигурирует в вычислениях и операциях деления
//поэтому он должен быть кратен степени 2
#define ANGLE_MULTIPLAYER            32                           

#define ANGLE_MAGNITUDE(a) ((a) * ANGLE_MULTIPLAYER)

void ckps_init_state(void);

//Тип фронта ДПКВ
// 0 - отрицательный, 1 - положительный
void ckps_set_edge_type(unsigned char edge_type);

//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то 
//по стандарту напротив середины сердечника ДПКВ должен находиться 20-й зуб диска синхронизации (считаем против 
//направления вращения от места выреза). Допустимые значения: 18,19,20,21,22.
void ckps_set_cogs_btdc(unsigned char cogs_btdc);

//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в меньшую сторону 
//возможен выход коммутатора из строя.  Если соединять два выхода вместе для одного коммутатора, то необходимо ставить
//значение 10, если двухканальный режим то 40.
void ckps_set_ignition_cogs(unsigned char cogs);

void ckps_set_dwell_angle(signed int angle);
unsigned int ckps_calculate_instant_freq(void);

unsigned char ckps_is_error(void);
void ckps_reset_error(void);
unsigned char ckps_is_cycle_cutover_r(void);
void ckps_init_state_variables(void);

//возвращает номер текущего зуба
unsigned char ckps_get_current_cog(void);

//возвращает 1, если номер текущего зуба изменился.
unsigned char ckps_is_cog_changed(void);


#endif //_CKPS_H_
