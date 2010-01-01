
#ifndef _CKPS_H_
#define _CKPS_H_

#include <stdint.h>

//uncomment following line if you wish to use 36-1 wheel
/*#define WHEEL_36_1*/

//коэффициент масштабирования углов поворота коленвала, фигурирует в вычислениях и операциях деления
//поэтому он должен быть кратен степени 2
#define ANGLE_MULTIPLAYER            32                           

void ckps_init_state(void);

//Тип фронта ДПКВ
// 0 - отрицательный, 1 - положительный
void ckps_set_edge_type(uint8_t edge_type);

//Если коленчатый вал установлен в положение, соответствующее верхней мертвой точке(t.d.c.) поршня первого цилиндра, то 
//по стандарту напротив середины сердечника ДПКВ должен находиться 20-й зуб диска синхронизации (считаем против 
//направления вращения от места выреза). Допустимые значения: 18,19,20,21,22 (для 60-2 шкива), 10,11,12,13,14 (для 36-1 шкива).
void ckps_set_cogs_btdc(uint8_t cogs_btdc);

//для стандартных коммутаторов длительность импульса запуска должна быть 1/3, при значительном отклонении в меньшую сторону 
//возможен выход коммутатора из строя.  Если соединять два выхода вместе для одного коммутатора, то необходимо ставить
//значение 10, если двухканальный режим то 40. Значения указаны для шкива 60-2.
void ckps_set_ignition_cogs(uint8_t cogs);

void ckps_set_dwell_angle(int16_t angle);
uint16_t ckps_calculate_instant_freq(void);

//установка окна фазовой селекции детонации. Параметры begin, end в градусах относительно в.м.т.  
void ckps_set_knock_window(int16_t begin, int16_t end);

//устанавливает обслуживать или необслуживать канал детонации
void ckps_use_knock_channel(uint8_t use_knock_channel);

uint8_t ckps_is_error(void);
void ckps_reset_error(void);
uint8_t ckps_is_cycle_cutover_r(void);
void ckps_init_state_variables(void);

//возвращает номер текущего зуба
uint8_t ckps_get_current_cog(void);

//возвращает 1, если номер текущего зуба изменился.
uint8_t ckps_is_cog_changed(void);

//установка кол-ва цилиндров двигателя (четное число)
//допустимые значения: 2,4,6,8 
void ckps_set_cyl_number(uint8_t i_cyl_number);

//производит инициализацию линий портов
void ckps_init_ports(void);

#endif //_CKPS_H_
