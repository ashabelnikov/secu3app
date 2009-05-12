
#ifndef _MEASURE_H_
#define _MEASURE_H_

#include "secu3.h"

void meas_update_values_buffers(ecudata* d);
void meas_average_measured_values(ecudata* d);
void meas_initial_measure(ecudata* d);

//производит считывание дискретных входов системы и переключение 
//типа топлива (набор таблиц).
void meas_take_discrete_inputs(ecudata *d);

#endif //_MEASURE_H_
