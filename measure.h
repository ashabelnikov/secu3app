
#ifndef _MEASURE_H_
#define _MEASURE_H_

struct ecudata;

void meas_update_values_buffers(struct ecudata* d);
void meas_average_measured_values(struct ecudata* d);
void meas_initial_measure(struct ecudata* d);

//производит считывание дискретных входов системы и переключение 
//типа топлива (набор таблиц).
void meas_take_discrete_inputs(struct ecudata *d);

#endif //_MEASURE_H_
