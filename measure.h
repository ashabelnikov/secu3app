
#ifndef _MEASURE_H_
#define _MEASURE_H_

struct ecudata_t;

void meas_update_values_buffers(struct ecudata_t* d);
void meas_average_measured_values(struct ecudata_t* d);
void meas_initial_measure(struct ecudata_t* d);

//производит считывание дискретных входов системы и переключение 
//типа топлива (набор таблиц).
void meas_take_discrete_inputs(struct ecudata_t *d);

#endif //_MEASURE_H_
