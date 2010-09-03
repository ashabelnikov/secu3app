
#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include <stdint.h>
#include "vstimer.h"

int16_t simple_interpolation(int16_t x,int16_t a1,int16_t a2,int16_t x_s,int16_t x_l);
int16_t bilinear_interpolation(int16_t x,int16_t y,int16_t a1,int16_t a2,int16_t a3,int16_t a4,int16_t x_s,int16_t y_s,int16_t x_l,int16_t y_l);

struct ecudata;

int16_t start_function(struct ecudata* d);
int16_t idling_function(struct ecudata* d);
int16_t work_function(struct ecudata* d, uint8_t i_update_airflow_only);
int16_t coolant_function(struct ecudata* d);
uint8_t knock_attenuator_function(struct ecudata* d);
void idling_regulator_init(void);
int16_t idling_pregulator(struct ecudata* d, volatile s_timer8* io_timer);
int16_t advance_angle_inhibitor(int16_t new_advance_angle, int16_t* ip_prev_state, int16_t intstep_p, int16_t intstep_m);
void restrict_value_to(int16_t *io_value, int16_t i_bottom_limit, int16_t i_top_limit);

#endif //_FUNCONV_H_



