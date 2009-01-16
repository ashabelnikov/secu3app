
#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include "secu3.h"
#include "vstimer.h"

int simple_interpolation(int x,int a1,int a2,int x_s,int x_l);
int bilinear_interpolation(int x,int y,int a1,int a2,int a3,int a4,int x_s,int y_s,int x_l,int y_l);


int start_function(ecudata* d);
int idling_function(ecudata* d);
int work_function(ecudata* d, char i_update_airflow_only);
int coolant_function(ecudata* d);
unsigned char knock_attenuator_function(ecudata* d);
void idling_regulator_init(void);
int idling_pregulator(ecudata* d, s_timer8* io_timer);
int advance_angle_inhibitor(int new_advance_angle, int* ip_prev_state, signed int intstep_p, signed int intstep_m);
void restrict_value_to(int *io_value, int i_bottom_limit, int i_top_limit);

#endif //_FUNCONV_H_



