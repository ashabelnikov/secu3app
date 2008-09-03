
#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include "secu3.h"

int simple_interpolation(int x,int a1,int a2,int x_s,int x_l);
int bilinear_interpolation(int x,int y,int a1,int a2,int a3,int a4,int x_s,int y_s,int x_l,int y_l);


int start_function(ecudata* d);
int idling_function(ecudata* d);
int work_function(ecudata* d);
int coolant_function(ecudata* d);
int idling_pregulator(ecudata* d);
int transient_state_integrator(int new_advance_angle, unsigned int intstep, char is_enabled);

#endif //_FUNCONV_H_



