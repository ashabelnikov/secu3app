
#ifndef _FUNCONV_H_
#define _FUNCONV_H_

#include "secu3.h"

int func_i2d(int n,int a1,int a2,int n_s,int n_l);
int func_i3d(int n,int p,int a1,int a2,int a3,int a4,int n_s,int p_s,int n_l,int p_l);


int str_func(ecudata* d);
int idl_func(ecudata* d);
int wrk_func(ecudata* d);
int tmp_func(ecudata* d);
int idl_pregul(ecudata* d);

#endif //_FUNCONV_H_



