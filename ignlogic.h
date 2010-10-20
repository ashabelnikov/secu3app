
#ifndef _IGNLOGIC_H_
#define _IGNLOGIC_H_

#include <stdint.h>

struct ecudata_t;

//режимы двигателя
#define EM_START 0   
#define EM_IDLE  1
#define EM_WORK  2

//конечный автомат режимов двигателя
void advance_angle_state_machine(int16_t* padvance_angle_inhibitor_state, struct ecudata_t* d);

#endif //_IGNLOGIC_H_
