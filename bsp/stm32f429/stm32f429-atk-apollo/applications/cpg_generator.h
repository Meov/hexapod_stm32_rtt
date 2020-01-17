#ifndef _CPG_GENERATOR_H
#define _CPG_GENERATOR_H
//CPG·¢ÉúÆ÷ 
#include "math.h"
#include "rtdef.h"
#define pi 3


typedef struct CPG_XY{
	rt_int16_t x[6];
	rt_int16_t y[6];
}cpg_xy;


struct Leg_info{
  int turn_dirction;
	int swing_theta;
	int stance_theta;
};
typedef struct Leg_info leg_info;
void cpg_generator_init(void);
cpg_xy get_cpg_data(void);

#endif
