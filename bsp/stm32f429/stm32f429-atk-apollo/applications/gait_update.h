#ifndef _GAIT_UPDATE_H
#define _GAIT_UPDATE_H


#include "usart.h"
#include "CpgGenerator.h"
#include <stdbool.h>
#include <string.h>
#include "timer.h"
#include "math.h"
#include "can.h"


#define THETA_CIRCLE 360

extern float omega_set;
extern float beta_set;
extern float theta[6][6];
extern float k_value[6][6];


void cpg_test(void);
void gait_mapping(void);

void get_cmd_from_pc(void);
void update_gait(void);
void gait_choose(int gait_to_set);
float cpg_to_angle(float cpg_position);

#endif
