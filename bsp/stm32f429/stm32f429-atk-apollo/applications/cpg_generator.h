#ifndef _CPG_GENERATOR_H
#define _CPG_GENERATOR_H
//CPG发生器 
#include "math.h"
#define pi 3.1415926

struct Function
{
		float x;
		float y;
};
typedef struct Function function;


//函数变量 设计6个Hopf振子 
struct Function_parameters  
{			
		float x;
		float y;				
};

typedef struct Function_parameters function_parameters;


struct desired_vals_container{
  float global_theta[6];
  short global_velocity[6];
};
typedef struct desired_vals_container vals;


struct CPG_XY{
	float x[6];
	float y[6];
};
typedef struct CPG_XY cpg_xy;


struct Leg_info{
  int turn_dirction;
	int swing_theta;
	int stance_theta;
};
typedef struct Leg_info leg_info;

cpg_xy function_coupling(cpg_xy values,float step);

function function_return(float x,float y);  //微分方程定义
//int Eular(funtion init, float a, float b, float h, float *result_x, float *result_y);//采用欧拉法解微分方程


//参数1 x,y的值 ；参数2 x,y的耦合值；参数3 积分步长
function_parameters hopf_function_return(function_parameters xy_value,function_parameters xy_coupling_value,float h,int i,int j);

int Eular(float a, float b, float h,function xy_value,float *result_x,float *result_y);

#endif
