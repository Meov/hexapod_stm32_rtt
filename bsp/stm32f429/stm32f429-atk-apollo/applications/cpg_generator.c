#include "cpg_generator.h"
#include "math.h"
#include <rtthread.h>
#include "cmd_parse.h"

static rt_int16_t omega_set = 20;
static rt_int16_t beta_set = 50;
static rt_int16_t omega_to_set = 10;//将要赋给omega的值
static rt_int16_t beta_to_set = 10;//将要给beta赋值

static rt_tick_t time_out_value = 200;
static cpg_xy cpg_values;	
cpg_xy xy_value = {{1,1,1,1,1,1},{-1,-1,-1,-1,-1,-1}};

float theta[6][6] = {
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
					};

float k_value[6][6] = 
					{
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,
						0,0,0,0,0,0,				
					};
									


static cpg_xy function_coupling(cpg_xy values, float_t step)
{
		rt_int16_t dx[6] = {0,0,0,0,0,0};
		rt_int16_t dy[6] = {0,0,0,0,0,0};
		rt_int16_t w_swing = 0;
		rt_int16_t beta = 5;
		rt_int16_t full_beta = 10;
		rt_int16_t u = 1;
		rt_int16_t omega[6] = {0};
		rt_int16_t b = 10;
		rt_int16_t rho = 10;
		rt_int16_t sigma = 10;
		rt_int16_t i = 0;
		w_swing = omega_set*3*pi;
		beta = beta_set;
		
		omega[0] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[0]) + full_beta) + w_swing / (exp(b*values.y[0]) + full_beta);
		omega[1] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[1]) + full_beta) + w_swing / (exp(b*values.y[1]) + full_beta);
		omega[2] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[2]) + full_beta) + w_swing / (exp(b*values.y[2]) + full_beta);
		omega[3] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[3]) + full_beta) + w_swing / (exp(b*values.y[3]) + full_beta);
		omega[4] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[4]) + full_beta) + w_swing / (exp(b*values.y[4]) + full_beta);
		omega[5] = (((full_beta - beta) / beta)*w_swing) / (exp(-b*values.y[5]) + full_beta) + w_swing / (exp(b*values.y[5]) + full_beta);
		
		for(i=0;i<6;i++)
		{
			dx[i] = rho*(u - values.x[i]*values.x[i] - values.y[i]*values.y[i])*values.x[i] - omega[i]*values.y[i] //函数自身
												+ k_value[i][0]*(cos(theta[i][0])*values.x[0] - sin(theta[i][0])*values.y[0])
												+ k_value[i][1]*(cos(theta[i][1])*values.x[1] - sin(theta[i][1])*values.y[1])
												+ k_value[i][2]*(cos(theta[i][2])*values.x[2] - sin(theta[i][2])*values.y[2])
												+ k_value[i][3]*(cos(theta[i][3])*values.x[3] - sin(theta[i][3])*values.y[3])
												+ k_value[i][4]*(cos(theta[i][4])*values.x[4] - sin(theta[i][4])*values.y[4])
												+ k_value[i][5]*(cos(theta[i][5])*values.x[5] - sin(theta[i][5])*values.y[5]);
			dy[i] = sigma*(u - values.x[i]*values.x[i] - values.y[i]*values.y[i])*values.y[i] + omega[i]*values.x[i]//函数自身
												+ k_value[i][0]*(sin(theta[i][0])*values.x[0] + cos(theta[i][0])*values.y[0])
												+ k_value[i][1]*(sin(theta[i][1])*values.x[1] + cos(theta[i][1])*values.y[1])
												+ k_value[i][2]*(sin(theta[i][2])*values.x[2] + cos(theta[i][2])*values.y[2])
												+ k_value[i][3]*(sin(theta[i][3])*values.x[3] + cos(theta[i][3])*values.y[3])
												+ k_value[i][4]*(sin(theta[i][4])*values.x[4] + cos(theta[i][4])*values.y[4])
												+ k_value[i][5]*(sin(theta[i][5])*values.x[5] + cos(theta[i][5])*values.y[5]); 
		
			values.x[i] = values.x[i] + dx[i] * step;
			values.y[i] = values.y[i] + dy[i] * step;
		}		
		return values;
}

static void cpg_monitor_thread_entry(void* parameter){
	
//	while(1){
//		//检测CPG的参数是否发生变化
//		
//		
//		
//	}

}


//定时器20ms进入一次 更新CPG的值
static void cpg_update(){
	
	//xy_value = function_coupling(xy_value,1);
	//cpg_values = xy_value;
	//rt_kprintf("%d\n",xy_value.x[0]);
	
}



cpg_xy get_cpg_data(){

	return cpg_values;

}



void cpg_generator_init(){
	rt_err_t result;
	static rt_timer_t timer1;
	
	rt_thread_t cpg_monitor_thread;
	
	cpg_monitor_thread = rt_thread_create("cpg",
											cpg_monitor_thread_entry,
	                                        RT_NULL,
											1024,
											 8,
	                                         5);
	result = rt_thread_startup(cpg_monitor_thread);

	if(result != RT_EOK)
	{
		
		rt_kprintf("created cpg generator thread failed \n");
		
	}
	timer1 = rt_timer_create("timer1",
                             cpg_update,
                             RT_NULL,
                             time_out_value,  //20ms定时器超时
                             RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER
                            );
    if (timer1 != RT_NULL) 
        rt_timer_start(timer1);
}



