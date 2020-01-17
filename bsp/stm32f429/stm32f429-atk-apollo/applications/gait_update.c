#include "gait_update.h"

#ifdef old

extern float omega_set;
extern float beta_set;
extern vals desired_values;
extern unsigned long Real_Position_Value[6];
int i = 0;


const int FORWARD = 1;  //正传
const int REVERSE = -1;	//反转
const int GAIT_INIT =  0;  //没什么卵用 用来占初始时刻的位置哒
const int TRIANGLE_FORWARD = 2;
const int TRIANGLE_BACKWARD = 3;
const int LEFT = 4;
const int RIGHT = 5;
const int STAIR = 6;
const int CLIMB = 7;

	
int turn_left[6] = {FORWARD,FORWARD,FORWARD,REVERSE,REVERSE,REVERSE};
int turn_right[6] = {REVERSE,REVERSE,REVERSE,FORWARD,FORWARD,FORWARD};		
int direction_reverse[6] = {REVERSE,REVERSE,REVERSE,REVERSE,REVERSE,REVERSE};
int direction_forward[6] = {FORWARD,FORWARD,FORWARD,FORWARD,FORWARD,FORWARD};  //三角步态	
unsigned int gait_mode[6] = {GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT};			  //初始时刻时三角步态模式
unsigned int gait_mode_last[6] = {GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT,GAIT_INIT};			
leg_info leg_info_set[6] = {0,0,0,0,0,0};
int direction_to_set[6] = {0,0,0,0,0,0};

//20190521 test
cpg_xy value_xy;

int direction_set[6] = {0,0,0,0,0,0};
		
bool turn_flag[6] = {false};	


//步态更新相关变量
function_parameters xy_value_new[6] = {0,0,0,0,0,0};
function_parameters coupling_xy_value[6] = {0,0,0,0,0,0};

float position[6] = {0,0,0,0,0,0};
float position_last[6] = {0,0,0,0,0,0};
short speed[6] = {0,0,0,0,0,0};
float position_delta[6] =  {0,0,0,0,0,0};
static unsigned long running_theta[6] = {0,0,0,0,0,0};
long theta_change[6] = {0,0,0,0,0,0};//运行间隔的电机实际位移量
unsigned long running_theta_last[6] = {0,0,0,0,0,0};
static float omega_last = 0;
float velecity_change_pro = 1;   //过度阶段速度变化比例
		
static unsigned long time_ms_last = 0;
extern unsigned long time_ms;

extern float omega_to_set;//将要赋给omega的值
extern float beta_to_set;//将要给beta赋值

long delta_time = 0;

int16_t speed_swing = 0;
int16_t speed_stance = 0;
bool state_swing[6] = {false,false,false,false,false,false};		
bool state_stance[6] = {false,false,false,false,false,false};		
unsigned int T_swing[6] = {0,0,0,0,0,0}; 
unsigned int T_stance[6] = {0,0,0,0,0,0}; 

static unsigned int T_swing_last[6] = {0,0,0,0,0,0};
static unsigned int T_stance_last[6] = {0,0,0,0,0,0};


float amplitude_up = 0;  //上升沿幅值
float amplitude_down = 0; //下降沿幅值
//转弯标志位 
bool is_change[6] = {false,false,false,false,false,false};		

bool gait_stair[6] = {false,false,false,false,false,false};	 //是否为爬台阶步态
int wait_flag[6] = {1,1,1,1,1,1};  //在竖直位置上等待步态变换
unsigned long gait_is_change_time[6] = {0,0,0,0,0,0};  //变换步态后的运行的次数  是为了获得第一次步态变换

unsigned int time_swing[6] = {0,0,0,0,0,0}; 
unsigned int time_stance[6] = {0,0,0,0,0,0};



//步态数据
//三角步态初始化
int legs_triangle_stance[6] = {45,45,45,45,45,45};
int legs_triangle_swing[6] = {315,315,315,315,315,315};

//爬坡步态初始化
int legs_climb_stance[6] = {0,0,0,0,0,0};
int legs_climb_swing[6] = {0,0,0,0,0,0};



int legs_stair_state_1[6] = {180,45,315,180,45,315};
int legs_stair_state_2[6] = {315,180,45,315,180,45};
int legs_stair_state_3[6] = {315,315,180,315,315,180};
int legs_stair_state_4[6] = {45,45,315,45,45,315};


int legs_climb_state_1[6] = {280,280,280,280,280,280};
int legs_climb_state_2[6] = {80,80,80,80,80,80};
int legs_climb_state_3[6] = {280,80,80,280,80,80};
int legs_climb_state_4[6] = {280,280,80,280,280,80};


int legs_stairx_state_1[6] = {60, 60, 300, 60,  60, 300};
int legs_stairx_state_2[6] = {300,180,300, 300, 180,300};
int legs_stairx_state_3[6] = {60, 225,0,   60,  225,0};
int legs_stairx_state_4[6] = {180,300,0,   180, 300,0};
int legs_stairx_state_5[6] = {270,270,45,  270, 300,45};
int legs_stairx_state_6[6] = {300,315,180, 300, 315,180};
int legs_stairx_state_7[6] = {45, 45 ,270, 300, 45, 270};
int legs_stairx_state_8[6] = {300,60, 300, 300, 60, 300};


int stair_speed[3] = {0,0,0}; //爬台阶步态速度匹配  前中后三组

int legs_zero[6] = { 0,0,0,0,0,0};
//int legs_stair_swing[6] = {270,270,270,270,270,270 };
static int gait_set_last = 0;

void cpg_test(void)  //产生6路耦合的CPG信号
{
	static unsigned long run_time = 0;	
	delta_time = time_ms - time_ms_last;
	time_ms_last = time_ms;	
	run_time++;
	value_xy = function_coupling(value_xy,delta_time/20000.1);
	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",run_time*0.114,value_xy.y[0],value_xy.y[1],value_xy.y[2],value_xy.y[3],value_xy.y[4],value_xy.y[5]);
	
}


void gait_mapping(void)
{	
	int16_t j = 0;
	static unsigned int T_2_up = 0;  //上升沿周期的一半
	static unsigned int T_2_down = 0;//下降沿周期的一半
	
	
	static unsigned int T_4_up = 0;  //上升沿周期的一半
	static unsigned int T_4_down = 0;//下降沿周期的一半
	
	
	
	
	
	
	
	bool gait_mode_change_location[6] = {false,false,false,false,false,false};  //表示步态切换位置
	for(i=0;i<6;i++)
	{
		
		j = i + 3;					
		if(i>2)
		{
				j = i - 3;											
		}	
		
		//if(gait_mode[i]==STAIR)
					//printf("stir %d\r\n",i);
		//printf("gait is %d\r\n",gait_mode[i]);
		
		position[i] = value_xy.y[i];
		position_delta[i] = position[i] - position_last[i];  //位置差  初始位置position并不是零  所以一开始 position_delta = 179 - 0 = 179
		position_last[i] = position[i];		
		if(position_delta[i]<0)   //下降沿  触地阶段
		{			

				//调速
				if(i == 0)   //以第0条腿作为参考，当第0条腿触地时进行omega赋值，同时给速度乘上速度变换的比例系数
				{
						omega_last = omega_set;
						omega_set = omega_to_set; //赋值给omega新的值														
						velecity_change_pro = omega_set/omega_last;
					//		printf("T_swing%d  is %d   amplitude_down:  %.2f\r\n",i,T_swing[i],amplitude_down);													
				}	

			
					//切换
					if((gait_mode_last[i] != gait_mode[i])||(is_change[i]))  //切换点
					{
						is_change[i] = true; //步态开始转变  什么时候进行转换？
						
						//printf("gait changing!\r\n");
						
						if((turn_flag[i])&&(abs(value_xy.y[i] - value_xy.y[j]) <= 0.02)) 		//一足与其耦合的足的位置差 来表明相位是否变换结束	转向 
						{
							leg_info_set[i].turn_dirction = direction_to_set[i];//切换点放在了这里 触地时刻的足反向悬空  悬空时刻的足反向触地 就开始转换方向
							is_change[i] = false; //转换完成
						}
						else if(!(turn_flag[i])&&(abs(value_xy.y[i]- value_xy.y[j]) >= 0.8)) //直行
						{
							leg_info_set[i].turn_dirction =  direction_to_set[i];
							is_change[i] = false;	
						}	
						else if((gait_mode[i]==STAIR)&&(abs(value_xy.y[i]- value_xy.y[j]) <= 0.02))  //如果是爬台阶步态 对侧足同步
						{ 
							leg_info_set[i].turn_dirction =  direction_to_set[i];
							is_change[i] = false;	
						}
	
					}
					
				gait_mode_last[i] = gait_mode[i];
	
				//running_theta[i] = -30*value_xy.y[i]+30;  //电机输出的位置  0----60度
				
				running_theta[i] = leg_info_set[i].stance_theta;	//40			
				
				if((Real_Position_Value[i]%32000) <= 10) //机器人此时足处于竖直位置附近
				{			
						//if(i==0)
							//printf("leg %d 到达切换点 目标值已经置零\r\n",i);	
						gait_mode_change_location[i] = true;									
				}
	
		/*
				
				if(time_stance[i]<T_4_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_1[i];      //爬台阶	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];       //爬坡步态模式
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式							
				}
				else if(T_4_up<time_stance[i]<T_2_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_2[i];      //爬台阶
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];      //爬坡步态模式
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式								 									
				
				
				}
				else if(T_2_up<time_stance[i]<T_2_up+T_4_up)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_3[i];      //爬台阶	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式						
				}
				else if(T_2_up+T_4_up<time_stance[i]<T_swing[i])
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_4[i];      //爬台阶	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式											
				}
				
	*/
				
				if(time_stance[i] < T_2_up)
				{
					
					if((gait_mode[i]==STAIR)&&(time_stance[i]<T_4_up))
						//running_theta[i] = legs_stair_state_1[i];      //爬太极步态模式
						running_theta[i] = legs_stairx_state_1[i];      //爬台阶	
					else if((gait_mode[i]==STAIR)&&(T_4_up<time_stance[i]<T_2_up))
						running_theta[i] = legs_stairx_state_2[i];      //爬台阶	
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_1[i];      //爬坡步态模式
					else 
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式								 									
				
				}
				else if(T_2_up<time_swing[i]<T_swing[i])
				{
					
					if((gait_mode[i]==STAIR)&&(T_2_up<time_stance[i]<T_2_up+T_4_up))
						running_theta[i] = legs_stairx_state_3[i];
					else if((gait_mode[i]==STAIR)&&(T_2_up+T_4_up<time_stance[i]<T_swing[i]))
						running_theta[i] = legs_stairx_state_4[i];      //爬台阶	
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_2[i];      //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].stance_theta;//三角步态模式								 									
				}

				
				
				if(state_swing[i])  //采集swing的时间
				{			
					T_swing[i] = time_swing[i];//获得下降沿的时间周期
					T_2_down = T_swing[i]/2;		//	获得下降沿时间的一半
					T_4_down = T_swing[i]/4;
				}
						
				if(T_stance[i]>10)
				{
					//speed_stance = 60/T_stance[i];  //触地的速度
					speed_stance = 95*running_theta[i]/T_stance[i];
					speed_stance  = speed_stance*velecity_change_pro;
				
				}
					
		
				state_swing[i] = false;
				state_stance[i] = true;										
				time_swing[i] = 0;  //下降时间置0 悬空时间置零				
				time_stance[i]++;				
				
			
		}
		else if(position_delta[i]>0)  //上升沿  悬空阶段
		{
		
				//running_theta[i] = 150*value_xy.y[i]+210; //电机输出的位置  60----360度
				
				running_theta[i] = leg_info_set[i].swing_theta;//										 							
			

				
				/*
				if(time_swing[i]<T_4_down)
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_5[i];     //爬太极步态模式
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];        //爬坡步态模式
					else  
						running_theta[i] = leg_info_set[i].swing_theta;  //三角步态模式									
				}
				else if(T_4_down<time_swing[i]<T_2_down)
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_6[i];      //爬太极步态模式
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];         //爬坡步态模式
					else 
						running_theta[i] = leg_info_set[i].swing_theta;   //三角步态模式					
				}
				else if(T_2_down<time_swing[i]<T_2_down+T_4_down)
				{
					if(gait_mode[i]==STAIR)
						running_theta[i] = legs_stairx_state_7[i];      //爬太极步态模式	
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];      //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].swing_theta;//三角步态模式						
				}else if(T_2_down+T_4_down<time_swing[i]<T_swing[i])
				{
					if(gait_mode[i]==STAIR)
							running_theta[i] = legs_stairx_state_8[i];     //爬太极步态模式
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];        //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].swing_theta;  //三角步态模式						
				}
							
				*/
				if(time_swing[i]<T_2_down)  //下降沿前半个周期
				{
					if((gait_mode[i]==STAIR)&&(time_swing[i]<T_4_down))
						running_theta[i] = legs_stairx_state_5[i];      //爬太极步态模式
					else if((gait_mode[i]==STAIR)&&(T_4_down<time_swing[i]<T_2_down))
						running_theta[i] = legs_stairx_state_6[i];      //爬太极步态模式
					
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_3[i];      //爬坡步态模式
					else 
						running_theta[i] = leg_info_set[i].swing_theta;//三角步态模式								 									
				
				}
				else if(T_2_down<time_swing[i]<T_swing[i]) //下降沿后半个周期
				{
					if((gait_mode[i]==STAIR)&&(T_2_down<time_swing[i]<T_2_down+T_4_down))
						running_theta[i] = legs_stairx_state_7[i];      //爬太极步态模式
					if((gait_mode[i]==STAIR)&&(T_2_down+T_4_down<time_swing[i]<T_swing[i]))
						running_theta[i] = legs_stairx_state_8[i];      //爬太极步态模式
					
					
					
					else if(gait_mode[i]==CLIMB)
						running_theta[i] = legs_climb_state_4[i];      //爬坡步态模式
					else
						running_theta[i] = leg_info_set[i].swing_theta;//三角步态模式								 									
				}
				


				if(state_stance[i])  //采集stance的时间
				{
					T_stance[i] = time_stance[i];	
					T_2_up = T_stance[i]/2;  //获得上升沿的周期的一半	
					T_4_up = T_stance[i]/4;
				}			
			
				if(T_swing[i]>10)
				{
					//speed_swing = 300/T_swing[i];  //悬空的速度
					speed_swing  = 70*running_theta[i]/T_swing[i]*1.0;  //360
					speed_swing  = speed_swing*velecity_change_pro;										
				}
								
				//if(i==0)
					//printf("T_swing is %d  speed swing is  %d\r\n",T_swing[i],speed_swing);
				
			
				state_swing[i] = true;
				state_stance[i] = false;	
				time_stance[i] = 0;	//下降时间置0 触地时间置零
				time_swing[i]++;		
		}
		
		
		
		theta_change[i]  = running_theta[i] - running_theta_last[i];  //刹停
		running_theta_last[i] = running_theta[i];  //电机位置的增量
	

	

		if(leg_info_set[i].turn_dirction == FORWARD) //正转时
		{
			
			
			//printf("forward state\r\n");	
		
			if(theta_change[i] < 0)          //进入触地阶段
			{							
				theta_change[i] += THETA_CIRCLE; //如果该值为负数 就将其变为正的角度	
				speed[i] = speed_stance;	
				
				//if((i == 0 )||(i == 3))
						//printf("forward state speed[%d] speed_stance is %d theta_change[%d]:%lu\r\n",j,speed_swing,j,theta_change[j]);								
				
			}					
			else if(theta_change[i] > 0)		 //进入悬空阶段
			{									
				
					theta_change[i] = theta_change[i];						
					speed[i] = speed_swing;								
					
				//if((i == 0 )||(i == 3))
						//printf("forward state speed[%d] speed_swing is %d theta_change[%d]:%lu\r\n",j,speed_swing,j,theta_change[j]);															
			}		
		
		}
																
		else if(leg_info_set[i].turn_dirction == REVERSE)  //向后转  
		{		
			//printf("reverse state\r\n");	
			
			if(theta_change[i] > 0)          //触地阶段  此处不能等于0 
			{								
					theta_change[i] -= THETA_CIRCLE; 							
					speed[i] = speed_stance;
																
					
				//	if((i == 0 )||(i == 3))
					//		printf("reverse state speed[%d] speed_stance is %d theta_change[%d]:%lu\r\n",i,speed_stance,i,theta_change[i]);															
			}					
			else if(theta_change[i] < 0)   	//悬空阶段
			{																
					theta_change[i] = theta_change[i];														
					speed[i] = speed_swing;																																	
					
																	
			}		
		}					
		
		
		if((gait_mode[i]==STAIR)||gait_mode[i]==CLIMB)
			speed[i] = 500;  //台阶步态就是快
		
		desired_values.global_theta[i] = theta_change[i];
		desired_values.global_velocity[i] = speed[i];					
	}	
}


void gait_choose(int gait_to_set)
{
	gait_set_last = gait_to_set;
	
	
	if(gait_to_set == STAIR)
	{
			
		
			theta[0][3] = 0;
			theta[3][0] = 0; //改变leg0和leg3的耦合相角										
				
			theta[1][4] = 0;
			theta[4][1] = 0; //改变leg0和leg3的耦合相角		
				
			theta[2][5] = 0;
			theta[5][2] = 0; //改变leg0和leg3的耦合相角		
			
			theta[0][2] = 0.7*pi;
			theta[2][0] = -0.7*pi;
			
				
			//同侧足相位关系
			theta[0][1] = 0.7*pi;
			theta[1][0] = -0.7*pi;
			theta[0][0] = 0; //改变leg0和leg3的耦合相角			
			
			theta[1][2] = 0.7*pi;
			theta[2][1] = -0.7*pi;
			theta[1][1] = 0; //改变leg0和leg3的耦合相角			
			
			theta[3][4] = 0.7*pi;
			theta[4][3] = -0.7*pi;
			
			theta[3][5] = 0.7*pi;
			theta[5][3] = -0.7*pi;             
										
										
			theta[4][5] = 0.7*pi;
			theta[5][4] = -0.7*pi;
			
			
			theta[0][4] = 0.7*pi;    
			theta[4][0] = -0.7*pi;   
			theta[0][5] = 0.7*pi;    
			theta[5][0] = -0.7*pi;  				
			
			
			theta[1][3] = -0.7*pi;    
			theta[3][1] = 0.7*pi;   
			theta[1][5] = 0.7*pi;    
			theta[5][1] = -0.7*pi;  				
			
			
			theta[2][3] = -0.7*pi;    
			theta[3][2] = 0.7*pi;   
			theta[2][4] = 0.7*pi;    
			theta[4][2] = -0.7*pi;  	
		
		
			for(i =0; i<6;i++)
			{	
				gait_mode[i] = gait_to_set;
			
				direction_to_set[i] = direction_forward[i];
				turn_flag[i] = false;		
				gait_stair[i] = true;	
			
			}
	
	}
	
	if(gait_to_set == CLIMB)
	{
		
		for(i =0; i<6;i++)
				{	
					gait_mode[i] = gait_to_set;
					theta[0][3] = 0;
					theta[3][0] = 0; //改变leg0和leg3的耦合相角										
						
					theta[1][4] = 0;
					theta[4][1] = 0; //改变leg0和leg3的耦合相角		
						
					theta[2][5] = 0;
					theta[5][2] = 0; //改变leg0和leg3的耦合相角		
					
					theta[0][2] = 0;
					theta[1][2] = 0;
					
								
					//同侧足相位关系
					theta[0][1] = 0;
					theta[1][0] = 0;
					theta[0][0] = 0; //改变leg0和leg3的耦合相角			
					
					theta[1][2] = 0;
					theta[2][1] = 0;
					theta[1][1] = 0; //改变leg0和leg3的耦合相角			
					
					theta[3][4] = 0;
					theta[4][3] = 0;
					
					theta[4][5] = 0;
					theta[5][4] = 0;
							
					
					
					leg_info_set[i].stance_theta = legs_climb_stance[i];
					leg_info_set[i].swing_theta = legs_climb_swing[i];
					
					direction_to_set[i] = direction_forward[i];
					turn_flag[i] = false;		
					gait_stair[i] = false;				
				}
	}
	
	else  //三角步态
	{
			//以下是新增加的
			theta[0][2] = 0;
			theta[0][4] = 0;
			theta[0][5] = 0;
			theta[1][3] = 0;
			theta[1][5] = 0;
			theta[2][0] = 0;
			theta[2][2] = 0;
			theta[2][3] = 0;
			theta[2][4] = 0;
			theta[3][1] = 0;
			theta[3][2] = 0;
			theta[3][3] = 0;
			theta[3][5] = 0;
			theta[4][0] = 0;
			theta[4][2] = 0;
			theta[4][4] = 0;
			theta[5][0] = 0;
			theta[5][1] = 0;
			theta[5][3] = 0;
			theta[5][5] = 0;
	
			
			for(i =0; i<6;i++)
			{			
				leg_info_set[i].stance_theta = legs_triangle_stance[i];
				leg_info_set[i].swing_theta = legs_triangle_swing[i];			
				gait_mode[i] = gait_to_set;
				gait_stair[i] = false;		
				
				if(gait_to_set == LEFT)
				{
					
					theta[0][3] = 0;
					theta[3][0] = 0; //改变leg0和leg3的耦合相角		
	
					theta[1][4] = 0;
					theta[4][1] = 0; //改变leg1和leg4的耦合相角
						
					theta[2][5] = 0;
					theta[5][2] = 0; //改变leg2和leg5的耦合相角									
					

						
					//同侧足相位关系
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //改变leg0和leg3的耦合相角			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //改变leg0和leg3的耦合相角			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
					
					
					//20190604新增
					theta[0][4] = pi;
					theta[4][0] = -pi;
					theta[1][3] = pi;
					theta[3][1] = -pi;
					theta[3][5] = pi;
					theta[5][3] = -pi;
					theta[2][4] = pi;
					theta[4][2] = -pi;
																								


					direction_to_set[i] = turn_left[i];
					turn_flag[i] = true;
					gait_stair[i] = false;							
				}
				else if(gait_to_set == RIGHT)
				{
					//转弯时均需将相角变为相同
						
					theta[0][3] = 0;
					theta[3][0] = 0; //改变leg0和leg3的耦合相角		
	
					theta[1][4] = 0;
					theta[4][1] = 0; //改变leg1和leg4的耦合相角		
					
					theta[2][5] = 0;
					theta[5][2] = 0; //改变leg2和leg5的耦合相角		
					
					
						
					
					//同侧足相位关系
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //改变leg0和leg3的耦合相角			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //改变leg0和leg3的耦合相角			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
					
					//20190604新增															
					theta[0][4] = pi;
					theta[4][0] = -pi;
					theta[1][3] = pi;
					theta[3][1] = -pi;
					theta[3][5] = pi;
					theta[5][3] = -pi;
					theta[2][4] = pi;
					theta[4][2] = -pi;
					
					direction_to_set[i] = turn_right[i];
					turn_flag[i] = true;
					gait_stair[i] = false;			
				}
				else if(gait_to_set == TRIANGLE_FORWARD)
				{			
					
					theta[0][3] = pi;
					theta[3][0] = -pi; //改变leg0和leg3的耦合相角					
					
					
					theta[1][4] = pi;
					theta[4][1] = -pi; //改变leg0和leg3的耦合相角		
					
					theta[2][5] = pi;
					theta[5][2] = -pi; //改变leg0和leg3的耦合相角		
					
					
					
					//同侧足相位关系
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //改变leg0和leg3的耦合相角			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //改变leg0和leg3的耦合相角			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
																										
					direction_to_set[i] = direction_forward[i];
					
					turn_flag[i] = false;
					gait_stair[i] = false;			
				}
				else if(gait_to_set == TRIANGLE_BACKWARD)
				{
									
					theta[0][3] = pi;
					theta[3][0] = -pi; //改变leg0和leg3的耦合相角										
					
					theta[1][4] = pi;
					theta[4][1] = -pi; //改变leg0和leg3的耦合相角		
					
					theta[2][5] = pi;
					theta[5][2] = -pi; //改变leg0和leg3的耦合相角		
						
					
					
					
					//同侧足相位关系
					theta[0][1] = pi;
					theta[1][0] = -pi;
					theta[0][0] = 0; //改变leg0和leg3的耦合相角			
					
					theta[1][2] = pi;
					theta[2][1] = -pi;
					theta[1][1] = 0; //改变leg0和leg3的耦合相角			
					
					theta[3][4] = pi;
					theta[4][3] = -pi;
					
					theta[4][5] = pi;
					theta[5][4] = -pi;
	
					
					direction_to_set[i] = direction_reverse[i];
					turn_flag[i] = false;
					gait_stair[i] = false;								
				}						
			}
		}
}

float cpg_to_angle(float cpg_position)
{
	int16_t position = 0;
	
	position = 360/2 * cpg_position;
	return position;
}



void get_cmd_from_pc()
{			
		u8 len;	
		int i = 0;
		int j = 0;
		char speedstr[6] = {0}; 				
		//float omega_to_set = omega_set;//将要给beta赋值
		if(USART_RX_STA&0x8000)
		{			
				len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
				//printf("\r\n您发送的消息为:\r\n");	
				//HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
				//while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束
				//printf("\r\n\r\n");//插入换行
				USART_RX_STA=0;	
				
				if(!strncmp((char*)USART_RX_BUF,"betaV",5))  //三角步态
				{																					
						strncpy(speedstr,(char*)USART_RX_BUF+5,1); //将速度内容赋值给speedstr															
						beta_set = atoi(speedstr)/10.0;								//betaV6
					//	printf("bate is %.2f\r\n",beta_set);					
				}					
		
				if(!strncmp((char*)USART_RX_BUF,"omgaV",5))  //移动
				{											
						strncpy(speedstr,(char*)USART_RX_BUF+5,1); //将速度内容赋值给speedstr
						omega_to_set = atoi(speedstr);						 		 //betaV6					
					//	printf("omega to set is %.2f\r\n",omega_to_set);
				}	
				
				
				if(!strncmp((char*)USART_RX_BUF,"k",1))  //移动
				{											
						strncpy(speedstr,(char*)USART_RX_BUF+1,1); //调整耦合矩阵的变换速率
						
											
	
					//test 20190521
					
					/*
					k_value[0][3]=k_value[3][0] 
					 =k_value[0][1]=k_value[1][0] 
					 =k_value[1][2]=k_value[2][1]
 					 =k_value[1][4]=k_value[4][1]																		
					 =k_value[2][5]=k_value[5][2]
					
					 =atoi(speedstr)*3.8;				
					*/
					for(i=0;i<6;i++)
					{
						for(j=0;j<6;j++)
						{
							k_value[i][j] = atoi(speedstr)*2.8;		
						}
					
					}
					
						//参数k7似乎挺合适的 20190425
						//k_value[0][0]	=	k_value[1][1] = k_value[2][2] 
					 
					 //=k_value[5][5] = k_value[4][4] = k_value[3][3]  
					 // = atoi(speedstr)*1.1;	
					
				}	
	
				if(!strncmp((char*)USART_RX_BUF,"left",4))  //左转
				{																															
					
					gait_choose(LEFT);	
					
					/*
					for(i=0;i<6;i++)
						{
							gait_mode[i] = LEFT;
							direction_to_set[i] = turn_left[i];
							turn_flag[i] = true;								
						}	
					
						theta[0][3] = 0;
						theta[3][0] = 0; //改变leg0和leg3的耦合相角		

						theta[1][4] = 0;
						theta[4][1] = 0; //改变leg1和leg4的耦合相角
					
						theta[2][5] = 0;
						theta[5][2] = 0; //改变leg2和leg5的耦合相角
					*/
		
						//printf("turn left \r\n");
				}		
				if(!strncmp((char*)USART_RX_BUF,"rigt",4))  //右转
				{																														
						
					
					gait_choose(RIGHT);	
						//转弯时均需将相角变为相同
						/*
						theta[0][3] = 0;
						theta[3][0] = 0; //改变leg0和leg3的耦合相角		

						theta[1][4] = 0;
						theta[4][1] = 0; //改变leg1和leg4的耦合相角		
					
						theta[2][5] = 0;
						theta[5][2] = 0; //改变leg2和leg5的耦合相角		
							

					
					
						for(i=0;i<6;i++)
							{
								gait_mode[i] = RIGHT;
								direction_to_set[i] = turn_right[i];
								turn_flag[i] = true;					
							}		
					*/
						//printf("turn right \r\n");
				}	
				if(!strncmp((char*)USART_RX_BUF,"fwrd",4))  //前进
				{																							
					/*	
					theta[0][3] = pi;
					theta[3][0] = pi; //改变leg0和leg3的耦合相角					
					
					
					theta[1][4] = pi;
					theta[4][1] = pi; //改变leg0和leg3的耦合相角		
					
					theta[2][5] = pi;
					theta[5][2] = pi; //改变leg0和leg3的耦合相角		
					
			
					//该系数只需设定一次 k_value[i][j] 表示第i条腿 与 第j条腿耦合时 变换的速度
					
					k_value[0][3] = 9.8;
					k_value[3][0] = 9.8;						
					k_value[0][0] = 1.8;
					k_value[3][3] = 1.8;
					
					
					k_value[0][1] = 6.8;
					k_value[1][0] = 6.8;						
					k_value[1][1] = 1.8;
					
					
					k_value[1][2] = 6.8;
					k_value[2][1] = 6.8;						
					k_value[2][2] = 1.8;
					
					
					k_value[1][4] = 6.8;
					k_value[4][1] = 6.8;						
					k_value[4][4] = 1.8;
					
					k_value[2][5] = 6.8;
					k_value[5][2] = 6.8;						
					k_value[5][5] = 1.8;
					*/
										
					
					
					gait_choose(TRIANGLE_FORWARD);	
					/*
					for(i=0;i<6;i++)
						{
							gait_mode[i] = TRIANGLE_FORWARD;
							direction_to_set[i] = direction_forward[i];
							turn_flag[i] = false;								
						}			
					*/
						//printf("forward \r\n");
				}	

				if(!strncmp((char*)USART_RX_BUF,"back",4))  //后退
				{																																
					gait_choose(TRIANGLE_BACKWARD);		
				}		
												
				if(!strncmp((char*)USART_RX_BUF,"stir",4))  //后退
				{	
					gait_choose(STAIR);
				}
				if(!strncmp((char*)USART_RX_BUF,"clim",4))  //后退
				{	
					gait_choose(CLIMB);
				}
	

				if(!strncmp((char*)USART_RX_BUF,"stop",4))
					HAL_TIM_Base_Stop_IT(&TIM3_Handler); //停止定时器中断：TIM_IT_UPDATE   
				if(!strncmp((char*)USART_RX_BUF,"start",5))
					HAL_TIM_Base_Start_IT(&TIM3_Handler); //开始定时器计数
			}
}
#endif

