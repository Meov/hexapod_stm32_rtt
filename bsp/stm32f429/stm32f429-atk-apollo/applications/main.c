/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "network.h"
#include "drv_usart.h"
#include "led_ctrl.h"
#include "basic_cmd.h"
#include "cmd_parse.h"
#include "cpg_generator.h"
static rt_thread_t thrad_main;


static void thread_main_entry(void *parameter){
		
		led_ctrl_init();
		
		rt_network_init();
		
		cmd_parse_init();

		cmd_hexapod_init();
	
		cpg_generator_init();
	
}

int rt_thread_start_up(void){
		
		rt_err_t result;
		thrad_main = rt_thread_create("thrad_main",
									thread_main_entry,
									RT_NULL,
									1024,
									5,
									20
									);
		
		result = rt_thread_startup(thrad_main);
		if(result != RT_EOK){
		 
			rt_kprintf("created thread1 failed!\n");
			return RT_ERROR;
		}
	return RT_EOK;
}



int main(void)
{	
   
	rt_thread_start_up();
	
    return RT_EOK;
}




