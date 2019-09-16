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
static rt_thread_t thread1;


/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 1)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(1000);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}

static void thread_main_entry(void *parameter){
	
		while(1){
			rt_kprintf("rt-thrad-1\n");
			rt_thread_delay(1);
		}
}

int rt_thread_start_up(void){
		rt_err_t result;
		thread1 = rt_thread_create("thrad_main",
															thread_main_entry,
															RT_NULL,
															1024,
															5,
															20
															);
		
		result = rt_thread_startup(thread1);
		if(result != RT_EOK){
		
			rt_kprintf("created thread1 failed!\n");
			
		}
}
