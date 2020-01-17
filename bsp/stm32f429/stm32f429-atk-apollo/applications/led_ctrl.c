#include "drv_led.h"
#include "led_ctrl.h"
#include "drv_led.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>



static rt_thread_t led_thread;


static void led_thread_entry(void *parameter){
	
	
	while (1)
    {
		rt_led_on();
		
		rt_thread_mdelay(1000);
		
		rt_led_off();
		
    }
}

void led_ctrl_init(void){

	rt_led_init();
	
	rt_err_t result;
	led_thread = rt_thread_create("rt_led_thread",
								led_thread_entry,
								RT_NULL,
								512,
								17,
								5);
	
	result = rt_thread_startup(led_thread);
	if(result != RT_EOK){
		
		rt_kprintf("creadted led thread failed!\n");
	}
}
