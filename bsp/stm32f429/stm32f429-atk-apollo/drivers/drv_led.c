#include "drv_led.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_led.h"


#define LED0_PIN	GET_PIN(B, 1)

void rt_led_on(){
	
	rt_pin_write(LED0_PIN, PIN_HIGH);

}
void rt_led_off(){

	rt_pin_write(LED0_PIN, PIN_LOW);

}

void rt_led_init(){
	
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
} 
