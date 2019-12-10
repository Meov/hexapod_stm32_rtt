#include "network.h"
rt_device_t uart1_dev = RT_NULL;
char buf[] = "12313\n";

struct serial_configure uart2_config = UART2_SERIAL_CONFIG;


static rt_err_t  dev_recv_buf(rt_device_t dev, rt_size_t size){

	rt_kprintf("recived\n");
	rt_device_write(dev,0,buf,sizeof(buf));
	return RT_EOK;
}

static rt_err_t uart_init(char dev_name[]){	
	rt_device_t dev = RT_NULL;
	dev = rt_device_find(dev_name);
	
	if(dev != RT_NULL){		
		rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &uart2_config);
		if(rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK){ //¶ÁÐ´ÖÐ¶ÏÄ£Ê½
			
			rt_device_set_rx_indicate(dev,dev_recv_buf);		
			rt_kprintf("open init uart2 success!\n");		
		return RT_EOK;
		}
	}
	return RT_ERROR;
}


void rt_network_init(){	
	uart_init("uart2");
	rt_kprintf("init uart2\n");
}
