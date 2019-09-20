#include "network.h"
rt_device_t uart1_dev = RT_NULL;
char buf[1024];


static rt_err_t  dev_recv_buf(rt_device_t dev, rt_size_t size){

	rt_kprintf("recived\n");
	rt_device_read(dev, 0, &buf, 1);
}




rt_err_t uart_recv_init(char dev_name[]){
	
	rt_device_t dev = RT_NULL;
	dev = rt_device_find(dev_name);
	if(dev != RT_NULL){
		if(rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK){
			rt_device_set_rx_indicate(dev,dev_recv_buf);
		}
	
	}

}




int uart_init(){

	uart_recv_init("uart2");
	


}