#include "network.h"
rt_device_t uart_dev = RT_NULL;
char buf[] = "12313\n";


static rt_sem_t _rx_sem;

struct serial_configure uart2_config = UART2_SERIAL_CONFIG;

static UartDataBufType _uart_buf;



void uart_recv_init(rt_sem_t sem){

	_uart_buf.state = RECV_EMPTY;
    _uart_buf.len = 0;
    _rx_sem = sem;
}




//协议为起始帧 0x66
//结束帧       0x77
static int uart_agreement_recv(uint8_t * rbuf, uint8_t * rlen)
{
    static uint8_t len = 0;
    uint8_t _buf;

    _buf = *rbuf;
    
    if(_uart_buf.state == RECV_EMPTY){
        if(_buf == 0x66){
            _uart_buf.state = RECV_START;
            _uart_buf.msg[_uart_buf.len] = _buf;
            _uart_buf.len++;
            return 0;
        }
    }
    if(_uart_buf.state == RECV_START){

        _uart_buf.msg[_uart_buf.len] = _buf;
		_uart_buf.len++;
		
		if(_uart_buf.len == 3){
			_uart_buf.state = RECV_FIN;
            rt_sem_release(_rx_sem);  //接收数据完成
			rt_kprintf("rcv ok!\n");
            len = 0;
            return 1;
        }
		
		
        if(_uart_buf.len > DATA_MAX_LEN){
            _uart_buf.state = RECV_EMPTY;
            _uart_buf.len = 0;
            len = 0;
        }
    }
    return 0;
}


int uart_data_recived_finish(uint8_t *buf, uint16_t* len){
	//在这里可以判断接收的数据是否正确 可使用CRC校验方法
	
	DataRecievd * r_data;
	
	if(_uart_buf.state != RECV_FIN){
	
		return -1;
		
	}else if(_uart_buf.len < 3){
		_uart_buf.len = 0;
		_uart_buf.state = RECV_EMPTY;
		rt_kprintf("data missing\n");
		return -1;
	
	}else{
		
		*len = 1;
		r_data = (DataRecievd*)(_uart_buf.msg);

		rt_memcpy(buf, &r_data->cmd, *len);
		_uart_buf.len = 0;
		_uart_buf.state = RECV_EMPTY;
		rt_kprintf("data ok len : %d\n",*len);
	}
	
	return RT_EOK;
}




static rt_err_t  uart_dev_recv_buf(rt_device_t dev, rt_size_t size){

    static uint8_t i;
    static uint8_t len = 1;
    rt_device_read(uart_dev,0,&i, len);
	
    uart_agreement_recv(&i, &len);
    return RT_EOK;
}


static rt_err_t uart_init(char *dev_name){	
	
	rt_device_t dev = RT_NULL;
	
	dev = rt_device_find(dev_name);
	
	if(dev != RT_NULL){
		
		
		if(rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK){ //读写中断模式
			
			if(uart_dev != RT_NULL){
				
				rt_device_close(uart_dev);
				
				rt_device_set_rx_indicate(uart_dev, RT_NULL);	
			}	
			uart_dev = dev;
			
			rt_device_set_rx_indicate(uart_dev,uart_dev_recv_buf);
			
			rt_kprintf("open init uart2 success!\n");
		}
		
		return RT_ERROR;
	}
	return RT_EOK;
}

void rt_network_init(){
	
	uart_init("uart2");

}
