#include "cmd_parse.h"
#include "rtdef.h"
#include <string.h>
#include "network.h"

static const struct CMD_PARSE_CLASS *cmd[MAX_PARSE_CLASS];

static rt_sem_t cmd_sem;
static rt_thread_t rt_uart_thread;

int register_cmd_class(const struct CMD_PARSE_CLASS *p)
{
    for(int i = 0;i < MAX_PARSE_CLASS;i++){
        if(cmd[i] == RT_NULL){
            cmd[i] = p;
            return 0;
        }
    }
    return -1;
}
static struct Cmd_Parse_Method *find_method(const struct CMD_PARSE_CLASS *c,uint16_t opcode)
{
    for(int i = 0;i < MAX_PARSE_METHODS;i++){
        if(c->methods[i].opcode == 0)break;
        if(c->methods[i].opcode == opcode && c->methods[i].func)
            return &c->methods[i];
    }
    return RT_NULL;
}
static int parse_cmd(uint8_t *data)
{ 	
	
    uint16_t opcode = *(uint16_t *)(data);  
	
	for(int i = 0;i < MAX_PARSE_CLASS;i++){
        if(cmd[i] && strlen((const char *)cmd[i]->name) > 0 && cmd[i]->methods){
            struct Cmd_Parse_Method *methods = find_method(cmd[i],opcode);
            if(methods){
                methods->func(data);  //执行该函数
                break;
            }
        }
    }
    return 0;
}


static void rt_uart_thread_entry(void *parameter){
	
    static uint8_t rbuf[100];
    static uint16_t rlen;
	
    while (1)
    {	
		rt_sem_take(cmd_sem, RT_WAITING_FOREVER);  				//阻塞等待信号量	
		if(uart_data_recived_finish(rbuf, &rlen) == RT_EOK)  	//获取收到的数据
			
		parse_cmd(rbuf);                                    //将接收到的数据从头至尾进行解析
        
	}
}


void cmd_parse_init(void){
	
	cmd_sem = rt_sem_create("cmd_sem", 0, 0);
	
	if (cmd_sem == RT_NULL) {
        rt_kprintf("cmd sem creation failed. \n");
    }
	
	uart_recv_init(cmd_sem);
	
	rt_err_t result;
	
	rt_uart_thread = rt_thread_create("uart_thread",
									 rt_uart_thread_entry,
									RT_NULL,
									512,
									11,
									5
									);
	result = rt_thread_startup(rt_uart_thread);

	if(result != RT_EOK){

		rt_kprintf("created uart thread failed\n");

	}
}



