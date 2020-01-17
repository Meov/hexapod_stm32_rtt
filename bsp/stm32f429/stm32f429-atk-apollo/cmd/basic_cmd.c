#include "cmd_parse.h"
#include "basic_cmd.h"

static uint16_t hexapod_start(uint8_t *p){

	rt_kprintf("hexapod start!\n");
	return 	RT_EOK;

}

static uint16_t hexapod_stop(uint8_t *p){

	rt_kprintf("hexapod stop!\n");
	return 	RT_EOK;
}
static uint16_t hexapod_left(uint8_t *p){

	rt_kprintf("hexapod turn left!\n");
	return 	RT_EOK;
}

static uint16_t hexapod_right(uint8_t *p){

	rt_kprintf("hexapod turn right!\n");
	return 	RT_EOK;
}
static uint16_t hexapod_back(uint8_t *p){

	rt_kprintf("hexapod turn back!\n");
	return 	RT_EOK;
}


static struct Cmd_Parse_Method methods[] = {
	 {START,&hexapod_start},
	 {STOP, &hexapod_stop},
	 {LEFT,	&hexapod_left},
     {RIGHT,&hexapod_right},
	 {BACK,&hexapod_back},
	 {0,RT_NULL}
};

const struct CMD_PARSE_CLASS hexapod_class = {
    .name = "HEXAPOD",
    .methods = methods
};
void cmd_hexapod_init(void)
{
    register_cmd_class(&hexapod_class);
}

