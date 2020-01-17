#ifndef _CMD_PARSE_
#define _CMD_PARSE_
#include <rtthread.h>


#define MAX_PARSE_CLASS    8
#define MAX_PARSE_METHODS    18

typedef uint16_t (*cmd_parser)(uint8_t *p);
struct Cmd_Parse_Method{
    uint16_t opcode;
    cmd_parser func;
};
struct CMD_PARSE_CLASS{
    char name[8];
    struct Cmd_Parse_Method *methods;
};
int register_cmd_class(const struct CMD_PARSE_CLASS *p);
int command_init(void);


void cmd_parse_init(void);
#endif
