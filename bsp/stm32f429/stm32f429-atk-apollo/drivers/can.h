#ifndef _CAN_H
#define _CAN_H
#include "rtdef.h"


#define CAN_DEV_NAME       "can1"      /* CAN 设备名称 */

struct rt_can_msg
{
    rt_uint32_t id  : 29;   /* CAN ID, 标志格式 11 位，扩展格式 29 位 */
    rt_uint32_t ide : 1;    /* 扩展帧标识位 */
    rt_uint32_t rtr : 1;    /* 远程帧标识位 */
    rt_uint32_t rsv : 1;    /* 保留位 */
    rt_uint32_t len : 8;    /* 数据段长度 */
    rt_uint32_t priv : 8;   /* 报文发送优先级 */
    rt_uint32_t hdr : 8;    /* 硬件过滤表号 */
    rt_uint32_t reserved : 8;
    rt_uint8_t data[8];     /* 数据段 */
};

struct rt_can_filter_item
{
    rt_uint32_t id  : 29;   /* 报文 ID */
    rt_uint32_t ide : 1;    /* 扩展帧标识位 */
    rt_uint32_t rtr : 1;    /* 远程帧标识位 */
    rt_uint32_t mode : 1;   /* 过滤表模式 */
    rt_uint32_t mask;       /* ID 掩码，0 表示对应的位不关心，1 表示对应的位必须匹配 */
    rt_int32_t hdr;         /* -1 表示不指定过滤表号，对应的过滤表控制块也不会被初始化，正数为过滤表号，对应的过滤表控制块会被初始化 */
#ifdef RT_CAN_USING_HDR
    /* 过滤表回调函数 */
    rt_err_t (*ind)(rt_device_t dev, void *args , rt_int32_t hdr, rt_size_t size);
    /* 回调函数参数 */
    void *args;
#endif /*RT_CAN_USING_HDR*/
};


struct rt_can_filter_config
{
    rt_uint32_t count;                  /* 过滤表数量 */
    rt_uint32_t actived;                /* 过滤表激活选项，1 表示初始化过滤表控制块，0 表示去初始化过滤表控制块 */
    struct rt_can_filter_item *items;   /* 过滤表指针，可指向一个过滤表数组 */
};


#endif