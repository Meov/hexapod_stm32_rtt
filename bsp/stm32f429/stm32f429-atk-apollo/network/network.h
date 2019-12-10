#ifndef __UART_H
#define __UART_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define UART2_SERIAL_CONFIG                     \
{                                               \
    BAUD_RATE_9600,     /* 9600 bits/s */       \
    DATA_BITS_8,        /* 8 databits */        \
    STOP_BITS_1,        /* 1 stopbit */         \
    PARITY_NONE,        /* No parity  */        \
    BIT_ORDER_LSB,      /* LSB first sent */    \
    NRZ_NORMAL,         /* Normal mode */       \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */       \
    0                                           \
}


void rt_network_init(void);
#endif

