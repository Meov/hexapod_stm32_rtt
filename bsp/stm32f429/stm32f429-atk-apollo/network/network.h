#ifndef __NETWORK_H
#define __NETWORK_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define DATA_MAX_LEN 3


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


typedef struct uart_data_buf_type {
	enum {
        RECV_EMPTY = 0,
        RECV_START,
        RECV_FIN,
        RECV_ERR
    } state;
    uint8_t len;
    uint8_t msg[DATA_MAX_LEN];
}UartDataBufType;

typedef struct datarecievd{
	uint8_t head;
	uint8_t cmd;
	uint8_t tail;
}DataRecievd;


int uart_data_recived_finish(uint8_t *buf, uint16_t* len);
void uart_recv_init(rt_sem_t sem);
void rt_network_init(void);
#endif
