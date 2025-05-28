#ifndef __UART_H__
#define __UART_H__

#include "uart_parser.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*uart_recv_callback)(unsigned char str);

typedef struct {
    void (*uart_recv_callback)(unsigned char str);
    void (*cus_recv_callback)(unsigned char str);
}recv_callback_func;

static int uart_fd = -1;
int qjy_uart_init(void* param, int8_t id);
void set_baud_rate(int fd, int speed);
int qjy_uart_deinit(void);
void rk_uart_sendbyte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif