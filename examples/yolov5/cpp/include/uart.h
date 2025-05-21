#ifndef __UART_H__
#define __UART_H__

#include "uart_parser.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*uart_recv_callback)(unsigned char str);

static int uart_fd = -1;
int qjy_uart_init(void* param);
void set_baud_rate(int fd, int speed);
int qjy_uart_deinit(void);

#ifdef __cplusplus
}
#endif

#endif