#ifndef HD_UART_PARSER
#define HD_UART_PARSER

#include <stdint.h>
#include <string.h>

typedef void (*uart_recv_action_id_callback)(const unsigned char *action_id,size_t action_id_size);

/**
 * 说明：在收到串口数据时，先调用此函数，再调用原有的解析函数。如果返回0：代表这条协议没有被处理；否则协议已被处理，就不需要再解析这条协议了。
 *
 * @param raw           串口原始数据包
 * @param raw_size      串口原始数据包大小
 * @return              是否已处理。1：已处理；0：未处理。
 */
int hd_uart_parser(const unsigned char *raw,size_t raw_size);

int hd_uart_init(uart_recv_action_id_callback callback);

#endif // __HD_UART_PARSER__