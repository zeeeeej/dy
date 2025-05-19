#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "hd_uart_parser.h"
//#include "hd_camera_protocol.h"
//#include "hd_camera_protocol_cmd.h"


static uart_recv_action_id_callback g_callback = NULL;

/**
 * 说明：在收到串口数据时，先调用此函数，再调用原有的解析函数。如果返回0：代表这条协议没有被处理；否则协议已被处理，就不需要再解析这条协议了。
 *
 * @param raw       串口原始数据包
 * @return          是否已处理。1：已处理；0：未处理。
 */
int hd_uart_parser(const unsigned char *raw, size_t raw_size) {
    if (g_callback != NULL) {
        g_callback(NULL, 0);
    }
    // if (NULL == raw) {

    //     return 0;
    // }
    // uint8_t ret;
    // uint8_t slave_addr_out;
    // uint8_t cmd_out;
    // uint32_t payload_data_size_out;
    // unsigned char *payload_data_out;
    // ret = hd_camera_protocol_decode(raw, raw_size, &slave_addr_out, &cmd_out, &payload_data_size_out,
    //                                 &payload_data_out);
    // if (ret) {
    //     printf("hd_camera_protocol_decode error \n");
    //     return 0;
    // }
    return 0;
}


int hd_uart_init(uart_recv_action_id_callback callback) {
    g_callback = callback;
}

