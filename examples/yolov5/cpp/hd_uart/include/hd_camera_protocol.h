
#ifndef __HD_CAMERA_PROTOCOL__
#define __HD_CAMERA_PROTOCOL__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#define DEBUG         0  // 1:打开 0：关闭
#define CONTEXT       0  // 1:mac 0:板子

#define PROTOCOL_VERSION                "1.0"                       // 协议版本
#define PROTOCOL_HEADER_0               0x5A                        // 协议头标识0
#define PROTOCOL_HEADER_1               0xAA                        // 协议头标识1
#define PROTOCOL_SLAVE_STATIC           0x01                        // 静态摄像头从机地址
#define PROTOCOL_SLAVE_DYNAMIC          0x02                        // 动态摄像头从机地址
#define PROTOCOL_RATE_DEFAULT           460800                      // 默认485串口波特率
#define PROTOCOL_UART_SUCCESS           0                           // result 成功
#define PROTOCOL_UART_FAIL              1                           // result 失败
#define PROTOCOL_MAX_FRAME_LEN          8192                        // 定义最大帧长度

uint16_t hd_crc16(const uint8_t *data, uint32_t length);

uint8_t hd_camera_protocol_decode(
        const unsigned char *recv_data_in,
        uint32_t recv_data_size_in,
        uint8_t *slave_addr_out,
        uint8_t *cmd_out,
        uint32_t *payload_data_size_out,
        unsigned char **payload_data_out
);

uint8_t hd_camera_protocol_encode(
        unsigned char **dest_data_output,
        uint32_t *dest_data_size_output,
        uint8_t slave_addr_in,
        uint8_t cmd_in,
        uint32_t payload_data_size_in,
        const unsigned char *payload_data_in
);

#ifdef __cplusplus
}
#endif

#endif // __HD_CAMERA_PROTOCOL__
