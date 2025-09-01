
#ifndef H__HD_CAMERA_PROTOCOL__H
#define H__HD_CAMERA_PROTOCOL__H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG         0  // 1:打开 0：关闭
#define CONTEXT       0  // 1:mac 0:板子

#define PROTOCOL_VERSION                "1.3"                       // 协议版本
#define PROTOCOL_HEADER_0               0x5A                        // 协议头标识0
#define PROTOCOL_HEADER_1               0xAA                        // 协议头标识1
#define PROTOCOL_SLAVE_STATIC           0x01                        // 静态摄像头从机地址
#define PROTOCOL_SLAVE_DYNAMIC          0x02                        // 动态摄像头从机地址
#define PROTOCOL_BROADCAST              0xFF                        // 广播地址
#define PROTOCOL_RATE_DEFAULT           460800                      // 默认485串口波特率
#define PROTOCOL_UART_SUCCESS           0                           // result 成功
#define PROTOCOL_UART_FAIL              1                           // result 失败
#define PROTOCOL_MAX_FRAME_LEN          (500*1024)                  // 定义最大帧长度

uint16_t hd_crc16(const uint8_t *data, uint32_t length);

uint8_t hd_camera_protocol_addr(
        uint8_t *out_addr,
        const unsigned char *in_recv_data,
        uint32_t in_recv_data_size);

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

typedef struct {
    uint32_t snapshot_timestamps;
    uint16_t pic_id;
    int file_size;
    unsigned char file_md5[16];
} hd_parse_pic_infos;

/**
 * 文件名格式：
 * id_md5_fileSize_bigTimestamp_triggerType_triggerAngel_addr_timestamp_picid.jpg
 * 其中md5为16进制字符串，filesize为整数字符串，timestamp为整数，picid为整数。
 * 根据文件名称解析数据得到hd_parse_pic_infos
 * 比如
 * 001_e3d99dcf615c61cd25b37b9f62989cce_123456_666_0_30_1_1234567788_002.jpg
 * 解除出来的hd_parse_pic_infos的值为：
 * snapshot_timestamps = 1234567788
 * pic_id = 002
 * file_size = 123456
 * file_size = 123456
 * file_md5 = [0xe3,0xd9,0x9d,0xcf,0x61,0x5c,0x61,0xcd,0x25,0xb3,0x7b,0x9f,0x62,0x98,0x9c,0xce]
 *
 * @param file_name     文件名称
 * @param infos         图片详情
 * @return
 */
int hd_camera_protocol_parse_pic_info(const char *file_name, hd_parse_pic_infos *infos);

// 001_e3d99dcf615c61cd25b37b9f62989cce_123456_666_0_30_1_1234567788_002.jpg
// %d_%s_%d_%d_%d_%d_%d_%d_%03d.jpg
int hd_camera_protocol_pic_info_encode(char result[1024], uint8_t index_1, unsigned char md5[16], uint32_t file_size,
                                       uint8_t index_2, uint8_t addr, uint8_t trigger_angel, uint8_t trigger_type,
                                       uint32_t timestamp, uint16_t pic_id
);

#ifdef __cplusplus
}
#endif

#endif // H__HD_CAMERA_PROTOCOL__H
