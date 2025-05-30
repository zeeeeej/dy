#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "hd_utils.h"
#include "hd_camera_protocol.h"

static void hd_camera_protocol_print_buffer(const unsigned char *buf, size_t len, const char *tag) {
    hd_printf_buff(buf,len,tag,0);
}

static const uint16_t ccitt_table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

 uint16_t hd_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0;

    while (length-- > 0)
        crc = (crc << 8) ^ ccitt_table[((crc >> 8) ^ *data++) & 0xff];

    return crc;
}

uint8_t hd_camera_protocol_decode(
        const unsigned char *recv_data_in,
        uint32_t recv_data_size_in,
         uint8_t *  slave_addr_out,
         uint8_t *  cmd_out,
         uint32_t *  payload_data_size_out,
         unsigned char **  payload_data_out
) {
    if (DEBUG) {
        hd_camera_protocol_print_buffer(recv_data_in, recv_data_size_in, "decode");
    }

    // 最小帧长度检查：帧头(2) + 地址(1) + 命令(1) + 长度(4) + CRC(2) = 10
    if (recv_data_size_in < 10) {
        return -1; // 数据长度不足
    }

    // 检查帧头
    if (recv_data_in[0] != PROTOCOL_HEADER_1 || recv_data_in[1] != PROTOCOL_HEADER_0) {
        LOGD("[uart]recv_data[0][1]:%02x,%02x\n", recv_data_in[0], recv_data_in[1]);
        return -2; // 帧头错误
    }

    // 获取从机地址
    *slave_addr_out = (uint8_t)recv_data_in[2];

    // 获取命令
    *cmd_out = recv_data_in[3];

    // 获取数据长度（小端格式）
    uint32_t length = (recv_data_in[7] << 24) | (recv_data_in[6] << 16) |
                      (recv_data_in[5] << 8) | recv_data_in[4];

    // 检查数据长度是否合理
    if (length > (recv_data_size_in - 10)) {
        return -4; // 数据长度超过实际数据
    }

    *payload_data_size_out = length;
    if (length > 0) {
        *payload_data_out = (unsigned char *)&recv_data_in[8];
    } else {
        *payload_data_out = NULL;
    }

    uint16_t received_crc = (recv_data_in[8 + length]) | recv_data_in[9 + length] << 8;
    uint16_t calculated_crc = hd_crc16(recv_data_in, 8 + length);

    if (received_crc != calculated_crc) {
        LOGE("%02x vs %02x \n", received_crc, calculated_crc);
        return -5; // CRC校验失败
    }
    if (DEBUG){
        LOGD("->cmd = %u \n",*cmd_out);
        LOGD("->slave_addr = %u \n",*slave_addr_out);
    }
    return 0; // 解析成功
}

uint8_t hd_camera_protocol_encode(
        unsigned char **dest_data_output,
        uint32_t *dest_data_size_output,
        uint8_t slave_addr_in,
        uint8_t cmd_in,
        uint32_t payload_data_size_in,
        const unsigned char *payload_data_in
) {
    // 计算总数据长度: 帧头(2) + 地址(1) + 命令(1) + 长度(4) + payload(n) + CRC(2)
    const uint32_t fixed_header_size = 8; // 帧头+地址+命令+长度
    const uint32_t crc_size = 2;
    const uint32_t total_size = fixed_header_size + payload_data_size_in + crc_size;
    // 分配内存
    uint8_t *buffer = (unsigned char *) malloc(total_size);
    if (buffer == NULL) {
        return -1; // 内存分配失败
    }

    // 填充帧头 (0x5AA5)
    buffer[0] = PROTOCOL_HEADER_1;
    buffer[1] = PROTOCOL_HEADER_0;

    // 填充从机地址 (由参数指定)
    buffer[2] = slave_addr_in;

    // 填充命令
    buffer[3] = cmd_in;

    // 填充数据长度 (小端模式)
    buffer[4] = (payload_data_size_in >> 0) & 0xFF;  // LSB
    buffer[5] = (payload_data_size_in >> 8) & 0xFF;
    buffer[6] = (payload_data_size_in >> 16) & 0xFF;
    buffer[7] = (payload_data_size_in >> 24) & 0xFF; // MSB

    // 填充payload数据
    if (payload_data_size_in > 0 && payload_data_in != NULL) {
        memcpy(&buffer[8], payload_data_in, payload_data_size_in);
    }

    // 计算CRC (覆盖: 帧头到payload)
    uint16_t crc = hd_crc16(buffer, fixed_header_size + payload_data_size_in);
    if (DEBUG){
        LOGD("crc = %02x \n", crc);
    }

    // 填充CRC (小端模式)
    buffer[fixed_header_size + payload_data_size_in] = crc & 0xFF;      // LSB
    buffer[fixed_header_size + payload_data_size_in + 1] = (crc >> 8) & 0xFF; // MSB

    // 返回生成的数据
    *dest_data_output = buffer;
    *dest_data_size_output = total_size;
    if (DEBUG) {
        hd_camera_protocol_print_buffer(buffer, total_size, "encode");
    }
    return 0; // 成功

}




