
#ifndef H__HD_CAMERA_PROTOCOL_PROPERTY__H
#define H__HD_CAMERA_PROTOCOL_PROPERTY__H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// property ids
#define PROPERTY_ID_SLAVE_ADDR                  0x01
#define PROPERTY_ID_FIRMWART_VERSION            0x02
#define PROPERTY_ID_CAMERA_SERIAL               0x03
#define PROPERTY_ID_UART_RATE                   0x04
#define PROPERTY_ID_IMG_SIZE                    0x05
#define PROPERTY_ID_IMG_Compression_ratio       0x06
#define PROPERTY_ID_IMG_Brightness              0x07
#define PROPERTY_ID_IMG_PARAMS                  0x08
#define PROPERTY_ID_UTC                         0x09
#define PROPERTY_ID_Gyroscope_argree            0x0A
#define PROPERTY_ID_Gyroscope_direction         0x0B
#define PROPERTY_ID_Gyroscope_config            0x0C
#define PROPERTY_ID_Gyroscope_state             0x0D
#define PROPERTY_ID_Gyroscope_save_img_max      0x0E
#define PROPERTY_ID_PWM_RATIO                   0x0F
#define PROPERTY_ID_Pitch_angel                 0x10
#define PROPERTY_ID_Roll_angel                  0x11
#define PROPERTY_ID_Yaw_Angle                   0x12
#define PROPERTY_ID_Acceleration                0x13
#define PROPERTY_ID_Opening_Angle_Latest        0x14
#define PROPERTY_ID_SYSTEM_VERSION              0x15


#define PROPERTY_HD_ID_DEBUG                    0xD1
#define PROPERTY_HD_ID_MODEL_FILE_NAME          0xCC
#define PROPERTY_HD_ID_DEBUG_ACTION_ID          0xD2
#define PROPERTY_HD_ID_DEBUG_ANGEL              0xD3
#define PROPERTY_HD_ID_DEBUG_HD_UART_VERSION    0xD4
#define PROPERTY_HD_ID_FACTORY_MODE             0xD5
#define PROPERTY_HD_ID_PUSH                     0xD6
#define PROPERTY_HD_ID_PULL                     0xD7

/* push模式 */
/**
 * push 文件
 * @param out_payload
 * @param out_payload_size
 * @param in_file_md5
 * @param in_file_size
 * @param in_file_path
 * @return
 */
uint8_t hd_host_property_set_push_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
);

uint8_t hd_host_property_set_push_encode_ext(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const char * src_file_path,
        const char * dest_file_path
);

/**
 * 解析 push 文件
 * @param out_file_md5
 * @param out_file_size
 * @param out_file_path
 * @param in_payload
 * @param in_payload_size
 * @return
 */
uint8_t hd_slave_property_set_push_decode(
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
);



#ifdef __cplusplus
}
#endif

#endif // H__HD_CAMERA_PROTOCOL_PROPERTY__H



