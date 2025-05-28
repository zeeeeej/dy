
#ifndef __HD_CAMERA_PROTOCOL_PROPERTY__
#define __HD_CAMERA_PROTOCOL_PROPERTY__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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
#define PROPERTY_HD_ID_DEBUG_ACTION_ID          0xD2
#define PROPERTY_HD_ID_DEBUG_ANGEL              0xD3
#define PROPERTY_HD_ID_DEBUG_HD_UART_VERSION    0xD4


#ifdef __cplusplus
}
#endif

#endif // __HD_CAMERA_PROTOCOL_PROPERTY__



