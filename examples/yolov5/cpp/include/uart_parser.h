#ifndef __UART_PARSER_H__
#define __UART_PARSER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define APP_VERSION  "0.0.1"

#define SYSTEM_HEART_BEART      0x01
#define SYSTEM_ATTR_READ        0x02
#define SYSTEM_ATTR_WRITE       0x03
#define SYSTEM_RESTORE_FACTORY  0x04
#define SYSTEM_REBOOT           0x05
#define SYSTEM_ACTIVE_CAPTURE   0x06
#define SYSTEM_PIC_INFO         0x07
#define SYSTEM_PIC_DELETE       0x08
#define SYSTEM_PIC_GET          0x09
#define SYSTEM_PIC_GET_SUCC     0x0A
#define SYSTEM_OTA_UPDATE       0x0B
#define SYSTEM_OTA_SEND         0x0C
#define SYSTEM_DOOR_STATE       0x0D

#define SYSTEM_FACTORY_TEST     0x0F

#define SYSTEM_APP_UPDATE       0x1B
#define SYSTEM_APP_SEND         0x1C

#define SUBSYSTEM_ATTR_ADDR     0x01  // R/W
#define SUBSYSTEM_ATTR_VERSION  0x02  // R
#define SUBSYSTEM_ATTR_SERIAL   0x03  // R/W
#define SUBSYSTEM_ATTR_BAUD     0x04  // W
#define SUBSYSTEM_ATTR_PIC_SIZE 0x05  // R/W
#define SUBSYSTEM_ATTR_PIC_COMP 0x06  // R/W
#define SUBSYSTEM_ATTR_PIC_BRIG 0x07  // R/W
#define SUBSYSTEM_ATTR_PIC_CONF 0x08
#define SUBSYSTEM_ATTR_UTC      0x09  // R/W
#define SUBSYSTEM_ATTR_GS_ANGLE 0x0A  // R/W
#define SUBSYSTEM_ATTR_GS_ROT   0x0B  // R/W
#define SUBSYSTEM_ATTR_GS_CONF  0x0C  
#define SUBSYSTEM_ATTR_GS_EN    0x0D  // R/W
#define SUBSYSTEM_ATTR_GS_NUM   0x0E  // R/W
#define SUBSYSTEM_ATTR_HEAT_PWM 0x0F  // R/W

#define SUBSYSTEM_ATTR_PITCH    0x10
#define SUBSYSTEM_ATTR_ROLL     0x11
#define SUBSYSTEM_ATTR_YAW      0x12
#define SUBSYSTEM_ATTR_GRAVITY  0x13
#define SUBSYSTEM_ATTR_DOOR_ANG 0x14


#define bswap_16(A) (( ((uint16_t)(A) & 0xff00) >> 8) | (( (uint16_t)(A) & 0x00ff) << 8))  
 
#define bswap_32(A) ((( (uint32_t)(A) & 0xff000000) >> 24) | \ 
                          (( (uint32_t)(A) & 0x00ff0000) >> 8)  | \ 
                          (( (uint32_t)(A) & 0x0000ff00) << 8)  | \ 
                          (( (uint32_t)(A) & 0x000000ff) << 24)) 

enum cmd_uart_state
{
    UART_WAIT_HEAD0,
    UART_WAIT_HEAD1,
    UART_WAIT_ADDR,
    UART_WAIT_CMD,
    UART_WAIT_LEN1,
    UART_WAIT_LEN2,
    UART_WAIT_LEN3,
    UART_WAIT_LEN4,
    UART_WAIT_DATA,
    UART_WAIT_CRC1,
    UART_WAIT_CRC2,
    UART_ERROR
};

void cmd_uart_init(void* param);
void *uart_test_jpeg_thread(void *arg);
void qjy_uart_parser(uint8_t str);
void reset_uart_data();

#ifdef __cplusplus
}
#endif

#endif