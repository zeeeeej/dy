#ifndef HD_CAMERA_H
#define HD_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HD_SERIAL_NORMAL_MODE = 0,      // 串口传输模式
    HD_SERIAL_SHELL_MODE,           // SHELL模式
    HD_SERIAL_PUSH_MODE,            // PUSH模式
    HD_SERIAL_PULL_MODE,            // PULL模式
    HD_SERIAL_HD_PUSH_MODE          // HD_PUSH模式
} HD_SERIAL_MODE;


void hd_camera_change_serial_mode(HD_SERIAL_MODE mode);

int hd_camera_uart_write(const unsigned char *raw, size_t raw_size);

#ifdef __cplusplus
}
#endif

#endif // HD_CAMERA_H
