#ifndef __HD_UTILS__
#define __HD_UTILS__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#define TAG "HDUART"

// 日志级别定义
typedef enum {
    HD_LOGGER_LEVEL_DEBUG,
    HD_LOGGER_LEVEL_INFO,
    HD_LOGGER_LEVEL_WARNING,
    HD_LOGGER_LEVEL_ERROR
} HDLoggerLevel;

// 设置日志级别
void hd_logger_set_level(HDLoggerLevel level);

HDLoggerLevel hd_logger_set_level_cur();

// 日志打印函数
void hd_logger_print(HDLoggerLevel level, const char *tag, const char *msg, ...);

// 简化宏定义
//#define LOGI(tag, msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_DEBUG, tag, msg, ##__VA_ARGS__)
//#define LOGD(tag, msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_INFO, tag, msg, ##__VA_ARGS__)
//#define LOGW(tag, msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_WARNING, tag, msg, ##__VA_ARGS__)
//#define LOGE(tag, msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_ERROR, tag, msg, ##__VA_ARGS__)

#define LOGD(msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_DEBUG, TAG, msg, ##__VA_ARGS__)
#define LOGI(msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_INFO, TAG, msg, ##__VA_ARGS__)
#define LOGW(msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_WARNING, TAG, msg, ##__VA_ARGS__)
#define LOGE(msg, ...)     hd_logger_print(HD_LOGGER_LEVEL_ERROR, TAG, msg, ##__VA_ARGS__)


// 计算3.5个字符时间（单位：微秒）
uint32_t calculate_3_5_char_time(uint32_t baud_rate, uint8_t data_bits, uint8_t parity, uint8_t stop_bits);

int hd_md5(const char *file_path, unsigned char result[16]);

void hd_printf_buff(const unsigned char *buf, size_t size, const char *tag, int full);

void hd_sleep_ms(uint32_t milliseconds);


#ifdef __cplusplus
}
#endif

#endif // __HD_UTILS__
