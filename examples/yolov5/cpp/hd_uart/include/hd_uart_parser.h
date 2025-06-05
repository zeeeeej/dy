#ifndef __HD_UART_PARSER__
#define __HD_UART_PARSER__

#ifdef __cplusplus
extern "C" {
#endif

#define  EVENT_SNAPSHOT                     0x01
#define  EVENT_DEBUG_WRITE                  0x02

/**
 * 收到3.17 广播门开事件（0x1E）
 * @param action_id_str action_id（时间戳（4字节）+序号（1字节））转成的字符串格式：时间戳（单位：秒）+序号（3位的10进制，范围0～255）
 */
typedef void (*hd_on_action_id_changed)(const char *action_id_str);

typedef void* (*hd_on_event)(int  event_id,void * event_value,size_t event_value_size);

/**
 * 初始化
 *
 * 初始化例子：
 * uint8_t addr = 0x01;
 * 1.初始化qjy_uart_init
 *      recv_callback_func func = {qjy_uart_parser, hd_uart_recv};
 *      qjy_uart_init(&func, addr);
 * 2.初始化hd_uart_init
 *      hd_uart_init(addr,"/userdata/jpeg",on_action_id_changed);
 *
 * @param addr                  从机地址。01:静态摄像头；02：动态摄像头。
 * @param pic_dir_path          图片保存的地址
 * @param callback              action_id回调
 * @return                      0：成功；1：失败。
 */
int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
        );

/**
 * 解析函数
 * @param byte
 */
void hd_uart_recv(uint8_t byte);

/**
 * 反初始化
 */
void hd_uart_deinit();

char * hd_uart_version();

//void hd_uart_debug_recv(const unsigned char *raw, size_t raw_size);
//void hd_uart_debug_write_self(int write_outside);

#ifdef __cplusplus
}
#endif

#endif // __HD_UART_PARSER__