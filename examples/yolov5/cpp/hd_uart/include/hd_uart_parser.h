#ifndef __HD_UART_PARSER__
#define __HD_UART_PARSER__

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hd_on_action_id_changed)(const char * action_id_str);

typedef void (*hd_debug_on_resp_changed)(const unsigned char *raw, uint32_t raw_size);

// 后期会隐藏
int handle_uart_data(const unsigned char *raw, size_t raw_size);

int hd_uart_init(const char* pic_dir_path,uint8_t addr, hd_on_action_id_changed  callback
                 ,hd_debug_on_resp_changed on_resp_changed);

void hd_uart_deinit();

#ifdef __cplusplus
}
#endif

#endif // __HD_UART_PARSER__