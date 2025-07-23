#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <ctype.h>
#include "hd_uart_parser.h"
#include "hd_camera_protocol.h"
#include "hd_camera_protocol_cmd.h"
#include "hd_utils.h"
#include <errno.h>
#include <sys/time.h>
#include <fcntl.h>
#include "uart.h"
#include "rs485.h"
#include "hd_camera_protocol_property.h"
#include "hd_camera_shell.h"
#include "hd_camera_protocol_extra_cmd.h"
#include "hd_queue.h"
#include <signal.h>
#include "hd_camera.h"
#include "hd_camera_ota.h"
#include "hd_c_log.h"

#define HD_UART_PARSER_DEBUG                    0                           // debug开启
#define FRAME_HEADER_H                          PROTOCOL_HEADER_1           // 头1
#define FRAME_HEADER_L                          PROTOCOL_HEADER_0           // 头2
#define MAX_FILE_SIZE                           (512*1024)                  // 最大图片传输大小
#define JPG_SUFFIX                              ".jpg"                      // 图片格式
#define JPG_SUFFIX_LEN                          4                           // 图片格式长度

// 状态机状态定义
typedef enum {
    STATE_WAIT_HEADER_H,
    STATE_WAIT_HEADER_L,
    STATE_WAIT_ADDR,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN_H,
    STATE_WAIT_LEN_MH,
    STATE_WAIT_LEN_ML,
    STATE_WAIT_LEN_L,
    STATE_WAIT_DATA,
    STATE_WAIT_CRC_H,
    STATE_WAIT_CRC_L,
} ParserState;

static volatile uint8_t g_addr = 0;                             // 当前从机地址
static char g_hd_app_version[1024];                             // HD app 版本
static volatile uint8_t g_serial_mode = HD_SERIAL_NORMAL_MODE;  // 串口模式
static pthread_mutex_t g_serial_mode_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_t g_frame_consume_t;                             // frame消费frame线程 //

static HDBlockingQueueUint8 *g_frame_queue = NULL;              // 协议帧队列

static volatile uint32_t g_delay = 0;                           // 485帧间隔delay
static char g_pic_dir_path[2048];                               // 当前照片存储目录
static volatile int g_running = 0;                              // 程序是否在运行
static hd_on_action_id_changed g_hd_on_action_id_changed = NULL;// 收到action_id回调
static hd_on_event g_hd_on_event = NULL;                        // 收到event回调

static unsigned char shell_resp_buff[MAX_RESULT_LENGTH];               // shell回复缓冲区
static unsigned g_file_buffer[MAX_FILE_SIZE] = {0};             // 图片文件缓冲区
static unsigned int g_file_buffer_size = -1;                    // 当前上传文件大小
static uint8_t g_file_pic_id = 0;                               // 当前上传文件pic_id
static int g_file_pulling = 0;                                  // 当前上传文件中

static char g_push_mode_file_path[1024];                        // push文件path
static uint64_t g_push_mode_file_size = -1;                     // push文件size
static unsigned char g_push_mode_file_md5[16];                  // push文件md5
static FILE *g_push_mode_file = NULL;                           // push file

static char *g_pull_mode_file_path[1024];                       // extra_pull文件path
static uint64_t g_pull_mode_file_size = -1;                     // extra_pull文件size
static unsigned char g_pull_mode_file_md5[16];                  // extra_pull文件md5
static FILE *g_pull_mode_file = NULL;                           // extra_pull file

static unsigned char PULL_MODE_FILE_HEADER_TAIL[8] = {          // hadlinks
        'h', 'a', 'd', 'l', 'i', 'n', 'k', 's'
};



// 隐藏的实现 a



static void do_uart_recv(uint8_t str);

static int do_parse_pic_info(const char *file_name, uint32_t *snapshot_timestamps, uint8_t *pic_id);

static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, uint32_t *frame_length);

static int do_str_2_action_id(const char *action_id_str, uint32_t *action_id_timestamps, uint8_t *action_id_index);

static int do_collect_all_pic_infos(hd_dynamic_pic_info **pic_infos, uint32_t *pic_infos_size);

static int do_collect_all_pic_infos_v2(hd_dynamic_pic_info *pic_infos, uint32_t *pic_infos_size);

static int do_find_pic_by_pic_id(uint8_t pic_id, char **file_path);

static int do_cut_file_data(const char *file_name, unsigned char **result, size_t *size,
                            uint32_t offset,
                            uint32_t read_len
);

static int handle_uart_data(const unsigned char *raw, size_t raw_size);
// 隐藏的实现 z

// 实现
static void reset_file_buffer() {
    g_file_pic_id = 0;
    g_file_pulling = 0;
    g_file_buffer_size = -1;
}

int hd_camera_uart_write(const unsigned char *raw, size_t raw_size) {
    if (g_running == 0) {
        return -5;
    }
    if (raw == NULL || raw_size <= 0) {
        return -1;
    }

    hd_printf_buff(raw, raw_size, "发送", 0);

    rs485_pwr_on();
    usleep(9000);
    for (int i = 0; i < raw_size; ++i) {
        if (!g_running)break;
        rk_uart_sendbyte(raw[i]);
        usleep(16);
    }
    usleep(4000);
    rs485_pwr_off();
    if (HD_UART_PARSER_DEBUG) {
        log_info("hd_camera_uart_write over \n");
    }
    return 0;
}

// 帧解析函数
static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, uint32_t *frame_length) {
    //log_info("<接受>%02x \n", str);
    // c语言实现
    // 串口数据帧格式为：
    // 帧头（2字节）:固定为0xaa5a
    // 从机地址(1字节)
    // 命令（1字节）
    // 数据长度（4字节）
    // 数据（N字节）
    // CRC16（2字节）
    // 比如aa 5a 01 1e 05 00 00 00 78 56 34 12 00 9c dd
    // 帧头：aa 5a
    // 从机地址：01
    // 命令：1e
    // 数据长度：05 00 00 00 （长度为5）
    // 数据：78 56 34 12 00
    // CRC16：9c dd

    // 目标：按照一个字节一个字节解析 从数据流中解析出：aa 5a 01 1e 05 00 00 00 78 56 34 12 00 9c dd整条数据。
    static ParserState state = STATE_WAIT_HEADER_H;
    static uint16_t data_index = 0;
    static uint32_t data_len = 0;
    static uint16_t expected_crc = 0;
    static uint16_t calculated_crc = 0;
    static uint16_t current_pos = 0;
    if (!g_running) {
        log_warn("not running\n");
        state = STATE_WAIT_HEADER_H;
        current_pos = 0;
        return 5;
    }
    switch (state) {
        case STATE_WAIT_HEADER_H:
            if (byte == FRAME_HEADER_H) {
                frame_buffer[0] = byte;
                current_pos = 1;
                state = STATE_WAIT_HEADER_L;
            }
            break;

        case STATE_WAIT_HEADER_L:
            if (byte == FRAME_HEADER_L) {
                frame_buffer[1] = byte;
                current_pos = 2;
                state = STATE_WAIT_ADDR;
            } else {
                current_pos = 0;
                state = STATE_WAIT_HEADER_H;
            }
            break;

        case STATE_WAIT_ADDR:

            // 判断addr
//            if ((byte != g_addr) && (byte != PROTOCOL_BROADCAST)) {
//                state = STATE_WAIT_HEADER_H;
//                current_pos = 0;
//                return -4;
//            }
            frame_buffer[2] = byte;
            current_pos = 3;
            state = STATE_WAIT_CMD;
            break;

        case STATE_WAIT_CMD:
//            if (
//
//                    (byte != CMD_HD_PROPERTY_GET)
//                    && (byte != CMD_HD_PROPERTY_SET)
//                    && (byte != CMD_HD_CAMERA_SNAPSHOT)
//                    && (byte != CMD_HD_PIC_INFO)
//                    && (byte != CMD_HD_PIC_DELETE)
//                    && (byte != CMD_HD_PIC_PULL)
//                    && (byte != CMD_HD_PIC_PULL_COMPLETED)
//                    && (byte != CMD_HD_BROADCAST_ACTION_ID)
//                    && (byte != CMD_HD_PUSH_FILE)
//                    && (byte != CMD_HD_PUSH_FILE_SEND)
//                    && (byte != CMD_HD_EXTRA_SHELL)
//                    && (byte != CMD_HD_EXTRA_PULL)
//                    && (byte != CMD_HD_EXTRA_PUSH)
//                    ) {
//                state = STATE_WAIT_HEADER_H;
//                current_pos = 0;
//                return -7;
//            }
            frame_buffer[3] = byte;
            current_pos = 4;
            state = STATE_WAIT_LEN_H;
            break;

        case STATE_WAIT_LEN_H:
            frame_buffer[4] = byte;
            current_pos = 5;
            state = STATE_WAIT_LEN_MH;
            break;

        case STATE_WAIT_LEN_MH:
            frame_buffer[5] = byte;
            current_pos = 6;
            state = STATE_WAIT_LEN_ML;
            break;

        case STATE_WAIT_LEN_ML:
            frame_buffer[6] = byte;
            current_pos = 7;
            state = STATE_WAIT_LEN_L;
            break;

        case STATE_WAIT_LEN_L:
            frame_buffer[7] = byte;
            current_pos = 8;
            // 解析数据长度 (小端格式)
            data_len = (uint32_t) frame_buffer[7] << 24 |
                       (uint32_t) frame_buffer[6] << 16 |
                       (uint32_t) frame_buffer[5] << 8 |
                       frame_buffer[4];

            if (data_len > PROTOCOL_MAX_FRAME_LEN) {
                log_warn("data_len = %u ,frame_length = %u,current_pos=%u\n", data_len, frame_length, current_pos);
                for (int i = 0; i < 40; ++i) {
                    log_warn("frame_buffer[%d] = %02x \n", i, frame_buffer[i]);
                }
                log_warn("全局信息：\n");
                log_warn("data_index        =  %02x (%d)\n", data_index, data_index);
                log_warn("data_len          =  %02x (%d)\n", data_len, data_len);
                log_warn("expected_crc      =  %02x (%d)\n", expected_crc, expected_crc);
                log_warn("calculated_crc    =  %02x (%d)\n", calculated_crc, calculated_crc);
                log_warn("current_pos       =  %02x (%d)\n", current_pos, current_pos);
                log_warn("state             =  %02x (%d)\n", state, state);
                log_warn("frame_length      =  %02x (%d)\n", frame_length, frame_length);

                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return -3;
            } else if (data_len == 0) {
                state = STATE_WAIT_CRC_H;
            } else {
                state = STATE_WAIT_DATA;
                data_index = 0;
            }
            break;

        case STATE_WAIT_DATA:
            frame_buffer[8 + data_index] = byte;
            current_pos = 8 + data_index + 1;
            data_index++;

            if (data_index >= data_len) {
                state = STATE_WAIT_CRC_H;
            }
            break;

        case STATE_WAIT_CRC_H:
            frame_buffer[8 + data_len] = byte;
            current_pos = 8 + data_len + 1;
            expected_crc = byte;
            state = STATE_WAIT_CRC_L;
            break;


        case STATE_WAIT_CRC_L: {
            frame_buffer[8 + data_len + 1] = byte;
            current_pos = 8 + data_len + 2;
            expected_crc |= (byte << 8);
            calculated_crc = hd_crc16(&frame_buffer[0], 8 + data_len);
            if (calculated_crc == expected_crc) {
                *frame_length = current_pos;
                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return 0;
            } else {

                log_warn("data_len = %u ,frame_length = %u,current_pos=%u\n", data_len, frame_length, current_pos);
                for (int i = 0; i < 20; ++i) {
                    log_warn("frame_buffer[%d] = %02x \n", i, frame_buffer[i]);
                }
                log_warn("全局信息：\n");
                log_warn("data_index        =  %02x (%d)\n", data_index, data_index);
                log_warn("data_len          =  %02x (%d)\n", data_len, data_len);
                log_warn("expected_crc      =  %02x (%d)\n", expected_crc, expected_crc);
                log_warn("calculated_crc    =  %02x (%d)\n", calculated_crc, calculated_crc);
                log_warn("current_pos       =  %02x (%d)\n", current_pos, current_pos);
                log_warn("state             =  %02x (%d)\n", state, state);
                log_warn("frame_length      =  %02x (%d)\n", frame_length, frame_length);

                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return -2;
            }
            break;
        }

        default:
            log_error("default current_pos = %d \n", current_pos);
            state = STATE_WAIT_HEADER_H;
            current_pos = 0;
            break;
    }

    return -1;
}

static void resetPushMode() {
    if (g_push_mode_file != NULL) {
        fclose(g_push_mode_file);
        g_push_mode_file = NULL;
    }
    memset(g_push_mode_file_path, 0, sizeof(g_push_mode_file_path));
    memset(g_push_mode_file_md5, 0, sizeof(g_push_mode_file_md5));
    g_push_mode_file_size = -1;
    hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
}

static void resetPullMode() {
    if (g_push_mode_file != NULL) {
        fclose(g_pull_mode_file);
        g_pull_mode_file = NULL;
    }
    memset(g_pull_mode_file_path, 0, sizeof(g_pull_mode_file_path));
    memset(g_pull_mode_file_md5, 0, sizeof(g_pull_mode_file_md5));
    g_pull_mode_file_size = -1;
    hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
}

static void on_push_delete_and_reply(int success) {
    unsigned char *out_protocol_data = NULL;
    uint32_t out_protocol_data_size;
    uint8_t in_slave_addr;
    uint8_t in_property_id;
    uint8_t in_result;
    int ret = hd_slave_property_set_encode(&out_protocol_data, &out_protocol_data_size,
                                           g_addr, PROPERTY_HD_ID_PUSH, success);
    if (ret)return;
    hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
    free(out_protocol_data);
}


static void do_uart_recv_with_pull(uint8_t str) {

}

static void do_uart_recv_with_push(uint8_t str) {
    if (g_push_mode_file == NULL)return;
    //log_debug("<do_uart_recv_to_file> %02x \n",str);
    static ssize_t file_size = 0;
    unsigned char uc = str;
    size_t written = fwrite(&uc, 1, 1, g_push_mode_file);
    if (written != 1) {
        perror("Failed to write byte\n");
        // todo 删除文件
        resetPushMode();
        on_push_delete_and_reply(1);
        return;
    }
    file_size++;
    if (file_size == g_push_mode_file_size) {
        fclose(g_push_mode_file);
        g_push_mode_file = NULL;
        sync();
        log_info("push 完毕！文件：%s ，大小：%d\n", g_push_mode_file_path, file_size);
        // 校验md5
        unsigned char result[16];
        int ret = hd_md5(g_push_mode_file_path, result);
        if (ret) {
            log_warn("md5生成 fail :%d\n", ret);
            // todo 删除文件
            resetPushMode();
            on_push_delete_and_reply(2);
            return;
        }
        if (hd_array_cmp(result, 16, g_push_mode_file_md5, 16) == 0) {
            log_info("文件push成功！\n");
            resetPushMode();
            on_push_delete_and_reply(0);
        } else {
            log_warn("md5不同");
            resetPushMode();
            // todo 删除文件
            on_push_delete_and_reply(3);
        }
    }
}

static void do_uart_recv_with_shell(uint8_t str) {
    if (str == 0x00 || str == 0x0d) { // 消除空格键、回车键
        return;
    }
    log_debug("[do_uart_recv_str]收到字符：%02x\n", str);
    static unsigned char shell_buff[MAX_COMMAND_LENGTH];

    static int do_uart_recv_str_index = 0;
    int ret;

    // 检查是否接收到回车换行符（假设'\n'表示结束）
    if (str == '\n') {
        // 确保字符串以null结尾
        if (do_uart_recv_str_index < MAX_COMMAND_LENGTH - 1) {
            shell_buff[do_uart_recv_str_index] = '\0';
        } else {
            shell_buff[MAX_COMMAND_LENGTH - 1] = '\0';
        }

        // 打印接收到的字符串
        log_info("Received Shell: %s\n", shell_buff);
        // 处理命令
        if (strcmp((char *) shell_buff, "#exit#") == 0) {
            pthread_mutex_lock(&g_serial_mode_mutex);
            int mode = g_serial_mode;
            pthread_mutex_unlock(&g_serial_mode_mutex);
            if (mode == HD_SERIAL_NORMAL_MODE) {
                log_info("已经取消工厂模式%d\n");
                unsigned char buf[] = {g_serial_mode};
                hd_camera_uart_write(buf, sizeof(buf));
                return;
            }
            hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
            log_info("取消工厂模式成功！\n");
            // 回复
            const char *resp = "取消工厂模式成功!\n";

            // 复制数据（包括 '\0' 结束符）
            unsigned long size = strlen(resp) + 1;
            log_info("发送shell命令长度=%d\n", size);
            memcpy(shell_resp_buff, resp, size);
            hd_camera_uart_write(shell_resp_buff, size);

            // 重置索引以便接收下一条命令
            do_uart_recv_str_index = 0;
        } else {
            log_info("shell命令 shell_buff=[%s] index=%d\n", shell_buff, do_uart_recv_str_index);
            for (int i = 0; i < do_uart_recv_str_index; ++i) {
                printf("%02x\n", shell_buff[i]);
            }
            char buff[1024];
            for (int i = 0; i <= do_uart_recv_str_index; ++i) {
                buff[i] = (char) shell_buff[i];
            }
            ret = hd_camera_shell_exec(buff, (char *) shell_resp_buff);
            if (ret) {
                log_warn("hd_camera_shell_exec error:%d\n", ret);
                // 重置索引以便接收下一条命令
                do_uart_recv_str_index = 0;
                return;
            }
            printf("执行结束！\n");
            log_info("exec result :\n");
            log_info("%s", shell_resp_buff);
            log_info("\n");
            hd_camera_uart_write(shell_resp_buff, strlen((char *) shell_resp_buff));
        }

        // 重置索引以便接收下一条命令
        do_uart_recv_str_index = 0;
    } else {
        // 将字符添加到缓冲区，但不要超过缓冲区大小
        if (do_uart_recv_str_index < MAX_COMMAND_LENGTH - 1) {
            shell_buff[do_uart_recv_str_index++] = str;
        } else {
            // 缓冲区已满，可以在这里处理错误或重置缓冲区
            printf("Error: Command too long!\n");
            do_uart_recv_str_index = 0;
        }
    }
}

static pthread_mutex_t g_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t g_frame_buffer[PROTOCOL_MAX_FRAME_LEN];  // 串口帧缓冲区
static uint32_t g_frame_length = 0;                     // 一个完整帧的数据长度

/*
 static void do_uart_recv_v2(uint8_t str) {
    int ret;
    ret = parse_serial_frame(str, g_frame_buffer, &g_frame_length);

    if (ret == 0) {
        uint32_t len = g_frame_length;
        uint8_t tmp[PROTOCOL_MAX_FRAME_LEN];

        pthread_mutex_lock(&g_buffer_mutex);
        memcpy(tmp, g_frame_buffer, len);
        pthread_mutex_unlock(&g_buffer_mutex);

        g_frame_length = 0;
        hd_printf_buff(tmp, len, "收到", 0);
        int result = handle_uart_data(tmp, len);
        log_debug("handle_uart_data = %d", result);
    } else if (ret == -1) {
        // 处理中。。。
    } else if (ret == 5) {
        g_frame_length = 0;
    } else if (ret == -2) {
        LOGW("CRC error \n");
        g_frame_length = 0;
    } else if (ret == -3) {
        LOGW("len error\n");
        g_frame_length = 0;
    } else {
        // 处理中。。。
    }


}*/

static void do_uart_recv(uint8_t str) {
    hd_queue_put_uint8(g_frame_queue, str);
//    do_uart_recv_v2(str);
}

static void do_uart_recv_take(uint8_t str) {
    int HD_PARSE_FRAME_QUEUE = 0;
    int ret;
    ret = parse_serial_frame(str, g_frame_buffer, &g_frame_length); // 产生完整的包

    if (ret == 0) {
        if (g_frame_length < 10) {
            log_warn("g_frame_length < 10 \n");
            return;
        }
//        hd_printf_buff(g_frame_buffer, g_frame_length, "接受", 0);
//        int result = handle_uart_data(g_frame_buffer, g_frame_length);
        uint32_t len = g_frame_length;
        uint8_t *tmp = malloc(len);
//        uint8_t tmp[PROTOCOL_MAX_FRAME_LEN];
        memcpy(tmp, g_frame_buffer, len);
        hd_printf_buff(tmp, len, "接受", 0);
        int result = handle_uart_data(tmp, len);
        free(tmp);
        //log_debug("handle_uart_data = %d", result);
    } else if (ret == -1) {
        // 处理中。。。
    } else if (ret == 5) {
        g_frame_length = 0;
    } else if (ret == -2) {
        log_warn("CRC error \n");
        g_frame_length = 0;
    } else if (ret == -3) {
        log_warn("len error\n");
        g_frame_length = 0;
    } else if (ret == -7) {
        log_warn("cmd error\n");
        g_frame_length = 0;
    } else {
        // 处理中。。。
    }


}

// 验证字符串是否以.jpg结尾
static int ends_with_jpg(const char *str) {
    size_t len = strlen(str);
    if (len < JPG_SUFFIX_LEN) return 0;
    return strcmp(str + len - JPG_SUFFIX_LEN, JPG_SUFFIX) == 0;
}

static int load_file_to_buffer(const char *filename) {
    if (filename == NULL) {
        fprintf(stderr, "错误：文件名不能为NULL\n");
        return -1;
    }

    FILE *file = fopen(filename, "rb");
    if (file == NULL) {
        fprintf(stderr, "无法打开文件 %s: %s\n", filename, strerror(errno));
        return -1;
    }

    // 获取文件大小
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    // 检查文件大小是否合适
    if (file_size < 0) {
        fprintf(stderr, "获取文件大小失败: %s\n", strerror(errno));
        fclose(file);
        return -1;
    }

    if ((size_t) file_size > sizeof(g_file_buffer)) {
        fprintf(stderr, "文件太大 (%ld字节)，最大支持 %zu字节\n",
                file_size, sizeof(g_file_buffer));
        fclose(file);
        return -1;
    }

    // 读取文件内容
    size_t bytes_read = fread(g_file_buffer, 1, file_size, file);
    if (bytes_read != (size_t) file_size) {
        log_warn("读取文件不完全，期望 %ld字节，实际读取 %zu字节\n",
                 file_size, bytes_read);
//        fclose(file);
//        return -1;
    }

    g_file_buffer_size = bytes_read;
    fclose(file);

    printf("成功加载 %u 字节数据到缓冲区\n", g_file_buffer_size);
    return 0;
}

/**
 * @brief 获取已加载文件的大小
 *
 * @return size_t 文件大小(字节数)
 */
static size_t get_loaded_file_size() {
    return g_file_buffer_size;
}

/**
 * @brief 获取文件缓冲区指针
 *
 * @return const unsigned char* 指向缓冲区的指针
 */
static const unsigned char *get_file_buffer() {
    return (const unsigned char *) g_file_buffer;
}

/*
// 解析图片
// 图片的格式:
// 文件名格式：prefix_01_TIMESTAMP_PICID.jpg
// 文件名格式：prefix_type_angel_01_TIMESTAMP_PICID.jpg
// prefix       :   其他
// TIMESTAMP    :   时间戳秒数
// PICID        :   0-255循环自增
// .jpg         :   图片默认格式
// 例子：解析字符串xxxxx_1747878695_112.jpg 解析为snapshot_timestamps：1747878695  pic_id：112
*/
static int do_parse_pic_info(const char *file_name, uint32_t *snapshot_timestamps, uint8_t *pic_id) {
    char *copy = strdup(file_name);
    if (!copy) {
        return -1; // 内存分配失败
    }

    char *token;
    char *last_token = NULL;
    char *second_last_token = NULL;

    // 第一次分割
    token = strtok(copy, "_");
    while (token != NULL) {
        second_last_token = last_token;
        last_token = token;
        token = strtok(NULL, "_");
    }

    // 检查是否至少有2个下划线分隔的部分
    if (!second_last_token || !last_token) {
        free(copy);
        return -1; // 格式不正确
    }

    // 处理时间戳部分
    char *endptr;
    unsigned long timestamp = strtoul(second_last_token, &endptr, 10);
    if (*endptr != '\0' || timestamp > UINT32_MAX) {
        free(copy);
        return -2; // 无效的时间戳
    }
    *snapshot_timestamps = (uint32_t) timestamp;

    // 处理图片ID部分（需要去掉.jpg后缀）
    char *dot = strchr(last_token, '.');
    if (!dot) {
        free(copy);
        return -3; // 没有文件扩展名
    }
    *dot = '\0'; // 截断.jpg部分

    unsigned long id = strtoul(last_token, &endptr, 10);
    if (*endptr != '\0' || id > UINT8_MAX) {
        free(copy);
        return -4; // 无效的图片ID
    }
    *pic_id = (uint8_t) id;

    free(copy);
    return 0; // 成功
}

/*
 * 解析
 * 比如字符串 1747814636001,1747814636解析为action_id_timestamps（单位秒 4个字节）；001解析为action_id_index
 * @param action_id_str
 * @param action_id_timestamps
 * @param action_id_index
 * @return
 */
static int do_str_2_action_id(const char *action_id_str, uint32_t *action_id_timestamps, uint8_t *action_id_index) {
    // 检查输入参数是否有效
    if (action_id_str == NULL || action_id_timestamps == NULL || action_id_index == NULL) {
        return -1;
    }

    size_t len = strlen(action_id_str);

    // 检查是否全是数字
//    for (size_t i = 0; i < len; i++) {
//        if (!isdigit(action_id_str[i])) {
//            return -1;
//        }
//    }

    // 时间戳部分至少需要10位（可以表示到2286年）
//    if (len < 10) {
//        return -1;
//    }

    // 分离时间戳和索引
    char ts_str[11] = {0};  // 10位时间戳 + null终止符
    char idx_str[4] = {0};   // 最多3位索引 + null终止符

    // 拷贝时间戳部分（前10位）
    strncpy(ts_str, action_id_str, 10);

    // 拷贝索引部分（剩余部分，最多3位）
    size_t idx_len = len - 10;
    if (idx_len > 3) {
        idx_len = 3;  // 索引最多3位
    }
    strncpy(idx_str, action_id_str + 10, idx_len);

    // 转换为数值
    char *endptr;
    unsigned long ts = strtoul(ts_str, &endptr, 10);
    if (*endptr != '\0' || ts > UINT32_MAX) {
        return -1;
    }

    unsigned long idx = strtoul(idx_str, &endptr, 10);
    if (*endptr != '\0' || idx > UINT8_MAX) {
        return -1;
    }

    *action_id_timestamps = (uint32_t) ts;
    *action_id_index = (uint8_t) idx;
    return 0;
}

static int do_action_id_2_str(char *str, size_t str_size, uint32_t action_id_timestamps, uint8_t action_id_index) {
    snprintf(str, str_size, "%d%03d", action_id_timestamps, action_id_index);
    return 0;
}

static int do_collect_all_pic_infos_v2(hd_dynamic_pic_info *pic_infos, uint32_t *pic_infos_size) {
    DIR *dir = opendir(g_pic_dir_path);                         // ###1
    if (!dir) {
        log_warn("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }
    char action_id_name_temp[1024];     // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息
    int total = 0;

    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        if (access(g_pic_dir_path, F_OK) == -1) {
            log_warn(" access error %s \n", g_pic_dir_path);
            break;
        }
        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {
            continue;
        }
        // 获取action_id文件夹名称
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            log_debug("%-20s %-4d %s\n", action_id_dir_entry->d_name, action_id_dir_entry->d_type, action_id_name_temp);
        }
        int ret;
        uint32_t temp_action_id_timestamps = 0;            // 当前temp_action_id_timestamps
        uint8_t temp_action_id_index = 0;            // 当前temp_action_id_index
        // 解析action_id
        ret = do_str_2_action_id(action_id_name_temp,
                                 &temp_action_id_timestamps, &temp_action_id_index);
        if (ret) {
            log_warn("      解析action_id失败 ： %s %d\n", action_id_name_temp, ret);
            continue;
        }

        // action_id文件夹信息
        struct dirent *pic_file_entry;      // 图片文件信息
        char pic_file_name[1024];           // 图片名称
        char action_id_dir_path[1024];      // action_id目录路径，比如/userdata/crop_images/121414141001
        char pic_file_path_tmp[1024];       // 图片path，比如/userdata/crop_images/121414141001/xxx22222xx_1747878695_2.jpg
        unsigned char md5_result_tmp[16];   // 图片md5
        uint32_t snapshot_timestamps_temp;  // 抓图时间戳
        uint8_t pic_id_temp;                // pic_id_temp

        snprintf(action_id_dir_path, sizeof(action_id_dir_path), "%s/%s", g_pic_dir_path, action_id_name_temp);
        DIR *action_id_dir = opendir(action_id_dir_path);           //######2

        if (!action_id_dir) {
            log_warn("无法打开目录 %s \n", action_id_dir_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {

            if (access(action_id_dir_path, F_OK) == -1) {
                log_warn(" access error action_id_dir %s \n", action_id_dir_path);
                break;
            }

            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            if (total >= 1024) {
                log_warn("Too many pictures, maximum is 1024");
                break;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            if (HD_UART_PARSER_DEBUG) {
                log_debug("开始解析图片文件:<%s> \n", pic_file_name);
            }
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                log_warn("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }

            // 创建图片信息。。。。。。。。。。开始

            if (HD_UART_PARSER_DEBUG) {
                log_warn("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp, pic_id_temp);
            }
            snprintf(pic_file_path_tmp, sizeof(pic_file_path_tmp), "%s/%s", action_id_dir_path, pic_file_name);
            if (HD_UART_PARSER_DEBUG) {
                log_debug("获取图片md5...%s\n", pic_file_path_tmp);
            }
            ret = hd_md5(pic_file_path_tmp, md5_result_tmp);
            if (ret) {
                log_error("hd_md5 fail.\n");
                continue;
            }

            if (HD_UART_PARSER_DEBUG) {
                log_debug("获取图片大小...\n");
            }
            struct stat st_tmp;
            if (stat(pic_file_path_tmp, &st_tmp) != 0) {
                log_error("stat fail\n");
                break;
            }

            uint32_t size = st_tmp.st_size;
            if (HD_UART_PARSER_DEBUG) {
                log_debug("图片信息：\n");
                log_debug("name                    =          %s\n", pic_file_name);
                log_debug("id                      =          %hhu\n", pic_id_temp);
                log_debug("size                    =          %u\n", size);
                log_debug("action_id_timestamps    =          %d\n", temp_action_id_timestamps);
                log_debug("action_id_index         =          %d\n", temp_action_id_index);
                log_debug("snapshot_timestamps     =          %u\n", temp_action_id_timestamps);
                log_debug("md5                     =          ");

                for (int j = 0; j < 16; ++j) {
                    printf("%02x ", md5_result_tmp[j]);
                }
                log_debug("\n");
            }
            if (HD_UART_PARSER_DEBUG) {
                log_debug("创建图片信息\n");
            }
            hd_dynamic_pic_info info;//= (hd_dynamic_pic_info *) malloc(sizeof(hd_dynamic_pic_info));
            info.action_id_index = temp_action_id_index;
            info.action_id_timestamps = temp_action_id_timestamps;
            info.snapshot_timestamps = snapshot_timestamps_temp;
            info.size = size;
            info.id = pic_id_temp;

            memcpy(info.md5, md5_result_tmp, sizeof(info.md5));

            pic_infos[total] = info;
            total++;
            if (HD_UART_PARSER_DEBUG) {
                log_debug("创建图片信息 结束\n");
            }
        }

        closedir(action_id_dir);                                //######2
    }

    closedir(dir);                                              //###1

    *pic_infos_size = total;
    return 0;
}

static int do_find_pic_by_pic_id(uint8_t pic_id, char **file_path) {
    if (HD_UART_PARSER_DEBUG) {
        log_debug("do_find_pic_by_pic_id pic_id = %d,g_pic_dir_path=%s\n", pic_id, g_pic_dir_path);
        log_debug("1.搜索目录[%s]\n", g_pic_dir_path);
    }
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        log_warn("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息

    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            log_debug("%-20s %-4d %s\n", action_id_dir_entry->d_name,
                      action_id_dir_entry->d_type,
                      action_id_name_temp);
        }
        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {

            continue;
        }
        // 继续校验名称
        // 分别打开文件夹查询pic_id匹配的图片
        struct dirent *pic_file_entry;  //   图片文件信息
        char pic_file_name[1024];       //   图片名称
        char result[1024];              // 图片path
        char action_id_path[1024];      // action_id目录路径temp
        int ret;
        uint32_t snapshot_timestamps_temp;
        uint8_t pic_id_temp;
        snprintf(action_id_path, sizeof(action_id_path), "%s/%s", g_pic_dir_path, action_id_name_temp);
        DIR *action_id_dir = opendir(action_id_path);
        if (!action_id_dir) {
            log_warn("无法打开目录 %s \n", action_id_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            if (HD_UART_PARSER_DEBUG) {
                log_debug("开始解析图片文件:<%s> \n", pic_file_name);
            }
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                log_warn("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }

            if (pic_id_temp == pic_id) {
                if (HD_UART_PARSER_DEBUG) {
                    log_info("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp,
                             pic_id_temp);
                }
                snprintf(result, sizeof(result), "%s/%s", action_id_path, pic_file_name);
                *file_path = strdup(result);
                closedir(action_id_dir);
                closedir(dir);
                return 0;
            }
        }
        closedir(action_id_dir);
    }
    log_warn("解析图片文件 没有找到：%d\n", pic_id);
    closedir(dir);
    return 2;
}

static int do_find_pic_by_pic_id_action_id(uint8_t pic_id, char **file_path, uint32_t action_id_stamp, uint8_t index) {
    if (HD_UART_PARSER_DEBUG) {
        log_debug("do_find_pic_by_pic_id pic_id = %d,g_pic_dir_path=%s\n", pic_id, g_pic_dir_path);
        log_debug("1.搜索目录[%s]\n", g_pic_dir_path);
    }
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        log_warn("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息

    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        if (access(g_pic_dir_path, F_OK) == -1) {
            log_warn(" access error %s \n", g_pic_dir_path);
            break;
        }

        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {
            continue;
        }

        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            log_debug("%-20s %-4d %s\n", action_id_dir_entry->d_name,
                      action_id_dir_entry->d_type,
                      action_id_name_temp);
        }
        // 继续校验名称
        // 分别打开文件夹查询pic_id匹配的图片
        struct dirent *pic_file_entry;  // 图片文件信息
        char pic_file_name[1024];       // 图片名称
        char result[1024];              // 图片path
        char action_id_dir_path[1024];  // action_id目录路径temp
        int ret;
        uint32_t snapshot_timestamps_temp;
        uint8_t pic_id_temp;
        snprintf(action_id_dir_path, sizeof(action_id_dir_path), "%s/%s", g_pic_dir_path, action_id_name_temp);

        // 判断action_id index
//        uint32_t temp_action_id_timestamps = 0; // 当前temp_action_id_timestamps
//        uint8_t temp_action_id_index = 0;             // 当前temp_action_id_index
//        ret = do_str_2_action_id(action_id_name_temp,
//                                 &temp_action_id_timestamps, &temp_action_id_index);
//        if (ret) {
//            log_warn("     %s 不存在此action_id %d %d ret=%d\n",action_id_name_temp, action_id_stamp, index, ret);
//            continue;
//        }
//        if ((action_id_stamp != temp_action_id_timestamps) || (temp_action_id_index != index)) {
//            log_warn("     %s找不到此action_id %d %d != %d %d\n",action_id_name_temp,temp_action_id_timestamps, temp_action_id_index,action_id_stamp, index);
//            continue;
//        }

        DIR *action_id_dir = opendir(action_id_dir_path);
        if (!action_id_dir) {
            log_warn("无法打开目录 %s \n", action_id_dir_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
            if (access(action_id_dir_path, F_OK) == -1) {
                log_warn(" access error %s \n", action_id_dir_path);
                break;
            }
            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            if (HD_UART_PARSER_DEBUG) {
                log_debug("开始解析图片文件:<%s> \n", pic_file_name);
            }
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                log_warn("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }

            if (pic_id_temp == pic_id) {
                if (HD_UART_PARSER_DEBUG) {
                    log_info("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp,
                             pic_id_temp);
                }
                snprintf(result, sizeof(result), "%s/%s", action_id_dir_path, pic_file_name);
                *file_path = strdup(result);
                closedir(action_id_dir);
                closedir(dir);
                return 0;
            }
        }
        closedir(action_id_dir);
    }
    log_warn("解析图片文件 没有找到：%d\n", pic_id);
    closedir(dir);
    return 2;
}

/*
 * 从文件file_name offset位置开始读取read_len长度的数据到result里面，size为实际读到的数据大小
 */
static int do_cut_file_data(const char *file_name, unsigned char **result, size_t *size,
                            uint32_t offset,
                            uint32_t read_len
) {
    FILE *file = NULL;
    unsigned char *buffer = NULL;
    size_t bytes_read = 0;

    // 打开文件
    file = fopen(file_name, "rb");
    if (file == NULL) {
        fprintf(stderr, "Failed to open file %s: %s\n", file_name, strerror(errno));
        return -1;
    }

    // 定位到offset位置
    if (fseek(file, offset, SEEK_SET) != 0) {
        fprintf(stderr, "Failed to seek to offset %u in file %s: %s\n",
                offset, file_name, strerror(errno));
        fclose(file);
        return -2;
    }

    // 分配内存
    buffer = (unsigned char *) malloc(read_len);
    if (buffer == NULL) {
        fprintf(stderr, "Failed to allocate memory for reading\n");
        fclose(file);
        return -3;
    }

    // 读取数据
    bytes_read = fread(buffer, 1, read_len, file);
    if (bytes_read == 0 && ferror(file)) {
        fprintf(stderr, "Failed to read from file %s: %s\n",
                file_name, strerror(errno));
        free(buffer);
        fclose(file);
        return -4;
    }

    // 设置输出参数
    *result = buffer;
    *size = bytes_read;

    // 关闭文件
    fclose(file);
    return 0;
}

/************ handles a **************/

static int
handle_property_get(
        const unsigned char *payload_data, uint32_t payload_data_size,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
) {
    uint8_t property_id_out;
    uint8_t ret = hd_slave_property_get_decode(&property_id_out, payload_data, payload_data_size);
    if (ret)return -1;
    switch (property_id_out) {
        case PROPERTY_HD_ID_MODEL_FILE_NAME: {
            log_info("[%d获取模型名称]%02x\n", g_addr, property_id_out);
            ret = hd_camera_ota_version(
                    property_id_out,
                    payload_data, payload_data_size, protocol_data_out, protocol_data_size_out);
            return ret;
        }
        case PROPERTY_HD_ID_DEBUG_ACTION_ID: {
            log_info("[%d获取属性]%02x DEBUG action_id\n", g_addr, property_id_out);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ANGEL: {
            log_info("[%d获取属性]%02x DEBUG angel\n", g_addr, property_id_out);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_HD_UART_VERSION: {
            log_info("[%d获取属性]%02x DEBUG hd_uart version\n", g_addr, property_id_out);
            char version[2048];
            snprintf(version, sizeof(version), "%s-%s", (g_hd_app_version == NULL) ? "" : g_hd_app_version,
                     hd_uart_version());
//            char *version = strcat(hd_uart_version(), (g_version == NULL) ? "" : g_version);
//            char *str = (char *)malloc(result_value_size_out + 1); // +1 用于 null 终止符
//            if (str == NULL) {
//                break;
//            }
//            memcpy(str, result_value_out, result_value_size_out);
//            str[result_value_size_out] = '\0';
//            free(str);

            log_info("hd_uart版本: %s\n", version);
            size_t len = strlen(version);
            unsigned char *result = (unsigned char *) malloc(len);
            if (!result) {
                break;
            }
            memcpy(result, version, len);
//            result[len] = '\0';
            ret = hd_slave_property_get_encode(protocol_data_out, protocol_data_size_out,
                                               g_addr, property_id_out, 0, result, len);
            free(result);
            return ret;
        }

        default: {
            break;
        }
    }
    return -1;
}

static int
handle_property_set(const unsigned char *payload_data, uint32_t payload_data_size) {

    if (g_addr == 0)return -1;
    log_debug("handle_property_set\n");
    uint8_t ret;
    unsigned char *protocol_data_out = NULL;
    uint32_t protocol_data_size_out;

    hd_printf_buff(payload_data, payload_data_size, "event-raw", 0);
    uint8_t property_id_out;
    unsigned char *result_value_out;
    uint32_t result_value_size_out = 0;
    // 解析
    ret = hd_slave_property_set_decode(&property_id_out, &result_value_out, &result_value_size_out,
                                       payload_data, payload_data_size);
    if (ret) {
        log_warn("hd_slave_property_set_decode error\n");
        return -1;
    }
    hd_printf_buff(result_value_out, result_value_size_out, "event", 0);
    if (HD_UART_PARSER_DEBUG) {
        log_debug("开始处理event\n");
    }
    switch (property_id_out) {
        case PROPERTY_HD_ID_DEBUG: {
            log_info("[%d设置属性]%02x DEBUG开关\n", g_addr, PROPERTY_HD_ID_DEBUG);
            if (result_value_size_out == 1) {
                uint8_t on = result_value_out[0];
                if (on >= HD_LOGGER_LEVEL_DEBUG && on <= HD_LOGGER_LEVEL_ERROR) {
                    hd_logger_set_level(on);
                    log_info("设置开关成功:%d\n", on);
                    ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                       g_addr, property_id_out, 1);
                    if (ret == 0) {
                        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                        if (ret) {
                            log_debug("hd_camera_uart_write error \n");
                        } else {
                            // 如果是pull 继续拉取文件
                        }
                    }
                }
            }

            break;
        }

        case PROPERTY_HD_ID_FACTORY_MODE: {
            log_info("[%设置工厂模式]%02x FACTORY_MODE\n", g_addr, PROPERTY_HD_ID_FACTORY_MODE);
            if (result_value_size_out == 1) {
                uint8_t on = result_value_out[0];
                g_serial_mode = on ? HD_SERIAL_SHELL_MODE : HD_SERIAL_NORMAL_MODE;
                log_info("设置工厂模式成功:%d\n", on);
                ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                   g_addr, property_id_out, 1);
                if (ret == 0) {
                    ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        log_debug("hd_camera_uart_write error \n");
                    } else {
                        // 如果是pull 继续拉取文件
                    }
                }
            }
            break;
        }

        case PROPERTY_HD_ID_PUSH: {
            log_info("[%d push]%02x PUSH\n", g_addr, PROPERTY_HD_ID_FACTORY_MODE);
            if (result_value_size_out <= 0) {

            } else {
                // 获取file_path
                // 获取md5
                log_debug("payload_data_size=%d\n", payload_data_size);
                ret = hd_slave_property_set_push_decode(g_push_mode_file_md5, &g_push_mode_file_size,
                                                        g_push_mode_file_path,
                                                        payload_data + 1,
                                                        payload_data_size - 1);
                if (ret) {
                    log_warn("hd_slave_property_set_push_decode error ret = %d\n", ret);
                    ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                       g_addr, property_id_out, ret);
                    if (ret == 0) {
                        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                    }
                    break;
                }
                log_debug("解析结果：\n");
                log_debug("file_size = %zu\n", g_push_mode_file_size);
                log_debug("md5 = [");
                for (int i = 0; i < 16; ++i) {
                    printf("%02x ", g_push_mode_file_md5[i]);
                }
                printf("]\n");
                log_debug("file_path = %s\n", g_push_mode_file_path);
                ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                   g_addr, property_id_out, g_serial_mode);


                // 创建文件夹，接受数据
                char *tmp = strdup(g_push_mode_file_path);
                ret = create_directory_if_not_exists(tmp);
                if (ret) {
                    free(tmp);
                    perror("create_directory_if_not_exists fail.\n");
                    break;
                }
                g_push_mode_file = fopen(tmp, "wb");  // 二进制写入模式
                if (!g_push_mode_file) {
                    free(tmp);
                    perror("Failed to open file");
                    break;
                }
                free(tmp);
                log_info("<<<<打开文件成功 准备接受数据>>>>\n");
                // 写入数据...
                // fwrite(data, 1, size, file);
                // fclose(file);  // 关闭文件
                log_info("<<<<切换到接受文件模式>>>>\n");
                hd_camera_change_serial_mode(HD_SERIAL_PUSH_MODE);
                break;
            }

            break;
        }

        case PROPERTY_HD_ID_DEBUG_ACTION_ID: {
            log_info("[%d设置属性]%02x DEBUG action_id\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ANGEL: {
            log_info("[%d设置属性]%02x DEBUG angel\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_HD_UART_VERSION: {
            log_info("[%d设置属性]%02x DEBUG hd_uart version\n", g_addr, PROPERTY_HD_ID_DEBUG);
//            if (result_value_out != NULL && result_value_size_out > 0) {
//
//
//                ret = hd_slave_property_set_encode(protocol_data_out, protocol_data_size_out,
//                                                   g_addr, property_id_out, 0);
//                return ret;
//            }
            break;
        }

        default: {
            log_warn("[%d设置属性]%02x 暂不支持\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
    }

//    if (ret == 0) {
//        ret = do_uart_write(protocol_data_out, protocol_data_size_out);
//        if (ret) {
//            log_debug("do_uart_write error \n");
//        } else {
//            // 如果是pull 继续拉取文件
//        }
//    }
    if (protocol_data_out != NULL) {
        free(protocol_data_out);
    }

    return 0;
}

static u_int8_t handle_snapshot_pic(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
) {
    // 拍照 超时？
    if (g_hd_on_event == NULL) {
        return -3;
    }
    void *result = g_hd_on_event(EVENT_HD_SNAPSHOT, NULL, 0);
    if (result == NULL) {
        return -4;
    }
    uint8_t *pic_id = (uint8_t *) result;
    if (pic_id == NULL) {
        return -2;
    }
    uint8_t in_result = PROTOCOL_UART_SUCCESS;
    int ret = hd_slave_snapshot_encode(protocol_data_out, protocol_data_size_out, g_addr, in_result, *pic_id);
    // free(pic_id);
    return ret;
}

/* 处理 3.9拉取图片（0x09）*/

static int read_from_buffer(unsigned char in_dest[], size_t in_size,
                            size_t in_offset, size_t *out_read_size
//                            ,const unsigned  char * src,size_t src_size
) {

    if (HD_UART_PARSER_DEBUG) {
        log_info("[read_from_buffer] g_file_buffer_size = %d\n", g_file_buffer_size);
    }
    // 参数检查
    if (in_dest == NULL) {
        fprintf(stderr, "错误：目标数组不能为NULL\n");
        return 1;
    }
    // 检查读取长度是否合理
    if (in_size == 0) {
        fprintf(stderr, "警告：请求读取0字节\n");
        *out_read_size = 0;
        return 2;
    }
    size_t real_offset = in_offset;

    // 检查请求是否超出缓冲区范围
    if (in_offset >= g_file_buffer_size) {
        log_error("[read_from_buffer] in_offset 大于文件最大大小<%d/%d> insize=%d\n", in_offset, g_file_buffer_size, in_size);
        return -2;
    }
    size_t real_read_size = in_size;
    if (HD_UART_PARSER_DEBUG) {
        log_info("[read_from_buffer] in_offset = %d \n", in_offset);
        log_info("[read_from_buffer] in_size = %d \n", in_size);
        log_info("[read_from_buffer] g_file_buffer_size = %d \n", g_file_buffer_size);
        log_info("[read_from_buffer] in_offset + in_size - g_file_buffer_size = %d \n", g_file_buffer_size - in_offset);
    }
    if (g_file_buffer_size - in_offset < in_size) {
        real_read_size = g_file_buffer_size - in_offset;
    }

    if (HD_UART_PARSER_DEBUG) {
        log_info("[read_from_buffer] real_read_size = %d \n", real_read_size);
    }
    if (real_read_size <= 0) {
        log_warn("real_read_size==0,没有数据可读了\n");
        return 1;
    }
    *out_read_size = real_read_size;
    memcpy(in_dest, (unsigned char *) g_file_buffer + in_offset, real_read_size);
    return 0;
}


// AA 5A 01 C9 05 09 00 00 00 2A A3 BB B7 D2 A8 2E 5C 8D C2 AF 26 AD 25 E6 CE 7B D6 47 CE 48 A0 B3
// AA 5A 01 C9 01 00 00 00 01 0D 59
// 319748
// +10240
//---------
// 329998
// 325372
// aa 5a 01 c9  09 00 00 00  01  00 a0 05 00  00 28 00 00  54 da

static int
handle_pull_pic(const unsigned char *payload_data,
                uint32_t payload_data_size,
                unsigned char **protocol_data_out,
                uint32_t *protocol_data_size_out
) {
    struct timespec start, end;
    double time_used;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // 1.解析收到的协议
    uint8_t out_pic_id;
    uint32_t out_offset = 0;
    uint32_t out_read_len;
    int ret = hd_slave_pull_pic_decode(&out_pic_id, &out_offset, &out_read_len, payload_data, payload_data_size);
    if (ret) {
        log_warn("hd_slave_pull_pic_decode error \n");
        return -1;
    }
    if (HD_UART_PARSER_DEBUG) {
        log_debug("解析到需要拉取的图片信息：\n");
        log_debug("pic_id            :       %d(0x%02x)\n", out_pic_id, out_pic_id);
        log_debug("offset            :       %d(0x%02x)\n", out_offset, out_offset);
        log_debug("read_len          :       %d(0x%02x)\n", out_read_len, out_read_len);
    }
    int goon = 0;
    if (g_file_pulling) { // 正在上传
        if (out_pic_id == g_file_pic_id) {
            //继续
            if (HD_UART_PARSER_DEBUG) {
                log_info("继续pull！\n");
            }
            goon = 1;
        } else {
            //清除掉 重新开始
            log_info("pic不相等 重新开始新的pull！\n");
            goon = 0;
        }
    } else { // 新的上传
        log_info("开始新的pull\n");
        goon = 0;
    }

    if (goon) {
        //ignore
    } else {
        // aa 5a 01 c9  09 00 00 00  01  81 29 05 00  00 28 00 00  33 e0  // 338305

        //                                                                // 347775
        // aa 5a 01 c9  09 00 00 00  01  81 29 05 00  00 28 00 00  33 e0
        if (out_offset == 0) { // 确保一开始从第0个位置拉取
            //查找图片
            char *filePath = NULL;
            ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
            if (ret) {
                log_warn("拉取的图片数据信息失败 %d\n", ret);
                return -1;
            }
            // 加载图片到缓存
            ret = load_file_to_buffer(filePath);
            if (ret) {
                log_warn("加载图片数据失败\n");
                return -2;
            }
            if (filePath != NULL) {
                free(filePath);
            }
            g_file_pic_id = out_pic_id;
            g_file_pulling = 1;
        } else {
            log_warn("从新拉取的数据:offset=%d应该从0开始\n", out_offset);
            return 10;
        }
    }

    if (HD_UART_PARSER_DEBUG) {
        log_debug("当前上传的文件pic_id   :      %d(0x%02x)\n", g_file_pic_id, g_file_pic_id);
        log_debug("当前上传的文件大小      :      %d(0x%02x)\n", g_file_buffer_size, g_file_buffer_size);
    }
    // 加载图片，从buff offset中读 read_len 数据
    unsigned char read_data[PROTOCOL_MAX_FRAME_LEN];
    size_t offset = out_offset;
    size_t read_len = out_read_len;
    size_t real_read_len = 0;
    ret = read_from_buffer(read_data, read_len, offset, &real_read_len);
    if (ret) {
        log_warn("[从机%d]读文件异常。\n", g_addr);
        reset_file_buffer();
        return 4;
    }
    if (HD_UART_PARSER_DEBUG) {
        log_debug("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", real_read_len, real_read_len);
    }
    if (real_read_len == 0) {
        log_warn("[从机%d]读取文件大小为0。\n", g_addr);
        reset_file_buffer();
        return 5;
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, read_data, real_read_len);
    if (ret) {
        log_warn("hd_slave_pull_pic_encode fail！\n");
        return -5;
    }
    if (real_read_len < out_read_len) {
        log_warn("[从机%d]文件读到结尾了。%d,%d\n", g_addr, real_read_len, out_read_len);
        reset_file_buffer();
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    time_used = (end.tv_sec - start.tv_sec) * 1e9;  // 秒转纳秒
    time_used += (end.tv_nsec - start.tv_nsec);     // 加上纳秒部分
    time_used /= 1e6;                               // 转换为毫秒

    printf("拉取图片处理总处理时间 耗时: %f 毫秒\n", time_used);
    return 0;
}

///* 处理 3.9拉取图片（0x09）*/
//static int
//handle_pull_pic_v1(const unsigned char *payload_data,
//                   uint32_t payload_data_size,
//                   unsigned char **protocol_data_out,
//                   uint32_t *protocol_data_size_out
//) {
//    uint8_t out_pic_id;
//    uint32_t out_offset;
//    uint32_t out_read_len;
//    int ret = hd_slave_pull_pic_decode(&out_pic_id, &out_offset, &out_read_len, payload_data, payload_data_size);
//    if (ret) {
//        log_warn("hd_slave_pull_pic_decode error \n");
//        return -1;
//    }
//    if (HD_UART_PARSER_DEBUG) {
//        log_debug("需要拉取的图片信息：\n");
//        log_debug("pic_id        :       %d(0x%02x)\n", out_pic_id, out_pic_id);
//        log_debug("offset        :       %d(0x%02x)\n", out_offset, out_offset);
//        log_debug("read_len      :       %d(0x%02x)\n", out_read_len, out_read_len);
//    }
//    char *filePath;
//    ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
//    if (ret) {
//        log_warn("拉取的图片数据信息失败\n");
//        return -1;
//    }
//    log_info("查找到的图片名称：%s\n", filePath);
//    unsigned char *result;
//    size_t size;
//
////    ret = do_cut_file_data(filePath, &result, &size, 246*1024, out_read_len);
//    ret = do_cut_file_data(filePath, &result, &size, out_offset, out_read_len);
//    if (ret) {
//        return -1;
//    }
//    log_debug("需要拉取的图片数据信息：\n");
//    log_debug("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", size, size);
//    if (size == 0) {
//        log_info("[从机%d]文件读完了。\n", g_addr);
//        return -1;
//    }
//    if (size < out_read_len) {
//        log_info("[从机%d]文件读到结尾了。\n", g_addr);
//    }
//
//    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, result, size);
//    if (ret) {
//        log_debug("hd_slave_pull_pic_encode fail！\n");
//        return -1;
//    }
//    return 0;
//}

/* 处理 3.16 查询摄像头图片信息（0x1D）*/
static int
handle_pic_infos(const unsigned char *payload_data, uint32_t payload_data_size, unsigned char **result,
                 uint32_t *result_size
) {
    int ret = hd_slave_pic_info_decode(payload_data, payload_data_size);
    if (ret) {
        log_warn("hd_slave_pic_info_decode error \n");
        return -1;
    }
    // 获取所有action_id图片
    hd_dynamic_pic_info pic_infos[1024] = {0};
    uint32_t pic_infos_size;
    ret = do_collect_all_pic_infos_v2(pic_infos, &pic_infos_size);
    if (ret) {
        log_warn("do_collect_all_pic_infos error \n");
        return -1;
    }
    log_info("图片数量：(%d):\n", pic_infos_size);
    if (pic_infos_size > 0) {
        if (HD_UART_PARSER_DEBUG) {
            log_debug("打印搜索结果(%d):\n", pic_infos_size);
            for (int i = 0; i < pic_infos_size; ++i) {
                log_debug("******\n");
                log_debug("id                    =   %d\n", pic_infos[i].id);
                log_debug("size                  =   %d\n", pic_infos[i].size);
                log_debug("action_id_index       =   %d\n", pic_infos[i].action_id_index);
                log_debug("action_id_timestamps  =   %d\n", pic_infos[i].action_id_timestamps);
                log_debug("snapshot_timestamps   =   %d\n", pic_infos[i].snapshot_timestamps);
                log_debug("md5                   =   ");
                for (int j = 0; j < 16; ++j) {
                    printf("%02x ", pic_infos[i].md5[j]);
                }
                log_debug("\n");
            }
        }
    }

    ret = hd_slave_pic_info_encode(result, result_size, g_addr, pic_infos,
                                   pic_infos_size);
    if (ret) {
        log_warn("hd_slave_pic_info_encode error %d \n", ret);
        return -1;
    }
    log_debug("hd_slave_pic_info_encode end! pic_infos_size = %d\n", pic_infos_size);
    return 0;
}

/* 处理 3.8 删除图片（0x08）*/
static int handle_delete_pic(const unsigned char *payload_data, uint32_t payload_data_size) {
    uint8_t out_pic_id = 0;
    uint32_t out_action_id_timestamp = 0;
    uint8_t out_action_id_index = 0;
    int ret = hd_slave_delete_pic_decode(&out_pic_id, &out_action_id_timestamp, &out_action_id_index, payload_data,
                                         payload_data_size);
    if (ret) {
        log_warn("hd_slave_delete_pic_decode error \n");
        return -1;
    }
    log_debug("需要删除的图片pic_id : <%d> action_id :<%d> <%d>\n", out_pic_id, out_action_id_timestamp, out_action_id_index);

    if (0xff == out_pic_id) {
        log_warn("删除所有图片 TODO\n");

        if (g_hd_on_event != NULL) {
            void *delete_result = g_hd_on_event(EVENT_HD_DELETE_ALL_FILE, NULL, 0);
            if (delete_result == NULL) {
                return -4;
            }
            int delete_result_int = *((int *) delete_result);
            if (delete_result_int) {
                return 0;
            } else {
                return -3;
            }
        }
        return -2;
    }
    // 遍历文件夹依次查询图片id
    char *filePath = NULL;
    ret = do_find_pic_by_pic_id_action_id(out_pic_id, &filePath, out_action_id_timestamp, out_action_id_index);
    if (ret != 0) {
        log_warn("do_find_pic_by_pic_id error \n");
        return -1;
    }
    if (HD_UART_PARSER_DEBUG) {
        log_debug("查询结果：图片地址=%s\n", filePath);
    }

    // 删除文件
    ret = remove(filePath);
    // TODO 检查当前文件夹是否为空 是则删除文件夹
    if (filePath != NULL) {
        free(filePath);
    }

    return ret == 0 ? 0 : -2;
}

/* 处理 3.10 图片拉取完成（0x0A）*/
static int handlePullPicComplete(const unsigned char *payload_data, uint32_t payload_data_size) {
    // 图片拉取完成 需要做什么？
    reset_file_buffer();
    return 0;
}

static void handle_extra_pull(unsigned char *payload_data, uint32_t payload_data_size) {
    int ret;
    int success = 0;
    unsigned char *protocol_data_out = NULL;
    uint32_t protocol_data_size_out;
    while (1) {
        log_debug("[handle_extra_pull] \n");
        ret = hd_slave_pull_decode((char *) g_pull_mode_file_path,
                                   payload_data,
                                   payload_data_size);

        if (ret) {
            log_warn("[handle_extra_pull] hd_slave_pull_decode error ret = %d\n", ret);
            success = 1;
            break;
        }
        log_debug("[handle_extra_pull] 解析结果：\n");
        log_debug("[handle_extra_pull] 需要拉取的文件file_path = %s\n", g_pull_mode_file_path);
        log_debug("[handle_extra_pull] 获取文件信息\n");
        unsigned char md5[16];
        log_debug("[handle_extra_pull] 获取文件信息 md5\n");
        ret = hd_md5((const char *) g_pull_mode_file_path, md5);
        if (ret) {
            log_warn("[handle_extra_pull]  hd_md5 error ret = %d\n", ret);
            success = 2;
            break;
        }
        log_debug("[handle_extra_pull] 获取文件信息 md5成功！\n");
        log_debug("[handle_extra_pull] 获取文件信息 file_size\n");
        int fd;
        fd = open((const char *) g_pull_mode_file_path, O_RDWR);
        if (fd == -1) {
            log_debug("[handle_extra_pull] 打开文件失败:%s 原因：%d->%s \n ", g_pull_mode_file_path, errno, strerror(errno));
            success = 3;
            break;
        }
        struct stat file_stat;
        if (fstat(fd, &file_stat) == -1) {
            log_debug("[handle_extra_pull] 获取文件大小失败。fd:%d\n", fd);
            success = 4;
            close(fd);
            break;
        }
        off_t file_size = file_stat.st_size;
        log_debug("[handle_extra_pull] 获取文件信息 file_size成功！\n");

        log_debug("[handle_extra_pull] file_size   :  < %d >bytes\n", file_size);
        log_debug("[handle_extra_pull] file_md5    :  ");
        for (int i = 0; i < sizeof(md5); ++i) {
            printf("%02x ", md5[i]);
        }
        printf("\n");

        log_debug("[handle_extra_pull] 准备应答... \n");
        ret = hd_slave_pull_encode(&protocol_data_out, &protocol_data_size_out, g_addr,
                                   0, md5, file_size, (const char *) g_pull_mode_file_path);
        if (ret) {
            close(fd);
            success = 5;
            break;
        }
        close(fd);
        hd_printf_buff(protocol_data_out, protocol_data_size_out, "pull应答", 0);
        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
        if (protocol_data_out != NULL) {
            free(protocol_data_out);
        }

        log_debug("[handle_extra_pull] 应答完毕! \n");
        g_pull_mode_file = fopen((const char *) g_pull_mode_file_path, "rb");
        if (g_pull_mode_file == NULL) {
            success = 6;
            fprintf(stderr, "[handle_extra_pull] 无法打开文件 %s: %s\n", (const char *) g_pull_mode_file_path,
                    strerror(errno));
            break;
        }
        // 先发送一个文件头：
        hd_camera_uart_write(PULL_MODE_FILE_HEADER_TAIL, sizeof(PULL_MODE_FILE_HEADER_TAIL));
        unsigned char buffer[1024];
        size_t bytes_read;
        log_info("[handle_extra_pull] 开始pull文件...\n");
        log_info("----------------------------\n");
        rs485_pwr_on();
        usleep(9000);
        while ((bytes_read = fread(buffer, 1, 1024, g_pull_mode_file)) > 0) {
            if (!g_running)break;

            for (size_t i = 0; i < bytes_read; i++) {
                printf("%02X ", buffer[i]);  // 处理每个字节

                rk_uart_sendbyte(buffer[i]);
//                usleep(16);
//                    do_uart_write(buffer, bytes_read);

            }
        }
        log_info("\n----------------------------\n");
        log_info("[handle_extra_pull] pull文件结束！\n");
        usleep(4000);
        rs485_pwr_off();
        hd_camera_uart_write(PULL_MODE_FILE_HEADER_TAIL, sizeof(PULL_MODE_FILE_HEADER_TAIL));

        hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
        log_info("[handle_extra_pull] pull文件完毕！应答...\n");
        ret = hd_slave_pull_encode(&protocol_data_out, &protocol_data_size_out, g_addr, 0xff,
                                   md5, file_size, (const char *) g_pull_mode_file_path);

        resetPullMode();
        if (ret) {
            success = 7;
            break;
        }
        log_info("[handle_extra_pull] pull文件完毕！应答成功！\n");
        hd_printf_buff(protocol_data_out, protocol_data_size_out, "pull完毕", 0);
        hd_camera_uart_write(protocol_data_out, protocol_data_size_out);

        break;
    }

    if (protocol_data_out) {
        free(protocol_data_out);
    }
}

static int handle_uart_data(const unsigned char *raw, size_t raw_size) {
    if (NULL == raw || raw_size <= 0) {
        log_warn("handle_uart_data raw == NULL || raw_size == 0 \n");
        return 0;
    }
    uint8_t ret;
    uint8_t out_addr;
    ret = hd_camera_protocol_addr(&out_addr, raw, raw_size);
    if (ret) {
        log_warn("hd_camera_protocol_addr error ret=%d \n", ret);
        return -2;
    }

    if (g_addr == 0 || (g_addr != out_addr && out_addr != PROTOCOL_BROADCAST)) {
        log_debug("从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, out_addr);
        return -3;
    }
    log_debug("------------------------------handle_uart_data------------------------%d------\n", raw_size);

    uint8_t slave_addr_out;
    uint8_t cmd_out;
    uint32_t payload_data_size_out;
    unsigned char *payload_data_out = NULL;
    ret = hd_camera_protocol_decode(raw, raw_size, &slave_addr_out, &cmd_out, &payload_data_size_out,
                                    &payload_data_out);
    if (ret) {
        log_warn("hd_camera_protocol_decode error  = %d\n", ret);
        return 0;
    }

    if (g_addr == 0 || (g_addr != slave_addr_out && slave_addr_out != PROTOCOL_BROADCAST)) {
        log_debug("从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, slave_addr_out);
        return -1;
    }

    switch (cmd_out) {
        case CMD_HD_PROPERTY_GET: {
            log_info("[从机%d] HD查询属性（0xC2）预留\n", g_addr);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = handle_property_get(payload_data_out, payload_data_size_out, &out_protocol_data,
                                      &out_protocol_data_size);
            if (ret == 0) {
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    log_debug("hd_camera_uart_write error \n");
                }
            }
            if (out_protocol_data) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PROPERTY_SET: {
            log_info("[从机%d] HD设置属性（0xC3）预留\n", g_addr);
            handle_property_set(payload_data_out, payload_data_size_out);
            break;
        }

        case CMD_HD_CAMERA_SNAPSHOT: {
            log_info("[从机%d] HD主动抓图（0xC6）\n", g_addr);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = handle_snapshot_pic(&out_protocol_data, &out_protocol_data_size);
            log_info("主动抓图ret:%d\n", ret);
            if (ret == 0) {
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    log_debug("hd_camera_uart_write error \n");
                }
            } else {
                ret = hd_slave_snapshot_encode(&out_protocol_data, &out_protocol_data_size, g_addr, PROTOCOL_UART_FAIL,
                                               0);
                if (ret) {
                    log_debug("hd_slave_snapshot_encode error \n");
                } else {
                    ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                    if (ret) {
                        log_debug("hd_camera_uart_write error \n");
                    }
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_INFO: {
            log_info("[从机%d] HD查询摄像头存储的图片信息（0xC7）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out = 0;
            ret = handle_pic_infos(payload_data_out, payload_data_size_out, &protocol_data_out,
                                   &protocol_data_size_out);
            if (ret == 0) {
                ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    log_warn("hd_camera_uart_write error \n");
                }
            } else {
                log_warn("handle_pic_infos error \n");
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
            }
            break;
        }

        case CMD_HD_PIC_DELETE: {
            log_info("[从机%d] HD删除图片（0xC8））\n", g_addr);
            int delete_ret = handle_delete_pic(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = hd_slave_delete_pic_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                             delete_ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                log_warn("hd_slave_delete_pic_encode error\n");
            } else {
                hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
            }

            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_PULL: {
            log_info("[从机%d] HD拉取图片（0xC9）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out;
            ret = handle_pull_pic(payload_data_out, payload_data_size_out, &protocol_data_out, &protocol_data_size_out);
            if (ret == 0) {
                ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    log_warn("hd_camera_uart_write error \n");
                }
            } else {
                log_warn("handle_pull_pic fail ! ret = %d \n", ret);
                ret = hd_slave_pull_pic_encode(&protocol_data_out, &protocol_data_size_out, g_addr, PROTOCOL_UART_FAIL,
                                               NULL, 0);
                if (ret) {
                    log_warn("hd_slave_pull_pic_encode error %d\n", ret);
                } else {
                    ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        log_warn("hd_camera_uart_write error \n");
                    }
                }
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
            }
            break;
        }

        case CMD_HD_PIC_PULL_COMPLETED: {
            log_info("[从机%d] HD图片拉取完成（0xCA）\n", g_addr);
            ret = handlePullPicComplete(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size;
            int result = hd_slave_pull_pic_complete_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                                           ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (result) {
                log_warn("hd_slave_pull_pic_complete_encode error\n");
            } else {
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    log_warn("hd_camera_uart_write error \n");
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_BROADCAST_ACTION_ID: {
            log_info("[从机%d] HD广播门开事件（0xCD）\n", g_addr);
            char buff[128] = {0};
            uint32_t out_action_id_timestamps;
            uint8_t out_action_id_index;
            uint8_t status;
            ret = hd_slave_action_id_decode(&status, &out_action_id_timestamps, &out_action_id_index, payload_data_out,
                                            payload_data_size_out);
            if (ret) {
                log_debug("hd_slave_action_id_decode error:%d \n", ret);
                break;
            }
            if (g_hd_on_action_id_changed != NULL) {
                do_action_id_2_str(buff, sizeof(buff), out_action_id_timestamps, out_action_id_index);
                log_info("action_id = %s\n", buff);
                g_hd_on_action_id_changed(status, buff);
            }
            // 广播不需要响应
            break;
        }

        case CMD_HD_EXTRA_PULL: {
            log_info("[从机%d] PULL（0x%02x）\n", g_addr, CMD_HD_EXTRA_PULL);
            handle_extra_pull(payload_data_out, payload_data_size_out);
            break;
        }

        case CMD_HD_PUSH_FILE: {
            log_info("[从机%d] HD PUSH File（0x%02x）\n", g_addr, CMD_HD_PUSH_FILE);
            hd_camera_ota_model_handle_cmd(payload_data_out, payload_data_size_out);
            break;
        }

        default:
            log_info("[从机%d] 暂不支持的CMD:%02x\n", g_addr, cmd_out);
            break;

    }
    return 0;
}

static void free_queue(HDBlockingQueueUint8 *queue) {
    if (queue != NULL) {
        hd_queue_destroy_uint8(queue);
    }
}

static void *handle_uart_data_thread(void *arg) {
    log_info("handle_uart_data_thread start...\n");
    while (g_running) {
        uint8_t item = hd_queue_take_uint8(g_frame_queue);
        do_uart_recv_take(item);
    }
    log_info("handle_uart_data_thread end.\n");
    return NULL;
}

/************ handles a **************/

static void init_log() {
    log_set_level(LOG_INFO);
    /*
    // 初始化本地日志
    char *log_path_prefix = "hd_log_";
    FILE *log_file = NULL;
    char buff[1024];
    time_t current_time = time(NULL);
    struct tm *local_time = localtime(&current_time);
    int year = local_time->tm_year + 1900;  // 年份从1900开始计数
    int month = local_time->tm_mon + 1;     // 月份从0开始计数
    int day = local_time->tm_mday;
    int hour = local_time->tm_hour;
    int minute = local_time->tm_min;
    int second = local_time->tm_sec;
//    char *log_dir_path = "/Users/xiangpengle/Downloads/";
    char *log_dir_path = "/userdata/log/";
    snprintf(buff, sizeof buff, "%s%s%d-%s-%s-%04d-%02d-%02d-%02d-%02d-%02d.log", log_dir_path, log_path_prefix, g_addr,
             hd_uart_version(), version, year, month, day, hour, minute,
             second);

    int ret = create_directory_if_not_exists(log_dir_path);
    if (ret) {
        printf("create_directory_if_not_exists fail. %s\n", buff);
        return 0;
    }
    log_file = fopen(buff, "wr");
    if (!log_file) {
        printf("Failed to open file");
        return 0;
    }
    log_add_fp(log_file, LOG_DEBUG);*/
}

/***************************************************************************************************/
/****************************** hd_uart.so *********************************************************/
/***************************************************************************************************/
#define HD_UART_PARSER_VERSION_INTERNAL         "0.2.42.2"                    // 库版本

int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        const char *version,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
) {
    hd_logger_set_level(HD_LOGGER_LEVEL_INFO);
    init_log();
    uint32_t delay = calculate_3_5_char_time(PROTOCOL_RATE_DEFAULT, 8, 0, 1);
    snprintf(g_pic_dir_path, sizeof(g_pic_dir_path), "%s", pic_dir_path);
    log_info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    log_info("hd_uart_init\n");
    log_info("hd_uart_init version              :       <%s/%s>   \n", version, hd_uart_version());
    log_info("hd_uart_init protocol_version     :       <%s>   \n", PROTOCOL_VERSION);
    log_info("hd_uart_init addr                 :       <%d> \n", addr);
    log_info("hd_uart_init pic_dir_path         :       <%s> \n", g_pic_dir_path);
    log_info("hd_uart_init delay                :       <%d> \n", delay);
    log_info("hd_uart_max               	    :       <%d> \n", PROTOCOL_MAX_FRAME_LEN);
    for (int i = 0; i < sizeof(PULL_MODE_FILE_HEADER_TAIL); ++i) {
        printf("%02x ", PULL_MODE_FILE_HEADER_TAIL[i]);
    }
    printf("\n");
    log_info(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    int ret = hd_camera_ota_init(g_addr);
    if (ret) {
        log_error("hd_camera_ota_init error ret=%d\n", ret);
        return 0;
    }

    g_frame_queue = hd_queue_create_uint8(10240);
    if (g_frame_queue == NULL) {
        log_error("createQueue g_frame_queue error!");
        return 0;
    }

    g_addr = addr;
    snprintf(g_hd_app_version, sizeof(g_hd_app_version), "%s", version == NULL ? "" : version);
    g_hd_on_action_id_changed = on_action_id_changed;
    g_hd_on_event = on_event;
    g_delay = delay;
    hd_camera_shell_init(addr);
    g_running = 1;

    pthread_create(&g_frame_consume_t, NULL, handle_uart_data_thread, NULL);

    log_info("hd_uart_init completed !!!\n");
    return 0;
}

void hd_uart_deinit() {
    log_info("hd_uart_deinit\n");
    g_hd_on_action_id_changed = NULL;
    g_hd_on_event = NULL;
    g_running = 0;
    hd_camera_ota_deinit();

    hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
    hd_camera_shell_deinit();

    if (g_frame_consume_t) {
        pthread_join(g_frame_consume_t, NULL);
    }

    free_queue(g_frame_queue);

    g_frame_queue = NULL;

}

char *hd_uart_version() {
    return HD_UART_PARSER_VERSION_INTERNAL;
}

void hd_camera_change_serial_mode(HD_SERIAL_MODE mode) {
    pthread_mutex_lock(&g_serial_mode_mutex);
    g_serial_mode = mode;
    log_info("############################################## \n", mode);
    log_info("############# serial_mode : [%d] ############## \n", mode);
    log_info("############################################## \n", mode);
    pthread_mutex_unlock(&g_serial_mode_mutex);
}

void hd_uart_recv(uint8_t byte) {
    if (g_running == 0)return;
    pthread_mutex_lock(&g_serial_mode_mutex);
    uint8_t mode = g_serial_mode;
    pthread_mutex_unlock(&g_serial_mode_mutex);
    switch (mode) {
        case HD_SERIAL_SHELL_MODE: {
            do_uart_recv_with_shell(byte);
            break;
        }
        case HD_SERIAL_NORMAL_MODE: {
            do_uart_recv(byte);
            break;
        }

        case HD_SERIAL_PULL_MODE: {
            do_uart_recv_with_pull(byte);
            break;
        }
        case HD_SERIAL_HD_PUSH_MODE: {
            hd_camera_ota_model_recv(byte);
            break;
        }

        case HD_SERIAL_PUSH_MODE: {
            // 接受文件
            do_uart_recv_with_push(byte);
            break;
        }
        default:
            break;

    }
}