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

#define HD_UART_PARSER_DEBUG                    0
#define HD_UART_PARSER_VERSION_INTERNAL         "0.2.40"
#define HD_PARSE_FRAME_QUEUE                    1
#define FRAME_HEADER_H                          PROTOCOL_HEADER_1
#define FRAME_HEADER_L                          PROTOCOL_HEADER_0
#define MAX_FILE_SIZE                           (512*1024)
#define JPG_SUFFIX                              ".jpg"                      // 图片格式
#define JPG_SUFFIX_LEN                          4                           // 图片格式长度
#define HD_FILE_PUSH_TIMEOUT                    30                          // 接受上传文件超时时间

#if(CONTEXT)
#define MODEL_DIR_PATH                          "/Users/xiangpengle/CLionProjects/hd_camera/test_case/oem/usr/shared"
#define MODEL_PREFIX                            ".rknn"
#define MODEL_PREFIX_DOWNLOADING                ".downloading"
#define MODEL_DEST_PATH                         "/Users/xiangpengle/Downloads"
#else
#define MODEL_DIR_PATH                          "/oem/usr/shared"
#define MODEL_PREFIX                            ".rknn"
#define MODEL_PREFIX_DOWNLOADING                ".downloading"
#define MODEL_DEST_PATH                         "/userdata"
#endif


typedef struct {
    unsigned char *data;
    uint32_t data_size;
} hd_frame_data;

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

typedef enum {
    HD_SERIAL_NORMAL_MODE = 0,   // 串口传输模式
    HD_SERIAL_SHELL_MODE,       // SHELL模式
    HD_SERIAL_PUSH_MODE,        // PUSH模式
    HD_SERIAL_PULL_MODE,         // PULL模式
    HD_SERIAL_HD_PUSH_MODE         // HD_PUSH模式
} HD_SERIAL_MODE;


static volatile uint8_t g_addr = 0;                     // 当前从机地址
static char g_version[1024];
static volatile uint8_t g_serial_mode = HD_SERIAL_NORMAL_MODE;

static pthread_t g_frame_consume_t = NULL;                  // 消费frame线程
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;   // 队列锁
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;      // 队列有新的数据条件
static int signal_sent = 0;                                 // 标志变量

static HDBlockingQueue *g_frame_queue = NULL;                          // 协议帧队列
static pthread_mutex_t g_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;  // 解析g_buffer锁，防止在memcpy时有错误

static volatile uint32_t g_delay = 0;                       // 485帧间隔delay
static char g_pic_dir_path[2048];                           // 当前照片存储目录
static volatile int g_running = 0;                          // 程序是否在运行
static hd_on_action_id_changed g_hd_on_action_id_changed = NULL;    // 收到action_id回调
static hd_on_event g_hd_on_event = NULL;                    // 收到event回调

unsigned char shell_resp_buff[MAX_RESULT_LENGTH];           // shell回复缓冲区
static unsigned g_file_buffer[MAX_FILE_SIZE] = {0};         // 图片文件缓冲区
static unsigned int g_file_buffer_size = -1;                // 当前上传文件大小
static uint8_t g_file_pic_id = 0;                           // 当前上传文件pic_id
static int g_file_pulling = 0;                              // 当前上传文件中

static char g_push_mode_file_path[1024];                   // push文件path
static uint64_t g_push_mode_file_size = -1;                 // push文件size
static unsigned char g_push_mode_file_md5[16];              // push文件md5
static FILE *g_push_mode_file = NULL;                       // push file

static char *g_pull_mode_file_path[1024];                   // extra_pull文件path
static uint64_t g_pull_mode_file_size = -1;                 // extra_pull文件size
static unsigned char g_pull_mode_file_md5[16];              // extra_pull文件md5
static FILE *g_pull_mode_file = NULL;                       // extra_pull file


static char g_hd_push_mode_file_path[1024];                   // hd_push文件path
static char g_hd_push_mode_file_path_downloading[2048];                   // hd_push文件path
static uint64_t g_hd_push_mode_file_size = -1;                 // hd_push文件size
static unsigned char g_hd_push_mode_file_md5[16];              // hd_push文件md5
static FILE *g_hd_push_mode_file = NULL;                       // hd_push file

static unsigned char PULL_MODE_FILE_HEADER_TAIL[8] = {      // hadlinks
        'h', 'a', 'd', 'l', 'i', 'n', 'k', 's'
};

// 隐藏的实现 a

static void on_serial_mode_changed(HD_SERIAL_MODE mode);

static int do_uart_write(const unsigned char *raw, size_t raw_size);

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

static void notify_frame_changed();

static void resetFileBuffer() {
    g_file_pic_id = 0;
    g_file_pulling = 0;
    g_file_buffer_size = -1;
}

static int do_uart_write(const unsigned char *raw, size_t raw_size) {
    if (g_running == 0) {
        return -5;
    }
    if (raw == NULL || raw_size <= 0) {
        return -1;
    }

    hd_printf_buff(raw, raw_size, "[应答]", 0);

//    for (int i = 0; i < raw_size; ++i) {
//
//        rs485_pwr_on();
//        usleep(g_delay*4);
//	rk_uart_sendbyte(raw[i]);
//        usleep(g_delay*4);
//        rs485_pwr_off();
//        usleep(g_delay*4);
//    }

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
        LOGI("do_uart_write over \n");
    }
    return 0;
}

// 帧解析函数
static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, uint32_t *frame_length) {
    //LOGI("<接受>%02x \n", str);
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
        state = STATE_WAIT_HEADER_H;
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
            frame_buffer[2] = byte;
            // 判断addr
            if ((frame_buffer[2] != g_addr) && (frame_buffer[2] != PROTOCOL_BROADCAST)) {
                current_pos = 0;
                state = STATE_WAIT_HEADER_H;
                return -4;
            }
            current_pos = 3;
            state = STATE_WAIT_CMD;
            break;

        case STATE_WAIT_CMD:
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
//                LOGW("data_len = %u ,frame_length = %u,current_pos=%u\n", data_len, frame_length, current_pos);
//                for (int i = 0; i < 40; ++i) {
//                    LOGW("frame_buffer[%d] = %02x \n", frame_buffer[i]);
//                }
//                LOGW("全局信息：\n");
//                LOGW("data_index        =  %02x (%d)\n", data_index, data_index);
//                LOGW("data_len          =  %02x (%d)\n", data_len, data_len);
//                LOGW("expected_crc      =  %02x (%d)\n", expected_crc, expected_crc);
//                LOGW("calculated_crc    =  %02x (%d)\n", calculated_crc, calculated_crc);
//                LOGW("current_pos       =  %02x (%d)\n", current_pos, current_pos);
//                LOGW("state             =  %02x (%d)\n", state, state);
//                LOGW("frame_length      =  %02x (%d)\n", frame_length, frame_length);

                current_pos = 0;
                state = STATE_WAIT_HEADER_H;
                return -3;
            }

            if (data_len == 0) {
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


        case STATE_WAIT_CRC_L:
            frame_buffer[8 + data_len + 1] = byte;
            current_pos = 8 + data_len + 2;
            expected_crc |= (byte << 8);


            calculated_crc = hd_crc16(&frame_buffer[0], 8 + data_len);
            //printf("calculated_crc   %02x \n", calculated_crc);
            //printf("expected_crc     %02x \n", expected_crc);
            if (calculated_crc == expected_crc) {
                *frame_length = current_pos;
                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return 0;
            } else {
                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return -2;
            }
            break;

        default:
            LOGE("default current_pos = %d \n", current_pos);
            state = STATE_WAIT_HEADER_H;
            current_pos = 0;
            data_index = 0;
            data_len = 0;
            expected_crc = 0;
            calculated_crc = 0;
            current_pos = 0;
            break;
    }

    return -1;
}

// 创建目录（如果不存在）
static int create_directory_if_not_exists(const char *path) {
    char *dir_path = strdup(path);
    char *p = strrchr(dir_path, '/');

    if (p != NULL) {
        *p = '\0'; // 截断文件名，只保留目录路径

        // 检查目录是否存在
        struct stat st;
        if (stat(dir_path, &st) != 0) {
            // 目录不存在，尝试创建
            if (mkdir(dir_path, 0755) != 0 && errno != EEXIST) {
                free(dir_path);
                return -1; // 创建失败
            }
        }
    }

    free(dir_path);
    return 0;
}

static void resetPushMode() {
    if (g_push_mode_file != NULL) {
        fclose(g_push_mode_file);
        g_push_mode_file = NULL;
    }
    memset(g_push_mode_file_path, 0, sizeof(g_push_mode_file_path));
    memset(g_push_mode_file_md5, 0, sizeof(g_push_mode_file_md5));
    g_push_mode_file_size = -1;
    on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
}

static void resetHDPushMode() {
    if (g_hd_push_mode_file != NULL) {
        fclose(g_hd_push_mode_file);
        g_hd_push_mode_file = NULL;
    }
    memset(g_hd_push_mode_file_path_downloading, 0, sizeof(g_hd_push_mode_file_path_downloading));
    memset(g_hd_push_mode_file_path, 0, sizeof(g_hd_push_mode_file_path));
    memset(g_hd_push_mode_file_md5, 0, sizeof(g_hd_push_mode_file_md5));
    g_hd_push_mode_file_size = -1;
    on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
}

static void resetPullMode() {
    if (g_push_mode_file != NULL) {
        fclose(g_pull_mode_file);
        g_pull_mode_file = NULL;
    }
    memset(g_pull_mode_file_path, 0, sizeof(g_pull_mode_file_path));
    memset(g_pull_mode_file_md5, 0, sizeof(g_pull_mode_file_md5));
    g_pull_mode_file_size = -1;
    on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
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
    do_uart_write(out_protocol_data, out_protocol_data_size);
    free(out_protocol_data);
}


static void on_hd_push_delete_and_reply(int result) {
    unsigned char *out_protocol_data = NULL;
    uint32_t out_protocol_data_size;
    uint8_t in_slave_addr;
    uint8_t in_type = 0x01;
    uint8_t in_result;
    uint32_t offset = 0;
    int ret = hd_slave_file_encode(&out_protocol_data, &out_protocol_data_size,
                                   g_addr, in_type, result, offset);

    if (ret) {
        LOGW("on_hd_push_delete_and_reply error = %d\n", ret);
        return;
    }
    do_uart_write(out_protocol_data, out_protocol_data_size);
    free(out_protocol_data);
}

static void do_uart_recv_with_pull(uint8_t str) {

}

static ssize_t do_uart_recv_with_hd_push_file_size = 0;

/**
 * 见 handle_hd_push_file
 */
static void do_uart_recv_with_hd_push(uint8_t str) {
    alarm(0);
    if (g_hd_push_mode_file == NULL)return;
//    LOGD("<do_uart_recv_with_hd_push> %02x \n",str);

    unsigned char uc = str;
    size_t written = fwrite(&uc, 1, 1, g_hd_push_mode_file);
    if (written != 1) {
        perror("Failed to write byte\n");
        // todo 删除文件
        resetHDPushMode();

        on_hd_push_delete_and_reply(1);
        return;
    }
    printf("do_uart_recv_with_hd_push file_size=%zd\n", do_uart_recv_with_hd_push_file_size);
    do_uart_recv_with_hd_push_file_size++;
    if (do_uart_recv_with_hd_push_file_size == g_hd_push_mode_file_size) {
        fclose(g_hd_push_mode_file);
        g_hd_push_mode_file = NULL;
        sync();
        LOGI("hd push 接受完毕！文件：%s ，大小：%d\n", g_hd_push_mode_file_path_downloading, do_uart_recv_with_hd_push_file_size);
        // 校验md5
        unsigned char result[16];
        int ret = hd_md5(g_hd_push_mode_file_path_downloading, result);
        if (ret) {
            LOGW("md5生成 fail :%d\n", ret);
            // todo 删除文件
            resetHDPushMode();
            on_hd_push_delete_and_reply(4);
            return;
        }
        if (hd_array_cmp(result, 16, g_hd_push_mode_file_md5, 16) == 0) {
            LOGI("文件push成功！\n");
            if (g_hd_push_mode_file != NULL) {
                fflush(g_hd_push_mode_file);  // 确保所有缓冲数据写入文件
                fclose(g_hd_push_mode_file);
                g_hd_push_mode_file = NULL;
            }
            // 修改名称
            ret = rename(g_hd_push_mode_file_path_downloading, g_hd_push_mode_file_path);
            if (ret == 0) {
                memset(g_hd_push_mode_file_path, 0, sizeof(g_hd_push_mode_file_path));
                memset(g_hd_push_mode_file_path_downloading, 0, sizeof(g_hd_push_mode_file_path_downloading));
                memset(g_hd_push_mode_file_md5, 0, sizeof(g_hd_push_mode_file_md5));
                g_hd_push_mode_file_size = -1;
                on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
                on_hd_push_delete_and_reply(0);
            } else {
                on_hd_push_delete_and_reply(5);
            }
        } else {
            LOGW("md5不同\n");
            resetHDPushMode();
            // todo 删除文件
            on_hd_push_delete_and_reply(4);
        }
    } else if (do_uart_recv_with_hd_push_file_size > g_hd_push_mode_file_size) {
        LOGW("file_size不同\n");
        resetHDPushMode();
        on_hd_push_delete_and_reply(3);
    } else {
        LOGW("继续接受 %02x\n", str);
        alarm(5);
    }
}

static void do_uart_recv_with_push(uint8_t str) {
    if (g_push_mode_file == NULL)return;
    //LOGD("<do_uart_recv_to_file> %02x \n",str);
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
        LOGI("push 完毕！文件：%s ，大小：%d\n", g_push_mode_file_path, file_size);
        // 校验md5
        unsigned char result[16];
        int ret = hd_md5(g_push_mode_file_path, result);
        if (ret) {
            LOGW("md5生成 fail :%d\n", ret);
            // todo 删除文件
            resetPushMode();
            on_push_delete_and_reply(2);
            return;
        }
        if (hd_array_cmp(result, 16, g_push_mode_file_md5, 16) == 0) {
            LOGI("文件push成功！\n");
            resetPushMode();
            on_push_delete_and_reply(0);
        } else {
            LOGW("md5不同");
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
    LOGD("[do_uart_recv_str]收到字符：%02x\n", str);
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
        LOGI("Received Shell: %s\n", shell_buff);
        // 处理命令
        if (strcmp(shell_buff, "#exit#") == 0) {
            if (g_serial_mode == HD_SERIAL_NORMAL_MODE) {
                LOGI("已经取消工厂模式%d\n");
                unsigned char buf[] = {g_serial_mode};
                do_uart_write(buf, sizeof(buf));
                return;
            }
            on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
            LOGI("取消工厂模式成功！\n");
            // 回复
            const char *resp = "取消工厂模式成功!\n";

            // 复制数据（包括 '\0' 结束符）
            unsigned long size = strlen(resp) + 1;
            LOGI("发送shell命令长度=%d\n", size);
            memcpy(shell_resp_buff, resp, size);
            do_uart_write(shell_resp_buff, size);

            // 重置索引以便接收下一条命令
            do_uart_recv_str_index = 0;
        } else {
            LOGI("shell命令 shell_buff=[%s] index=%d\n", shell_buff, do_uart_recv_str_index);
            for (int i = 0; i < do_uart_recv_str_index; ++i) {
                printf("%02x\n", shell_buff[i]);
            }
            char buff[1024];
            for (int i = 0; i <= do_uart_recv_str_index; ++i) {
                buff[i] = shell_buff[i];
            }
            ret = hd_camera_shell_exec(buff, shell_resp_buff);
            if (ret) {
                LOGW("hd_camera_shell_exec error:%d\n", ret);
                // 重置索引以便接收下一条命令
                do_uart_recv_str_index = 0;
                return;
            }
            printf("执行结束！\n");
            LOGI("exec result :\n");
            LOGI("%s", shell_resp_buff);
            LOGI("\n");
            do_uart_write(shell_resp_buff, strlen(shell_resp_buff));
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

static void do_uart_recv(uint8_t str) {
    uint8_t g_frame_buffer[PROTOCOL_MAX_FRAME_LEN];  // 串口帧缓冲区
    uint32_t g_frame_length = 0;                     // 一个完整帧的数据长度
    int ret;
    pthread_mutex_lock(&g_buffer_mutex);
    ret = parse_serial_frame(str, g_frame_buffer, &g_frame_length);
    pthread_mutex_unlock(&g_buffer_mutex);
    if (ret == 0) {
        uint32_t len2 = g_frame_length;
        LOGI("收到完整帧.......%d....\n", len2);
        if (HD_PARSE_FRAME_QUEUE) {
//            if(         g_frame_queue->size>20480){
//                LOGI("[QUEUE]满了 \n", len2);
//                notify_frame_changed();
//                return;
//            }
//            if (isFull(g_frame_queue)) {
//                return;
//            }
            // 放入队列
            unsigned char *buf = (unsigned char *) malloc(len2);
            // 加锁
//            LOGI("memcpy...sizeof(buf)=%d....len = %d....\n", sizeof(buf), len2);
            pthread_mutex_lock(&g_buffer_mutex);
            memcpy(buf, g_frame_buffer, len2);
            pthread_mutex_unlock(&g_buffer_mutex);
            hd_frame_data *data = malloc(sizeof(hd_frame_data));
            data->data = buf;
            data->data_size = len2;
            hd_queue_put(g_frame_queue, data);
        } else {
            uint32_t len = g_frame_length;
            uint8_t tmp[PROTOCOL_MAX_FRAME_LEN];
            memcpy(tmp, g_frame_buffer, len);
            g_frame_length = 0;

            // LOGI("收到完整帧.......%d....\n", len);
            hd_printf_buff(tmp, len, "收到", 0);
            ret = handle_uart_data(tmp, len);
//        ret = handle_uart_data(g_frame_buffer, g_frame_length); // 多线程问题
        }
    } else if (ret == -1) {
        // 处理中。。。
    } else if (ret == 5) {
        g_frame_length = 0;
//        memset(g_frame_buffer, 0, PROTOCOL_MAX_FRAME_LEN);
    } else if (ret == -2) {
        LOGW("CRC error \n");
        g_frame_length = 0;
//        memset(g_frame_buffer, 0, PROTOCOL_MAX_FRAME_LEN);
    } else if (ret == -3) {
        LOGW("len error\n");
        g_frame_length = 0;
//        memset(g_frame_buffer, 0, PROTOCOL_MAX_FRAME_LEN);
    } else {
        // 处理中。。。
    }


}

static void notify_frame_changed() {
//    // sem_post(&g_semaphore); // 发送信号（信号量 +1）
//    pthread_mutex_lock(&mutex);
//    LOGI("notify_frame_changed...\n");
//    signal_sent = 1;
//    pthread_cond_signal(&cond); // 发送信号
//    pthread_mutex_unlock(&mutex);
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
        LOGW("读取文件不完全，期望 %ld字节，实际读取 %zu字节\n",
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

// 解析文件名并返回分割后的字符串数组
static int parse_filename(const char *filename, char *tokens[], int max_tokens) {
    // 1. 验证输入
    if (!ends_with_jpg(filename)) {
        LOGE("错误：文件名必须以.jpg结尾 %s\n", filename);
        return -1;
    }

    // 2. 创建可修改的副本（因为strtok会修改原字符串）
    char *str = strdup(filename);
    if (!str) {
        perror("内存分配失败");
        return -1;
    }

    // 3. 去掉.jpg后缀
    str[strlen(str) - JPG_SUFFIX_LEN] = '\0';

    // 4. 分割字符串
    int count = 0;
    char *token = strtok(str, "_");
    while (token && count < max_tokens) {
        tokens[count++] = strdup(token);  // 复制每个token
        token = strtok(NULL, "_");
    }

    free(str);  // 释放临时字符串
    return count;
}

/*
// 解析图片
// 图片的格式:
// 文件名格式：prefix_01_TIMESTAMP_PICID.jpg
//                 ^         ^     ^
//                 |         |     |
//              第一个_     第二个_  .
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
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        LOGW("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息
    int total = 0;
    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            LOGD("%-20s %-4d %s\n", action_id_dir_entry->d_name,
                 action_id_dir_entry->d_type,
                 action_id_name_temp);
        }
        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {

            continue;
        }
        int ret;
        uint32_t temp_action_id_timestamps = 0; // 当前temp_action_id_timestamps
        uint8_t temp_action_id_index = 0;             // 当前temp_action_id_index
        ret = do_str_2_action_id(action_id_name_temp,
                                 &temp_action_id_timestamps, &temp_action_id_index);
        if (ret) {
            LOGW("      解析action_id失败 ： %s %d\n", action_id_name_temp, ret);
            continue;
        }

        // 继续校验名称
        // 分别打开文件夹查询pic_id匹配的图片
        struct dirent *pic_file_entry;  //   图片文件信息
        char pic_file_name[1024];       //   图片名称
        char result[1024];              //   图片path
        char action_id_path[1024];      //   action_id目录路径temp
        char file_path_tmp[1024];       //
        unsigned char md5_result_tmp[16];

        uint32_t snapshot_timestamps_temp;
        uint8_t pic_id_temp;
        snprintf(action_id_path, sizeof(action_id_path), "%s/%s", g_pic_dir_path, action_id_name_temp);
        DIR *action_id_dir = opendir(action_id_path);
        if (!action_id_dir) {
            LOGW("无法打开目录 %s \n", action_id_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            if (total >= 1024) {
                LOGW("Too many pictures, maximum is 1024");
                break;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            if (HD_UART_PARSER_DEBUG) {
                LOGD("开始解析图片文件:<%s> \n", pic_file_name);
            }
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                LOGW("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }
            // 创建图片信息。。。。。。。。。。开始

            if (HD_UART_PARSER_DEBUG) {
                LOGW("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp, pic_id_temp);
            }
            snprintf(file_path_tmp, sizeof(file_path_tmp), "%s/%s", action_id_path, pic_file_name);
            if (HD_UART_PARSER_DEBUG) {
                LOGD("获取图片md5...%s\n", file_path_tmp);
            }
            ret = hd_md5(file_path_tmp, md5_result_tmp);

            if (ret) {
                LOGE("hd_md5 fail.\n");
                continue;
            }

            if (HD_UART_PARSER_DEBUG) {
                LOGD("获取图片大小...\n");
            }
            struct stat st_tmp;
            if (stat(file_path_tmp, &st_tmp) != 0) {
                LOGE("stat fail\n");
                //return 1;//st_tmp.st_size;  // 返回文件大小（字节）
                break;
            }

            uint32_t size = st_tmp.st_size;
            if (HD_UART_PARSER_DEBUG) {
                LOGD("图片信息：\n");
                LOGD("name                    =          %s\n", pic_file_name);
                LOGD("id                      =          %hhu\n", pic_id_temp);
                LOGD("size                    =          %u\n", size);
                LOGD("action_id_timestamps    =          %d\n", temp_action_id_timestamps);
                LOGD("action_id_index         =          %d\n", temp_action_id_index);
                LOGD("snapshot_timestamps     =          %u\n", temp_action_id_timestamps);
                LOGD("md5                     =          ");

                for (int j = 0; j < 16; ++j) {
                    printf("%02x ", md5_result_tmp[j]);
                }
                LOGD("\n");
            }
            if (HD_UART_PARSER_DEBUG) {
                LOGD("创建图片信息\n");
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


            // 创建图片信息。。。。。。。。。。。。。。。结束

        }
        closedir(action_id_dir);
    }
    closedir(dir);
    *pic_infos_size = total;
    return 0;
}

static int do_collect_all_pic_infos(hd_dynamic_pic_info **pic_infos, uint32_t *pic_infos_size) {
    LOGD(">>>>>>>>>>>>>>>>>>>>>>\n");
    /* 一、搜索目录 */
    LOGD("1.搜索目录[%s]\n", g_pic_dir_path);
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        LOGW("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char *action_id_dir_names[1024] = {0}; // action_id目录名称数组
    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息
    int action_id_count = 0;
    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            LOGD("%-20s %-4d %s\n", action_id_dir_entry->d_name,
                 action_id_dir_entry->d_type,
                 action_id_name_temp);
        }

        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {
            continue;
        }
        // 继续校验名称
        action_id_dir_names[action_id_count] = strdup(action_id_name_temp);
        action_id_count++;
    }
    LOGI("2.搜索目录完毕！aciont_id目录数量为：%d\n", action_id_count);
    if (action_id_count > 0) {
        for (int i = 0; i < action_id_count; ++i) {
            LOGI("%s,", action_id_dir_names[i]);
        }
        LOGI("\n");
    }


    uint32_t total = 0; // 所有的图片
    if (action_id_count == 0) {
        *pic_infos = NULL;
        *pic_infos_size = 0;
        LOGW("4.解析文件夹完毕！！！图片为空\n");
    } else {
        /* 二、遍历action_id目录，搜索图片文件 */
        uint32_t MAX = 0xff; // 不超过255
        char action_id_path[1024]; // action_id目录路径temp
        size_t temp_size = sizeof(hd_dynamic_pic_info) * MAX;
        hd_dynamic_pic_info *infos = (hd_dynamic_pic_info *) malloc(temp_size);
        memset(infos, 0, temp_size);
        LOGD("3.遍历action_id目录，搜索图片文件。\n");
        uint32_t temp_action_id_timestamps = 0; // 当前temp_action_id_timestamps
        uint8_t temp_action_id_index = 0;             // 当前temp_action_id_index
        int ret;
        for (int i = 0; i < action_id_count; ++i) {
            // 解析action_id
            LOGD("------------------------------\n");
            LOGD("<%d>解析文件夹[%s]\n", i, action_id_dir_names[i]);
            ret = do_str_2_action_id(action_id_dir_names[i],
                                     &temp_action_id_timestamps, &temp_action_id_index);
            if (ret) {
                LOGW("      解析action_id失败 ： %s %d\n", action_id_dir_names[i], ret);
                continue;
            }
            if (HD_UART_PARSER_DEBUG) {
                LOGD("解析文件夹[%s]成功! timestamps：%d ,index：%d \n", action_id_dir_names[i], temp_action_id_timestamps,
                     temp_action_id_index);
            }
            snprintf(action_id_path, sizeof(action_id_path), "%s/%s", g_pic_dir_path, action_id_dir_names[i]);
            struct dirent *pic_file_entry;  //   图片文件信息
            char pic_file_name[1024];       //   图片名称
            DIR *action_id_dir = opendir(action_id_path);
            if (!action_id_dir) {
                LOGW("无法打开目录 %s \n", action_id_path);
                return -1;
            }
            // 遍历action_id目录下的图片
            unsigned char md5_result_tmp[16];
            char file_path_tmp[1024];
            uint32_t snapshot_timestamps_temp;
            uint8_t pic_id_temp;

            while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
                if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                    continue;
                }
                // 根据文件名称 解析pic_id
                snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
                LOGD("-->开始解析图片文件:<%s> \n", pic_file_name);
                //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
                ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
                if (ret) {
                    LOGW("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                    continue;
                }
                LOGW("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp, pic_id_temp);

                snprintf(file_path_tmp, sizeof(file_path_tmp), "%s/%s", action_id_path, pic_file_name);
                LOGD("获取图片md5...%s\n", file_path_tmp);
                ret = hd_md5(file_path_tmp, md5_result_tmp);

                if (ret) {
                    LOGE("hd_md5 fail.\n");
                    continue;
                }

                LOGD("获取图片大小...\n");
                struct stat st_tmp;
                if (stat(file_path_tmp, &st_tmp) != 0) {
                    LOGE("stat fail\n");
                    return 1;//st_tmp.st_size;  // 返回文件大小（字节）
                }

                uint32_t size = st_tmp.st_size;
                if (HD_UART_PARSER_DEBUG) {
                    LOGD("图片信息：\n");
                    LOGD("name                    =          %s\n", pic_file_name);
                    LOGD("id                      =          %hhu\n", pic_id_temp);
                    LOGD("size                    =          %u\n", size);
                    LOGD("action_id_timestamps    =          %d\n", temp_action_id_timestamps);
                    LOGD("action_id_index         =          %d\n", temp_action_id_index);
                    LOGD("snapshot_timestamps     =          %u\n", temp_action_id_timestamps);
                    LOGD("md5                     =          ");

                    for (int j = 0; j < 16; ++j) {
                        printf("%02x ", md5_result_tmp[j]);
                    }
                    LOGD("\n");
                }

                LOGD("创建图片信息\n");
                hd_dynamic_pic_info info;//= (hd_dynamic_pic_info *) malloc(sizeof(hd_dynamic_pic_info));
                info.action_id_index = temp_action_id_index;
                info.action_id_timestamps = temp_action_id_timestamps;
                info.snapshot_timestamps = snapshot_timestamps_temp;
                info.size = size;
                info.id = pic_id_temp;

                memcpy(info.md5, md5_result_tmp, sizeof(info.md5));

                infos[total] = info;
                total++;
            }

            closedir(action_id_dir);
            if (action_id_dir_names[i] != NULL) {
                free(action_id_dir_names[i]);
            }
        }

        // 赋值
        *pic_infos = infos;
        *pic_infos_size = total;

        LOGI("4.解析文件夹完毕！！！结果：%d张图片。\n", total);
    }
    closedir(dir);

    LOGD("查找所有pic完毕！！！\n");
    LOGI("<<<<<<<<<<<<<<<<<<<<<<\n");
    return 0;
}

static int do_find_pic_by_pic_id(uint8_t pic_id, char **file_path) {
    if (HD_UART_PARSER_DEBUG) {
        LOGD("do_find_pic_by_pic_id pic_id = %d,g_pic_dir_path=%s\n", pic_id, g_pic_dir_path);
        LOGD("1.搜索目录[%s]\n", g_pic_dir_path);
    }
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        LOGW("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息

    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        if (HD_UART_PARSER_DEBUG) {
            LOGD("%-20s %-4d %s\n", action_id_dir_entry->d_name,
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
            LOGW("无法打开目录 %s \n", action_id_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            if (HD_UART_PARSER_DEBUG) {
                LOGD("开始解析图片文件:<%s> \n", pic_file_name);
            }
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                LOGW("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }

            if (pic_id_temp == pic_id) {
                if (HD_UART_PARSER_DEBUG) {
                    LOGI("解析图片文件成功  snapshot_timestamps:%d ，pic_id_temp=%d \n", snapshot_timestamps_temp, pic_id_temp);
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
    LOGW("解析图片文件 没有找到：%d\n", pic_id);
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
            LOGI("[%d获取模型名称]%02x\n", g_addr, property_id_out);
            // 查找模型
            char version[1024];
            ret = hd_find_model_name(MODEL_DIR_PATH, version, MODEL_PREFIX);
            if (ret) {
                return ret;
            }
            LOGI("模型名称: %s\n", version);
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
        case PROPERTY_HD_ID_DEBUG_ACTION_ID: {
            LOGI("[%d获取属性]%02x DEBUG action_id\n", g_addr, property_id_out);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ANGEL: {
            LOGI("[%d获取属性]%02x DEBUG angel\n", g_addr, property_id_out);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_HD_UART_VERSION: {
            LOGI("[%d获取属性]%02x DEBUG hd_uart version\n", g_addr, property_id_out);
            char version[2048];
            snprintf(version, sizeof(version), "%s-%s", (g_version == NULL) ? "" : g_version, hd_uart_version());
//            char *version = strcat(hd_uart_version(), (g_version == NULL) ? "" : g_version);
//            char *str = (char *)malloc(result_value_size_out + 1); // +1 用于 null 终止符
//            if (str == NULL) {
//                break;
//            }
//            memcpy(str, result_value_out, result_value_size_out);
//            str[result_value_size_out] = '\0';
//            free(str);

            LOGI("hd_uart版本: %s\n", version);
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
    LOGD("handle_property_set\n");
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
        LOGW("hd_slave_property_set_decode error\n");
        return -1;
    }
    hd_printf_buff(result_value_out, result_value_size_out, "event", 0);
    if (HD_UART_PARSER_DEBUG) {
        LOGD("开始处理event\n");
    }
    switch (property_id_out) {
        case PROPERTY_HD_ID_DEBUG: {
            LOGI("[%d设置属性]%02x DEBUG开关\n", g_addr, PROPERTY_HD_ID_DEBUG);
            if (result_value_size_out == 1) {
                uint8_t on = result_value_out[0];
                if (on >= HD_LOGGER_LEVEL_DEBUG && on <= HD_LOGGER_LEVEL_ERROR) {
                    hd_logger_set_level(on);
                    LOGI("设置开关成功:%d\n", on);
                    ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                       g_addr, property_id_out, 1);
                    if (ret == 0) {
                        ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                        if (ret) {
                            LOGD("do_uart_write error \n");
                        } else {
                            // 如果是pull 继续拉取文件
                        }
                    }
                }
            }

            break;
        }

        case PROPERTY_HD_ID_FACTORY_MODE: {
            LOGI("[%设置工厂模式]%02x FACTORY_MODE\n", g_addr, PROPERTY_HD_ID_FACTORY_MODE);
            if (result_value_size_out == 1) {
                uint8_t on = result_value_out[0];
                g_serial_mode = on ? HD_SERIAL_SHELL_MODE : HD_SERIAL_NORMAL_MODE;
                LOGI("设置工厂模式成功:%d\n", on);
                ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                   g_addr, property_id_out, 1);
                if (ret == 0) {
                    ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        LOGD("do_uart_write error \n");
                    } else {
                        // 如果是pull 继续拉取文件
                    }
                }
            }
            break;
        }

        case PROPERTY_HD_ID_PUSH: {
            LOGI("[%d push]%02x PUSH\n", g_addr, PROPERTY_HD_ID_FACTORY_MODE);
            if (result_value_size_out <= 0) {

            } else {
                // 获取file_path
                // 获取md5
                LOGD("payload_data_size=%d\n", payload_data_size);
                ret = hd_slave_property_set_push_decode(g_push_mode_file_md5, &g_push_mode_file_size,
                                                        g_push_mode_file_path,
                                                        payload_data + 1,
                                                        payload_data_size - 1);
                if (ret) {
                    LOGW("hd_slave_property_set_push_decode error ret = %d\n", ret);
                    ret = hd_slave_property_set_encode(&protocol_data_out, &protocol_data_size_out,
                                                       g_addr, property_id_out, ret);
                    if (ret == 0) {
                        ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                    }
                    break;
                }
                LOGD("解析结果：\n");
                LOGD("file_size = %zu\n", g_push_mode_file_size);
                LOGD("md5 = [");
                for (int i = 0; i < 16; ++i) {
                    printf("%02x ", g_push_mode_file_md5[i]);
                }
                printf("]\n");
                LOGD("file_path = %s\n", g_push_mode_file_path);
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
                LOGI("<<<<打开文件成功 准备接受数据>>>>\n");
                // 写入数据...
                // fwrite(data, 1, size, file);
                // fclose(file);  // 关闭文件
                LOGI("<<<<切换到接受文件模式>>>>\n");
                on_serial_mode_changed(HD_SERIAL_PUSH_MODE);
                break;
            }

            break;
        }

        case PROPERTY_HD_ID_DEBUG_ACTION_ID: {
            LOGI("[%d设置属性]%02x DEBUG action_id\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ANGEL: {
            LOGI("[%d设置属性]%02x DEBUG angel\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_HD_UART_VERSION: {
            LOGI("[%d设置属性]%02x DEBUG hd_uart version\n", g_addr, PROPERTY_HD_ID_DEBUG);
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
            LOGW("[%d设置属性]%02x 暂不支持\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
    }

//    if (ret == 0) {
//        ret = do_uart_write(protocol_data_out, protocol_data_size_out);
//        if (ret) {
//            LOGD("do_uart_write error \n");
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
    uint8_t *pic_id = g_hd_on_event(EVENT_SNAPSHOT, NULL, 0);
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
        LOGI("[read_from_buffer] g_file_buffer_size = %d\n", g_file_buffer_size);
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
        LOGE("[read_from_buffer] in_offset 大于文件最大大小<%d/%d> insize=%d\n", in_offset, g_file_buffer_size, in_size);
        return -2;
    }
    size_t real_read_size = in_size;
    LOGI("[read_from_buffer] in_offset = %d \n", in_offset);
    LOGI("[read_from_buffer] in_size = %d \n", in_size);
    LOGI("[read_from_buffer] g_file_buffer_size = %d \n", g_file_buffer_size);
    LOGI("[read_from_buffer] in_offset + in_size - g_file_buffer_size = %d \n", g_file_buffer_size - in_offset);

    if (g_file_buffer_size - in_offset < in_size) {
        real_read_size = g_file_buffer_size - in_offset;
    }

    if (HD_UART_PARSER_DEBUG) {
        LOGI("[read_from_buffer] real_read_size = %d \n", real_read_size);
    }
    if (real_read_size <= 0) {
        LOGW("real_read_size==0,没有数据可读了\n");
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
        LOGW("hd_slave_pull_pic_decode error \n");
        return -1;
    }
    if (HD_UART_PARSER_DEBUG) {
        LOGD("解析到需要拉取的图片信息：\n");
        LOGD("pic_id            :       %d(0x%02x)\n", out_pic_id, out_pic_id);
        LOGD("offset            :       %d(0x%02x)\n", out_offset, out_offset);
        LOGD("read_len          :       %d(0x%02x)\n", out_read_len, out_read_len);
    }
    int goon = 0;
    if (g_file_pulling) { // 正在上传
        if (out_pic_id == g_file_pic_id) {
            //继续
            if (HD_UART_PARSER_DEBUG) {
                LOGI("继续pull！\n");
            }
            goon = 1;
        } else {
            //清除掉 重新开始
            LOGI("pic不相等 重新开始新的pull！\n");
            goon = 0;
        }
    } else { // 新的上传
        LOGI("开始新的pull\n");
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
                LOGW("拉取的图片数据信息失败 %d\n", ret);
                return -1;
            }
            // 加载图片到缓存
            ret = load_file_to_buffer(filePath);
            if (ret) {
                LOGW("加载图片数据失败\n");
                return -2;
            }
            if (filePath != NULL) {
                free(filePath);
            }
            g_file_pic_id = out_pic_id;
            g_file_pulling = 1;
        } else {
            LOGW("从新拉取的数据:offset=%d应该从0开始\n", out_offset);
            return 10;
        }
    }

    if (HD_UART_PARSER_DEBUG) {
        LOGD("当前上传的文件pic_id   :      %d(0x%02x)\n", g_file_pic_id, g_file_pic_id);
        LOGD("当前上传的文件大小      :      %d(0x%02x)\n", g_file_buffer_size, g_file_buffer_size);
    }
    // 加载图片，从buff offset中读 read_len 数据
    unsigned char read_data[PROTOCOL_MAX_FRAME_LEN];
    size_t offset = out_offset;
    size_t read_len = out_read_len;
    size_t real_read_len = 0;
    ret = read_from_buffer(read_data, read_len, offset, &real_read_len);
    if (ret) {
        LOGW("[从机%d]读文件异常。\n", g_addr);
        resetFileBuffer();
        return 4;
    }
    if (HD_UART_PARSER_DEBUG) {
        LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", real_read_len, real_read_len);
    }
    if (real_read_len == 0) {
        LOGW("[从机%d]读取文件大小为0。\n", g_addr);
        resetFileBuffer();
        return 5;
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, read_data, real_read_len);
    if (ret) {
        LOGW("hd_slave_pull_pic_encode fail！\n");
        return -5;
    }
    if (real_read_len < out_read_len) {
        LOGW("[从机%d]文件读到结尾了。%d,%d\n", g_addr, real_read_len, out_read_len);
        resetFileBuffer();
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
//        LOGW("hd_slave_pull_pic_decode error \n");
//        return -1;
//    }
//    if (HD_UART_PARSER_DEBUG) {
//        LOGD("需要拉取的图片信息：\n");
//        LOGD("pic_id        :       %d(0x%02x)\n", out_pic_id, out_pic_id);
//        LOGD("offset        :       %d(0x%02x)\n", out_offset, out_offset);
//        LOGD("read_len      :       %d(0x%02x)\n", out_read_len, out_read_len);
//    }
//    char *filePath;
//    ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
//    if (ret) {
//        LOGW("拉取的图片数据信息失败\n");
//        return -1;
//    }
//    LOGI("查找到的图片名称：%s\n", filePath);
//    unsigned char *result;
//    size_t size;
//
////    ret = do_cut_file_data(filePath, &result, &size, 246*1024, out_read_len);
//    ret = do_cut_file_data(filePath, &result, &size, out_offset, out_read_len);
//    if (ret) {
//        return -1;
//    }
//    LOGD("需要拉取的图片数据信息：\n");
//    LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", size, size);
//    if (size == 0) {
//        LOGI("[从机%d]文件读完了。\n", g_addr);
//        return -1;
//    }
//    if (size < out_read_len) {
//        LOGI("[从机%d]文件读到结尾了。\n", g_addr);
//    }
//
//    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, result, size);
//    if (ret) {
//        LOGD("hd_slave_pull_pic_encode fail！\n");
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
        LOGW("hd_slave_pic_info_decode error \n");
        return -1;
    }
    // 获取g_addr下所有action_id图片
    hd_dynamic_pic_info pic_infos[1024] = {0};
    uint32_t pic_infos_size;
//    ret = do_collect_all_pic_infos(&pic_infos, &pic_infos_size);
    ret = do_collect_all_pic_infos_v2(pic_infos, &pic_infos_size);
    if (ret) {
        LOGW("do_collect_all_pic_infos error \n");
        return -1;
    }
    LOGI("搜索图片结果，大小=(%d):\n", pic_infos_size);
    if (pic_infos_size > 0) {
        if (HD_UART_PARSER_DEBUG) {
            LOGD("打印搜索结果(%d):\n", pic_infos_size);
            for (int i = 0; i < pic_infos_size; ++i) {
                LOGD("******\n");
                LOGD("id                    =   %d\n", pic_infos[i].id);
                LOGD("size                  =   %d\n", pic_infos[i].size);
                LOGD("action_id_index       =   %d\n", pic_infos[i].action_id_index);
                LOGD("action_id_timestamps  =   %d\n", pic_infos[i].action_id_timestamps);
                LOGD("snapshot_timestamps   =   %d\n", pic_infos[i].snapshot_timestamps);
                LOGD("md5                   =   ");
                for (int j = 0; j < 16; ++j) {
                    printf("%02x ", pic_infos[i].md5[j]);
                }
                LOGD("\n");
            }
        }
    }

    ret = hd_slave_pic_info_encode(result, result_size, g_addr, pic_infos,
                                   pic_infos_size);
//    if (pic_infos != NULL) {
//        LOGW(" free(pic_infos %p \n", pic_infos);
//        free(pic_infos);
//        LOGW(" free(pic_infos end! \n");
//    }

    if (ret) {
        LOGW("hd_slave_pic_info_encode error %d \n", ret);
        return -1;
    }

//    LOGI("hd_slave_pic_info_encode end! pic_infos_size = %d\n", pic_infos_size);
    return 0;
}

/* 处理 3.8 删除图片（0x08）*/
static int handle_delete_pic(const unsigned char *payload_data, uint32_t payload_data_size) {
    uint8_t out_pic_id;
    int ret = hd_slave_delete_pic_decode(&out_pic_id, payload_data, payload_data_size);
    if (ret) {
        LOGW("hd_slave_delete_pic_decode error \n");
        return -1;

    }
    LOGD("需要删除的图片pic_id : <%d> \n", out_pic_id);

    // 遍历文件夹依次查询图片id
    char *filePath = NULL;
    ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
    if (ret != 0) {
        LOGW("do_find_pic_by_pic_id error \n");
        return -1;
    }
    if (HD_UART_PARSER_DEBUG) {
        LOGD("查询结果：图片地址=%s\n", filePath);
    }

    // 删除文件
    ret = remove(filePath);
    // 检查当前文件夹是否为空 是则删除文件夹
    if (filePath != NULL) {
        free(filePath);
    }

    return ret == 0 ? 0 : -2;
}

/* 处理 3.10 图片拉取完成（0x0A）*/
static int handlePullPicComplete(const unsigned char *payload_data, uint32_t payload_data_size) {
    // 图片拉取完成 需要做什么？
    resetFileBuffer();
    return 0;
}

static void timeout_handler(int sig) {
    if (g_serial_mode == HD_SERIAL_HD_PUSH_MODE) {
        LOGW("传输文件超时%s！！！\n", g_hd_push_mode_file_path);
        resetHDPushMode();
    }
}

// 见 do_uart_recv_with_hd_push
static void handle_hd_push_file(unsigned char *payload_data, uint32_t payload_data_size) {
    // 解析
    LOGI("handle_hd_push_file\n");
    uint8_t type;
    uint32_t file_size;
    unsigned char file_md5[16] = {0};
    char file_name[2048] = {0};
    int ret;
    ret = hd_slave_file_decode_payload(&type, &file_size, file_md5, file_name, payload_data, payload_data_size);
    if (ret) {
        LOGW("handle_hd_push_file hd_slave_file_decode_payload error = %d\n", ret);
        return;
    }
    switch (type) {
        case 0x01: {
            if (file_size <= 0) {
                LOGW("handle_hd_push_file hd_slave_file_decode_payload file_size error\n");
                return;
            }

            if (strlen(file_name) <= 0) {
                LOGW("handle_hd_push_file hd_slave_file_decode_payload file_name error\n");
                return;
            }

            LOGI("============ 准备接受文件的信息a ============ \n");
            LOGI("type           :           %02x \n", type);
            LOGI("file_size      :           %02x \n", file_size);
            LOGI("file_md5       :           ", file_size);
            hd_printf_buff(file_md5, 16, "md5", 0);
            LOGI("file_name      :           %s \n", file_name);

            LOGI("============ 准备接受文件的信息z ============\n");

            g_hd_push_mode_file_size = file_size;
            memcpy(g_hd_push_mode_file_md5, file_md5, 16);
            snprintf(g_hd_push_mode_file_path, sizeof(g_hd_push_mode_file_path), "%s/%s%s", MODEL_DEST_PATH, file_name,
                     MODEL_PREFIX);
            snprintf(g_hd_push_mode_file_path_downloading, sizeof(g_hd_push_mode_file_path_downloading), "%s%s",
                     g_hd_push_mode_file_path, MODEL_PREFIX_DOWNLOADING);
            // 创建文件夹，准备接受数据
            LOGI("g_hd_push_mode_file_path_downloading      :           %s \n", g_hd_push_mode_file_path_downloading);
            ret = create_directory_if_not_exists(g_hd_push_mode_file_path_downloading);
            if (ret) {
                perror("handle_hd_push_file create_directory_if_not_exists fail.\n");
                return;
            }
            g_hd_push_mode_file = fopen(g_hd_push_mode_file_path_downloading, "wb");  // 二进制写入模式
            if (!g_hd_push_mode_file) {
                perror("Failed to open file");
                return;
            }
            LOGI("<<<<打开文件成功 准备接受数据>>>>\n");
            on_serial_mode_changed(HD_SERIAL_HD_PUSH_MODE);
            // 回复
            unsigned char *out_payload;
            uint32_t out_payload_size;
            uint8_t int_type = 0x01;
            uint8_t int_result = 0;
            ret = hd_slave_file_encode_payload(&out_payload, &out_payload_size, int_type, int_result, 0);
            if (ret) {
                LOGW("handle_hd_push_file hd_slave_file_encode_payload error = %d\n", ret);
                resetHDPushMode();
                return;
            }
            unsigned char *out_p;
            uint32_t out_p_size;
            ret = hd_camera_protocol_encode(&out_p, &out_p_size, g_addr, CMD_HD_PUSH_FILE, out_payload_size,
                                            out_payload);
            free(out_payload);
            if (ret) {
                LOGW("handle_hd_push_file hd_camera_protocol_encode error = %d\n", ret);
                resetHDPushMode();
                return;
            }
            ret = do_uart_write(out_p, out_p_size);
            free(out_p);
            if (ret) {
                LOGW("handle_hd_push_file do_uart_write error = %d\n", ret);
                resetHDPushMode();
                return;
            }
            LOGI("<%s>(%d)等待上传至<%s> ...\n", file_name, file_size, g_hd_push_mode_file_path_downloading);
            // 准备接受数据 do_uart_recv_with_hd_push
            // 开启超时 TODO
            alarm(HD_FILE_PUSH_TIMEOUT);
        }
        default:
            break;
    }
}

static void handle_extra_pull(unsigned char *payload_data, uint32_t payload_data_size) {
    int ret;
    int success = 0;
    unsigned char *protocol_data_out = NULL;
    uint32_t protocol_data_size_out;
    while (1) {
        LOGD("[handle_extra_pull] \n");
        ret = hd_slave_pull_decode(g_pull_mode_file_path,
                                   payload_data,
                                   payload_data_size);

        if (ret) {
            LOGW("[handle_extra_pull] hd_slave_pull_decode error ret = %d\n", ret);
            success = 1;
            break;
        }
        LOGD("[handle_extra_pull] 解析结果：\n");
        LOGD("[handle_extra_pull] 需要拉取的文件file_path = %s\n", g_pull_mode_file_path);
        LOGD("[handle_extra_pull] 获取文件信息\n");
        unsigned char md5[16];
        LOGD("[handle_extra_pull] 获取文件信息 md5\n");
        ret = hd_md5(g_pull_mode_file_path, md5);
        if (ret) {
            LOGW("[handle_extra_pull]  hd_md5 error ret = %d\n", ret);
            success = 2;
            break;
        }
        LOGD("[handle_extra_pull] 获取文件信息 md5成功！\n");
        LOGD("[handle_extra_pull] 获取文件信息 file_size\n");
        int fd;
        fd = open(g_pull_mode_file_path, O_RDWR);
        if (fd == -1) {
            LOGD("[handle_extra_pull] 打开文件失败:%s 原因：%d->%s \n ", g_pull_mode_file_path, errno, strerror(errno));
            success = 3;
            break;
        }
        struct stat file_stat;
        if (fstat(fd, &file_stat) == -1) {
            LOGD("[handle_extra_pull] 获取文件大小失败。fd:%d\n", fd);
            success = 4;
            close(fd);
            break;
        }
        off_t file_size = file_stat.st_size;
        LOGD("[handle_extra_pull] 获取文件信息 file_size成功！\n");

        LOGD("[handle_extra_pull] file_size   :  < %d >bytes\n", file_size);
        LOGD("[handle_extra_pull] file_md5    :  ");
        for (int i = 0; i < sizeof(md5); ++i) {
            printf("%02x ", md5[i]);
        }
        printf("\n");

        LOGD("[handle_extra_pull] 准备应答... \n");
        ret = hd_slave_pull_encode(&protocol_data_out, &protocol_data_size_out, g_addr,
                                   0, md5, file_size, (const char *) g_pull_mode_file_path);
        if (ret) {
            close(fd);
            success = 5;
            break;
        }
        close(fd);
        hd_printf_buff(protocol_data_out, protocol_data_size_out, "pull应答", 0);
        ret = do_uart_write(protocol_data_out, protocol_data_size_out);
        if (protocol_data_out != NULL) {
            free(protocol_data_out);
        }

        LOGD("[handle_extra_pull] 应答完毕! \n");
        g_pull_mode_file = fopen(g_pull_mode_file_path, "rb");
        if (g_pull_mode_file == NULL) {
            success = 6;
            fprintf(stderr, "[handle_extra_pull] 无法打开文件 %s: %s\n", g_pull_mode_file_path, strerror(errno));
            break;
        }
        // 先发送一个文件头：
        do_uart_write(PULL_MODE_FILE_HEADER_TAIL, sizeof(PULL_MODE_FILE_HEADER_TAIL));
        unsigned char buffer[1024];
        size_t bytes_read;
        LOGI("[handle_extra_pull] 开始pull文件...\n");
        LOGI("----------------------------\n");
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
        LOGI("\n----------------------------\n");
        LOGI("[handle_extra_pull] pull文件结束！\n");
        usleep(4000);
        rs485_pwr_off();
        do_uart_write(PULL_MODE_FILE_HEADER_TAIL, sizeof(PULL_MODE_FILE_HEADER_TAIL));

        on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
        LOGI("[handle_extra_pull] pull文件完毕！应答...\n");
        ret = hd_slave_pull_encode(&protocol_data_out, &protocol_data_size_out, g_addr, 0xff,
                                   md5, file_size, (const char *) g_pull_mode_file_path);

        resetPullMode();
        if (ret) {
            success = 7;
            break;
        }
        LOGI("[handle_extra_pull] pull文件完毕！应答成功！\n");
        hd_printf_buff(protocol_data_out, protocol_data_size_out, "pull完毕", 0);
        do_uart_write(protocol_data_out, protocol_data_size_out);

        break;
    }

    if (protocol_data_out) {
        free(protocol_data_out);
    }
}

static int handle_uart_data(const unsigned char *raw, size_t raw_size) {
    if (NULL == raw || raw_size <= 0) {
        LOGW("handle_uart_data raw == NULL || raw_size == 0\n");
        return 0;
    }
    LOGD("------------------------------handle_uart_data------------------------%d------\n", raw_size);
    uint8_t ret;
    uint8_t slave_addr_out;
    uint8_t cmd_out;
    uint32_t payload_data_size_out;
    unsigned char *payload_data_out = NULL;
    // 可以只先解析addr
    ret = hd_camera_protocol_decode(raw, raw_size, &slave_addr_out, &cmd_out, &payload_data_size_out,
                                    &payload_data_out);
    if (ret) {
        LOGW("hd_camera_protocol_decode error \n");
        return 0;
    }
    if (HD_UART_PARSER_DEBUG) {
        LOGD("slave_addr         :           %d \n", slave_addr_out);
        LOGD("cmd_out            :           %02x \n", cmd_out);
    }
    if (g_addr == 0 || (g_addr != slave_addr_out && slave_addr_out != PROTOCOL_BROADCAST)) {
        LOGW("从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, slave_addr_out);
        return -1;
    }

    char buff[2048];
    memset(buff, 0, sizeof(buff));

    switch (cmd_out) {

        case CMD_HD_PROPERTY_GET: {
            LOGI("[从机%d] HD查询属性（0xC2）预留\n", g_addr);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = handle_property_get(payload_data_out, payload_data_size_out, &out_protocol_data,
                                      &out_protocol_data_size);
            if (ret == 0) {
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            }
            if (out_protocol_data) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PROPERTY_SET: {
            LOGI("[从机%d] HD设置属性（0xC3）预留\n", g_addr);
            handle_property_set(payload_data_out, payload_data_size_out);
            break;
        }

        case CMD_HD_CAMERA_SNAPSHOT: {
            LOGI("[从机%d] HD主动抓图（0xC6）\n", g_addr);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = handle_snapshot_pic(&out_protocol_data, &out_protocol_data_size);
            LOGI("主动抓图ret:%d\n", ret);
            if (ret == 0) {
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            } else {
                ret = hd_slave_snapshot_encode(&out_protocol_data, &out_protocol_data_size, g_addr, PROTOCOL_UART_FAIL,
                                               0);
                if (ret) {
                    LOGD("hd_slave_snapshot_encode error \n");
                } else {
                    ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                    if (ret) {
                        LOGD("do_uart_write error \n");
                    }
                }

            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_INFO: {
            LOGI("[从机%d] HD查询摄像头存储的图片信息（0xC7）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out = 0;
            ret = handle_pic_infos(payload_data_out, payload_data_size_out, &protocol_data_out,
                                   &protocol_data_size_out);
            if (ret == 0) {
                // hd_printf_buff(protocol_data_out, protocol_data_size_out, "-", 1);
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGW("do_uart_write error \n");
                }
            } else {
                LOGW("handle_pic_infos error \n");
            }
            if (protocol_data_out != NULL) {
                LOGD("free(protocol_data_out) start... \n");
                free(protocol_data_out);
                LOGD("free(protocol_data_out) end \n");
            }
            break;
        }

        case CMD_HD_PIC_DELETE: {
            LOGI("[从机%d] HD删除图片（0xC8））\n", g_addr);
            int delete_ret = handle_delete_pic(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = hd_slave_delete_pic_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                             delete_ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                LOGW("hd_slave_delete_pic_encode error\n");
            } else {
                do_uart_write(out_protocol_data, out_protocol_data_size);
            }

            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_PULL: {
            LOGI("[从机%d] HD拉取图片（0xC9）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out;
            ret = handle_pull_pic(payload_data_out, payload_data_size_out, &protocol_data_out, &protocol_data_size_out);
            if (ret == 0) {
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGW("do_uart_write error \n");
                }
            } else {
                LOGW("handle_pull_pic fail ! ret = %d \n", ret);
                ret = hd_slave_pull_pic_encode(&protocol_data_out, &protocol_data_size_out, g_addr, PROTOCOL_UART_FAIL,
                                               NULL, 0);
                if (ret) {
                    LOGW("hd_slave_pull_pic_encode error %d\n", ret);
                } else {
                    ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        LOGW("do_uart_write error \n");
                    }
                }
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
            }
            break;
        }

        case CMD_HD_PIC_PULL_COMPLETED: {
            LOGI("[从机%d] HD图片拉取完成（0xCA）\n", g_addr);
            ret = handlePullPicComplete(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size;
            int result = hd_slave_pull_pic_complete_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                                           ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (result) {
                LOGW("hd_slave_pull_pic_complete_encode error\n");
            } else {
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGW("do_uart_write error \n");
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_BROADCAST_ACTION_ID: {
            LOGI("[从机%d] HD广播门开事件（0xCD）\n", g_addr);
            uint32_t out_action_id_timestamps;
            uint8_t out_action_id_index;
            uint8_t status;
            ret = hd_slave_action_id_decode(&status, &out_action_id_timestamps, &out_action_id_index, payload_data_out,
                                            payload_data_size_out);
            if (ret) {
                LOGD("hd_slave_action_id_decode error:%d \n", ret);
                break;
            }
            if (g_hd_on_action_id_changed != NULL) {

                do_action_id_2_str(buff, sizeof(buff), out_action_id_timestamps, out_action_id_index);
                LOGI("action_id = %s\n", buff);
                g_hd_on_action_id_changed(status, buff);
            }
            // 广播不需要响应
            break;
        }

        case CMD_HD_EXTRA_PULL: {
            LOGI("[从机%d] PULL（0x%02x）\n", g_addr, CMD_HD_EXTRA_PULL);
            handle_extra_pull(payload_data_out, payload_data_size_out);
            break;
        }

        case CMD_HD_PUSH_FILE: {
            LOGI("[从机%d] HD PUSH File（0x%02x）\n", g_addr, CMD_HD_PUSH_FILE);
            handle_hd_push_file(payload_data_out, payload_data_size_out);
            break;
        }

        default:
            LOGI("[从机%d] 暂不支持的CMD:%d\n", g_addr, cmd_out);
            break;

    }
    // free(payload_data_out); // 不需要free 因为没有用malloc

    return 0;
}

static void free_hd_frame_data(hd_frame_data *data) {
    if (data == NULL)return;
    if (data->data == NULL)return;
    free(data->data);
    free(data);
}

static void free_queue(HDBlockingQueue *queue) {
    if (queue != NULL) {
        for (int i = 0; i < queue->size; ++i) {
            void *per = queue->items[i];
            if (per != NULL) {
                hd_frame_data *data = (hd_frame_data *) per;
                free_hd_frame_data(data);
            }
        }
        hd_queue_destroy(queue);
    }
}

static void *handle_uart_data_thread(void *arg) {
    LOGI("handle_uart_data_thread start...\n");
    while (g_running) {
        void *item = hd_queue_take(g_frame_queue);
        if (item == NULL)continue;
        hd_frame_data *frame = (hd_frame_data *) item;
        if (frame->data_size <= 0 || frame->data == NULL) {
            free(frame);
            continue;
        }
        hd_printf_buff(frame->data, frame->data_size, "handle_uart_data_thread", 0);
        handle_uart_data(frame->data, frame->data_size);
        free_hd_frame_data(frame);
    }
    LOGI("handle_uart_data_thread end.\n");
    return NULL;
}

/************ handles a **************/

int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        const char *version,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
) {
    signal(SIGALRM, timeout_handler);
    hd_logger_set_level(HD_LOGGER_LEVEL_DEBUG);
    uint32_t delay = calculate_3_5_char_time(PROTOCOL_RATE_DEFAULT, 8, 0, 1);
    snprintf(g_pic_dir_path, sizeof(g_pic_dir_path), "%s", pic_dir_path);
    LOGI("calculate_3_5_char_time = %d\n", delay);
    LOGI(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    LOGI("hd_uart_init\n");
    LOGI("hd_uart_init version              :       <%s/%s>   \n", version, hd_uart_version());
    LOGI("hd_uart_init protocol_version     :       <%s>   \n", PROTOCOL_VERSION);
    LOGI("hd_uart_init addr                 :       <%d> \n", addr);
    LOGI("hd_uart_init pic_dir_path         :       <%s> \n", g_pic_dir_path);
    LOGI("hd_uart_init delay                :       <%d> \n", delay);
    LOGI("hd_uart_max               	    :       <%d> \n", PROTOCOL_MAX_FRAME_LEN);
    for (int i = 0; i < sizeof(PULL_MODE_FILE_HEADER_TAIL); ++i) {
        printf("%02x ", PULL_MODE_FILE_HEADER_TAIL[i]);
    }
    printf("\n");
    LOGI(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    g_frame_queue = hd_queue_create(1024);
    if (g_frame_queue == NULL) {
        LOGE("createQueue error!");
        return 0;
    }
    g_addr = addr;
    snprintf(g_version, sizeof(g_version), "%s", version == NULL ? "" : version);
    g_hd_on_action_id_changed = on_action_id_changed;
    g_hd_on_event = on_event;
    g_delay = delay;
    hd_camera_shell_init(addr);
    g_running = 1;
    pthread_create(&g_frame_consume_t, NULL, handle_uart_data_thread, NULL);
    LOGI("hd_uart_init completed !!!\n");
    return 0;
}

void hd_uart_recv(uint8_t byte) {
    if (g_running == 0)return;
    switch (g_serial_mode) {
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
            do_uart_recv_with_hd_push(byte);
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

void hd_uart_deinit() {
    LOGI("hd_uart_deinit\n");
    g_hd_on_action_id_changed = NULL;
    g_hd_on_event = NULL;
    g_running = 0;
    signal_sent = 0;
    if (g_frame_consume_t) {
        pthread_join(g_frame_consume_t, NULL);
    }
    on_serial_mode_changed(HD_SERIAL_NORMAL_MODE);
    hd_camera_shell_deinit();
    free_queue(g_frame_queue);
    g_frame_queue = NULL;

}

char *hd_uart_version() {
    return HD_UART_PARSER_VERSION_INTERNAL;
}

static void on_serial_mode_changed(HD_SERIAL_MODE mode) {
    g_serial_mode = mode;
    LOGI("############################################## \n", mode);
    LOGI("############# serial_mode : [%d] ############## \n", mode);
    LOGI("############################################## \n", mode);
}
