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
#include "hd_pic_infos.h"
#include "hd_c_log.h"

#define HD_UART_PARSER_DEBUG_APp                0                           // debug开启
#define HD_UART_PARSER_DEBUG_Uart               0                           // debug开启
#define HD_UART_PARSER_DEBUG                    0                           // debug开启
#define FRAME_HEADER_H                          PROTOCOL_HEADER_1           // 头1
#define FRAME_HEADER_L                          PROTOCOL_HEADER_0           // 头2
#define MAX_FILE_SIZE                           PROTOCOL_MAX_FRAME_LEN      // 最大图片传输大小
#define MAX_FILE_PATH_SIZE                      1024                        // path大小
#define JPG_SUFFIX                              ".jpg"                      // 图片格式
#define JPG_SUFFIX_LEN                          4                           // 图片格式长度

#define SNAP_TEST_WITH_PURE                         0
#define SNAP_PATH                                   "/userdata/hadlinks"           // 图片地址
#define SNAP_DEMO_CROP_PIC                          "/userdata/crop.jpg"           // 动态截图副本
//#define SNAP_PATH                                     "/Users/xiangpengle/CLionProjects/hd_camera/test_case/userdata/hadlinks"
//#define SNAP_DEMO_CROP_PIC                            "/Users/xiangpengle/CLionProjects/hd_camera/test_case/userdata/crop.jpg"


#define BUFFER_SIZE 4096


//<editor-fold desc="debug">
// 复制文件函数
static int copy_file(const char *src_path, const char *dest_path) {
    FILE *src_file = fopen(src_path, "rb");
    if (src_file == NULL) {
        perror("无法打开源文件");
        return -1;
    }

    FILE *dest_file = fopen(dest_path, "wb");
    if (dest_file == NULL) {
        perror("无法创建目标文件");
        fclose(src_file);
        return -1;
    }

    char buffer[BUFFER_SIZE];
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, BUFFER_SIZE, src_file)) > 0) {
        if (fwrite(buffer, 1, bytes_read, dest_file) != bytes_read) {
            perror("写入目标文件失败");
            fclose(src_file);
            fclose(dest_file);
            return -1;
        }
    }

    fclose(src_file);
    fclose(dest_file);
    return 0;
}
//</editor-fold>

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

//<editor-fold desc="全局变量">
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
//</editor-fold>

//<editor-fold desc="局部函数定义">
static void do_uart_recv(uint8_t str);

static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, size_t *frame_length);

static int do_str_2_action_id(const char *action_id_str, uint32_t *action_id_timestamps, uint8_t *action_id_index);

static int handle_uart_data(const unsigned char *raw, size_t raw_size);

static void my_remove_directory(const char *path);
//</editor-fold>

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
        LOGI("hd_camera_uart_write over \n");
    }
    return 0;
}

// 帧解析函数
static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, size_t *frame_length) {
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
    static size_t data_index = 0;
    static size_t data_len = 0;
    static uint16_t expected_crc = 0;
    static uint16_t calculated_crc = 0;
    static size_t current_pos = 0;
    if (!g_running) {
        LOGW("not running\n");
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
                LOGW("data_len = %u ,frame_length = %u,current_pos=%u\n", data_len, frame_length, current_pos);
                for (int i = 0; i < 40; ++i) {
                    LOGW("frame_buffer[%d] = %02x \n", i, frame_buffer[i]);
                }
                LOGW("len error : 全局信息：\n");
                LOGW("data_index        =  %02x (%d)\n", data_index, data_index);
                LOGW("data_len          =  %02x (%d)\n", data_len, data_len);
                LOGW("expected_crc      =  %02x (%d)\n", expected_crc, expected_crc);
                LOGW("calculated_crc    =  %02x (%d)\n", calculated_crc, calculated_crc);
                LOGW("current_pos       =  %02x (%d)\n", current_pos, current_pos);
                LOGW("state             =  %02x (%d)\n", state, state);
                LOGW("frame_length      =  %02x (%d)\n", frame_length, frame_length);

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

                LOGW("data_len = %u ,frame_length = %u,current_pos=%u\n", data_len, frame_length, current_pos);
                for (int i = 0; i < 20; ++i) {
                    LOGW("frame_buffer[%d] = %02x \n", i, frame_buffer[i]);
                }
                LOGW("crc error 全局信息：\n");
                LOGW("data_index        =  %02x (%d)\n", data_index, data_index);
                LOGW("data_len          =  %02x (%d)\n", data_len, data_len);
                LOGW("expected_crc      =  %02x (%d)\n", expected_crc, expected_crc);
                LOGW("calculated_crc    =  %02x (%d)\n", calculated_crc, calculated_crc);
                LOGW("current_pos       =  %02x (%d)\n", current_pos, current_pos);
                LOGW("state             =  %02x (%d)\n", state, state);
                LOGW("frame_length      =  %02x (%d)\n", frame_length, frame_length);

                state = STATE_WAIT_HEADER_H;
                current_pos = 0;
                return -2;
            }
            break;
        }

        default:
            LOGE("default current_pos = %d \n", current_pos);
            state = STATE_WAIT_HEADER_H;
            current_pos = 0;
            break;
    }

    return 1;
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
    if (out_protocol_data){
        free(out_protocol_data);
        out_protocol_data = NULL;
    }

}


static void do_uart_recv_with_pull(uint8_t str) {

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
        if (strcmp((char *) shell_buff, "#exit#") == 0) {
            pthread_mutex_lock(&g_serial_mode_mutex);
            int mode = g_serial_mode;
            pthread_mutex_unlock(&g_serial_mode_mutex);
            if (mode == HD_SERIAL_NORMAL_MODE) {
                LOGI("已经取消工厂模式%d\n");
                unsigned char buf[] = {g_serial_mode};
                hd_camera_uart_write(buf, sizeof(buf));
                return;
            }
            hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
            LOGI("取消工厂模式成功！\n");
            // 回复
            const char *resp = "取消工厂模式成功!\n";

            // 复制数据（包括 '\0' 结束符）
            unsigned long size = strlen(resp) + 1;
            LOGI("发送shell命令长度=%d\n", size);
            memcpy(shell_resp_buff, resp, size);
            hd_camera_uart_write(shell_resp_buff, size);

            // 重置索引以便接收下一条命令
            do_uart_recv_str_index = 0;
        } else {
            LOGI("shell命令 shell_buff=[%s] index=%d\n", shell_buff, do_uart_recv_str_index);
            for (int i = 0; i < do_uart_recv_str_index; ++i) {
                printf("%02x\n", shell_buff[i]);
            }
            char buff[1024];
            for (int i = 0; i <= do_uart_recv_str_index; ++i) {
                buff[i] = (char) shell_buff[i];
            }
            ret = hd_camera_shell_exec(buff, (char *) shell_resp_buff);
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
        LOGD("handle_uart_data = %d", result);
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
    //hd_queue_put_uint8(g_frame_queue, str);
    hd_queue_offer_uint8(g_frame_queue, str);
//    do_uart_recv_v2(str);
}

static void do_uart_recv_take(uint8_t str) {
    static pthread_mutex_t g_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
    static uint8_t g_frame_buffer[PROTOCOL_MAX_FRAME_LEN];  // 串口帧缓冲区
    static size_t g_frame_length = 0;                     // 一个完整帧的数据长度

    int HD_PARSE_FRAME_QUEUE = 0;
    int ret = parse_serial_frame(str, g_frame_buffer, &g_frame_length); // 产生完整的包
    if (ret == 0) {
        size_t len = g_frame_length;
        hd_printf_buff(g_frame_buffer, len, "接受", 0);
        int result = handle_uart_data(g_frame_buffer, len);
        /* uint32_t len = g_frame_length;
         uint8_t *tmp = malloc(len);
 //        uint8_t tmp[PROTOCOL_MAX_FRAME_LEN];
         memcpy(tmp, g_frame_buffer, len);
         hd_printf_buff(tmp, len, "接受", 0);
         int result = handle_uart_data(tmp, len);
         free(tmp);*/
        //LOGD("handle_uart_data = %d", result);
        return;
    } else {
        // LOGW("parse_serial_frame doing = %d\n", ret);
    }

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

//<editor-fold desc="解析文件名称">
#include <string.h>
#include <stdbool.h>

/**
 * 从文件的绝对路径中解析文件名称和父目录名称
 * @param file_path 输入的文件路径（必须是绝对路径）
 * @param file_name 输出的文件名
 * @param file_parent_dir_name 输出的父目录名
 * @return 成功返回0，失败返回-1（路径无效、以'/'结尾、没有'/'或在根目录下、相对路径）
 */
/**
* 从文件的绝对路径中解析文件名称和父目录名称
* @param file_path 输入的文件路径（必须是绝对路径）
* @param file_name 输出的文件名
* @param file_parent_dir_name 输出的父目录名
* @param parent_name 输出的父目录的最后一级名称
* @return 成功返回0，失败返回-1（路径无效、以'/'结尾、没有'/'或在根目录下、相对路径）
*/
static int decode_pic_file_path(const char *file_path, char *file_name, char *file_parent_dir_name, char *parent_name) {
    // 检查输入参数是否有效
    if (file_path == NULL || file_name == NULL ||
        file_parent_dir_name == NULL || parent_name == NULL) {
        return -1;
    }

    // 检查是否是绝对路径（以'/'开头）
    if (file_path[0] != '/') {
        return -2;
    }
    int len = strlen(file_path);
    // 检查路径是否以'/'结尾
    if (len > 1 && file_path[len - 1] == '/') {
        return -3;
    }
    // 查找最后一个'/'的位置
    char *last_slash = strrchr(file_path, '/');
    if (last_slash == NULL) {
        return -4;
    }
    // 检查是否在根目录下（如"/file.txt"）
    if (last_slash == file_path) {
        return -5;
    }
    // 提取文件名（最后一个'/'后面的部分）
    strcpy(file_name, last_slash + 1);
    // 提取父目录路径（从开头到最后一个'/'之前的部分）
    strncpy(file_parent_dir_name, file_path, last_slash - file_path);
    file_parent_dir_name[last_slash - file_path] = '\0';

    // 提取父目录的最后一级名称
    // 创建一个临时缓冲区来处理父目录路径
    char temp_path[1024];
    strncpy(temp_path, file_path, last_slash - file_path);
    temp_path[last_slash - file_path] = '\0';

    // 在临时路径中查找最后一个'/'
    char *second_last_slash = strrchr(temp_path, '/');
    if (second_last_slash == NULL) {
        // 只有一级父目录（如"/dir/file.txt"）
        strcpy(parent_name, temp_path + 1);
    } else {
        // 多级目录（如"/dir1/dir2/file.txt"）
        strcpy(parent_name, second_last_slash + 1);
    }
    return 0;
}
//static int decode_pic_file_path(const char *file_path, char *file_name, char *file_parent_dir_name, char *parent_name) {
//    // 检查输入参数是否有效
//    if (file_path == NULL || file_name == NULL ||
//        file_parent_dir_name == NULL || parent_name == NULL) {
//        return -1;
//    }
//
//    // 检查是否是绝对路径（以'/'开头）
//    if (file_path[0] != '/') {
//        return -2;
//    }
//    int len = strlen(file_path);
//    // 检查路径是否以'/'结尾
//    if (len > 1 && file_path[len - 1] == '/') {
//        return -3;
//    }
//    // 查找最后一个'/'的位置
//    char *last_slash = strrchr(file_path, '/');
//    if (last_slash == NULL) {
//        return -4;
//    }
//    // 检查是否在根目录下（如"/file.txt"）
//    if (last_slash == file_path) {
//        return -5;
//    }
//    // 提取文件名（最后一个'/'后面的部分）
//    strcpy(file_name, last_slash + 1);
//    // 提取父目录路径（从开头到最后一个'/'之前的部分）
//    strncpy(file_parent_dir_name, file_path, last_slash - file_path);
//    file_parent_dir_name[last_slash - file_path] = '\0';
//    // 提取父目录的最后一级名称
//    char *prev_slash = last_slash;
//    *last_slash = '\0'; // 临时截断字符串
//    char *second_last_slash = strrchr(file_path, '/');
//    *last_slash = '/';  // 恢复字符串
//    if (second_last_slash == NULL) {
//        // 只有一级父目录（如"/dir/file.txt"）
//        strcpy(parent_name, file_path + 1);
//    } else {
//        // 多级目录（如"/dir1/dir2/file.txt"）
//        strcpy(parent_name, second_last_slash + 1);
//    }
//    return 0;
//
//}

//// 测试代码
//#include <stdio.h>
//
//void test_case(const char *path) {
//    char file_name[256] = {0};
//    char parent_dir[256] = {0};
//
//    printf("Testing: %s\n", path);
//    int ret = do_str_2_pic_file_name(path, file_name, parent_dir);
//    if (ret == 0) {
//        printf("  File name: %s\n", file_name);
//        printf("  Parent dir: %s\n", parent_dir);
//    } else {
//        printf("  Invalid path (returned %d)\n", ret);
//    }
//    printf("----------------\n");
//}
//
//int main() {
//    // 有效测试用例
//    test_case("/userdata/dir/file.txt");
//
//    // 无效测试用例
//    test_case("/");                     // 根目录
//    test_case("/file");                 // 直接在根目录下
//    test_case("relative/path/file.txt");// 相对路径
//    test_case("/path/ends/with/slash/");// 以'/'结尾
//    test_case("/no_filename/");         // 以'/'结尾且无文件名
//    test_case("");                      // 空路径
//    test_case(NULL);                    // NULL指针
//
//    return 0;
//}
//</editor-fold>

static int do_action_id_2_str(char *str, size_t str_size, uint32_t action_id_timestamps, uint8_t action_id_index) {
    snprintf(str, str_size, "%d%03d", action_id_timestamps, action_id_index);
    return 0;
}

static int do_collect_all_pic_infos_from_pic_infos(hd_dynamic_pic_info *pic_infos, uint32_t *pic_infos_size) {
    HD_PIC_INFO **caches;
    size_t caches_size = 0;
    int ret = hd_pic_infos_get_all(&caches, &caches_size);
    if (ret) {
        LOGW("hd_pic_infos_get_all error ret = %d \n", ret);
        return -1;
    }
    HD_PIC_INFO cache;
    hd_dynamic_pic_info info;
    for (int i = 0; i < caches_size; ++i) {
        cache = *caches[i];
        info.action_id_index = cache.action_id_index;
        info.action_id_timestamps = cache.action_id_timestamps;
        info.snapshot_timestamps = cache.snapshot_timestamps;
        info.size = cache.size;
        info.id = cache.id;
        memcpy(info.md5, cache.md5, sizeof(info.md5));
        pic_infos[i] = info;
    }
    *pic_infos_size = caches_size;
    if (caches != NULL) {
        HD_PIC_INFOs_free(caches, caches_size);
        caches = NULL;
    }
    return 0;
}

static int on_pic_info_removed(const HD_PIC_INFO *info) {
    if (info == NULL)return 0;
    LOGI("on_pic_info_removed %s\n", info->path);
    if (SNAP_TEST_WITH_PURE) {
        if (info->path) {
            if (access(info->path,F_OK)){
                remove(info->path);
            }
        }
    } else {
        if (g_hd_on_event != NULL) {
            void *result = g_hd_on_event(EVENT_HD_DELETE_PIC, info->path, strlen(info->path));
            if (result == NULL) {
                return 0;
            }
            if (*(int *) result == 0) {
                return 0;
            } else {
                return 1;
            }
        }
    }
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
            ret = hd_camera_ota_version(
                    property_id_out,
                    payload_data, payload_data_size, protocol_data_out, protocol_data_size_out);
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
            if (result){
                free(result);
                result = NULL;
            }
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
                        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                        if (ret) {
                            LOGD("hd_camera_uart_write error \n");
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
                    ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        LOGD("hd_camera_uart_write error \n");
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
                        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
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
                    if (tmp){
                        free(tmp);
                        tmp = NULL;
                    }
                    perror("create_directory_if_not_exists fail.\n");
                    break;
                }
                g_push_mode_file = fopen(tmp, "wb");  // 二进制写入模式
                if (!g_push_mode_file) {
                    if (tmp){
                        free(tmp);
                        tmp = NULL;
                    }
                    perror("Failed to open file");
                    break;
                }
                if (tmp){
                    free(tmp);
                    tmp = NULL;
                }
                LOGI("<<<<打开文件成功 准备接受数据>>>>\n");
                // 写入数据...
                // fwrite(data, 1, size, file);
                // fclose(file);  // 关闭文件
                LOGI("<<<<切换到接受文件模式>>>>\n");
                hd_camera_change_serial_mode(HD_SERIAL_PUSH_MODE);
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
        protocol_data_out = NULL;
    }

    return 0;
}

static u_int8_t handle_snapshot_pic(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
) {

    if (SNAP_TEST_WITH_PURE) {
        uint16_t  pic_id;
        int ret = hd_camera_produce_take_photos_actively(&pic_id);
        if (ret){
            hd_slave_snapshot_encode(protocol_data_out,protocol_data_size_out,g_addr,1,0);
        }else{
            printf("handle_snapshot_pic pic_id = %d\n",pic_id);
            hd_slave_snapshot_encode(protocol_data_out,protocol_data_size_out,g_addr,0,pic_id);
        }
        return 0;
    }


    // 拍照 超时？
    if (g_hd_on_event == NULL) {
        return -3;
    }
    void *result = g_hd_on_event(EVENT_HD_SNAPSHOT, NULL, 0);
    if (result == NULL) {
        return -4;
    }
    uint16_t *pic_id = (uint16_t *) result;
    if (pic_id == NULL) {
        return -2;
    }
    uint16_t in_result = PROTOCOL_UART_SUCCESS;
    int ret = hd_slave_snapshot_encode(protocol_data_out, protocol_data_size_out, g_addr, in_result, *pic_id);
    return ret;
}

/* 处理 3.9拉取图片（0x09）*/
static int
handle_pull_pic(const unsigned char *payload_data,
                uint32_t payload_data_size,
                unsigned char **protocol_data_out,
                uint32_t *protocol_data_size_out
) {
    // 1.解析收到的协议
    uint16_t out_pic_id;
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
    unsigned char read_data[PROTOCOL_MAX_FRAME_LEN];
    size_t offset = out_offset;
    size_t read_len = out_read_len;
    size_t real_read_len = 0;
    ret = hd_pic_infos_pull(out_pic_id, -1, -1, read_data, read_len, offset, &real_read_len, on_pic_info_removed);
    if (ret) {
        LOGW("[从机%d]读文件异常。\n", g_addr);
        return 4;
    }
    if (HD_UART_PARSER_DEBUG) {
        LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", real_read_len, real_read_len);
    }
    if (real_read_len == 0) {
        LOGW("[从机%d]读取文件大小为0。\n", g_addr);
        return 5;
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, read_data, real_read_len);
    if (ret) {
        LOGW("hd_slave_pull_pic_encode fail！\n");
        return -5;
    }
    return 0;
}

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
    // 获取所有action_id图片
    hd_dynamic_pic_info pic_infos[1024] = {0};
    uint32_t pic_infos_size;
//    ret = do_collect_all_pic_infos_v2(pic_infos, &pic_infos_size);
    ret = do_collect_all_pic_infos_from_pic_infos(pic_infos, &pic_infos_size);
    if (ret) {
        LOGW("do_collect_all_pic_infos error \n");
        return -1;
    }
    LOGI("图片数量：(%d):\n", pic_infos_size);
    if (pic_infos_size > 0) {
        if (1) {
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
    if (ret) {
        LOGW("hd_slave_pic_info_encode error %d \n", ret);
        return -1;
    }
    LOGD("hd_slave_pic_info_encode end! pic_infos_size = %d\n", pic_infos_size);
    return 0;
}

/* 处理 3.8 删除图片（0x08）*/

static int handle_delete_pic(const unsigned char *payload_data, uint32_t payload_data_size) {
    uint16_t out_pic_id = 0;
    uint32_t out_action_id_timestamp = 0;
    uint8_t out_action_id_index = 0;
    int ret = hd_slave_delete_pic_decode(&out_pic_id, &out_action_id_timestamp, &out_action_id_index, payload_data,
                                         payload_data_size);
    if (ret) {
        LOGW("hd_slave_delete_pic_decode error \n");
        return -1;
    }
    LOGD("需要删除的图片pic_id : <%d> action_id :<%d> <%d>\n", out_pic_id, out_action_id_timestamp, out_action_id_index);

    if (PIC_ID_DELETE_ALL == out_pic_id) {
        LOGW("删除所有图片 TODO\n");

        if (SNAP_TEST_WITH_PURE) {
            my_remove_directory(SNAP_PATH);
        } else {
            if (g_hd_on_event != NULL) {
                void *delete_result = g_hd_on_event(EVENT_HD_DELETE_ALL_FILE, NULL, 0);
                if (delete_result == NULL) {
                    return -4;
                }
                int delete_result_int = *((int *) delete_result);
                if (delete_result_int) {
//                    free(delete_result);
//                    delete_result = NULL;
                    return 0;
                } else {
//                    free(delete_result);
//                    delete_result = NULL;
                    return -3;
                }
            }
        }
        return -2;
    }
    ret = hd_pic_infos_delete(out_pic_id, out_action_id_timestamp, out_action_id_index, 0, on_pic_info_removed);
    return ret;
}

/* 处理 3.10 图片拉取完成（0x0A）*/
static int handlePullPicComplete(const unsigned char *payload_data, uint32_t payload_data_size) {
    // 图片拉取完成 需要做什么？
//    reset_file_buffer();
    return 0;
}

static void handle_extra_pull(unsigned char *payload_data, uint32_t payload_data_size) {
    int ret;
    int success = 0;
    unsigned char *protocol_data_out = NULL;
    uint32_t protocol_data_size_out;
    while (1) {
        LOGD("[handle_extra_pull] \n");
        ret = hd_slave_pull_decode((char *) g_pull_mode_file_path,
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
        ret = hd_md5((const char *) g_pull_mode_file_path, md5);
        if (ret) {
            LOGW("[handle_extra_pull]  hd_md5 error ret = %d\n", ret);
            success = 2;
            break;
        }
        LOGD("[handle_extra_pull] 获取文件信息 md5成功！\n");
        LOGD("[handle_extra_pull] 获取文件信息 file_size\n");
        int fd;
        fd = open((const char *) g_pull_mode_file_path, O_RDWR);
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
        ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
        if (protocol_data_out != NULL) {
            free(protocol_data_out);
            protocol_data_out = NULL;
        }

        LOGD("[handle_extra_pull] 应答完毕! \n");
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
        hd_camera_uart_write(PULL_MODE_FILE_HEADER_TAIL, sizeof(PULL_MODE_FILE_HEADER_TAIL));

        hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
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
        hd_camera_uart_write(protocol_data_out, protocol_data_size_out);

        break;
    }

    if (protocol_data_out) {
        free(protocol_data_out);
        protocol_data_out = NULL;
    }
}

static char *debug_file_1 = "/userdata/0_284d99fa41805371ed361adc43e31280_222173_666_0_51_0_1753422800_1.jpg";

static char *debug_file_2 = "/userdata/1_e99a18c428cb38d5f260853678922e03_123456_799_0_45_2_1753262446_254.jpg";

static char *debug_file_dest = "/userdata/hadlinks/1753422798";

static void my_remove_directory(const char *path) {
    if (!access(path,F_OK)){
        perror("my_remove_directory access error ");
        return;
    }
    DIR *dir = opendir(path);
    if (!dir) {
        perror("无法打开目录");
        printf("path=%s\n",path);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        char full_path[1024];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);

        struct stat statbuf;
        if (lstat(full_path, &statbuf) == -1) {
            perror("无法获取文件状态");
            continue;
        }

        if (S_ISDIR(statbuf.st_mode)) {
            my_remove_directory(full_path); // 递归删除子目录
        } else {
            if (unlink(full_path)) {
                perror("删除文件失败");
            }
        }
    }

    closedir(dir);

    if (rmdir(path)) {
        perror("删除目录失败");
    }
}

static void cp_file_function() {
    int ret;
    static uint8_t aciton_id_index = 0;
    aciton_id_index++;
    char action_id_dir_buff[1024];
    snprintf(action_id_dir_buff, sizeof(action_id_dir_buff), "%s%03d", debug_file_dest, aciton_id_index);
    LOGI("cp_file_function action_id_dir=%s \n", action_id_dir_buff);
    ret = create_directory_if_not_exists(action_id_dir_buff);
    if (ret) {
        LOGI("create_directory_if_not_exists  fail !ret = %d\n", ret);
        return;
    }
    static uint16_t pic_id_index = 0;
    pic_id_index++;
    char file_name[1024];

    if (g_addr == 1) {

        snprintf(file_name, sizeof(file_name), "%s/%s%d.jpg", action_id_dir_buff,
                 "0_284d99fa41805371ed361adc43e31280_222173_666_0_51_0_1753422800_", pic_id_index);
        LOGI("cp_file_function %s => %s \n", debug_file_1, file_name);
        ret = create_directory_if_not_exists(file_name);
        if (ret) {
            LOGI("create_directory_if_not_exists  fail !ret = %d\n", ret);
            return;
        }
        ret = copy_file(debug_file_1, file_name);
    } else {
        snprintf(file_name, sizeof(file_name), "%s/%s%d.jpg", action_id_dir_buff,
                 "1_e99a18c428cb38d5f260853678922e03_123456_799_0_45_2_1753262446_", 254);
        LOGI("cp_file_function %s => %s \n", debug_file_2, file_name);
        ret = create_directory_if_not_exists(file_name);
        if (ret) {
            LOGI("create_directory_if_not_exists  fail !ret = %d\n", ret);
            return;
        }
        ret = copy_file(debug_file_2, file_name);
    }
    LOGI("cp_file_function ret = %d\n", ret);

}

static void handle_signal(int sig) {
    if (sig == SIGUSR1) {
        cp_file_function();
    }
}

static int handle_uart_data(const unsigned char *raw, size_t raw_size) {
    if (NULL == raw || raw_size <= 0) {
        LOGW("handle_uart_data raw == NULL || raw_size == 0 \n");
        return 1;
    }
    uint8_t ret;
    uint8_t out_addr;
    ret = hd_camera_protocol_addr(&out_addr, raw, raw_size);
    if (ret) {
        LOGW("hd_camera_protocol_addr error ret=%d \n", ret);
        return 2;
    }

    if (g_addr == 0 || (g_addr != out_addr && out_addr != PROTOCOL_BROADCAST)) {
        LOGD(">从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, out_addr);
        return 3;
    }
    LOGD("------------------------------handle_uart_data------------------------%d------\n", raw_size);

    uint8_t slave_addr_out;
    uint8_t cmd_out;
    uint32_t payload_data_size_out;
    unsigned char *payload_data_out = NULL;
    ret = hd_camera_protocol_decode(raw, raw_size, &slave_addr_out, &cmd_out, &payload_data_size_out,
                                    &payload_data_out);
    if (ret) {
        LOGW("hd_camera_protocol_decode error  = %d\n", ret);
        return 4;
    }

    if (g_addr == 0 || (g_addr != slave_addr_out && slave_addr_out != PROTOCOL_BROADCAST)) {
        LOGD("从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, slave_addr_out);
        return 5;
    }

    switch (cmd_out) {
        case CMD_HD_PROPERTY_GET: {
            LOGI("[从机%d] HD查询属性（0xC2）预留\n", g_addr);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = handle_property_get(payload_data_out, payload_data_size_out, &out_protocol_data,
                                      &out_protocol_data_size);
            if (ret == 0) {
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("hd_camera_uart_write error \n");
                }
            }
            if (out_protocol_data) {
                free(out_protocol_data);
                out_protocol_data = NULL;
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
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("hd_camera_uart_write error \n");
                }
            } else {
                ret = hd_slave_snapshot_encode(&out_protocol_data, &out_protocol_data_size, g_addr, PROTOCOL_UART_FAIL,
                                               0);
                if (ret) {
                    LOGD("hd_slave_snapshot_encode error \n");
                } else {
                    ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                    if (ret) {
                        LOGD("hd_camera_uart_write error \n");
                    }
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
                out_protocol_data = NULL;
            }
            break;
        }

        case CMD_HD_PIC_INFO: {

            if (HD_UART_PARSER_DEBUG_APp)return 0;
            LOGI("[从机%d] HD查询摄像头存储的图片信息（0xC7）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out = 0;
            ret = handle_pic_infos(payload_data_out, payload_data_size_out, &protocol_data_out,
                                   &protocol_data_size_out);
            if (ret == 0) {
                ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGW("hd_camera_uart_write error \n");
                }
            } else {
                LOGW("handle_pic_infos error \n");
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
                protocol_data_out = NULL;
            }
            break;
        }

        case CMD_HD_PIC_DELETE: {
            if (HD_UART_PARSER_DEBUG_APp)return 0;
            LOGI("[从机%d] HD删除图片（0xC8））\n", g_addr);
            int delete_ret = handle_delete_pic(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size = 0;
            ret = hd_slave_delete_pic_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                             delete_ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                LOGW("hd_slave_delete_pic_encode error\n");
            } else {
                hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
            }

            if (out_protocol_data != NULL) {
                free(out_protocol_data);
                out_protocol_data = NULL;
            }
            break;
        }

        case CMD_HD_PIC_PULL: {
            if (HD_UART_PARSER_DEBUG_APp)return 0;
            LOGI("[从机%d] HD拉取图片（0xC9）\n", g_addr);
            unsigned char *protocol_data_out = NULL;
            uint32_t protocol_data_size_out;
            ret = handle_pull_pic(payload_data_out, payload_data_size_out, &protocol_data_out, &protocol_data_size_out);
            if (ret == 0) {
                ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGW("hd_camera_uart_write error \n");
                }
            } else {
                LOGW("handle_pull_pic fail ! ret = %d \n", ret);
                ret = hd_slave_pull_pic_encode(&protocol_data_out, &protocol_data_size_out, g_addr, PROTOCOL_UART_FAIL,
                                               NULL, 0);
                if (ret) {
                    LOGW("hd_slave_pull_pic_encode error %d\n", ret);
                } else {
                    ret = hd_camera_uart_write(protocol_data_out, protocol_data_size_out);
                    if (ret) {
                        LOGW("hd_camera_uart_write error \n");
                    }
                }
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
                protocol_data_out = NULL;
            }
            break;
        }

        case CMD_HD_PIC_PULL_COMPLETED: {
            if (HD_UART_PARSER_DEBUG_APp)return 0;
            LOGI("[从机%d] HD图片拉取完成（0xCA）\n", g_addr);
            ret = handlePullPicComplete(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data = NULL;
            uint32_t out_protocol_data_size;
            int result = hd_slave_pull_pic_complete_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                                           ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (result) {
                LOGW("hd_slave_pull_pic_complete_encode error\n");
            } else {
                ret = hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGW("hd_camera_uart_write error \n");
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
                out_protocol_data = NULL;
            }
            break;
        }

        case CMD_HD_BROADCAST_ACTION_ID: {
            LOGI("[从机%d] HD广播门开事件（0xCD）\n", g_addr);

            char buff[128] = {0};
            uint32_t out_action_id_timestamps;
            uint8_t out_action_id_index;
            uint8_t status;
            ret = hd_slave_action_id_decode(&status, &out_action_id_timestamps, &out_action_id_index, payload_data_out,
                                            payload_data_size_out);
            if (ret) {
                LOGD("hd_slave_action_id_decode error:%d \n", ret);
                break;
            }

            if (SNAP_TEST_WITH_PURE) {
                hd_camera_produce_on_action_id_changed(out_action_id_timestamps, out_action_id_index, status,0);
                return 0;
            }

            if (g_hd_on_action_id_changed != NULL) {
                do_action_id_2_str(buff, sizeof(buff), out_action_id_timestamps, out_action_id_index);
                LOGI("action_id = %s\n", buff);
                if (HD_UART_PARSER_DEBUG_Uart) {

                } else {
                    g_hd_on_action_id_changed(status, buff);
                }

            }
            // 广播不需要响应

            if (HD_UART_PARSER_DEBUG_Uart) {
//            // 触发复制
                if (status == 1) {
                    if (kill(getpid(), SIGUSR1)) {
                        perror("kill failed");
                        return 1;
                    }
//                cp_file_function();
                }
            }
            break;
        }

        case CMD_HD_EXTRA_PULL: {
            LOGI("[从机%d] PULL（0x%02x）\n", g_addr, CMD_HD_EXTRA_PULL);
            handle_extra_pull(payload_data_out, payload_data_size_out);
            break;
        }

        case CMD_HD_PUSH_FILE: {
            LOGI("[从机%d] HD PUSH File（0x%02x）\n", g_addr, CMD_HD_PUSH_FILE);
            hd_camera_ota_model_handle_cmd(payload_data_out, payload_data_size_out);
            break;
        }

        default:
            LOGI("[从机%d] 暂不支持的CMD:%02x\n", g_addr, cmd_out);
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
    LOGI("handle_uart_data_thread start...\n");
    while (g_running) {
        uint8_t item = hd_queue_take_uint8(g_frame_queue);
        do_uart_recv_take(item);
    }
    LOGI("handle_uart_data_thread end.\n");
    return NULL;
}

/************ handles a **************/

static void init_log() {
    //log_set_level(LOG_INFO);
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
#define HD_UART_PARSER_VERSION_INTERNAL         "0.3.4.8"                    // 库版本


int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        const char *version,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
       ,int(*transform_pic)(const char *, char *)
) {
    // 注册信号处理函数
    if (HD_UART_PARSER_DEBUG_Uart) {
        signal(SIGUSR1, handle_signal);
    }
    hd_logger_set_level(HD_LOGGER_LEVEL_DEBUG);
    init_log();
    uint32_t delay = calculate_3_5_char_time(PROTOCOL_RATE_DEFAULT, 8, 0, 1);


    if (HD_UART_PARSER_DEBUG_Uart) {
        my_remove_directory("/userdata/hadlinks");
        snprintf(g_pic_dir_path, sizeof(g_pic_dir_path), "%s", "/userdata/hadlinks");
    } else {

        if (SNAP_TEST_WITH_PURE) {
            // 测试hd_c_log.c
            snprintf(g_pic_dir_path, sizeof(g_pic_dir_path), "%s", SNAP_PATH);
            create_directory_if_not_exists(SNAP_PATH);
        } else {
            snprintf(g_pic_dir_path, sizeof(g_pic_dir_path), "%s", pic_dir_path);
        }

    }

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
    int ret = hd_camera_ota_init(g_addr);
    if (ret) {
        LOGE("hd_camera_ota_init error ret=%d\n", ret);
        return 0;
    }

    ret = hd_pic_infos_init(0);
    if (ret) {
        LOGE("hd_pic_infos_init error! %d\n", ret);
        return 0;
    }
    if (SNAP_TEST_WITH_PURE) {
        my_remove_directory(SNAP_PATH);

        ret = hd_camera_produce_init(addr, SNAP_PATH, SNAP_DEMO_CROP_PIC, hd_uart_on_pic_add, NULL);
        if (ret) {
            LOGE("hd_camera_produce_init error! %d\n", ret);
            return 0;
        }
    }

    g_frame_queue = hd_queue_create_uint8(1024 * 20);
    if (g_frame_queue == NULL) {
        LOGE("createQueue g_frame_queue error!");
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


    LOGI("hd_uart_init completed !!!\n");
    return 0;
}

void hd_uart_deinit() {
    LOGI("hd_uart_deinit\n");
    g_hd_on_action_id_changed = NULL;
    g_hd_on_event = NULL;
    g_running = 0;
    hd_camera_ota_deinit();
    if (SNAP_TEST_WITH_PURE) {
        hd_camera_produce_deinit(g_addr);
    }
    hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
    hd_camera_shell_deinit();

    if (g_frame_consume_t) {
        pthread_join(g_frame_consume_t, NULL);
    }

    free_queue(g_frame_queue);
    hd_pic_infos_deinit();

    g_frame_queue = NULL;

}

char *hd_uart_version() {
    return HD_UART_PARSER_VERSION_INTERNAL;
}

void hd_camera_change_serial_mode(HD_SERIAL_MODE mode) {
    pthread_mutex_lock(&g_serial_mode_mutex);
    g_serial_mode = mode;
    LOGI("############################################## \n", mode);
    LOGI("############# serial_mode : [%d] ############## \n", mode);
    LOGI("############################################## \n", mode);
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


static int on_action_info_removed(const HD_ACTION_ID_INFO *item) {
    if (item == NULL)return 0;
    LOGI("=====on_action_info_removed %s \n", item->path);
    LOGI("=====on_action_info_removed %s \n", item->path);
    LOGI("=====on_action_info_removed %s \n", item->path);
    if (SNAP_TEST_WITH_PURE) {
        my_remove_directory(item->path);
    } else {
        if (g_hd_on_event != NULL) {
            void *result = g_hd_on_event(EVENT_HD_DELETE_ACTION_ID, item->path, strlen(item->path));
            if (result == NULL) {
                return 0;
            }
            if (*(int *) result == 0) {
                //free(result);
                return 0;
            } else {
                // ret = 2;
                //free(result);
                return 1;
            }
        }
    }

    return 0;
}


int hd_uart_on_pic_add(char *action_id_str, char **array, int array_size) {
    if (action_id_str == NULL || array == NULL || array_size == 0) {
        LOGW("check error");
        return 1;
    }
    LOGI("+++++++++++++++++++++++\n");
    LOGI("图片资源已生成，action_id:<%s> ，size:<%d>\n", action_id_str, array_size);
    LOGI("1.生成HD_ACTION_ID_INFO\n");
    HD_ACTION_ID_INFO *new_action_id_info = malloc(sizeof(HD_ACTION_ID_INFO));
    if (new_action_id_info == NULL) {
        LOGW("HD_ACTION_ID_INFO malloc error\n");
        return 1;
    }

    new_action_id_info->path = strdup(action_id_str);
    new_action_id_info->empty = 0;

    LOGI("2.填充HD_ACTION_ID_INFO\n");
    char tmp_action_id_name_str[1024];
    char tmp_pic_name_str[1024];
    // 取出pic_name
    char s_pic_file_name[MAX_FILE_PATH_SIZE];
    char s_action_id_path[MAX_FILE_PATH_SIZE];
    char s_action_id_name[MAX_FILE_PATH_SIZE];

    int pic_total = 0;
    for (int i = 0; i < array_size; ++i) {
        LOGI("解析PATH中 ....... %d <%s>\n", i, array[i]);
        char *tmp = strdup(array[i]);
        LOGI("  +解析PATH中...[%s]\n", tmp);

        int ret = decode_pic_file_path(tmp, s_pic_file_name, s_action_id_path, s_action_id_name);

        int tmp_decode_result = 0;
        if (ret) {
            LOGI("      ->解析PATH失败！ret=%d\n", ret);
            tmp_decode_result = 1;
        } else {
            LOGI("      ->解析PATH完成！@%s @%s @%s\n", s_action_id_path, s_action_id_name, s_pic_file_name);
            if (strcmp(s_action_id_path, action_id_str) != 0) {
                LOGI("      ->解析PATH失败！action_id不一致 %s != %s\n", s_action_id_path, action_id_str);
                tmp_decode_result = 3;
            } else {
                tmp_decode_result = 0;
                LOGI("      ->解析PATH成功！文件名称=%s ，父文件名:%s\n", s_pic_file_name, s_action_id_name);
            }
        }
        if (tmp_decode_result) {
            // error
        } else {
            //LOGI("图片资源已生成完毕 x1\n");
            //<editor-fold desc="添加图片信息">
            while (1) {
                LOGI("      ->解析文件信息 ...\n");
                // 解析完action_id名称和pic名称
                uint32_t action_id_timestamps;
                uint8_t action_id_index;
                ret = do_str_2_action_id(s_action_id_name, &action_id_timestamps, &action_id_index);
                if (ret) {
                    LOGW("      ->解析action_id失败 do_str_2_action_id error : %d\n", ret);
                    break;
                }
                hd_parse_pic_infos infos;
                ret = hd_camera_protocol_parse_pic_info(s_pic_file_name, &infos);
                if (ret) {
                    LOGW("      ->解析文件名失败 hd_camera_protocol_parse_pic_info error %d\n", ret);
                    break;
                }
                HD_PIC_INFO *new_pic_info = malloc(sizeof(HD_PIC_INFO));
                if (new_pic_info == NULL) {
                    LOGW("      ->解析文件名失败 HD_PIC_INFO malloc error\n");
                    break;
                }
                new_pic_info->action_id_timestamps = action_id_timestamps;
                new_pic_info->action_id_index = action_id_index;
                //info->trigger_type = infos->trigger_type;
                //info->trigger_angel = infos->trigger_angel;
                new_pic_info->snapshot_timestamps = infos.snapshot_timestamps;
                new_pic_info->size = infos.file_size;
                new_pic_info->id = infos.pic_id;
                new_pic_info->path = tmp;
                memcpy(new_pic_info->md5, infos.file_md5, sizeof(new_pic_info->md5));
                // 添加到action_id_info
                new_action_id_info->pics[pic_total] = new_pic_info;
                // NOTE:多次赋值了
                new_action_id_info->action_id_timestamps = action_id_timestamps;
                new_action_id_info->action_id_index = action_id_index;
                pic_total++;
                LOGW("      ->解析文件名成功 ！\n");
                break; // DO NOT FORGET！！！
            }
            //LOGI("图片资源已生成完毕 x4444\n");
            //</editor-fold>
        }
    }
    // 3.检查HD_ACTION_ID_INFO
    int result = 0;
    if (pic_total != 0) {
        LOGI("图片资源已生成完毕 <%s> size:<%d> total:<%d>\n", action_id_str, array_size, pic_total);
        new_action_id_info->pic_count = pic_total;
        result = hd_pic_infos_add(new_action_id_info, on_action_info_removed);
    } else {
        LOGW("空的pic_info\n");
        if (new_action_id_info){
            free(new_action_id_info);
            new_action_id_info = NULL;
        }
        result = 2;
    }

    LOGI("+++++++++++++++++++++++\n");
    return 0;

}