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
#include <limits.h>
#include <sys/time.h>
#include "uart.h"
#include "485.h"
#include "uart_parser.h"
#include "hd_camera_protocol_property.h"

#define HD_UART_PARSER_VERSION_INTERNAL "0.1.9"
// 帧头定义
#define FRAME_HEADER_H PROTOCOL_HEADER_1
#define FRAME_HEADER_L PROTOCOL_HEADER_0

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

// 当前从机地址
static volatile uint8_t g_addr = 0;
static volatile uint32_t g_delay = 0;
// 当前照片存储目录
static const char *g_pic_dir_path = NULL;
static volatile int g_running = 0;
static pthread_t g_uart_t;
static hd_on_action_id_changed g_hd_on_action_id_changed = NULL;
static hd_on_event g_hd_on_event = NULL;

static uint8_t g_frame_buffer[PROTOCOL_MAX_FRAME_LEN];

// 待上传的文件缓冲
#define MAX_FILE_SIZE (400*1024)
static unsigned g_file_buffer[MAX_FILE_SIZE] = {0};
static unsigned g_file_buffer_size = -1;
static uint8_t g_file_pic_id = 0;
static int g_file_pulling = 0;

static uint16_t g_frame_length = 0;

static int g_write_outside = 0;

// 隐藏的实现 a
static int do_uart_write(const unsigned char *raw, size_t raw_size);

//static void do_uart_recv(uint8_t str);

static int do_parse_pic_info(const char *file_name, uint32_t *snapshot_timestamps, uint8_t *pic_id);

static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, uint16_t *frame_length);

static int do_str_2_action_id(const char *action_id_str, uint32_t *action_id_timestamps, uint8_t *action_id_index);

static int do_collect_all_pic_infos(hd_dynamic_pic_info ***pic_infos, uint8_t *pic_infos_size);

static int do_find_pic_by_pic_id(uint8_t pic_id, char **file_path);

static int do_cut_file_data(const char *file_name, unsigned char **result, size_t *size,
                            uint32_t offset,
                            uint32_t read_len
);

static int handle_uart_data(const unsigned char *raw, size_t raw_size);
// 隐藏的实现 z

// 实现

static void resetFileBuffer() {
    g_file_pic_id = 0;
    g_file_pulling = 0;
    g_file_buffer_size = -1;
    memset(g_file_buffer, 0, MAX_FILE_SIZE);
}

static int do_uart_write(const unsigned char *raw, size_t raw_size) {
    if (g_running == 0) {
        return -5;
    }
    if (raw == NULL || raw_size <= 0) {
        return -1;
    }
    if (g_write_outside) { // 由外部来实现写操作。
        if (g_hd_on_event != NULL) {
            g_hd_on_event(EVENT_DEBUG_WRITE, (void *) raw, raw_size);
        }
        return 0;
    }

    rs485_pwr_on();
    for (int i = 0; i < raw_size; ++i) {
        rk_uart_sendbyte(raw[i]);
        usleep(g_delay);
    }
    rs485_pwr_off();
    return 0;
}

// 帧解析函数
static int parse_serial_frame(uint8_t byte, uint8_t *frame_buffer, uint16_t *frame_length) {
    //LOGI("parse_serial_frame\n");
    static ParserState state = STATE_WAIT_HEADER_H;
    static uint16_t data_index = 0;
    static uint32_t data_len = 0;
    static uint16_t expected_crc = 0;
    static uint16_t calculated_crc = 0;
    static uint16_t current_pos = 0;

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
                state = STATE_WAIT_HEADER_H;
            }
            break;

        case STATE_WAIT_ADDR:
            frame_buffer[2] = byte;
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

            if (data_len > PROTOCOL_MAX_FRAME_LEN - 10) { // 10 = 帧头(2)+地址(1)+命令(1)+长度(4)+CRC(2)
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
                return 0;
            } else {
                state = STATE_WAIT_HEADER_H;
                return -2;
            }
            break;

        default:
            state = STATE_WAIT_HEADER_H;
            break;
    }

    return -1;
}

static void do_uart_recv(uint8_t str) {
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


    int ret;
    ret = parse_serial_frame(str, g_frame_buffer, &g_frame_length);
    if (ret == 0) {
        LOGI("收到完整帧.......\n");
        hd_printf_buff(g_frame_buffer, g_frame_length, "收到完整帧", 0);
        // 处理数据
        handle_uart_data(g_frame_buffer, g_frame_length);
        g_frame_length = 0;
        memset(g_frame_buffer, 0, PROTOCOL_MAX_FRAME_LEN);
    } else if (ret == -1) {

    } else if (ret == -2) {
        LOGW("CRC error \n");
    } else if (ret == -3) {
        LOGW("len error\n");
    }

}

#define MAX_TOKENS 20  // 最大分割数量
#define JPG_SUFFIX ".jpg"
#define JPG_SUFFIX_LEN 4  // ".jpg"的长度

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
        fprintf(stderr, "读取文件不完全，期望 %ld字节，实际读取 %zu字节\n",
                file_size, bytes_read);
        fclose(file);
        return -1;
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
    if (!file_name || !snapshot_timestamps || !pic_id) {
        return -1;
    }
    char cpy[256];

    snprintf(cpy, sizeof(cpy), "%s", file_name);  // 安全复制
//
//    // 查找第一个和第二个'_'
//    const char *first_underscore = strchr(file_name, '_');
//    const char *second_underscore = first_underscore ? strchr(first_underscore + 1, '_') : NULL;
//    const char *last_dot = strrchr(file_name, '.');
//
//    // 基础格式检查
//    if (!first_underscore || !second_underscore || !last_dot ||
//        second_underscore >= last_dot) {
//        return -2;
//    }
//
//    /* 解析时间戳（第一个_和第二个_之间的部分） */
//    char *timestamp_end;
//    errno = 0;
//    long timestamp = strtol(first_underscore + 1, &timestamp_end, 10);
//
//    // 检查时间戳转换结果
//    if ((errno == ERANGE && (timestamp == LONG_MAX || timestamp == LONG_MIN))) {
//        return -3; // 数值溢出
//    }
//    if (timestamp_end != second_underscore) {  // 关键修改点
//        return -4; // 时间戳未正确终止于第二个_
//    }
//    if (timestamp < 0 || timestamp > UINT32_MAX) {
//        return -5;
//    }
//
//    /* 解析图片ID（第二个_和.之间的部分） */
//    char *pic_id_end;
//    errno = 0;
//    long id = strtol(second_underscore + 1, &pic_id_end, 10);
//
//    if ((errno == ERANGE && (id == LONG_MAX || id == LONG_MIN)) ||
//        pic_id_end != last_dot ||  // 关键修改点
//        id < 0 || id > UINT8_MAX) {
//        return -6;
//    }
    char *tokens[MAX_TOKENS];
    int token_count = 0;
    token_count = parse_filename(file_name, tokens, MAX_TOKENS);
    if (token_count <= 2)return -1;
    char *snapshot_timestamps_str = tokens[token_count - 2];
    char *id_str = tokens[token_count - 1];
    *snapshot_timestamps = (uint32_t) atoi(snapshot_timestamps_str);
    *pic_id = (uint8_t) atoi(id_str);
    LOGD("==>解析结果 时间 ：<%s> \n", snapshot_timestamps_str);
    LOGD("==>解析结果 id  ：<%s> \n", id_str);
    return 0;
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
    for (size_t i = 0; i < len; i++) {
        if (!isdigit(action_id_str[i])) {
            return -1;
        }
    }

    // 时间戳部分至少需要10位（可以表示到2286年）
    if (len < 10) {
        return -1;
    }

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

static int do_collect_all_pic_infos(hd_dynamic_pic_info ***pic_infos, uint8_t *pic_infos_size) {
    /* 一、搜索目录 */
    if (g_pic_dir_path == NULL) {
        LOGD("目录未设置\n");
        return -1;
    }
    LOGD("1.搜索目录[%s]\n", g_pic_dir_path);
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        LOGD("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char *action_id_dir_names[1024] = {0}; // action_id目录名称数组
    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息
    int action_id_count = 0;
    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        LOGI("%-20s %-4d %s\n", action_id_dir_entry->d_name,
             action_id_dir_entry->d_type,
             action_id_name_temp);
        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {
            continue;
        }
        // 继续校验名称
        action_id_dir_names[action_id_count] = strdup(action_id_name_temp);
        action_id_count++;
    }
    LOGD("2.搜索目录完毕！aciont_id目录数量为：%d\n", action_id_count);


    uint8_t total = 0; // 所有的图片
    if (action_id_count == 0) {
        *pic_infos = NULL;
        *pic_infos_size = 0;
        LOGD("解析文件夹完毕！！！图片为空\n ");
    } else {
        /* 二、遍历action_id目录，搜索图片文件 */
        uint8_t MAX = 0xff; // 不超过255
        char action_id_path[1024]; // action_id目录路径temp
        size_t temp_size = sizeof(hd_dynamic_pic_info **) * MAX;
        hd_dynamic_pic_info **infos = (hd_dynamic_pic_info **) malloc(temp_size);
        memset(infos, 0, temp_size);
        LOGD("3.遍历action_id目录，搜索图片文件。\n");
        uint32_t temp_action_id_timestamps = 0; // 当前temp_action_id_timestamps
        uint8_t temp_action_id_index = 0;             // 当前temp_action_id_index
        int ret;
        for (int i = 0; i < action_id_count; ++i) {
            // 解析action_id
            LOGD("------------------------------\n");
            LOGD("<%d>解析文件夹[%s]\n ", i, action_id_dir_names[i]);
            ret = do_str_2_action_id(action_id_dir_names[i],
                                     &temp_action_id_timestamps, &temp_action_id_index);
            if (ret) {
                //LOGD("      解析action_id失败 ： %s %d\n ",action_id_dirs[i],ret);
                continue;
            }
            LOGD("解析文件夹[%s]成功! timestamps：%d ,index：%d \n", action_id_dir_names[i], temp_action_id_timestamps,
                 temp_action_id_index);
            snprintf(action_id_path, sizeof(action_id_path), "%s/%s", g_pic_dir_path, action_id_dir_names[i]);
            struct dirent *pic_file_entry;  //   图片文件信息
            char pic_file_name[1024];       //   图片名称
            DIR *action_id_dir = opendir(action_id_path);
            if (!action_id_dir) {
                LOGD("无法打开目录 %s \n", action_id_path);
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
                LOGD("开始解析图片文件:<%s> \n", pic_file_name);
                //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
                ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
                if (ret) {
                    LOGD("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                    continue;
                }


                snprintf(file_path_tmp, sizeof(file_path_tmp), "%s/%s", action_id_path, pic_file_name);
                LOGD("获取图片md5...%s\n", file_path_tmp);
                ret = hd_md5(file_path_tmp, md5_result_tmp);

                if (ret) {
                    fprintf(stderr, "hd_md5 fail.\n");
                    continue;
                }

                LOGD("获取图片大小...\n");
                struct stat st_tmp;
                if (stat(file_path_tmp, &st_tmp) != 0) {
                    fprintf(stderr, "stat fail\n");
                    return 1;//st_tmp.st_size;  // 返回文件大小（字节）
                }

                uint32_t size = st_tmp.st_size;
                LOGD("图片信息：\n");


                LOGD("name                    =          %s\n", pic_file_name);
                LOGD("id                      =          %hhu\n", pic_id_temp);
                LOGD("size                    =          %u\n", size);
                LOGD("action_id_timestamps    =          %d\n", temp_action_id_timestamps);
                LOGD("action_id_index         =          %d\n", temp_action_id_index);
                LOGD("snapshot_timestamps     =          %u\n", temp_action_id_timestamps);
                LOGD("md5                     =          ");
                for (int j = 0; j < 16; ++j) {
                    LOGD("%02x ", md5_result_tmp[j]);
                }
                LOGD("\n");

                LOGD("创建图片信息\n");
                hd_dynamic_pic_info *info = (hd_dynamic_pic_info *) malloc(sizeof(hd_dynamic_pic_info));
                info->action_id_index = temp_action_id_index;
                info->action_id_timestamps = temp_action_id_timestamps;
                info->snapshot_timestamps = snapshot_timestamps_temp;
                info->size = size;
                info->id = pic_id_temp;

                memcpy(info->md5, md5_result_tmp, sizeof(info->md5));

                infos[total] = info;
                total++;
            }


            closedir(action_id_dir);
        }

        // 赋值
        *pic_infos = infos;
        *pic_infos_size = total;

        LOGD("解析文件夹完毕！！！图片结果大小：%d\n ", total);
    }
    closedir(dir);

    LOGD("查找所有pic完毕！！！\n");
    return 0;
}

static int do_find_pic_by_pic_id(uint8_t pic_id, char **file_path) {
    /* 一、搜索目录 */
    if (g_pic_dir_path == NULL) {
        LOGD("目录未设置\n");
        return -1;
    }
    LOGD("1.搜索目录[%s]\n", g_pic_dir_path);
    // 获取所有的action_id的目录
    // 比如
    // 0    1747814636001
    // 1    1747814636002
    // 2    1747814636003
    DIR *dir = opendir(g_pic_dir_path);
    if (!dir) {
        LOGD("无法打开目录 %s \n", g_pic_dir_path);
        return -1;
    }

    char action_id_name_temp[1024]; // action_id目录名称temp
    struct dirent *action_id_dir_entry; // action_id目录文件信息
    while ((action_id_dir_entry = readdir(dir)) != NULL) {
        snprintf(action_id_name_temp, sizeof(action_id_name_temp), "%s", action_id_dir_entry->d_name);
        LOGD("%-20s %-4d %s\n", action_id_dir_entry->d_name,
             action_id_dir_entry->d_type,
             action_id_name_temp);
        // 跳过 "." 和 ".." 目录
        if (strcmp(action_id_dir_entry->d_name, ".") == 0 || strcmp(action_id_dir_entry->d_name, "..") == 0 ||
            (action_id_dir_entry->d_type != DT_DIR)) {
            continue;
        }
        // 继续校验名称
        // 分别打开文件夹查询pic_id匹配的图片
        struct dirent *pic_file_entry;  //   图片文件信息
        char pic_file_name[1024];       //   图片名称
        char result[1024];       // 图片path
        char action_id_path[1024]; // action_id目录路径temp
        int ret;
        uint32_t snapshot_timestamps_temp;
        uint8_t pic_id_temp;
        snprintf(action_id_path, sizeof(action_id_path), "%s/%s", g_pic_dir_path, action_id_name_temp);
        DIR *action_id_dir = opendir(action_id_path);
        if (!action_id_dir) {
            LOGD("无法打开目录 %s \n", action_id_path);
            continue;
        }
        while ((pic_file_entry = readdir(action_id_dir)) != NULL) {
            if (strcmp(pic_file_entry->d_name, ".") == 0 || strcmp(pic_file_entry->d_name, "..") == 0) {
                continue;
            }
            // 根据文件名称 解析pic_id
            snprintf(pic_file_name, sizeof(pic_file_name), "%s", pic_file_entry->d_name);
            LOGD("开始解析图片文件:<%s> \n", pic_file_name);
            //char * debug_pic_file_name = "xxx22222xx_1747878695_2.jpg";
            ret = do_parse_pic_info(pic_file_name, &snapshot_timestamps_temp, &pic_id_temp);
            if (ret) {
                LOGD("解析图片文件失败 error( %d) :  %s \n", ret, pic_file_name);
                continue;
            }
            if (pic_id_temp == pic_id) {
                snprintf(result, sizeof(result), "%s/%s", action_id_path, pic_file_name);
                *file_path = strdup(result);
                closedir(action_id_dir);
                closedir(dir);
                return 0;
            }
        }
        closedir(action_id_dir);
    }
    closedir(dir);
    return -2;
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
        case PROPERTY_HD_ID_DEBUG: {
            LOGI("[%d获取属性]%02x DEBUG开关\n", g_addr, PROPERTY_HD_ID_DEBUG);
            unsigned char buff[] = {hd_logger_set_level_cur()};
            ret = hd_slave_property_get_encode(protocol_data_out, protocol_data_size_out,
                                               g_addr, property_id_out, 1, buff, sizeof(buff));
            return ret;
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ACTION_ID: {
            LOGI("[%d获取属性]%02x DEBUG action_id\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_ANGEL: {
            LOGI("[%d获取属性]%02x DEBUG angel\n", g_addr, PROPERTY_HD_ID_DEBUG);
            break;
        }
        case PROPERTY_HD_ID_DEBUG_HD_UART_VERSION: {
            LOGI("[%d获取属性]%02x DEBUG hd_uart version\n", g_addr, PROPERTY_HD_ID_DEBUG);
            char *version = hd_uart_version();
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
            ret = hd_slave_property_get_encode(protocol_data_out, protocol_data_size_out,
                                               g_addr, property_id_out, 1, result, len);
            return ret;

            break;
        }

        default: {
            break;
        }
    }
    return -1;
}

static int
handle_property_set(
        const unsigned char *payload_data, uint32_t payload_data_size,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
) {
    if (g_addr == 0)return -1;
    LOGD("handle_property_set\n");
    hd_printf_buff(payload_data, payload_data_size, "event-raw", 0);
    uint8_t property_id_out;
    unsigned char *result_value_out;
    uint32_t result_value_size_out;
    uint8_t ret = hd_slave_property_set_decode(&property_id_out, &result_value_out, &result_value_size_out,
                                               payload_data, payload_data_size);
    if (ret) {
        LOGD("hd_slave_property_set_decode error\n");
        return -1;
    }
    hd_printf_buff(result_value_out, result_value_size_out, "event", 0);
    LOGD("开始处理event\n");

    switch (property_id_out) {
        case PROPERTY_HD_ID_DEBUG: {
            LOGI("[%d设置属性]%02x DEBUG开关\n", g_addr, PROPERTY_HD_ID_DEBUG);
            if (result_value_size_out == 1) {
                uint8_t on = result_value_out[0];
                if (on >= HD_LOGGER_LEVEL_DEBUG && on <= HD_LOGGER_LEVEL_ERROR) {
                    hd_logger_set_level(on);
                    LOGI("设置开关成功:%d\n", on);
                    hd_slave_property_set_encode(protocol_data_out, protocol_data_size_out,
                                                 g_addr, property_id_out, 1);
                    return 0;
                }
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
    return -1;
}

static u_int8_t
handle_snapshot_pic(
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
                     size_t in_offset, size_t *out_read_size) {
    // 参数检查
    if (in_dest == NULL) {
        fprintf(stderr, "错误：目标数组不能为NULL\n");
        return 1;
    }

    // 检查请求是否超出缓冲区范围
    if (in_offset >= g_file_buffer_size) {
        fprintf(stderr, "错误：偏移量 %zu 超出文件大小 %u\n",
                in_offset, g_file_buffer_size);
        return 1;
    }

    // 检查读取长度是否合理
    if (in_size == 0) {
        fprintf(stderr, "警告：请求读取0字节\n");
        return 1;
    }

    size_t real_read_size = in_size;
    if (in_offset + in_size > g_file_buffer_size) {
        real_read_size = in_offset + in_size - g_file_buffer_size;
    }
    *out_read_size = real_read_size;
    memcpy(in_dest, (unsigned char *) g_file_buffer + in_offset, real_read_size);
    return 0;
}


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
    uint32_t out_offset;
    uint32_t out_read_len;
    int ret = hd_slave_pull_pic_decode(&out_pic_id, &out_offset, &out_read_len, payload_data, payload_data_size);
    if (ret) {
        LOGD("hd_slave_pull_pic_decode error \n");
        return -1;
    }
    LOGD("解析到需要拉取的图片信息：\n");
    LOGD("pic_id        :       %d(0x%02x)\n", out_pic_id, out_pic_id);
    LOGD("offset        :       %d(0x%02x)\n", out_offset, out_offset);
    LOGD("read_len      :       %d(0x%02x)\n", out_read_len, out_read_len);

    int goon = 0;
    if (g_file_pulling) { // 正在上传
        if (out_pic_id == g_file_pic_id) {
            //继续
            LOGI("继续pull！\n");
            goon = 1;
        } else {
            //清除掉 重新开始
            LOGI("pic不相等 重新开始新的pull！\n");
        }
    } else { // 新的上传
        LOGI("开始新的pull\n");
    }

    if (goon) {
        //ignore
    } else {
        // 查找图片
        char *filePath;
        ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
        if (ret) {
            LOGD("拉取的图片数据信息失败\n");
            return -1;
        }
        // 加载图片到缓存
        ret = load_file_to_buffer(filePath);
        if (ret) {
            LOGD("加载图片数据失败\n");
            return -1;
        }
        g_file_pic_id = out_pic_id;
        g_file_pulling = 1;
    }

    // 加载图片，从buff offset中读 read_len 数据
    unsigned char read_data[out_read_len];
    size_t offset = out_offset;
    size_t read_len = sizeof(read_data);
    size_t real_read_len = 0;
    ret = read_from_buffer(read_data, read_len, offset, &real_read_len);
    if (ret) {
        return -1;
    }

    LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", real_read_len, real_read_len);
    if (real_read_len == 0) {
        LOGI("[从机%d]文件读完了。\n", g_addr);
        resetFileBuffer();
        return -1;
    }
    if (real_read_len < out_read_len) {
        LOGI("[从机%d]文件读到结尾了。\n", g_addr);
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, read_data, real_read_len);
    if (ret) {
        LOGD("hd_slave_pull_pic_encode fail！\n");
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    time_used = (end.tv_sec - start.tv_sec) * 1e9;  // 秒转纳秒
    time_used += (end.tv_nsec - start.tv_nsec);     // 加上纳秒部分
    time_used /= 1e6;                               // 转换为毫秒

    printf("拉取图片处理总处理时间 耗时: %f 毫秒\n", time_used);
    return 0;
}

/* 处理 3.9拉取图片（0x09）*/
static int
handle_pull_pic_v1(const unsigned char *payload_data,
                   uint32_t payload_data_size,
                   unsigned char **protocol_data_out,
                   uint32_t *protocol_data_size_out
) {
    uint8_t out_pic_id;
    uint32_t out_offset;
    uint32_t out_read_len;
    int ret = hd_slave_pull_pic_decode(&out_pic_id, &out_offset, &out_read_len, payload_data, payload_data_size);
    if (ret) {
        LOGD("hd_slave_pull_pic_decode error \n");
        return -1;
    }
    LOGD("需要拉取的图片信息：\n");
    LOGD("pic_id        :       %d(0x%02x)\n", out_pic_id, out_pic_id);
    LOGD("offset        :       %d(0x%02x)\n", out_offset, out_offset);
    LOGD("read_len      :       %d(0x%02x)\n", out_read_len, out_read_len);
    char *filePath;
    ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
    if (ret) {
        LOGD("拉取的图片数据信息失败\n");
        return -1;
    }
    LOGI("查找到的图片名称：%s\n", filePath);
    unsigned char *result;
    size_t size;

//    ret = do_cut_file_data(filePath, &result, &size, 246*1024, out_read_len);
    ret = do_cut_file_data(filePath, &result, &size, out_offset, out_read_len);
    if (ret) {
        return -1;
    }
    LOGD("需要拉取的图片数据信息：\n");
    LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", size, size);
    if (size == 0) {
        LOGI("[从机%d]文件读完了。\n", g_addr);
        return -1;
    }
    if (size < out_read_len) {
        LOGI("[从机%d]文件读到结尾了。\n", g_addr);
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 0, result, size);
    if (ret) {
        LOGD("hd_slave_pull_pic_encode fail！\n");
        return -1;
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
        LOGD("hd_slave_pic_info_decode error \n");
        return -1;
    }
    // 获取g_addr下所有action_id图片
    hd_dynamic_pic_info **pic_infos;
    uint8_t pic_infos_size;
    ret = do_collect_all_pic_infos(&pic_infos, &pic_infos_size);
    if (ret) {
        LOGD("do_collect_all_pic_infos error \n");
        return -1;
    }
    if (pic_infos_size > 0) {
        LOGD("打印搜索结果(%d):\n", pic_infos_size);
        for (int i = 0; i < pic_infos_size; ++i) {
            LOGD("******\n");
            LOGD("id                    =   %d\n", pic_infos[i]->id);
            LOGD("size                  =   %d\n", pic_infos[i]->size);
            LOGD("action_id_index       =   %d\n", pic_infos[i]->action_id_index);
            LOGD("action_id_timestamps  =   %d\n", pic_infos[i]->action_id_timestamps);
            LOGD("snapshot_timestamps   =   %d\n", pic_infos[i]->snapshot_timestamps);
            LOGD("md5                   =   ");
            for (int j = 0; j < 16; ++j) {
                LOGD("%02x ", pic_infos[i]->md5[j]);
            }
            LOGD("\n");

        }
    }

    unsigned char *protocol_data_out;
    uint32_t protocol_data_size_out;
    ret = hd_slave_pic_info_encode(&protocol_data_out, &protocol_data_size_out, g_addr, pic_infos,
                                   pic_infos_size);

    if (pic_infos) {
        free(pic_infos);
    }
    *result = protocol_data_out;
    *result_size = protocol_data_size_out;
    // AA 5A
    // 01
    // 1D
    // 79 00 00 00
    // 04
    // 01   D4 88 2D 68 03  27 83 2E 68     20 D8 03 00     0B 51 31 EA 3B 30 A7 CD 3F 7A 7B 4B E9 36 1F FF
    // 03   D4 88 2D 68 03  27 83 2E 68     20 D8 03 00     0B 51 31 EA 3B 30 A7 CD 3F 7A 7B 4B E9 36 1F FF
    // 04   EC 88 2D 68 01  27 83 2E 68     20 D8 03 00     0B 51 31 EA 3B 30 A7 CD 3F 7A 7B 4B E9 36 1F FF
    // 05   EC 88 2D 68 01  27 83 2E 68     20 D8 03 00     0B 51 31 EA 3B 30 A7 CD 3F 7A 7B 4B E9 36 1F FF
    // 98 17
    if (ret) {
        LOGD("hd_slave_pic_info_encode error \n");
        return -1;
    }

    return 0;
}

/* 处理 3.8 删除图片（0x08）*/
static int handle_delete_pic(const unsigned char *payload_data, uint32_t payload_data_size) {
    uint8_t out_pic_id;
    int ret = hd_slave_delete_pic_decode(&out_pic_id, payload_data, payload_data_size);
    if (ret) {
        LOGD("hd_slave_delete_pic_decode error \n");
        return -1;

    }
    LOGD("需要删除的图片pic_id : <%d> \n", out_pic_id);
    // 遍历文件夹依次查询图片id
    char *filePath;
    ret = do_find_pic_by_pic_id(out_pic_id, &filePath);
    if (ret != 0) {
        LOGD("do_find_pic_by_pic_id error \n");
        return -1;
    }
    LOGD("查询结果：图片地址=%s\n", filePath);

    // 删除文件
    ret = remove(filePath);
    // 检查当前文件夹是否为空 是则删除文件夹

    free(filePath);

    return ret == 0 ? 0 : -2;
}

/* 处理 3.10 图片拉取完成（0x0A）*/
static int handlePullPicComplete(const unsigned char *payload_data, uint32_t payload_data_size) {
    // 图片拉取完成 需要做什么？
    resetFileBuffer();
    return 0;
}

static int handle_uart_data(const unsigned char *raw, size_t raw_size) {
    if (NULL == raw) {
        return 0;
    }
    LOGD("------------------------------handle_uart_data------------------------------\n");
    uint8_t ret;
    uint8_t slave_addr_out;
    uint8_t cmd_out;
    uint32_t payload_data_size_out;
    unsigned char *payload_data_out;
    // 可以只先解析addr
    ret = hd_camera_protocol_decode(raw, raw_size, &slave_addr_out, &cmd_out, &payload_data_size_out,
                                    &payload_data_out);
    if (ret) {
        LOGD("hd_camera_protocol_decode error \n");
        return 0;
    }
    LOGD("slave_addr         :           %d \n", slave_addr_out);
    LOGD("cmd_out            :           %02x \n", cmd_out);
    if (g_addr == 0 || g_addr != slave_addr_out) {
        LOGD("从机地址错误。当前地址：%d , 接收到的数据地址:%d \n", g_addr, slave_addr_out);
        return -1;
    }

    char buff[2048];
    memset(buff, 0, sizeof(buff));

    switch (cmd_out) {

        case CMD_HD_PROPERTY_GET: {
            LOGI("[从机%d] HD查询属性（0xC2）预留\n", g_addr);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            ret = handle_property_get(payload_data_out, payload_data_size_out, &out_protocol_data,
                                      &out_protocol_data_size);
            if (ret == 0) {
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("do_uart_write error \n");
                    break;
                }
            }
            break;
        }

        case CMD_HD_PROPERTY_SET: {
            LOGI("HD设置属性（0xC3）预留\n", g_addr);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            ret = handle_property_set(payload_data_out, payload_data_size_out, &out_protocol_data,
                                      &out_protocol_data_size);
            if (ret == 0) {
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_CAMERA_SNAPSHOT: {
            LOGI("[从机%d] HD主动抓图（0xC6）\n", g_addr);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
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
                }
                ret = do_uart_write(out_protocol_data, out_protocol_data_size);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            }
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_INFO: {
            LOGI("[从机%d] HD查询摄像头存储的图片信息（0xC7）\n", g_addr);
            unsigned char *protocol_data_out;
            uint32_t protocol_data_size_out;
            ret = handle_pic_infos(payload_data_out, payload_data_size_out, &protocol_data_out,
                                   &protocol_data_size_out);
            LOGI("查询图片信息结果：");
            hd_printf_buff(protocol_data_out, protocol_data_size_out, "-", 0);
            if (ret == 0) {
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            }
            if (protocol_data_out != NULL) {
                free(protocol_data_out);
            }
            break;
        }

        case CMD_HD_PIC_DELETE: {
            LOGI("[从机%d] HD删除图片（0xC8））\n", g_addr);
            int delete_ret = handle_delete_pic(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            ret = hd_slave_delete_pic_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                             delete_ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                LOGD("hd_slave_delete_pic_encode error\n");
            }
            do_uart_write(out_protocol_data, out_protocol_data_size);
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_PIC_PULL: {
            LOGI("[从机%d] HD拉取图片（0xC9）\n", g_addr);
            unsigned char *protocol_data_out;
            uint32_t protocol_data_size_out;
            ret = handle_pull_pic(payload_data_out, payload_data_size_out, &protocol_data_out, &protocol_data_size_out);
            if (ret == 0) {
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGD("do_uart_write error \n");
                }
            } else {
                ret = hd_slave_pull_pic_encode(&protocol_data_out, &protocol_data_size_out, g_addr, PROTOCOL_UART_FAIL,
                                               NULL, 0);
                if (ret) {
                    break;
                }
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGD("do_uart_write error \n");
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
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            int result = hd_slave_pull_pic_complete_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                                           ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (result) {
                LOGD("hd_slave_pull_pic_complete_encode error\n");
            }
            do_uart_write(out_protocol_data, out_protocol_data_size);
            if (out_protocol_data != NULL) {
                free(out_protocol_data);
            }
            break;
        }

        case CMD_HD_BROADCAST_ACTION_ID: {
            LOGI("[从机%d] HD广播门开事件（0xCD）\n", g_addr);
            uint32_t out_action_id_timestamps;
            uint8_t out_action_id_index;
            ret = hd_slave_action_id_decode(&out_action_id_timestamps, &out_action_id_index, payload_data_out,
                                            payload_data_size_out);
            if (ret) {
                LOGD("hd_slave_action_id_decode error \n");
                break;
            }
            if (g_hd_on_action_id_changed != NULL) {

                do_action_id_2_str(buff, sizeof(buff), out_action_id_timestamps, out_action_id_index);
                LOGI("action_id = %s\n", buff);
                g_hd_on_action_id_changed(buff);
            }
            // 广播不需要响应
            break;
        }

        default:
            LOGI("[从机%d] 暂不支持的CMD:%d\n", g_addr, cmd_out);
            break;

    }
    // free(payload_data_out); // 不需要free 因为没有用malloc

    return 0;
}

/************ handles a **************/

static void *work_thread(void *arg) {
    while (g_running) {
        sleep(2);
    }
    LOGI("hd_uart_work_over...\n");
    return NULL;
}

int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
) {
    hd_logger_set_level(HD_LOGGER_LEVEL_DEBUG);
    LOGI("do_uart_write...\n");
    uint32_t delay = calculate_3_5_char_time(PROTOCOL_RATE_DEFAULT, 8, 0, 1);
    LOGI("calculate_3_5_char_time = %d\n", delay);
    LOGI(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    LOGI("hd_uart_init\n");
    LOGI("hd_uart_init version              :       <%s>   \n", hd_uart_version());
    LOGI("hd_uart_init protocol_version     :       <%s>   \n", PROTOCOL_VERSION);
    LOGI("hd_uart_init addr                 :       <%d> \n", addr);
    LOGI("hd_uart_init pic_dir_path         :       <%s> \n", pic_dir_path);
    LOGI("hd_uart_init delay                :       <%d> \n", delay);
    LOGI(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    g_pic_dir_path = pic_dir_path;
    g_addr = addr;
    g_hd_on_action_id_changed = on_action_id_changed;
    g_hd_on_event = on_event;
    g_delay = delay;
    g_running = 1;
    pthread_create(&g_uart_t, NULL, work_thread, NULL);
    LOGI("hd_uart_init completed !!!\n");
    return 0;
}

void hd_uart_recv(uint8_t byte) {
    //LOGI("|%02x", byte);
    if (g_running == 0)return;
    do_uart_recv(byte);
}

void hd_uart_deinit() {
    LOGI("hd_uart_deinit\n");
    g_hd_on_action_id_changed = NULL;
    g_hd_on_event = NULL;
    g_running = 0;
    pthread_join(g_uart_t, NULL);
}

char *hd_uart_version() {
    return HD_UART_PARSER_VERSION_INTERNAL;
}

void hd_uart_debug_recv(const unsigned char *raw, size_t raw_size) {
    handle_uart_data(raw, raw_size);
}

void hd_uart_debug_write_self(int write_outside) {
    g_write_outside = write_outside;
}
