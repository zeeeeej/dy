
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "hd_utils.h"
#include <time.h>
#include <dirent.h>  // 目录操作头文件
#include <string.h>  // 用于 strcmp
#include <sys/stat.h>
#include <errno.h>

static HDLoggerLevel g_HDLoggerLevel = HD_LOGGER_LEVEL_INFO;

// 设置日志级别
void hd_logger_set_level(HDLoggerLevel level) {
    g_HDLoggerLevel = level;
}

HDLoggerLevel hd_logger_set_level_cur() {
    return g_HDLoggerLevel;
}

// 日志打印函数
void hd_logger_print(HDLoggerLevel level, const char *tag, const char *msg, ...) {
    if (level < g_HDLoggerLevel) {
        return;
    }
    if (tag != NULL) {
        printf("[%s]", tag);
    }

    va_list args;
    va_start(args, msg);     // 初始化 args，指向 msg 之后的参数
    vprintf(msg, args);     // 使用 vprintf 打印格式化字符串和可变参数
    va_end(args);           // 清理 args
}

/**
 * 计算3.5个字符时间（单位：微秒）
 */
uint32_t calculate_3_5_char_time(uint32_t baud_rate, uint8_t data_bits, uint8_t parity, uint8_t stop_bits) {
    uint8_t bits_per_char = 1 + data_bits + (parity != 0 ? 1 : 0) + stop_bits;
    float char_time_us = (float) bits_per_char * 1000000.0 / (float) baud_rate;
    return (uint32_t) (3.5f * char_time_us);
}

/**
 * 使用系统调用system()生成文件的md5
 */
int hd_md5(const char *file_path, unsigned char result[16]) {
    if (file_path == NULL || result == NULL) {
        return -1; // 参数错误
    }

    // 构造命令字符串
    char command[256];
    snprintf(command, sizeof(command), "md5sum %s", file_path);

    // 执行命令并获取输出
    FILE *pipe = popen(command, "r");
    if (pipe == NULL) {
        return -2; // 命令执行失败
    }

    // 读取命令输出
    char output[64];
    if (fgets(output, sizeof(output), pipe) == NULL) {
        pclose(pipe);
        return -3; // 读取输出失败
    }

    pclose(pipe);

    // 解析 MD5 哈希值
    if (sscanf(output, "%32s", output) != 1) {
        return -4; // 解析失败
    }

    // 将十六进制字符串转换为字节数组
    for (int i = 0; i < 16; i++) {
        char hex[3] = {output[2 * i], output[2 * i + 1], '\0'};
        result[i] = (unsigned char) strtol(hex, NULL, 16);
    }

    return 0; // 成功
}


void hd_printf_buff(const unsigned char *buf, size_t buf_size, const char *tag, int full) {

//    printf("打印开始<%s> \n", tag);
    size_t size = buf_size;
    if (buf_size > 64) {
        size = 64;
    }

    if (full) {
        printf("size : %zu\n", size);

        for (int i = 0; i < size; ++i) {
            printf("[%-3d]%02x \n", i, buf[i]);
        }
    }
    if (full) {
        printf("[%s][i]", tag);
        for (int i = 0; i < size; ++i) {
            if (i > 0xff) {
                printf("%-1s%04x", "", i);
            } else {
                printf("%-1s%02x", "", i);
            }

        }
    }
    if (full) {
        printf("\n");
    }
    printf("[%s][%zu]", tag, size);
    for (int i = 0; i < size; ++i) {
//        if (i > 0xff) {
//            printf("%-1s%04x", "", buf[i]);
//        } else {
        printf("%-1s%02x", "", buf[i]);
//        }
    }
    printf("\n");
    if (full) {
        printf("打印结束<%s> \n", tag);
    }
    //printf("\n");

}

// 毫秒级睡眠函数
void hd_sleep_ms(uint32_t milliseconds) {

    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);  // POSIX API
}

int hd_array_cmp(const unsigned char *a1, size_t len1,
                 const unsigned char *a2, size_t len2) {
    // 检查指针有效性
    if (a1 == NULL || a2 == NULL) {
        return 1; // NULL指针与任何数组都不相等
    }

    // 首先比较长度
    if (len1 != len2) {
        return 2; // 长度不同直接返回不相等
    }

    // 逐个字节比较内容
    for (size_t i = 0; i < len1; i++) {
        if (a1[i] != a2[i]) {
            return 3; // 发现不相等字节
        }
    }

    return 0; // 长度和内容都相等
}

static void extract_filename(const char *full_name, char *name_only) {
    const char *dot = strrchr(full_name, '.');  // 查找最后一个 '.' 的位置
    if (dot != NULL) {
        size_t length = dot - full_name;  // 计算主文件名长度
        strncpy(name_only, full_name, length);  // 复制到新缓冲区
        name_only[length] = '\0';  // 添加字符串结束符
    } else {
        strcpy(name_only, full_name);  // 如果没有扩展名，直接复制
    }
}

int hd_find_model_name(const char *dir_path, char *model_version, const char *prefix) {
    LOGI("%s %s %s", dir_path, model_version, prefix);
    DIR *dir = opendir(dir_path);
    if (!dir) {
        perror("opendir failed");
        return 1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // 跳过 "." 和 ".." 目录
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (strstr(entry->d_name, prefix) != NULL) {
            extract_filename(entry->d_name, model_version);
            return 0;
        }
    }

    closedir(dir);  // 关闭目录
    return 2;
}

// 创建目录（如果不存在）
int create_directory_if_not_exists(const char *path) {
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

 int delete_file_if_exists(const char *filename) {
    if (remove(filename) == 0) {
        printf("文件 %s 已删除\n", filename);
        return 0; // 成功
    } else {
        if (errno == ENOENT) {
            printf("文件 %s 不存在\n", filename);
        } else {
            perror("删除文件失败");
        }
        return -1; // 失败
    }
}

