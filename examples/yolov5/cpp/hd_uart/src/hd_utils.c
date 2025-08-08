
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "hd_utils.h"

#include <time.h>
#include <dirent.h>  // 目录操作头文件
#include <string.h>  // 用于 strcmp
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>


//<editor-fold desc="hd_md5">
#include <stdint.h>

// 左循环移位函数
#define LEFTROTATE(x, c) (((x) << (c)) | ((x) >> (32 - (c))))

// MD5算法的四个基本函数
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))

// MD5的每一轮操作
#define MD5_ROUND1(a, b, c, d, x, s, ac) { \
    (a) += F((b), (c), (d)) + (x) + (uint32_t)(ac); \
    (a) = LEFTROTATE((a), (s)); \
    (a) += (b); \
}

#define MD5_ROUND2(a, b, c, d, x, s, ac) { \
    (a) += G((b), (c), (d)) + (x) + (uint32_t)(ac); \
    (a) = LEFTROTATE((a), (s)); \
    (a) += (b); \
}

#define MD5_ROUND3(a, b, c, d, x, s, ac) { \
    (a) += H((b), (c), (d)) + (x) + (uint32_t)(ac); \
    (a) = LEFTROTATE((a), (s)); \
    (a) += (b); \
}

#define MD5_ROUND4(a, b, c, d, x, s, ac) { \
    (a) += I((b), (c), (d)) + (x) + (uint32_t)(ac); \
    (a) = LEFTROTATE((a), (s)); \
    (a) += (b); \
}

// MD5上下文结构
typedef struct {
    uint32_t state[4];    // 状态 (ABCD)
    uint32_t count[2];    // 位数计数器，低32位在前
    uint8_t buffer[64];   // 输入缓冲区
} HD_MD5_CTX;

// MD5填充字节
static const uint8_t PADDING[64] = {
        0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// 辅助函数：将字节数组编码为32位整数数组
static void Encode(uint8_t *output, const uint32_t *input, size_t len) {
    size_t i, j;

    for (i = 0, j = 0; j < len; i++, j += 4) {
        output[j] = (uint8_t) (input[i] & 0xff);
        output[j + 1] = (uint8_t) ((input[i] >> 8) & 0xff);
        output[j + 2] = (uint8_t) ((input[i] >> 16) & 0xff);
        output[j + 3] = (uint8_t) ((input[i] >> 24) & 0xff);
    }
}

// 辅助函数：将字节数组解码为32位整数数组
static void Decode(uint32_t *output, const uint8_t *input, size_t len) {
    size_t i, j;

    for (i = 0, j = 0; j < len; i++, j += 4)
        output[i] = ((uint32_t) input[j]) | (((uint32_t) input[j + 1]) << 8) |
                    (((uint32_t) input[j + 2]) << 16) | (((uint32_t) input[j + 3]) << 24);
}

// MD5核心变换
static void MD5_Transform(uint32_t state[4], const uint8_t block[64]) {
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3], x[16];

    Decode(x, block, 64);

    // 第1轮
    MD5_ROUND1(a, b, c, d, x[0], 7, 0xd76aa478);
    MD5_ROUND1(d, a, b, c, x[1], 12, 0xe8c7b756);
    MD5_ROUND1(c, d, a, b, x[2], 17, 0x242070db);
    MD5_ROUND1(b, c, d, a, x[3], 22, 0xc1bdceee);
    MD5_ROUND1(a, b, c, d, x[4], 7, 0xf57c0faf);
    MD5_ROUND1(d, a, b, c, x[5], 12, 0x4787c62a);
    MD5_ROUND1(c, d, a, b, x[6], 17, 0xa8304613);
    MD5_ROUND1(b, c, d, a, x[7], 22, 0xfd469501);
    MD5_ROUND1(a, b, c, d, x[8], 7, 0x698098d8);
    MD5_ROUND1(d, a, b, c, x[9], 12, 0x8b44f7af);
    MD5_ROUND1(c, d, a, b, x[10], 17, 0xffff5bb1);
    MD5_ROUND1(b, c, d, a, x[11], 22, 0x895cd7be);
    MD5_ROUND1(a, b, c, d, x[12], 7, 0x6b901122);
    MD5_ROUND1(d, a, b, c, x[13], 12, 0xfd987193);
    MD5_ROUND1(c, d, a, b, x[14], 17, 0xa679438e);
    MD5_ROUND1(b, c, d, a, x[15], 22, 0x49b40821);

    // 第2轮
    MD5_ROUND2(a, b, c, d, x[1], 5, 0xf61e2562);
    MD5_ROUND2(d, a, b, c, x[6], 9, 0xc040b340);
    MD5_ROUND2(c, d, a, b, x[11], 14, 0x265e5a51);
    MD5_ROUND2(b, c, d, a, x[0], 20, 0xe9b6c7aa);
    MD5_ROUND2(a, b, c, d, x[5], 5, 0xd62f105d);
    MD5_ROUND2(d, a, b, c, x[10], 9, 0x02441453);
    MD5_ROUND2(c, d, a, b, x[15], 14, 0xd8a1e681);
    MD5_ROUND2(b, c, d, a, x[4], 20, 0xe7d3fbc8);
    MD5_ROUND2(a, b, c, d, x[9], 5, 0x21e1cde6);
    MD5_ROUND2(d, a, b, c, x[14], 9, 0xc33707d6);
    MD5_ROUND2(c, d, a, b, x[3], 14, 0xf4d50d87);
    MD5_ROUND2(b, c, d, a, x[8], 20, 0x455a14ed);
    MD5_ROUND2(a, b, c, d, x[13], 5, 0xa9e3e905);
    MD5_ROUND2(d, a, b, c, x[2], 9, 0xfcefa3f8);
    MD5_ROUND2(c, d, a, b, x[7], 14, 0x676f02d9);
    MD5_ROUND2(b, c, d, a, x[12], 20, 0x8d2a4c8a);

    // 第3轮
    MD5_ROUND3(a, b, c, d, x[5], 4, 0xfffa3942);
    MD5_ROUND3(d, a, b, c, x[8], 11, 0x8771f681);
    MD5_ROUND3(c, d, a, b, x[11], 16, 0x6d9d6122);
    MD5_ROUND3(b, c, d, a, x[14], 23, 0xfde5380c);
    MD5_ROUND3(a, b, c, d, x[1], 4, 0xa4beea44);
    MD5_ROUND3(d, a, b, c, x[4], 11, 0x4bdecfa9);
    MD5_ROUND3(c, d, a, b, x[7], 16, 0xf6bb4b60);
    MD5_ROUND3(b, c, d, a, x[10], 23, 0xbebfbc70);
    MD5_ROUND3(a, b, c, d, x[13], 4, 0x289b7ec6);
    MD5_ROUND3(d, a, b, c, x[0], 11, 0xeaa127fa);
    MD5_ROUND3(c, d, a, b, x[3], 16, 0xd4ef3085);
    MD5_ROUND3(b, c, d, a, x[6], 23, 0x04881d05);
    MD5_ROUND3(a, b, c, d, x[9], 4, 0xd9d4d039);
    MD5_ROUND3(d, a, b, c, x[12], 11, 0xe6db99e5);
    MD5_ROUND3(c, d, a, b, x[15], 16, 0x1fa27cf8);
    MD5_ROUND3(b, c, d, a, x[2], 23, 0xc4ac5665);

    // 第4轮
    MD5_ROUND4(a, b, c, d, x[0], 6, 0xf4292244);
    MD5_ROUND4(d, a, b, c, x[7], 10, 0x432aff97);
    MD5_ROUND4(c, d, a, b, x[14], 15, 0xab9423a7);
    MD5_ROUND4(b, c, d, a, x[5], 21, 0xfc93a039);
    MD5_ROUND4(a, b, c, d, x[12], 6, 0x655b59c3);
    MD5_ROUND4(d, a, b, c, x[3], 10, 0x8f0ccc92);
    MD5_ROUND4(c, d, a, b, x[10], 15, 0xffeff47d);
    MD5_ROUND4(b, c, d, a, x[1], 21, 0x85845dd1);
    MD5_ROUND4(a, b, c, d, x[8], 6, 0x6fa87e4f);
    MD5_ROUND4(d, a, b, c, x[15], 10, 0xfe2ce6e0);
    MD5_ROUND4(c, d, a, b, x[6], 15, 0xa3014314);
    MD5_ROUND4(b, c, d, a, x[13], 21, 0x4e0811a1);
    MD5_ROUND4(a, b, c, d, x[4], 6, 0xf7537e82);
    MD5_ROUND4(d, a, b, c, x[11], 10, 0xbd3af235);
    MD5_ROUND4(c, d, a, b, x[2], 15, 0x2ad7d2bb);
    MD5_ROUND4(b, c, d, a, x[9], 21, 0xeb86d391);

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;

    // 清零敏感信息
    memset(x, 0, sizeof(x));
}

// 初始化MD5上下文
void hd_MD5_Init(HD_MD5_CTX *context) {
    context->count[0] = context->count[1] = 0;

    // 初始化状态
    context->state[0] = 0x67452301;
    context->state[1] = 0xEFCDAB89;
    context->state[2] = 0x98BADCFE;
    context->state[3] = 0x10325476;
}

// 对输入数据进行MD5处理
void hd_MD5_Update(HD_MD5_CTX *context, const uint8_t *input, size_t inputLen) {
    size_t i, index, partLen;

    // 计算当前buffer中有多少字节
    index = (size_t) ((context->count[0] >> 3) & 0x3F);

    // 更新位数计数器
    if ((context->count[0] += ((uint32_t) inputLen << 3)) < ((uint32_t) inputLen << 3))
        context->count[1]++;
    context->count[1] += ((uint32_t) inputLen >> 29);

    partLen = 64 - index;

    // 如果输入数据足够填满buffer
    if (inputLen >= partLen) {
        memcpy(&context->buffer[index], input, partLen);
        MD5_Transform(context->state, context->buffer);

        for (i = partLen; i + 63 < inputLen; i += 64)
            MD5_Transform(context->state, &input[i]);

        index = 0;
    } else {
        i = 0;
    }

    // 将剩余数据存入buffer
    memcpy(&context->buffer[index], &input[i], inputLen - i);
}

// 完成MD5计算，输出结果
void hd_MD5_Final(HD_MD5_CTX *context, uint8_t digest[16]) {
    uint8_t bits[8];
    size_t index, padLen;

    // 保存位数
    Encode(bits, context->count, 8);

    // 填充到448位（模512）
    index = (size_t) ((context->count[0] >> 3) & 0x3f);
    padLen = (index < 56) ? (56 - index) : (120 - index);
    hd_MD5_Update(context, PADDING, padLen);

    // 附加长度
    hd_MD5_Update(context, bits, 8);

    // 存储状态到digest
    Encode(digest, context->state, 16);

    // 清零敏感信息
    memset(context, 0, sizeof(*context));
}

//// 打印MD5结果
//void print_md5(uint8_t digest[16]) {
//    for (int i = 0; i < 16; i++)
//        printf("%02x", digest[i]);
//    printf("\n");
//}

//int main() {
//    char *test = "Hello, world!";
//    uint8_t digest[16];
//
//    MD5((uint8_t *) test, strlen(test), digest);
//
//    printf("MD5 (\"%s\") = ", test);
//    print_md5(digest);
//
//    return 0;
//}

//</editor-fold>

//<editor-fold desc="hd_log">
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
//</editor-fold>

//<editor-fold desc="other">
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
static int hd_md5_str(const char *file_path, unsigned char result[16]) {
    if (file_path == NULL)return -1;
    size_t size = strlen(file_path);
    printf("size = %zu\n",size);
    if (size==0)return -1;
    unsigned char buff[1024];
    for (int i = 0; i < size; ++i) {
        buff[i] = file_path[i];
    }
    HD_MD5_CTX context;
    hd_MD5_Init(&context);
    hd_MD5_Update(&context, buff, size);
    hd_MD5_Final(&context, result);
    return 0;
}

int hd_md5_file(const char *file_name, uint8_t *result){
    FILE *file = NULL ;
    if ((file = fopen(file_name, "rb")) == NULL) {
        perror("fopen");
        return -1;
    }
    unsigned char *input_buffer = malloc(1024);
    size_t input_size = 0;

    HD_MD5_CTX ctx;
    hd_MD5_Init(&ctx);

    while((input_size = fread(input_buffer, 1, 1024, file)) > 0){
        hd_MD5_Update(&ctx, (uint8_t *)input_buffer, input_size);
    }

    hd_MD5_Final(&ctx,result);
    return 0;
}

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
//</editor-fold>

//<editor-fold desc="hd_file">
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

void hd_delete_directory(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
        perror("opendir failed");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue; // 跳过 . 和 ..
        }

        char full_path[1024];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);

        if (entry->d_type == DT_DIR) {
            // 递归删除子目录
            hd_delete_directory(full_path);
        } else {
            // 删除文件
            if (unlink(full_path)) {
                perror("unlink failed");
            }
        }
    }

    closedir(dir);

    // 最后删除空目录
    if (rmdir(path)) {
        perror("rmdir failed");
    }
}
//</editor-fold>



