#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include "hd_utils.h"

// 属性ID TODO
static struct {

    uint8_t id;
    /**
     * 数据类型：
     * 0:uint8_t
     * 1:string
     * 2:uint32_t
     * 3:struct
     */
    uint8_t type;
    /**
     * 数据长度
     * -1 : n
     */
    uint8_t len;
    /**
     * 读写
     * 0b11
     * bit0:读
     * bit1:写
     */
    uint8_t rw;
} HDCameraProperty;

static uint8_t hd_host_property_set_push_pull_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    if (out_payload == NULL || out_payload_size == NULL) {
        return 1;
    }
    uint32_t size = 16 + sizeof(in_file_size) + strlen(in_file_path) + 1;
    unsigned char *payload = (unsigned char *) malloc(size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, size);
    int pos = 0;
    for (int i = 0; i < 16; ++i) {
        payload[i] = in_file_md5[i];
        pos++;
    }
    for (int i = 0; i < 8; i++) {
        payload[pos + i] = (in_file_size >> (i * 8)) & 0xFF;
    }
    pos += 8;
    for (int i = 0; i < strlen(in_file_path); ++i) {
        payload[pos++] = in_file_path[i];
    }
    payload[pos] = '\0';
    *out_payload_size = size;
    *out_payload = payload;
    return 0;
}

uint8_t hd_slave_property_set_push_pull_decode(
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    if (out_file_size == NULL || out_file_md5 == NULL || out_file_path == NULL) {
        return 1;
    }

    if (in_payload == NULL || in_payload_size <= 16 + 8) {
        return 2;
    }
    int pos = 0;
    LOGD("解析file_md5 pos=%d\n", pos);
    for (int i = 0; i < 16; ++i) {
        out_file_md5[i] = in_payload[i];
        pos++;
    }
    LOGD("解析file_size pos=%d\n", pos);
    *out_file_size = (uint64_t) in_payload[pos] |
                     (uint64_t) in_payload[pos + 1] << 8 |
                     (uint64_t) in_payload[pos + 2] << 16 |
                     (uint64_t) in_payload[pos + 3] << 24 |
                     (uint64_t) in_payload[pos + 4] << 32 |
                     (uint64_t) in_payload[pos + 5] << 40 |
                     (uint64_t) in_payload[pos + 6] << 48 |
                     (uint64_t) in_payload[pos + 7] << 56;
    pos += 8;

    LOGD("解析file_path pos=%d\n", pos);
    size_t file_path_size = in_payload_size - 16 - 8;
    LOGD("解析file_path size=%zu\n", file_path_size);
    for (int i = 0; i < file_path_size; ++i) {
        out_file_path[i] = in_payload[pos + i];
    }
    LOGD("md5 = [");
    for (int i = 0; i < 16; ++i) {
        LOGD("%02x ", out_file_md5[i]);
        LOGD("]\n");
        LOGD("out_file_size = %llx\n", *out_file_size);
        LOGD("out_file_path = %s\n", out_file_path);
    }
    return 0;

}


// 开启push模式
uint8_t hd_host_property_set_push_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    return hd_host_property_set_push_pull_encode(out_payload,out_payload_size,in_file_md5,in_file_size,in_file_path);
}

// 解析开启push模式
uint8_t hd_slave_property_set_push_decode(
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    return hd_slave_property_set_push_pull_decode(out_file_md5,out_file_size,out_file_path,in_payload,in_payload_size);
}



uint8_t hd_host_property_set_push_encode_ext(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const char *src_file_path,
        const char *dest_file_path
) {
    // 1。打开文件
    int fd;
    fd = open(src_file_path, O_RDWR);
    if (fd == -1) {
        LOGD("打开文件失败:%s 原因：%d->%s \n ", src_file_path, errno, strerror(errno));
        return 11;
    }
    // 2。获取文件长度
    // 获取文件长度
    struct stat file_stat;
    if (fstat(fd, &file_stat) == -1) {
        LOGD("获取文件大小失败。fd:%d\n", fd);
        close(fd);
        return 12;
    }
    off_t file_size = file_stat.st_size;
    LOGD("file_size   :  < %lld >bytes\n", file_size);
    // 3。获取文件md5
    unsigned char md5[16];
    int ret = hd_md5(src_file_path, md5);
    if (ret) {
        LOGD("获取文件md5失败。fd:%d\n", fd);
        close(fd);
        return 13;
    }
    LOGD("file_md5    :  ");
    for (int i = 0; i < sizeof(md5); ++i) {
        LOGD("%02x ", md5[i]);
    }
    LOGD("\n");
    return hd_host_property_set_push_encode(
            out_payload, out_payload_size, md5, file_size, dest_file_path
    );
}
