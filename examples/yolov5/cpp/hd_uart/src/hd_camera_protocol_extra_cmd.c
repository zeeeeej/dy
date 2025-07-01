#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <printf.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include "hd_utils.h"
#include "hd_camera_protocol_extra_cmd.h"

static uint8_t hd_host_push_pull_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    if (out_payload == NULL || out_payload_size == NULL) {
        return 1;
    }
    int pos = 0;
    if (in_result == 1) {
        uint32_t size = 1 + strlen(in_file_path) + 1;
        unsigned char *payload = (unsigned char *) malloc(size);
        memset(payload, 0, size);
        payload[0] = in_result;
        pos++;
        for (int i = 0; i < strlen(in_file_path); ++i) {
            payload[pos++] = in_file_path[i];
        }
        payload[pos] = '\0';
        *out_payload = payload;
        *out_payload_size = size;
        return 0;
    }
    uint32_t size = 1 + 16 + sizeof(in_file_size) + strlen(in_file_path) + 1;
//    printf("size = %d\n", size);
//    printf("len  = %lu\n", strlen(in_file_path));
    unsigned char *payload = (unsigned char *) malloc(size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, size);

    payload[pos] = in_result;
    pos++;
//    printf("组装file_md5.pos = %d\n", pos);
    for (int i = 0; i < 16; ++i) {
        payload[pos] = in_file_md5[i];
        pos++;
    }
//    printf("组装file_size.pos = %d ,file_size = %llx\n", pos, in_file_size);
    for (int i = 0; i < 8; i++) {
        payload[pos + i] = (in_file_size >> (i * 8)) & 0xFF;
    }
    pos += 8;
//    printf("组装file_name.pos = %d\n", pos);
    for (int i = 0; i < strlen(in_file_path); ++i) {
        payload[pos++] = in_file_path[i];
    }
    payload[pos] = '\0';
    *out_payload_size = size;
    *out_payload = payload;
    return 0;
}

uint8_t hd_slave_push_pull_decode(
        uint8_t *out_result,
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    if (out_file_size == NULL || out_file_md5 == NULL || out_file_path == NULL) {
        return 1;
    }

    if (in_payload == NULL || in_payload_size <= 16 + 8 + 1) {
        return 2;
    }
    int pos = 0;
    *out_result = in_payload[pos];
    pos++;
//    printf("解析file_md5 pos=%d\n", pos);
    for (int i = 0; i < 16; ++i) {
        out_file_md5[i] = in_payload[pos+i];

    }
    pos+=16;
//    printf("解析file_size pos=%d\n", pos);
    *out_file_size = (uint64_t) in_payload[pos] |
                     (uint64_t) in_payload[pos + 1] << 8 |
                     (uint64_t) in_payload[pos + 2] << 16 |
                     (uint64_t) in_payload[pos + 3] << 24 |
                     (uint64_t) in_payload[pos + 4] << 32 |
                     (uint64_t) in_payload[pos + 5] << 40 |
                     (uint64_t) in_payload[pos + 6] << 48 |
                     (uint64_t) in_payload[pos + 7] << 56;
    pos += 8;

//    printf("解析file_path pos=%d\n", pos);
    size_t file_path_size = in_payload_size - 16 - 8;
//    printf("解析file_path size=%zu\n", file_path_size);
    for (int i = 0; i < file_path_size; ++i) {
        out_file_path[i] = in_payload[pos + i];
    }
//    printf("md5 = [");
//    for (int i = 0; i < 16; ++i) {
//        printf("%02x ", out_file_md5[i]);
//    }
//    printf("]\n");
//    printf("out_file_size = %llx\n", *out_file_size);
//    printf("out_file_path = %s\n", out_file_path);

    return 0;
}


// 开启push模式
uint8_t hd_host_push_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    return hd_host_push_pull_encode(out_payload, out_payload_size, in_result, in_file_md5, in_file_size, in_file_path);
}

// 解析开启push模式
uint8_t hd_slave_push_decode(
        uint8_t * out_result,
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    return hd_slave_push_pull_decode(out_result,out_file_md5, out_file_size, out_file_path, in_payload, in_payload_size);
}


uint8_t hd_host_push_encode_ext(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const char *src_file_path,
        const char *dest_file_path
) {
    // 1。打开文件
    int fd;
    fd = open(src_file_path, O_RDWR);
    if (fd == -1) {
        printf("extra_cmd.c 打开文件失败:%s 原因：%d->%s \n ", src_file_path, errno, strerror(errno));
        return 11;
    }
    // 2。获取文件长度
    // 获取文件长度
    struct stat file_stat;
    if (fstat(fd, &file_stat) == -1) {
        printf("extra_cmd.c 获取文件大小失败。fd:%d\n", fd);
        close(fd);
        return 12;
    }
    off_t file_size = file_stat.st_size;
//    printf("file_size   :  < %lld >bytes\n", file_size);
    // 3。获取文件md5
    unsigned char md5[16];
    int ret = hd_md5(src_file_path, md5);
    if (ret) {
        printf("extra_cmd.c 获取文件md5失败。fd:%d\n", fd);
        close(fd);
        return 13;
    }
//    printf("file_md5    :  ");
//    for (int i = 0; i < sizeof(md5); ++i) {
//        printf("%02x ", md5[i]);
//    }
//    printf("\n");
    return hd_host_push_encode(
            out_payload, out_payload_size, 0, md5, file_size, dest_file_path
    );
}

/* pull模式 */
/**
 * 拉取文件
 * @param out_payload
 * @param out_payload_size
 * @param in_file_path
 * @return
 */
uint8_t hd_host_pull_encode_payload(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const char *in_file_path
) {
    if (out_payload == NULL || out_payload_size == NULL) {
        return 1;
    }
    uint32_t size = strlen(in_file_path) + 1;
//    printf("pull size = %d\n", size);
//    printf("pull len  = %lu\n", strlen(in_file_path));
    unsigned char *payload = (unsigned char *) malloc(size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, size);
    int pos = 0;
//    printf("pull file_name.pos = %d\n", pos);
    for (int i = 0; i < strlen(in_file_path); ++i) {
        payload[pos++] = in_file_path[i];
    }
    payload[pos] = '\0';
    *out_payload_size = size;
    *out_payload = payload;
    return 0;
}

/**
 * 解析拉取文件
 * @param out_file_path
 * @param in_payload
 * @param in_payload_size
 * @return
 */
uint8_t hd_slave_pull_decode(
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    if (out_file_path == NULL) {
        return 1;
    }

    if (in_payload == NULL || in_payload_size <= 0) {
        return 2;
    }
    printf("解析pull file_path\n");
    size_t file_path_size = in_payload_size;
    printf("解析pull file_path size=%zu\n", file_path_size);
    for (int i = 0; i < file_path_size; ++i) {
        out_file_path[i] = in_payload[i];
    }
    printf("解析pull out_file_path = %s\n", out_file_path);
    return 0;
}

/**
 * 应答pull文件信息
 * @param out_file_md5
 * @param out_file_size
 * @param out_file_path
 * @param in_payload
 * @param in_payload_size
 * @return
 */
uint8_t hd_host_pull_decode(
        uint8_t *out_result,
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
) {
    return hd_slave_push_pull_decode(out_result, out_file_md5, out_file_size, out_file_path, in_payload,
                                     in_payload_size);
}

/**
 * 解析应答pull文件信息
 * @param out_payload
 * @param out_payload_size
 * @param in_file_md5
 * @param in_file_size
 * @param in_file_path
 * @return
 */
uint8_t hd_slave_pull_encode_payload(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    return hd_host_push_pull_encode(out_payload, out_payload_size, in_result, in_file_md5, in_file_size, in_file_path);
}

uint8_t hd_slave_pull_encode(
        unsigned char **out_protocol_payload,
        uint32_t *out_protocol_payload_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
) {
    unsigned char *out_payload_resp;
    uint32_t out_payload_size_resp;
    uint8_t ret;
    ret = hd_slave_pull_encode_payload(&out_payload_resp, &out_payload_size_resp, in_result, in_file_md5, in_file_size,
                                       in_file_path);
    if (ret)return -1;
    ret = hd_camera_protocol_encode(out_protocol_payload, out_protocol_payload_size, in_slave_addr, CMD_HD_EXTRA_PULL,
                                    out_payload_size_resp, out_payload_resp);
    free(out_payload_resp);
    return ret;
}

uint8_t hd_host_pull_encode(
        unsigned char **out_protocol,
        uint32_t *out_protocol_size,
        uint8_t in_slave_addr,
        const char *in_file_path
){
    unsigned char *out_payload_resp;
    uint32_t out_payload_size_resp;
    uint8_t ret;
    ret = hd_host_pull_encode_payload(&out_payload_resp, &out_payload_size_resp,
                                       in_file_path);
    hd_printf_buff(out_payload_resp,out_payload_size_resp,"hd_host_pull_encode",0);
    if (ret)return -1;
    ret = hd_camera_protocol_encode(out_protocol, out_protocol_size, in_slave_addr, CMD_HD_EXTRA_PULL,
                                    out_payload_size_resp, out_payload_resp);
    free(out_payload_resp);
    return ret;
}