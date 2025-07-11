
#ifndef H__HD_CAMERA_PROTOCOL_EXTRA_CMD__H
#define H__HD_CAMERA_PROTOCOL_EXTRA_CMD__H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "hd_camera_protocol.h"

#define CMD_HD_EXTRA_SHELL              0xF1
#define CMD_HD_EXTRA_PULL               0xF2
#define CMD_HD_EXTRA_PUSH               0xF3


/* push模式 */
/**
 * push 文件
 * @param out_payload
 * @param out_payload_size
 * @param in_file_md5
 * @param in_file_size
 * @param in_file_path
 * @return
 */
uint8_t hd_host_push_encode(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
);

uint8_t hd_host_push_encode_ext(
        unsigned char **out_payload,
        uint32_t *out_payload_size,
        const char *src_file_path,
        const char *dest_file_path
);

/**
 * 解析 push 文件
 * @param out_file_md5
 * @param out_file_size
 * @param out_file_path
 * @param in_payload
 * @param in_payload_size
 * @return
 */
uint8_t hd_slave_push_decode(
        uint8_t * out_result,
        unsigned char out_file_md5[16],
        uint64_t *out_file_size,
        char *out_file_path,
        const unsigned char *in_payload,
        uint32_t in_payload_size
);


/* ************* */
/* <0xF2 pull> */
/* ************* */
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
);

uint8_t hd_host_pull_encode(
        unsigned char **out_protocol,
        uint32_t *out_protocol_size,
        uint8_t in_slave_addr,
        const char *in_file_path
);

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
);

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
);

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
);


uint8_t hd_slave_pull_encode(
        unsigned char **out_protocol_payload,
        uint32_t *out_protocol_payload_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char in_file_md5[16],
        uint64_t in_file_size,
        const char *in_file_path
);


#ifdef __cplusplus
}
#endif

#endif // H__HD_CAMERA_PROTOCOL_EXTRA_CMD__H
