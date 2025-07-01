#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hd_camera_protocol_cmd.h"
#include "hd_camera_protocol.h"
#include "hd_utils.h"

#define DEBUG_PROTOCOL_CMD              0

static void hd_camera_protocol_print_buffer(const unsigned char *buf, size_t len, const char *tag) {
    hd_printf_buff(buf, len, tag, 0);
}

/* 心跳 */

uint8_t hd_host_heartbeat_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_ack_number
) {
    uint8_t payload[] = {in_ack_number};
    uint8_t ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            CMD_HEARTBEAT,
            sizeof(payload),
            payload
    );
    return ret;
}

uint8_t hd_slave_heartbeat_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_ack_number
) {
    uint8_t payload[] = {in_ack_number};
    uint8_t ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            CMD_HEARTBEAT,
            sizeof(payload),
            payload
    );
    return ret;
}

uint8_t hd_slave_heartbeat_decode(
        uint8_t *out_ack_number,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data_size == 1) {
        *out_ack_number = in_payload_data[0];
        return 0;
    } else {
        return -1;
    }
}

uint8_t hd_host_heartbeat_decode(
        uint8_t *out_ack_number,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data_size == 1) {
        *out_ack_number = in_payload_data[0];
        return 0;
    } else {
        return -1;
    }
}

/* 查询属性 */

static uint8_t common_host_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id,
        int hd
) {
    uint8_t payload[] = {in_property_id};
    uint8_t ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PROPERTY_GET : CMD_QJY_PROPERTY_GET,
            sizeof(payload),
            payload
    );
    return ret;
}

static uint8_t common_slave_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id,
        uint8_t in_result,
        const unsigned char *in_property_value,
        uint32_t in_property_value_size,
        int hd
) {
    if (DEBUG_PROTOCOL_CMD) {
        hd_printf_buff(in_property_value, in_property_value_size, "hd_slave_property_get_encode", 0);
    }
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("    sizeof (property_id_in) = %lu\n", sizeof(in_property_id));
        LOGD("    sizeof (result_in)      = %lu\n", sizeof(in_result));
        LOGD("    property_value_size_in  = %u\n", in_property_value_size);
    }
    uint8_t len = sizeof(in_property_id) + sizeof(in_result) + in_property_value_size;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("    len                     = %u\n", len);
    }
    uint8_t payload_size = len + 1;
    uint8_t *payload = (unsigned char *) malloc(payload_size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, payload_size);
    payload[0] = len;
    payload[1] = in_property_id;
    payload[2] = in_result;
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(in_property_value, in_property_value_size, "property_value_in");
        hd_camera_protocol_print_buffer(payload, len, "hd_slave_property_get_encode=1");
    }
    // 填充属性值
    if (in_property_value_size > 0 && in_property_value != NULL) {
        memcpy(&payload[3], in_property_value, in_property_value_size);
    }

    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload, len, "hd_slave_property_get_encode=2");
        LOGD("    sizeof (payload) = %hhu\n", payload_size);
    }

    uint8_t ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PROPERTY_GET : CMD_QJY_PROPERTY_GET,
            payload_size,
            payload
    );
    free(payload);
    return ret;
}



uint8_t qjy_host_property_get_decode(
        uint8_t *out_property_id,
        uint8_t *out_result,
        unsigned char **out_property_value,
        uint32_t *out_property_value_size,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size

){

    // AA 5A | 01 | 02 | 08 00 00 00 | 0C 02 00 64 65 76 2D 30 | CC 47

    //aa 5a  01  02   07 00 00 00    02 00 30 2e 34 2e 36    4f aa
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(in_payload_data, in_payload_data_size, "[hd_host_property_get_decode]");
    }
    if (in_payload_data == NULL || out_property_id == NULL ||
        out_result == NULL || out_property_value == NULL ||
        out_property_value_size == NULL) {
        return 1;
    }

    if (in_payload_data_size < 3) {
        return 2;
    }

//    uint8_t len = in_payload_data[0];
//    if (DEBUG_PROTOCOL_CMD) {
//        LOGD("    len                  = %d %0X\n", len, len);
//        LOGD("    payload_data_size_in = %d %0X\n", in_payload_data_size, in_payload_data_size);
//    }
//
//    // 检查声明的长度是否与实际数据长度一致
//    if (len > in_payload_data_size) {
//        return 3;
//    }

    *out_property_id = in_payload_data[0];
    *out_result = in_payload_data[1];

    *out_property_value_size = in_payload_data_size - 2;
    // 将属性值指针指向payload中属性值开始的位置(跳过前3字节)
    *out_property_value = (unsigned char *) (in_payload_data + 2);

    return 0;
}

uint8_t hd_host_property_get_decode(
        uint8_t *out_property_id,
        uint8_t *out_result,
        unsigned char **out_property_value,
        uint32_t *out_property_value_size,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size


) {
    // AA 5A | 01 | 02 | 08 00 00 00 | 0C 02 00 64 65 76 2D 30 | CC 47

    //aa 5a  01  02   07 00 00 00    02 00 30 2e 34 2e 36    4f aa
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(in_payload_data, in_payload_data_size, "[hd_host_property_get_decode]");
    }
    if (in_payload_data == NULL || out_property_id == NULL ||
        out_result == NULL || out_property_value == NULL ||
        out_property_value_size == NULL) {
        return 1;
    }

    if (in_payload_data_size < 3) {
        return 2;
    }

//    uint8_t len = in_payload_data[0];
//    if (DEBUG_PROTOCOL_CMD) {
//        LOGD("    len                  = %d %0X\n", len, len);
//        LOGD("    payload_data_size_in = %d %0X\n", in_payload_data_size, in_payload_data_size);
//    }
//
//    // 检查声明的长度是否与实际数据长度一致
//    if (len > in_payload_data_size) {
//        return 3;
//    }
    // todo fix 按照协议 [08 d4 01 30 2e 32 2e 33 36]
    // todo fix 08不需要
    *out_property_id = in_payload_data[1];
    *out_result = in_payload_data[2];

    *out_property_value_size = in_payload_data_size - 3;
    // 将属性值指针指向payload中属性值开始的位置(跳过前3字节)
    *out_property_value = (unsigned char *) (in_payload_data + 3);

    return 0;
}

uint8_t hd_slave_property_get_decode(
        uint8_t *out_property_id,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data_size == 1) {
        *out_property_id = in_payload_data[0];
        return 0;
    } else {
        return -1;
    }
}

/**
 * 查询属性 请求
 *
 * | 0x02 | 0x01 | ID |
 *
 * @param out_protocol_data             encode结果
 * @param out_protocol_data_size        encode结果大小
 * @param in_slave_addr                 从机地址
 * @param in_property_id                属性ID
 */
uint8_t qyj_host_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id
) {
    return common_host_property_get_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_property_id,
                                           0);
}

/**
 * 查询属性 应答
 *
 * | 0x02 | len | ID | result | value |
 *
 * @param out_protocol_data             encode结果
 * @param out_protocol_data_size        encode结果大小
 * @param in_slave_addr                 从机地址
 * @param in_property_id                属性ID
 * @param in_result                     返回值。0x00：成功，其他：失败
 * @param in_property_value             属性值
 * @param in_property_value_size             属性值大小
 */
uint8_t qjy_slave_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id,
        uint8_t in_result,
        const unsigned char *in_property_value,
        uint32_t in_property_value_size
) {
    return common_slave_property_get_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_property_id,
                                            in_result, in_property_value, in_property_value_size, 0);
}


/**
 * 查询属性 请求
 *
 * | 0x02 | 0x01 | ID |
 *
 * @param out_protocol_data             encode结果
 * @param out_protocol_data_size        encode结果大小
 * @param in_slave_addr                 从机地址
 * @param in_property_id                属性ID
 */
uint8_t hd_host_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id
) {
    return common_host_property_get_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_property_id,
                                           1);
}

/**
 * 查询属性 应答
 *
 * | 0x02 | len | ID | result | value |
 *
 * @param out_protocol_data             encode结果
 * @param out_protocol_data_size        encode结果大小
 * @param in_slave_addr                 从机地址
 * @param in_property_id                属性ID
 * @param in_result                     返回值。0x00：成功，其他：失败
 * @param in_property_value             属性值
 * @param in_property_value_size             属性值大小
 */
uint8_t hd_slave_property_get_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id,
        uint8_t in_result,
        const unsigned char *in_property_value,
        uint32_t in_property_value_size
) {
    return common_slave_property_get_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_property_id,
                                            in_result, in_property_value, in_property_value_size, 1);
}

/* 设置属性 */

/**
 * 设置属性 请求
 *
 * | 0x03 | len | ID | value |
 *
 * @param slave_addr_in             从机地址
 * @param property_id_in            属性ID
 * @param property_value_in         属性值
 * @param property_value_size_in    属性值size
 */
static uint8_t common_host_property_set_encode(
        uint8_t slave_addr_in,
        uint8_t property_id_in,
        const unsigned char *property_value_in,
        uint32_t property_value_size_in,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        int hd
) {
    if (DEBUG_PROTOCOL_CMD) {
        LOGD(" hd_host_property_set_encode ........\n");
    }
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(property_value_in, property_value_size_in, "[hd_host_property_set_encode]");
    }
    if (
            property_value_in == NULL || protocol_data_out == NULL ||
            protocol_data_size_out == NULL) {
        return 1;
    }

    uint8_t payload_size = sizeof(property_id_in) + property_value_size_in;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("payload_size                     = %u\n", payload_size);
    }
    uint8_t *payload = (unsigned char *) malloc(payload_size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, payload_size);
    payload[0] = property_id_in;

    //unsigned char *reverse = reverse_array(property_value_in, property_value_size_in);
//    if(DEBUG_PROTOCOL_CMD) {
//        hd_camera_protocol_print_buffer(property_value_in, property_value_size_in, "[property_value_in]");
//        hd_camera_protocol_print_buffer(reverse, property_value_size_in, "[reverse]");
//    }

    memcpy(&payload[1], property_value_in, property_value_size_in);
//    free(property_value_in);

    // 填充属性值 小端模式
//    copy_little_endian(property_value_in,property_value_size_in,
//                       payload,payload_size,2);

    uint8_t ret = hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            hd ? CMD_HD_PROPERTY_SET : CMD_QJY_PROPERTY_SET,
            payload_size,
            payload
    );
    free(payload);
    return ret;
}

/**
 * 设置属性 应答
 *
 * | 0x03 | 0x02 | ID | result |
 *
 * @param slave_addr        从机地址
 * @param property_id       属性ID
 * @param result            返回值。0x00：成功，其他：失败
 */
uint8_t common_slave_property_set_encode(
        uint8_t slave_addr_in,
        uint8_t property_id_in,
        uint8_t result_in,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        int hd
) {

    if (protocol_data_out == NULL || protocol_data_size_out == NULL) {
        return 1;
    }

    uint8_t payload_size = 3;
    uint8_t *payload = (unsigned char *) malloc(payload_size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, payload_size);
    payload[0] = 2;
    payload[1] = property_id_in;
    payload[2] = result_in;

    uint8_t ret = hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            hd ? CMD_HD_PROPERTY_SET : CMD_QJY_PROPERTY_SET,
            payload_size,
            payload
    );
    free(payload);
    return ret;

}


/**
 * 设置属性 请求
 *
 * | 0x03 | len | ID | value |
 *
 * @param slave_addr_in             从机地址
 * @param property_id_in            属性ID
 * @param property_value_in         属性值
 * @param property_value_size_in    属性值size
 */
uint8_t qjy_host_property_set_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint8_t property_id_in,
        const unsigned char *property_value_in,
        uint32_t property_value_size_in

) {
    return common_host_property_set_encode(slave_addr_in, property_id_in, property_value_in, property_value_size_in,
                                           protocol_data_out, protocol_data_size_out, 0);
}

/**
 * 设置属性 应答
 *
 * | 0x03 | 0x02 | ID | result |
 *
 * @param slave_addr        从机地址
 * @param property_id       属性ID
 * @param result            返回值。0x00：成功，其他：失败
 */
uint8_t qjy_slave_property_set_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint8_t property_id_in,
        uint8_t result_in

) {
    return common_slave_property_set_encode(slave_addr_in, property_id_in, result_in, protocol_data_out,
                                            protocol_data_size_out, 0);
}

/**
 * 设置属性 请求
 *
 * | 0xC3 | len | ID | value |
 *
 * @param slave_addr_in             从机地址
 * @param property_id_in            属性ID
 * @param property_value_in         属性值
 * @param property_value_size_in    属性值size
 */
uint8_t hd_host_property_set_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint8_t property_id_in,
        const unsigned char *property_value_in,
        uint32_t property_value_size_in

) {
    return common_host_property_set_encode(slave_addr_in, property_id_in, property_value_in, property_value_size_in,
                                           protocol_data_out, protocol_data_size_out, 1);
}

/**
 * 设置属性 应答
 *
 * | 0xC3 | 0x02 | ID | result |
 *
 * @param slave_addr        从机地址
 * @param property_id       属性ID
 * @param result            返回值。0x00：成功，其他：失败
 */
uint8_t hd_slave_property_set_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_property_id,
        uint8_t in_result
) {
    return common_slave_property_set_encode(in_slave_addr, in_property_id, in_result, out_protocol_data,
                                            out_protocol_data_size, 1);
}

/**
 * 设置属性应答 解析
 * @param payload_data_in           payload
 * @param payload_data_size_in      payload size
 * @param property_id_out           属性ID
 * @param result_out                返回值。0x00：成功，其他：失败
 * @return
 */
uint8_t hd_host_property_set_decode(
        uint8_t *property_id_out,
        uint8_t *result_out,
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in

) {

    if (payload_data_in == NULL || property_id_out == NULL || result_out == NULL) {
        return 1;
    }

    // AA 5A 01 03 03 00 00 00  | 02 09 00 |  DF 4A
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload_data_in, payload_data_size_in, "[hd_host_property_set_decode]");
    }
    if (payload_data_size_in < 2) {
        return 2;
    }

    uint8_t len = payload_data_in[0];
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("    len                  = %d %0x\n", len, len);
        LOGD("    payload_data_size_in = %d %0x\n", payload_data_size_in, payload_data_size_in);
    }

    // 检查声明的长度是否与实际数据长度一致
    if (len > payload_data_size_in) {
        return 3;
    }

    *property_id_out = payload_data_in[1];
    *result_out = payload_data_in[2];
    return 0;
}

/**
 * 设置属性请求 解析
 * @param payload_data_in           payload
 * @param payload_data_size_in      payload size
 * @param property_id_out           属性ID
 * @param result_value              属性值
 * @return
 */
uint8_t hd_slave_property_set_decode(
        uint8_t *property_id_out,
        unsigned char **result_value_out,
        uint32_t *result_value_size_out,
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in


) {

//    if (payload_data_in == NULL || property_id_out == NULL ||
//        result_value_out == NULL || result_value_size_out == NULL) {
//        return 1;
//    }
//
//    // AA 5A 01 03 06 00 00 00 05 09 04 03 02 01 26 AD
//    if (DEBUG_PROTOCOL_CMD) {
//        hd_camera_protocol_print_buffer(payload_data_in, payload_data_size_in, "[hd_slave_property_set_decode]");
//    }
//
//    if (payload_data_size_in < 2) {
//        return 2;
//    }
//
//    uint8_t id = payload_data_in[0];
//
//
//    *property_id_out = id;
//    size_t  size =  payload_data_size_in - 1;
//    *result_value_size_out = size;
//    if (DEBUG_PROTOCOL_CMD) {
//        LOGD("    id                      = %d %0x\n", id, id);
//        LOGD("    payload_data_size_in    = %d %0x\n", payload_data_size_in, payload_data_size_in);
//        LOGD("    result_value_size_out   = %d %0x\n", *result_value_size_out, *result_value_size_out);
//    }
//
//    if (*result_value_size_out == 0) {
//        result_value_out = NULL;
//        return 0;
//    }
//    // 将属性值指针指向payload中属性值开始的位置(跳过前3字节)
////    unsigned char *tmp = (unsigned char *) malloc(size );
//
//    for (int i = 0; i < size; ++i) {
//        LOGD("%d---> %02x\n",i,payload_data_in[i + 1]);
//        *result_value_out[i] = payload_data_in[i + 1];
//    }
//    if (DEBUG_PROTOCOL_CMD) {
//        hd_printf_buff(result_value_out, *result_value_size_out, "x", 0);
//    }
    // 1. 参数有效性检查
    if (payload_data_in == NULL || property_id_out == NULL ||
        result_value_out == NULL || result_value_size_out == NULL) {
        return 1; // 无效参数
    }

    // 2. 检查最小数据长度(至少需要1字节的属性ID)
    if (payload_data_size_in < 1) {
        return 2; // 数据长度不足
    }

    // 3. 提取属性ID(第一个字节)
    *property_id_out = payload_data_in[0];

    // 4. 计算属性值数据大小
    *result_value_size_out = payload_data_size_in - 1;

    // 5. 处理属性值数据
    if (*result_value_size_out > 0) {
        // 5.1 分配内存存放属性值
        unsigned char *tmp = (unsigned char *)malloc(*result_value_size_out);
        if (tmp == NULL) {
            return 3; // 内存分配失败
        }

        // 5.2 复制属性值数据(跳过第一个字节)
        memcpy(tmp, payload_data_in + 1, *result_value_size_out);

        // 5.3 设置输出指针
        *result_value_out = tmp;
    } else {
        // 没有属性值数据
        *result_value_out = NULL;
    }

    return 0; // 成功
}

/* 恢复出厂 */

uint8_t hd_camera_protocol_cmd_property_recovery_req(uint8_t slave_addr) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_recovery_resp(uint8_t *slave_addr, uint8_t *result) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_reboot_req(uint8_t slave_addr) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_reboot_resp(uint8_t *slave_addr, uint8_t *result) {
    return 0;
}

/* ************* */
/* <主动抓图> */
/* ************* */
static uint8_t common_host_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        int hd
) {
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_CAMERA_SNAPSHOT : CMD_QJY_CAMERA_SNAPSHOT,
            0,
            NULL
    );
}


/**
 * 主动抓图 请求
 *
 * | 0x06 | 0x00 |
 *
 * @param slave_addr        从机地址
 */
uint8_t qjy_host_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
) {
    return common_host_snapshot_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, 0);
}


static uint8_t common_slave_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        uint8_t in_pic_id,
        int hd
) {
    unsigned char default_value[2] = {in_result, in_pic_id};
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_CAMERA_SNAPSHOT : CMD_QJY_CAMERA_SNAPSHOT,
            sizeof(default_value),
            default_value
    );
}

uint8_t qjy_slave_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        uint8_t in_pic_id
) {
    return common_slave_snapshot_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result, in_pic_id,
                                        0);
}

uint8_t hd_host_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
) {
    return common_host_snapshot_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, 1);
}

uint8_t hd_slave_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        uint8_t in_pic_id
) {
    return common_slave_snapshot_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result, in_pic_id,
                                        1);
}

/**
 * 主动抓图 应答
 *
 * | 0x06 | 0x02 | result | pic_id |
 *
 * @param result            返回值。0x00：成功，其他：失败
 * @param pic_id            图片id，取图片用
 */
uint8_t hd_slave_snapshot_decode(
        uint8_t *out_result,
        uint8_t *out_pic_id,
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
) {
    if (payload_data_in == NULL || payload_data_size_in != 2) {
        return -2;
    }
    *out_result = payload_data_in[0];
    *out_pic_id = payload_data_in[1];
    return 0;
}

uint8_t hd_host_snapshot_decode(
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
) {
    // ignore
    return 0;
}

void hd_dynamic_pic_info_print(const hd_dynamic_pic_info **infos, size_t size) {
    LOGD("=================图片信息==========================\n");
    if (size > 0) {

        LOGD("大小：(%zu):\n", size);
        for (int i = 0; i < size; ++i) {
            LOGD("------------%d-------------\n", i);
            LOGD("id                    =     %02x   %d\n", infos[i]->id, infos[i]->id);
            LOGD("size                  =     %02x   %d\n", infos[i]->size, infos[i]->size);
            LOGD("action_id_index       =     %02x   %d\n", infos[i]->action_id_index, infos[i]->action_id_index);
            LOGD("action_id_timestamps  =     %02x   %d\n", infos[i]->action_id_timestamps,
                 infos[i]->action_id_timestamps);
            LOGD("trigger_type          =     %02x   %d\n", infos[i]->trigger_type, infos[i]->trigger_type);
            LOGD("trigger_angel         =     %02x   %d\n", infos[i]->trigger_angel, infos[i]->trigger_angel);
            LOGD("snapshot_timestamps   =     %02x   %d\n", infos[i]->snapshot_timestamps,
                 infos[i]->snapshot_timestamps);
            LOGD("md5                   =     ");
            for (int j = 0; j < 16; ++j) {
                LOGD("%02x ", infos[i]->md5[j]);
            }
            LOGD("\n");

        }
    } else {
        LOGD("图片信息为空\n");
    }
    LOGD("====================end=======================\n");
}

uint8_t common_host_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        int hd
) {
    return hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            hd ? CMD_HD_PIC_INFO : CMD_QJY_PIC_INFO,
            0,
            NULL
    );
}

uint8_t hd_host_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in
) {
    return common_host_pic_info_encode(protocol_data_out, protocol_data_size_out, slave_addr_in, 1);
}

uint8_t qjy_host_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in
) {
    return common_host_pic_info_encode(protocol_data_out, protocol_data_size_out, slave_addr_in, 0);
}


uint8_t hd_host_pic_info_decode(
        hd_dynamic_pic_info ***pic_info_out,
        uint32_t *pic_info_size_out,
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
) {
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload_data_in, payload_data_size_in, "[hd_host_pic_info_decode]");
    }
    uint32_t pic_number = payload_data_in[0];
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("len            =   %d(%02x)\n", payload_data_size_in, payload_data_size_in);
        LOGD("pic_number     =   %d(%02x)\n", pic_number, pic_number);
    }
    if (payload_data_size_in - pic_number * hd_dynamic_pic_info_real_size() != 1) {
        perror("len != pic_number");
        return -2;
    }
    size_t count = pic_number * sizeof(hd_dynamic_pic_info *);
    hd_dynamic_pic_info **infos = (hd_dynamic_pic_info **) malloc(count);
    int pos = 1;
    for (int i = 0; i < pic_number; ++i) {
        if (DEBUG_PROTOCOL_CMD) {
            LOGD("pos=%d \n", pos);
        }
        hd_dynamic_pic_info *info = (hd_dynamic_pic_info *) malloc(sizeof(hd_dynamic_pic_info));
        uint8_t id = payload_data_in[pos];
        pos++;
        uint32_t action_id_timestamps = (uint32_t) payload_data_in[pos + 0] |  // 最低字节在最低地址
                                        (uint32_t) payload_data_in[pos + 1] << 8 |
                                        (uint32_t) payload_data_in[pos + 2] << 16 |
                                        (uint32_t) payload_data_in[pos + 3] << 24;
        pos += 4;
        uint8_t action_id = payload_data_in[pos++];
        uint8_t trigger_type = payload_data_in[pos++];
        uint8_t trigger_angel = payload_data_in[pos++];
        uint32_t snapshot_timestamps = (uint32_t) payload_data_in[pos + 0] |  // 最低字节在最低地址
                                       (uint32_t) payload_data_in[pos + 1] << 8 |
                                       (uint32_t) payload_data_in[pos + 2] << 16 |
                                       (uint32_t) payload_data_in[pos + 3] << 24;
        pos += 4;
        uint32_t size = (uint32_t) payload_data_in[pos + 0] |  // 最低字节在最低地址
                        (uint32_t) payload_data_in[pos + 1] << 8 |
                        (uint32_t) payload_data_in[pos + 2] << 16 |
                        (uint32_t) payload_data_in[pos + 3] << 24;
        pos += 4;
        if (DEBUG_PROTOCOL_CMD) {
            LOGD("    id                        :       %02x\n", id);
            LOGD("    action_id_timestamps      :       %02x\n", action_id_timestamps);
            LOGD("    action_id                 :       %02x\n", action_id);
            LOGD("    trigger_type              :       %02x\n", trigger_type);
            LOGD("    trigger_angel             :       %02x\n", trigger_angel);
            LOGD("    snapshot_timestamps       :       %02x\n", snapshot_timestamps);
            LOGD("    size                      :       %02x\n", size);
        }
        info->id = id;
        info->action_id_timestamps = action_id_timestamps;
        info->action_id_index = action_id;
        info->trigger_type = trigger_type;
        info->trigger_angel = trigger_angel;
        info->snapshot_timestamps = snapshot_timestamps;
        info->size = size;
        memcpy(info->md5, payload_data_in + pos, 16);
        pos += 16;
        infos[i] = info;
    }
    *pic_info_size_out = pic_number;
    *pic_info_out = infos;
    return 0;
}

static int _is_big_endian() {
    union {
        uint32_t i;
        uint8_t c[4];
    } test = {0x01020304};

    return test.c[0] == 0x01; // 大端返回1，小端返回0
}

uint8_t hd_dynamic_pic_infos_encode(hd_dynamic_pic_info *infos,
                                    size_t count,
                                    unsigned char **result,
                                    size_t *result_size,
                                    uint8_t is_big_endian
) {
    if (infos == NULL || count == 0) return -1;


    if(DEBUG){
        hd_dynamic_pic_info_print( infos, count);
    }

    const size_t real_size = hd_dynamic_pic_info_real_size();
    const size_t struct_size = sizeof(hd_dynamic_pic_info);
    const size_t total_size = real_size * count;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("hd_dynamic_pic_info     count           =    %zu\n", count);
        LOGD("hd_dynamic_pic_info     real_size       =    %zu\n", real_size);
        LOGD("hd_dynamic_pic_info     struct_size     =    %zu\n", struct_size);
        LOGD("hd_dynamic_pic_infos    total_size      =    %zu\n", total_size);
    }
    unsigned char *buffer = (unsigned char *) malloc(total_size);
    if (buffer == NULL) {
        perror("hd_dynamic_pic_infos_encode malloc fail ");
        return -1;
    }
    // 逐个复制结构体到缓冲区
    unsigned char *ptr = buffer;
    for (size_t i = 0; i < count; i++) {
        const hd_dynamic_pic_info info = infos[i];

        // 复制id (1字节)
        *ptr++ = info.id;

//        if (is_big_endian) {
//            uint32_t r = swap_uint32((info->action_id_timestamps));
//            memcpy(ptr, &r, sizeof(uint32_t));
//        } else {
//            memcpy(ptr, &info->action_id_timestamps, sizeof(uint32_t));
//        }
//        ptr += sizeof(uint32_t);
        *ptr++ = (info.action_id_timestamps >> 0) & 0xFF;  // LSB
        *ptr++ = (info.action_id_timestamps >> 8) & 0xFF;
        *ptr++ = (info.action_id_timestamps >> 16) & 0xFF;
        *ptr++ = (info.action_id_timestamps >> 24) & 0xFF; // MSB




        *ptr++ = info.action_id_index;
        *ptr++ = info.trigger_type;
        *ptr++ = info.trigger_angel;

        // 复制snapshot_timestamps (4字节)
//        if (is_big_endian) {
//            uint32_t r = swap_uint32((info->snapshot_timestamps));
//            memcpy(ptr, &r, sizeof(uint32_t));
//        } else {
//            memcpy(ptr, &info->snapshot_timestamps, sizeof(uint32_t));
//        }
//        ptr += sizeof(uint32_t);

        *ptr++ = (info.snapshot_timestamps >> 0) & 0xFF;  // LSB
        *ptr++ = (info.snapshot_timestamps >> 8) & 0xFF;
        *ptr++ = (info.snapshot_timestamps >> 16) & 0xFF;
        *ptr++ = (info.snapshot_timestamps >> 24) & 0xFF; // MSB



        // 复制size (4字节)
//        if (is_big_endian) {
//            uint32_t r = swap_uint32((info->size));
//            memcpy(ptr, &r, sizeof(uint32_t));
//        } else {
//            memcpy(ptr, &info->size, sizeof(uint32_t));
//        }
//        ptr += sizeof(uint32_t);

        *ptr++ = (info.size >> 0) & 0xFF;  // LSB
        *ptr++ = (info.size >> 8) & 0xFF;
        *ptr++ = (info.size >> 16) & 0xFF;
        *ptr++ = (info.size >> 24) & 0xFF; // MSB

        // 复制md5 (16字节) 数组 不受大小端影响
        memcpy(ptr, info.md5, 16);
        ptr += 16;
    }
    *result_size = total_size;
    *result = buffer;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("hd_dynamic_pic_infos 转换成字节大小: %d  = (%d x %d)\n", total_size, real_size, count);
    }
    return 0;
}


static uint8_t common_slave_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        hd_dynamic_pic_info *info_in,
        uint32_t info_size_in,
        int hd
) {
    if (protocol_data_out == NULL || protocol_data_size_out == NULL ) {
        return -1;
    }

    if (info_size_in == 0 || info_in == NULL) {
        unsigned char default_value[] = { 0};
        return hd_camera_protocol_encode(
                protocol_data_out,
                protocol_data_size_out,
                slave_addr_in,
                hd ? CMD_HD_PIC_INFO : CMD_QJY_PIC_INFO,
                sizeof(default_value),
                default_value
        );
    }
    int ret;
//    uint32_t pic_info_size = info_size_in * sizeof(hd_dynamic_pic_info);
    unsigned char *pic_info;
    size_t pic_info_size;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("encode pic ....\n");
    }
    ret = hd_dynamic_pic_infos_encode(info_in, info_size_in, &pic_info, &pic_info_size, 0);
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("encode end ! pic_info_size=%d\n", pic_info_size);
        //hd_printf_buff(pic_info, pic_info_size, "<图片数据>", 1);
    }
    if (ret) {
        perror("hd_dynamic_pic_infos_encode error .");
        return -1;
    }

    if (DEBUG_PROTOCOL_CMD) {

        hd_camera_protocol_print_buffer(pic_info, pic_info_size, "[pic_info]");
    }
    // len + pic_num + hd_camera_protocol_pic_infos
    uint32_t payload_size = 1 + pic_info_size;
    unsigned char *payload = (unsigned char *) malloc(payload_size);
    if (payload==NULL){
        LOGW("malloc payload fail.\n");
        return 2;
    }
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("malloc payload size = %d\n", payload_size);
    }
    payload[0] = info_size_in;
    memcpy(&payload[1], pic_info, pic_info_size);
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload, payload_size, "[hd_slave_pic_info_encode]");
    }
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("hd_camera_protocol_encode ....\n");
    }
    ret =  hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            CMD_HD_PIC_INFO,
            payload_size,
            payload
    );
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("hd_camera_protocol_encode end \n");
    }
    free(payload);
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("common_slave_pic_info_encode end!! \n");
    }
    return ret;
}


uint8_t hd_slave_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        hd_dynamic_pic_info *info_in,
        uint32_t info_size_in
) {
    return common_slave_pic_info_encode(protocol_data_out, protocol_data_size_out, slave_addr_in, info_in, info_size_in,
                                        1);
}

uint8_t qjy_slave_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        hd_dynamic_pic_info *info_in,
        uint32_t info_size_in
) {
    return common_slave_pic_info_encode(protocol_data_out, protocol_data_size_out, slave_addr_in, info_in, info_size_in,
                                        0);
}

uint8_t hd_slave_pic_info_decode(
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
) {
    // ignore
    return 0;
}

/* ************* */
/* <0x08 删除图片> */
/* ************* */
static uint8_t
common_host_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        int hd
) {

    uint8_t payload[] = {in_pic_id};
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_DELETE : CMD_QJY_PIC_DELETE,
            sizeof(payload),
            payload
    );
}

uint8_t hd_host_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id
) {
    return common_host_delete_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_pic_id, 1);
}

uint8_t qjy_host_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id
) {
    return common_host_delete_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_pic_id, 0);
}

uint8_t
hd_slave_delete_pic_decode(
        uint8_t *out_pic_id,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data == NULL || in_payload_data_size != 1) {
        return -2;
    }
    *out_pic_id = in_payload_data[0];
    return 0;
}

static uint8_t
common_slave_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        int hd
) {
    uint8_t payload[] = {in_result};
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_DELETE : CMD_QJY_PIC_DELETE,
            sizeof(payload),
            payload
    );
}

uint8_t
hd_slave_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
) {
    return common_slave_delete_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result, 1);
}

uint8_t
qjy_slave_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
) {
    return common_slave_delete_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result, 0);
}

uint8_t
hd_host_delete_pic_decode(
        uint8_t *out_result,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data == NULL || in_payload_data_size != 1) {
        return -2;
    }
    *out_result = in_payload_data[0];
    return 0;
}

/* ************* */
/* <0x09 拉取图片> */
/* ************* */

static uint8_t
common_host_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        uint32_t in_offset,
        uint32_t in_read_len,
        int hd
) {
    if (out_protocol_data == NULL || out_protocol_data_size == NULL) {
        return -1;
    }
    uint32_t payload_size = sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint32_t);
    unsigned char *payload = (unsigned char *) malloc(payload_size);

    payload[0] = in_pic_id;

    payload[1] = (in_offset >> 0) & 0xFF;  // LSB
    payload[2] = (in_offset >> 8) & 0xFF;
    payload[3] = (in_offset >> 16) & 0xFF;
    payload[4] = (in_offset >> 24) & 0xFF; // MSB

    payload[5] = (in_read_len >> 0) & 0xFF;  // LSB
    payload[6] = (in_read_len >> 8) & 0xFF;
    payload[7] = (in_read_len >> 16) & 0xFF;
    payload[8] = (in_read_len >> 24) & 0xFF; // MSB

    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload, payload_size, "[hd_host_pull_pic_encode]");
    }
    int ret =  hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_PULL : CMD_QJY_PIC_PULL,
            payload_size,
            payload
    );
    free(payload);
    return ret;
}

uint8_t
hd_host_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        uint32_t in_offset,
        uint32_t in_read_len
) {
    return common_host_pull_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_pic_id, in_offset,
                                       in_read_len, 1);
}

uint8_t
qjy_host_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        uint32_t in_offset,
        uint32_t in_read_len
) {
    return common_host_pull_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_pic_id, in_offset,
                                       in_read_len, 0);
}

uint8_t hd_slave_pull_pic_decode(
        uint8_t *out_pic_id,
        uint32_t *out_offset,
        uint32_t *out_read_len,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data == NULL || in_payload_data_size != (sizeof(uint8_t) + sizeof(uint32_t) * 2)) {
        return -2;
    }
    *out_pic_id = in_payload_data[0];
    *out_offset = (uint32_t) in_payload_data[1] |  // 最低字节在最低地址
                  (uint32_t) in_payload_data[2] << 8 |
                  (uint32_t) in_payload_data[3] << 16 |
                  (uint32_t) in_payload_data[4] << 24;

    *out_read_len = (uint32_t) in_payload_data[5] |  // 最低字节在最低地址
                    (uint32_t) in_payload_data[6] << 8 |
                    (uint32_t) in_payload_data[7] << 16 |
                    (uint32_t) in_payload_data[8] << 24;

    return 0;
}

static uint8_t
common_slave_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char *in_pic_data,
        size_t in_pic_data_size,
        int hd
) {
    if (out_protocol_data == NULL || out_protocol_data_size == NULL) {
        return -1;
    }
    uint32_t payload_size = sizeof(in_result) + in_pic_data_size;
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("payload_size  = %d \n", payload_size);
    }
    unsigned char *payload = (unsigned char *) malloc(payload_size);
//    for (int i = 0; i < payload_size; ++i) {
//         LOGD(" %d--->%02x\n",i,payload[i]);
//    }
    payload[0] = in_result;
    memcpy(&payload[1], in_pic_data, in_pic_data_size);

//    for (int i = 0; i < payload_size; ++i) {
//         LOGD(" %d--->%02x\n",i,payload[i]);
//    }
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(payload, payload_size, "[hd_slave_pull_pic_encode]");
    }
//
    int ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_PULL : CMD_QJY_PIC_PULL,
            payload_size,
            payload
    );
//     LOGD("ret = %d \n",ret);
    // AA 5A
    // 01
    // 09
    // 31 00 00 00
    // 01
    // 01 00 00 01 00 01 00 00 FF DB 00 43
    // 00 01 01 01 01 01 01 01 01 01 01 01
    // 01 01 01 01 01 01 01 01 01 01 01 01
    // 01 01 01 01 01 01 01 01 01 01 01 01
    // 8D 6F
    return ret;
}

uint8_t
hd_slave_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char *in_pic_data,
        size_t in_pic_data_size
) {
    return common_slave_pull_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result,
                                        in_pic_data, in_pic_data_size, 1);
}

uint8_t
qjy_slave_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char *in_pic_data,
        size_t in_pic_data_size
) {
    return common_slave_pull_pic_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result,
                                        in_pic_data, in_pic_data_size, 0);
}

uint8_t hd_host_pull_pic_decode(
        uint8_t *out_result,
        unsigned char **out_pic_data,
        uint32_t *out_pic_data_size,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data == NULL || in_payload_data_size < (sizeof(uint8_t) + sizeof(uint32_t))) {
        return -2;
    }
    // 11 00 00 00  01  43 02 53 62 04 3A AE 9F AE 9B 33 96 BD F2 E1 D7
    if (DEBUG_PROTOCOL_CMD) {
        hd_camera_protocol_print_buffer(in_payload_data, in_payload_data_size, "hd_host_pull_pic_decode");
    }
//    uint32_t len = (uint32_t) in_payload_data[0] |  // 最低字节在最低地址
//                   (uint32_t) in_payload_data[1] << 8 |
//                   (uint32_t) in_payload_data[2] << 16 |
//                   (uint32_t) in_payload_data[3] << 24;
    *out_result = in_payload_data[0];
    *out_pic_data_size = in_payload_data_size - sizeof(uint8_t);
    if (DEBUG_PROTOCOL_CMD) {
        LOGD("out_result          =   %d\n", *out_result);
        LOGD("out_pic_data_size   =   %d\n", *out_pic_data_size);
    }
    unsigned char *result = (unsigned char *) malloc(*out_pic_data_size);
    uint32_t pos = 1;
    for (uint32_t i = 0; i < *out_pic_data_size; ++i) {
        result[i] = in_payload_data[pos];
        pos++;
    }
    *out_pic_data = result;
    return 0;
}

/* ************* */
/* <0x0A 拉取图片完成> */
/* ************* */
static uint8_t
common_host_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        int hd
) {
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_PULL_COMPLETED : CMD_QJY_PIC_PULL_COMPLETED,
            0,
            NULL
    );
}

uint8_t
hd_host_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
) {
    return common_host_pull_pic_complete_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, 1);
}

uint8_t
qjy_host_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
) {
    return common_host_pull_pic_complete_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, 0);
}

uint8_t
hd_slave_pull_pic_complete_decode(
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    // ignore
    return 0;
}

static uint8_t
common_slave_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        int hd
) {
    uint8_t result[] = {in_result};
    return hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            hd ? CMD_HD_PIC_PULL_COMPLETED : CMD_QJY_PIC_PULL_COMPLETED,
            sizeof(result),
            result
    );
}

uint8_t
hd_slave_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
) {
    return common_slave_pull_pic_complete_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result,
                                                 1);
}

uint8_t
qjy_slave_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
) {
    return common_slave_pull_pic_complete_encode(out_protocol_data, out_protocol_data_size, in_slave_addr, in_result,
                                                 0);
}

uint8_t
hd_host_pull_pic_complete_decode(
        uint8_t *out_result,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {
    if (in_payload_data == NULL || in_payload_data_size != 1) {
        return -2;
    }
    *out_result = in_payload_data[0];
    return 0;
}

uint8_t
hd_host_ota_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint32_t file_size,
        const unsigned char file_md5[16]) {


    if (protocol_data_out == NULL || protocol_data_size_out == NULL) {
        return 1;
    }

    uint8_t payload_size = 0x14;
    uint8_t *payload = (unsigned char *) malloc(payload_size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, payload_size);

    // 填充数据长度 (小端模式)
    payload[0] = (file_size >> 0) & 0xFF;  // LSB
    payload[1] = (file_size >> 8) & 0xFF;
    payload[2] = (file_size >> 16) & 0xFF;
    payload[3] = (file_size >> 24) & 0xFF; // MSB

    memcpy(&payload[4], file_md5, 16);

    uint8_t ret = hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            CMD_OTA,
            payload_size,
            payload
    );
    free(payload);
    return ret;
}

uint8_t hd_camera_protocol_cmd_property_camera_ota_resp(uint8_t *slave_addr, uint8_t *result, uint32_t *offset) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_ota_push_req(uint8_t slave_addr, uint32_t offset, uint32_t data_len,
                                                            const unsigned char *data) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_ota_push_resp(uint8_t *slave_addr, uint8_t *result) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_door_signal_req(uint8_t slave_addr) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_door_signal_resp(uint8_t *slave_addr, uint8_t *result) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_app_upgrade_req(uint8_t slave_addr, uint32_t file_size,
                                                               unsigned char *file_md5) {
    return 0;
}

uint8_t hd_host_app_upgrade_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint32_t file_size,
        const unsigned char file_md5[16]
) {


    if (protocol_data_out == NULL || protocol_data_size_out == NULL) {
        return 1;
    }

    uint8_t payload_size = 0x14;
    uint8_t *payload = (unsigned char *) malloc(payload_size);
    if (payload == NULL) {
        return -1; // 内存分配失败
    }
    memset(payload, 0, payload_size);

    // 填充数据长度 (小端模式)
    payload[0] = (file_size >> 0) & 0xFF;  // LSB
    payload[1] = (file_size >> 8) & 0xFF;
    payload[2] = (file_size >> 16) & 0xFF;
    payload[3] = (file_size >> 24) & 0xFF; // MSB

    memcpy(&payload[4], file_md5, 16);

    uint8_t ret = hd_camera_protocol_encode(
            protocol_data_out,
            protocol_data_size_out,
            slave_addr_in,
            CMD_APP_UPGRADE,
            payload_size,
            payload
    );
    free(payload);
    return ret;
}

uint8_t
hd_camera_protocol_cmd_property_camera_app_upgrade_push_req(uint8_t slave_addr, uint32_t offset, uint32_t data_len,
                                                            unsigned char *data) {
    return 0;
}

uint8_t hd_camera_protocol_cmd_property_camera_app_upgrade_push_resp(uint8_t *slave_addr, uint8_t *result) {
    return 0;
}


static int is_little_endian2() {
    uint32_t num = 0x01020304;
    uint8_t *p = (uint8_t *) &num;
    return p[0] == 0x04; // 小端返回1，大端返回0
}

uint8_t hd_host_action_id_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_status,
        uint32_t in_action_id_timestamps,
        uint8_t in_action_id_index
) {

    uint8_t size =  sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint8_t);
    unsigned char *default_value = (unsigned char *) malloc(size);
    default_value[0] = in_status;
    // 填充数据长度 (小端模式)
    default_value[1] = (in_action_id_timestamps >> 0) & 0xFF;  // LSB
    default_value[2] = (in_action_id_timestamps >> 8) & 0xFF;
    default_value[3] = (in_action_id_timestamps >> 16) & 0xFF;
    default_value[4] = (in_action_id_timestamps >> 24) & 0xFF; // MSB

//    if (is_little_endian2()) {
//        memcpy(default_value, &in_action_id_timestamps, sizeof(uint32_t)); // 小端直接复制
//    } else {
//        uint32_t swapped = htonl(in_action_id_timestamps);         // 大端需转换
//        memcpy(default_value, &swapped, sizeof(uint32_t));
//    }
//    uint32_t r = swap_uint32(in_action_id_timestamps);
//    memcpy(default_value, &r, sizeof(uint32_t));
    default_value[5] = in_action_id_index;
    int ret = hd_camera_protocol_encode(
            out_protocol_data,
            out_protocol_data_size,
            in_slave_addr,
            CMD_HD_BROADCAST_ACTION_ID,
            size,
            default_value
    );
    free(default_value);
    return ret;
}

uint8_t hd_slave_action_id_decode(
        uint8_t *out_status,
        uint32_t *out_action_id_timestamps,
        uint8_t *out_action_id_index,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
) {

    if (in_payload_data == NULL || in_payload_data_size != 6) {
        return -1;
    }

    *out_status = in_payload_data[0];
    // 解析数据（小端序）
    *out_action_id_timestamps = ((uint32_t) in_payload_data[4] << 24) |
                                ((uint32_t) in_payload_data[3] << 16) |
                                ((uint32_t) in_payload_data[2] << 8) |
                                ((uint32_t) in_payload_data[1]);
    *out_action_id_index = in_payload_data[5];
    return 0;
}
