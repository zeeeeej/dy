
#ifndef __HD_CAMERA_PROTOCOL_CMD__
#define __HD_CAMERA_PROTOCOL_CMD__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#define HEARTBEAT_INTERVAL              5           // 单位:秒

// cmds
#define CMD_HEARTBEAT                   0x01
#define CMD_QJY_PROPERTY_GET            0x02
#define CMD_QJY_PROPERTY_SET            0x03
#define CMD_RECOVERY                    0x04
#define CMD_CAMERA_REBOOT               0x05
#define CMD_QJY_CAMERA_SNAPSHOT         0x06
#define CMD_QJY_PIC_INFO                0x07
#define CMD_QJY_PIC_DELETE              0x08
#define CMD_QJY_PIC_PULL                0x09
#define CMD_QJY_PIC_PULL_COMPLETED      0x0A
#define CMD_OTA                         0x0B
#define CMD_OTA_PUSH                    0x0C
#define CMD_DOOR_SIGNAL                 0x0D
#define CMD_APP_UPGRADE                 0x1B
#define CMD_APP_UPGRADE_PUSH            0x1C
#define CMD_HD_PROPERTY_GET             0xC2
#define CMD_HD_PROPERTY_SET             0xC3
#define CMD_HD_CAMERA_SNAPSHOT          0xC6
#define CMD_HD_PIC_INFO                 0xC7
#define CMD_HD_PIC_DELETE               0xC8
#define CMD_HD_PIC_PULL                 0xC9
#define CMD_HD_PIC_PULL_COMPLETED       0xCA
#define CMD_HD_BROADCAST_ACTION_ID      0xCD


typedef struct {
    /** pic_id 1BYTE */
    uint8_t id;
    /** action_id timestamps 4BYTE */
    uint32_t action_id_timestamps;
    /** action_id index BYTE */
    uint8_t action_id_index;
    /** 触发方式 0x00:陀螺仪触发；0x01:主动触发 */
    uint8_t trigger_type;
    /** 触发角度 */
    uint8_t trigger_angel;
    /** 抓取时间 4BYTE */
    uint32_t snapshot_timestamps;
    /** 图片大小 4BYTE */
    uint32_t size;
    /** MD5 16BYTE */
    unsigned char md5[16];
} hd_dynamic_pic_info;

static size_t hd_dynamic_pic_info_real_size() {
    return
            sizeof(uint8_t) // id
            + sizeof(uint32_t) //  action_id_timestamps
            + sizeof(uint8_t) // action_id_index
            + sizeof(uint8_t) // trigger_type
            + sizeof(uint8_t) // trigger_angel
            + sizeof(uint32_t) // snapshot_timestamps
            + sizeof(uint32_t) // size
            + 16; // md5
}

void hd_dynamic_pic_info_print(const hd_dynamic_pic_info **infos, size_t size);

// 交换32位整数的字节序
static uint32_t swap_uint32(uint32_t val) {
    return ((val << 24) & 0xff000000) |
           ((val << 8) & 0x00ff0000) |
           ((val >> 8) & 0x0000ff00) |
           ((val >> 24) & 0x000000ff);
}


/* ************* */
/* <0x01 心跳> */
/* ************* */
/**
 * 心跳 请求
 *
 * ｜ 0x01 | 0x01 | N |
 *
 * @param in_slave_addr        从机地址
 * @param in_ack_number        请求序号
 */
uint8_t hd_host_heartbeat_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_ack_number
);

/**
 * 心跳 应答
 *
 * ｜ 0x01 | 0x01 | N+1 |
 *
 * @param in_slave_addr        从机地址
 * @param in_ack_number        返回序号 = 请求序号+1
 */
uint8_t hd_slave_heartbeat_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_ack_number
);

/**
 * 解析心跳 请求
 * @param payload_data_in           payload
 * @param payload_data_size_in      payload size
 * @param ack_number_out            序号
 * @return
 */
uint8_t hd_slave_heartbeat_decode(
        uint8_t *out_ack_number,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

/**
 * 解析心跳 应答
 * @param in_payload_data          整条协议数据
 * @param in_payload_data_size     整条协议数据大小
 * @param out_ack_number            序号
 * @return
 */
uint8_t hd_host_heartbeat_decode(
        uint8_t *out_ack_number,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size

);


/* ************* */
/* <0x02 查询属性> */
/* ************* */
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
);

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
);


/**
 * 查询属性 请求
 *
 * | 0xC2 | 0x01 | ID |
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
);

/**
 * 查询属性 应答
 *
 * | 0xC2 | len | ID | result | value |
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
);

/**
 * 查询属性 应答 解析
 * @param in_payload_data
 * @param in_payload_data_size
 * @param out_property_id
 * @param out_result
 * @param out_property_value
 * @param out_property_value_size
 * @return
 */
uint8_t hd_host_property_get_decode(
        uint8_t *out_property_id,
        uint8_t *out_result,
        unsigned char **out_property_value,
        uint32_t *out_property_value_size,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size

);

/**
 * 查询属性 请求 解析
 * @param in_payload_data
 * @param in_payload_data_size
 * @param out_property_id
 * @return
 */
uint8_t hd_slave_property_get_decode(
        uint8_t *out_property_id,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size

);

/* ************* */
/* <0x03 设置属性> */
/* ************* */
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

);

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

);

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

);

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
);

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

);

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

);

/* ************* */
/* <0x04 恢复出厂> */
/* ************* */
/**
 * 恢复出厂 请求
 *
 * | 0x04 | 0x00 |
 *
 * @param slave_addr        从机地址
 */
uint8_t hd_camera_protocol_cmd_property_recovery_req(uint8_t slave_addr);

/**
 * 恢复出厂 应答
 *
 * | 0x04| 0x01 | result |
 *
 * @param slave_addr        从机地址
 * @param result            返回值。0x00：成功，其他：失败
 */
uint8_t hd_camera_protocol_cmd_property_recovery_resp(
        uint8_t *slave_addr,
        uint8_t *result
);

/**
 * 重启摄像头 请求
 *
 * | 0x05 | 0x00 |
 *
 * @param slave_addr        从机地址
 */
uint8_t hd_camera_protocol_cmd_property_camera_reboot_req(uint8_t slave_addr);

/**
 * 重启摄像头 应答
 *
 * | 0x05 | 0x00 |
 *
 * @param slave_addr        从机地址
 * @param result            返回值。0x00：成功，其他：失败
 */

uint8_t hd_camera_protocol_cmd_property_camera_reboot_resp(
        uint8_t *slave_addr,
        uint8_t *result
);

/* ************* */
/* <主动抓图> */
/* ************* */
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
);

uint8_t qjy_slave_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        uint8_t in_pic_id
);

uint8_t hd_host_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
);

uint8_t hd_slave_snapshot_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        uint8_t in_pic_id
);

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
);

uint8_t hd_host_snapshot_decode(
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
);


// 将hd_dynamic_pic_info列表转成字符数组
uint8_t hd_dynamic_pic_infos_encode(hd_dynamic_pic_info *infos,
                                    size_t count,
                                    unsigned char **result,
                                    size_t *result_size,
                                    uint8_t is_big_endian
);

/* 查询图片 */
/**
 * 查询摄像头存贮的图片信息 请求
 *
 * | 0x07 | 0x00 |
 *
 * @param protocol_data_out
 * @param protocol_data_size_out
 * @param slave_addr                    从机地址
 */
uint8_t qjy_host_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in
);

uint8_t hd_host_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in
);

uint8_t hd_host_pic_info_decode(
        hd_dynamic_pic_info ***pic_info_out,
        uint8_t *pic_info_size_out,
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
);

/**
 * 查询摄像头存贮的图片信息 应答
 *
 * | 0x07 | len | pic_num | pic_info0 ... pic_infoN |
 *
 * @param slave_addr        从机地址
 * @param len               1 + pic_num * 27
 * @param pic_num           图片数量
 * @param pic_infos         pic_info [ pic_id（1BYTE） ｜ 触发方式（1BYTE） ｜ 触发角度（1BYTE） ｜ 抓取时间（4BYTE） ｜ 图片大小（4BYTE） ｜ MD5（16BYTE） ]
 */
uint8_t qjy_slave_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        hd_dynamic_pic_info *info_in,
        uint8_t info_size_in
);

uint8_t hd_slave_pic_info_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        hd_dynamic_pic_info *info_in,
        uint8_t info_size_in
);

uint8_t hd_slave_pic_info_decode(
        const unsigned char *payload_data_in,
        uint32_t payload_data_size_in
);

/* ************* */
/* <0x1E 广播门开事件> */
/* ************* */
uint8_t hd_host_action_id_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint32_t in_action_id_timestamps,
        uint8_t in_action_id_index
);

uint8_t hd_slave_action_id_decode(
        uint32_t *out_action_id_timestamps,
        uint8_t *out_action_id_index,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);


/* ************* */
/* <0x08 删除图片> */
/* ************* */

uint8_t
hd_host_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id
);

uint8_t
qjy_host_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id
);

uint8_t
hd_slave_delete_pic_decode(
        uint8_t *out_pic_id,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

uint8_t
qjy_slave_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
);

uint8_t
hd_slave_delete_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
);

uint8_t
hd_host_delete_pic_decode(
        uint8_t *out_result,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

/* ************* */
/* <0x09 拉取图片> */
/* ************* */
/**
 * 获取图片 请求
 *
 * | 0x09 | 0x01 | pic_id | offset | read_len |
 *
 * @param out_protocol_data
 * @param out_protocol_data_size
 * @param in_slave_addr        从机地址
 * @param in_pic_id            图片id
 * @param in_offset            表示从图片数据的第offset字节开始读取
 * @param in_read_len          表示读取字节个数
 */
uint8_t
hd_host_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        uint32_t in_offset,
        uint32_t in_read_len);

uint8_t
qjy_host_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_pic_id,
        uint32_t in_offset,
        uint32_t in_read_len);

uint8_t hd_slave_pull_pic_decode(
        uint8_t *out_pic_id,
        uint32_t *out_offset,
        uint32_t *out_read_len,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

/**
 *
 * @param out_protocol_data
 * @param out_protocol_data_size
 * @param in_slave_addr
 * @param in_result
 * @param in_pic_data 数据
 * @return
 */
uint8_t
qjy_slave_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char *in_pic_data,
        size_t in_pic_data_size
);

uint8_t
hd_slave_pull_pic_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result,
        const unsigned char *in_pic_data,
        size_t in_pic_data_size
);

uint8_t hd_host_pull_pic_decode(
        uint8_t *out_result,
        unsigned char **out_pic_data,
        uint32_t *out_pic_data_size,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

/* ************* */
/* <0x0A 拉取图片完成> */
/* ************* */

uint8_t
qjy_host_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
);

uint8_t
hd_host_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr
);

uint8_t
hd_slave_pull_pic_complete_decode(
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);

uint8_t
hd_slave_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
);

uint8_t
qjy_slave_pull_pic_complete_encode(
        unsigned char **out_protocol_data,
        uint32_t *out_protocol_data_size,
        uint8_t in_slave_addr,
        uint8_t in_result
);

uint8_t
hd_host_pull_pic_complete_decode(
        uint8_t *out_result,
        const unsigned char *in_payload_data,
        uint32_t in_payload_data_size
);


/* ********************* */
/* <0x0B 系统固件升级通知> */
/* ********************* */
/**
 * 固件升级通知 请求
 *
 * | 0x0B | 0x14 | fileSize | fileMd5 |
 *
 *  @param slave_addr        从机地址
 *  @param file_size         固件大小。4字节
 *  @param file_md5          固件MD5。16字节
 */
uint8_t hd_host_ota_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint32_t file_size,
        const unsigned char file_md5[16]
);

/**
 * 固件升级通知 应答
 *
 * | 0x0B | 0x05 | result | offset |
 *
 * 相机收到通知后，首次传输固件返回offset=0。如果上次传输中断，MD5与上次一致，返回offset=已接收固件大小，进行断点续传。
 * 主机从第offset字节开始，分包发送固件。
 *
 *  @param slave_addr        从机地址
 *  @param result            返回值。0x00：可以固件升级，其他：升级未准备就绪
 *  @param offset            已接收固件大小（偏移量），范围0~fileSize。用于断点续传。
 */
uint8_t hd_camera_protocol_cmd_property_camera_ota_resp(
        uint8_t *slave_addr,
        uint8_t *result,
        uint32_t *offset
);

/**
 * 发送固件升级包 请求
 *
 * | 0x0C | len | offset | dataLen | data |
 *
 * @param slave_addr        从机地址
 * @param offset            固件数据包偏移量
 * @param data_len          0x000000000：发送完毕;0xFFFFFFFF:  终止OTA升级;其他：固件数据包长度。
 * @param data              固件数据包
 */
uint8_t hd_camera_protocol_cmd_property_camera_ota_push_req(
        uint8_t slave_addr,
        uint32_t offset,
        uint32_t data_len,
        const unsigned char *data
);


/**
 * 发送固件升级包 应答
 *
 * | 0x0C | 0x01 | result |
 *
 * @param slave_addr        从机地址
 * @param result            0x00：完整固件接收完毕。0x01：发送下一包数据；0x02：重发；其他：OTA失败，终止升级
 */
uint8_t hd_camera_protocol_cmd_property_camera_ota_push_resp(
        uint8_t *slave_addr,
        uint8_t *result
);

/**
 * 门开关信号 请求
 *
 * | 0x0D | 0x00 |
 *
 * @param slave_addr        从机地址
 */
uint8_t hd_camera_protocol_cmd_property_camera_door_signal_req(uint8_t slave_addr);

/**
 * 门开关信号 应答
 *
 * | 0x0D| 0x01 | result |
 *
 * @param slave_addr        从机地址
 * @param result            返回值。0x00：门关状态，0x01：门开状态
 */
uint8_t hd_camera_protocol_cmd_property_camera_door_signal_resp(
        uint8_t *slave_addr,
        uint8_t *result
);

/* ********************* */
/* <0x0B app固件升级通知> */
/* ********************* */
/**
 * app升级通知 请求
 *
 * | 0x1B | 0x14 | fileSize | fileMd5 |
 *
 *  @param slave_addr        从机地址
 *  @param file_size         app大小。4字节
 *  @param file_md5          appMD5。16字节
 */
uint8_t hd_host_app_upgrade_encode(
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out,
        uint8_t slave_addr_in,
        uint32_t file_size,
        const unsigned char file_md5[16]
);

/**
 * app升级通知 应答
 *
 * | 0x1B | 0x05 | result | offset |
 *
 * 相机收到通知后，首次传输app返回offset=0。如果上次传输中断，MD5与上次一致，返回offset=已接收app大小，进行断点续传。
 * 主机从第offset字节开始，分包发送app。
 *
 *  @param slave_addr        从机地址
 *  @param result            返回值。0x00：可以app升级，其他：升级未准备就绪
 *  @param offset            已接收app大小（偏移量），范围0~fileSize。用于断点续传。
 */
uint8_t hd_camera_protocol_cmd_property_camera_app_upgrade_resp(
        uint8_t *slave_addr,
        uint8_t *result,
        uint32_t *offset
);

/**
 * 发送app升级包 请求
 *
 * | 0x1C | len | offset | dataLen | data |
 *
 * @param slave_addr        从机地址
 * @param offset            app数据包偏移量
 * @param data_len          0x000000000：发送完毕;0xFFFFFFFF:  终止OTA升级;其他：app数据包长度。
 * @param data              app数据包
 */
uint8_t hd_camera_protocol_cmd_property_camera_app_upgrade_push_req(
        uint8_t slave_addr,
        uint32_t offset,
        uint32_t data_len,
        unsigned char *data
);

/**
 * 发送app升级包 应答
 *
 * | 0x1C | 0x01 | result |
 *
 * APP升级通知（0x1B）收到应答后，不再先发送请求，而是直接给摄像头发送升级包原始数据，摄像头接收完原始数据后仍需应答告知升级结果.
 *
 * @param slave_addr        从机地址
 * @param result            0x00：完整app接收完毕。0x01：发送下一包数据；0x02：重发；其他：OTA失败，终止升级
 */
uint8_t hd_camera_protocol_cmd_property_camera_app_upgrade_push_resp(
        uint8_t *slave_addr,
        uint8_t *result
);


#ifdef __cplusplus
}
#endif

#endif // __HD_CAMERA_PROTOCOL_CMD__
