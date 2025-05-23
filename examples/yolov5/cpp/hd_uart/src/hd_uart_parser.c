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

// 当前从机地址
static uint8_t g_addr = 0;
// 当前照片存储目录
static const char *g_pic_dir_path = NULL;
static volatile int g_running = 0;
static pthread_t g_uart_t;
static hd_on_action_id_changed g_hd_on_action_id_changed = NULL;
static hd_debug_on_resp_changed g_hd_debug_on_resp_changed = NULL;

// 写串口
static int do_uart_write(const unsigned char *raw, size_t raw_size) {
    //hd_camera_protocol_print_buffer(raw, raw_size, "hd_uart_write");
    g_hd_debug_on_resp_changed(raw, raw_size);
    return -1;
}

/*
// 解析图片
// 图片的格式:
// 文件名格式：prefix_TIMESTAMP_PICID.jpg
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

    // 查找第一个和第二个'_'
    const char *first_underscore = strchr(file_name, '_');
    const char *second_underscore = first_underscore ? strchr(first_underscore + 1, '_') : NULL;
    const char *last_dot = strrchr(file_name, '.');

    // 基础格式检查
    if (!first_underscore || !second_underscore || !last_dot ||
        second_underscore >= last_dot) {
        return -2;
    }

    /* 解析时间戳（第一个_和第二个_之间的部分） */
    char *timestamp_end;
    errno = 0;
    long timestamp = strtol(first_underscore + 1, &timestamp_end, 10);

    // 检查时间戳转换结果
    if ((errno == ERANGE && (timestamp == LONG_MAX || timestamp == LONG_MIN))) {
        return -3; // 数值溢出
    }
    if (timestamp_end != second_underscore) {  // 关键修改点
        return -4; // 时间戳未正确终止于第二个_
    }
    if (timestamp < 0 || timestamp > UINT32_MAX) {
        return -5;
    }

    /* 解析图片ID（第二个_和.之间的部分） */
    char *pic_id_end;
    errno = 0;
    long id = strtol(second_underscore + 1, &pic_id_end, 10);

    if ((errno == ERANGE && (id == LONG_MAX || id == LONG_MIN)) ||
        pic_id_end != last_dot ||  // 关键修改点
        id < 0 || id > UINT8_MAX) {
        return -6;
    }

    *snapshot_timestamps = (uint32_t) timestamp;
    *pic_id = (uint8_t) id;
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

                LOGD("获取图片md5...\n");
                snprintf(file_path_tmp, sizeof(file_path_tmp), "%s/%s", action_id_path, pic_file_name);
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

/* 处理 3.9拉取图片（0x09）*/
static int
handle_pull_pic(const unsigned char *payload_data,
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
    unsigned char *result;
    size_t size;

//    ret = do_cut_file_data(filePath, &result, &size, 246*1024, out_read_len);
    ret = do_cut_file_data(filePath, &result, &size, out_offset, out_read_len);
    if (ret) {
        return -1;
    }
    LOGD("需要拉取的图片数据信息：\n");
    LOGD("实际读取的文件数据大小size        :       %zu(0x%02zx)\n", size, size);
    if (size < out_read_len) {
        LOGD("文件读到结尾了。\n");
    }

    ret = hd_slave_pull_pic_encode(protocol_data_out, protocol_data_size_out, g_addr, 1, result, size);
    if (ret) {
        LOGD("hd_slave_pull_pic_encode fail！\n");
        return -1;
    }
    return 0;
}

/* 处理 3.16 查询摄像头图片信息（0x1D）*/
static int
handle_pic_infos(const unsigned char *payload_data, uint32_t payload_data_size, unsigned char **protocol_data_out,
                 uint32_t *protocol_data_size_out
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


    ret = hd_slave_pic_info_encode(protocol_data_out, protocol_data_size_out, g_addr, pic_infos,
                                   pic_infos_size);

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
    free(filePath);
    return ret == 0 ? 0 : -2;
}

/* 处理 3.10 图片拉取完成（0x0A）*/
static int handlePullPicComplete(const unsigned char *payload_data, uint32_t payload_data_size) {
    // 图片拉取完成 需要做什么？
    return 0;
}

int handle_uart_data(const unsigned char *raw, size_t raw_size) {
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
        case CMD_BROADCAST_ACTION_ID: {
            LOGI("CMD 3.17 广播门开事件（0x1E）\n");
            uint32_t out_action_id_timestamps;
            uint8_t out_action_id_index;
            ret = hd_slave_action_id_decode(&out_action_id_timestamps, &out_action_id_index, payload_data_out,
                                            payload_data_size_out);
            if (ret) {
                LOGD("hd_slave_action_id_decode error \n");
                return -1;
            }
            if (g_hd_on_action_id_changed != NULL) {

                do_action_id_2_str(buff, sizeof(buff), out_action_id_timestamps, out_action_id_index);
                LOGI("action_id = %s\n",buff);
                g_hd_on_action_id_changed(buff);
            }
            // 广播不需要响应
            break;
        }
        case CMD_PIC_PULL_COMPLETED: {
            LOGI("CMD 3.10 图片拉取完成（0x0A）\n");
            ret = handlePullPicComplete(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            ret = hd_slave_pull_pic_complete_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                                    ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                LOGD("hd_slave_pull_pic_complete_encode error\n");
                return -1;
            }
            do_uart_write(out_protocol_data, out_protocol_data_size);
            break;
        }

        case CMD_PIC_DELETE: {
            LOGI("CMD 3.8 删除图片（0x08）\n");
            int delete_ret = handle_delete_pic(payload_data_out, payload_data_size_out);
            unsigned char *out_protocol_data;
            uint32_t out_protocol_data_size;
            ret = hd_slave_delete_pic_encode(&out_protocol_data, &out_protocol_data_size, slave_addr_out,
                                             delete_ret == 0 ? PROTOCOL_UART_SUCCESS : PROTOCOL_UART_FAIL);
            if (ret) {
                LOGD("hd_slave_delete_pic_encode error\n");
                return -1;
            }
            do_uart_write(out_protocol_data, out_protocol_data_size);
            break;
        }
        case CMD_COMMON_PIC_INFO: {
            LOGI("CMD 3.16 查询摄像头图片信息（0x1D）\n");
            unsigned char *protocol_data_out;
            uint32_t protocol_data_size_out;
            ret = handle_pic_infos(payload_data_out, payload_data_size_out, &protocol_data_out,
                                   &protocol_data_size_out);
            if (ret == 0) {
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGD("do_uart_write error \n");
                    return -1;
                }
            }
            break;
        }

        case CMD_PIC_PULL: {
            LOGI("CMD 3.9拉取图片（0x09）\n");
            unsigned char *protocol_data_out;
            uint32_t protocol_data_size_out;
            ret = handle_pull_pic(payload_data_out, payload_data_size_out, &protocol_data_out, &protocol_data_size_out);
            if (ret == 0) {
                ret = do_uart_write(protocol_data_out, protocol_data_size_out);
                if (ret) {
                    LOGD("do_uart_write error \n");
                    return -1;
                }
            }
            free(protocol_data_out);
            break;
        }

        default:
            LOGI("CMD 暂不支持的CMD:%d\n", cmd_out);
            break;

    }

    return 0;
}

static void *work_thread(void *arg) {
    while (g_running) {
        // LOGD("work_thread running ...\n");
        sleep(1);

    }
    LOGD("work_thread end\n");
    return NULL;
}

int hd_uart_init(const char *pic_dir_path,
                 uint8_t addr,
                 hd_on_action_id_changed callback,
                 hd_debug_on_resp_changed on_resp_changed
) {
    LOGI("hd_uart_init\n");
    LOGI("hd_uart_init addr             :       <%d> \n", addr);
    LOGI("hd_uart_init pic_dir_path     :       <%s> \n", pic_dir_path);
    g_pic_dir_path = pic_dir_path;
    g_addr = addr;
    g_hd_on_action_id_changed = callback;
    g_hd_debug_on_resp_changed = on_resp_changed;
    g_running = 1;
    pthread_create(&g_uart_t, NULL, work_thread, NULL);
    return 0;
}


void hd_uart_deinit() {
    LOGI("hd_uart_deinit\n");
    g_hd_on_action_id_changed = NULL;
    g_running = 0;
    pthread_join(g_uart_t, NULL);

}