
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <ctype.h>
#include "hd_camera_protocol.h"
#include "hd_camera_protocol_cmd.h"
#include "hd_utils.h"
#include <sys/time.h>
#include <fcntl.h>
#include "hd_camera_shell.h"
#include "hd_queue.h"
#include <signal.h>
#include "hd_camera_ota.h"
#include "hd_camera.h"


#define hd_camera_ota_debug                     0
#define hd_camera_ota_tag                       "hd_camera_ota.c"

#define HD_FILE_PUSH_TIMEOUT                    10                          // 接受上传文件超时时间
#if(CONTEXT)
#define MODEL_DIR_PATH                          "/Users/xiangpengle/CLionProjects/hd_camera/test_case/oem/usr/shared"
#define MODEL_PREFIX                            ".rknn"
#define MODEL_PREFIX_DOWNLOADING                ".downloading"
#define MODEL_DEST_PATH                         "/Users/xiangpengle/Downloads"
#else
#define MODEL_DIR_PATH                          "/oem/usr/share"            // 模型位置
#define MODEL_PREFIX                            ".rknn"                     // 模型文件格式
#define MODEL_PREFIX_DOWNLOADING                ".downloading"              // 模型正在下载
#define MODEL_DEST_PATH                         "/userdata"                 // 模型下载文件夹
#endif

static pthread_t g_hd_push_progress_pthread_t;
static pthread_t g_hd_push_t;
static uint8_t g_addr;

static volatile int g_running = 0;
static HDBlockingQueueUint8 *g_hd_push_frame_queue = NULL;
static char g_hd_push_mode_file_path[2048];                     // hd_push文件path
static char g_hd_push_mode_file_path_downloading[2048];         // hd_push文件path
static size_t g_hd_push_recv_count = 0;
static volatile size_t g_hd_push_mode_file_size = 0;                  // hd_push文件size
static unsigned char g_hd_push_mode_file_md5[16];               // hd_push文件md5
static FILE *g_hd_push_mode_file = NULL;                        // hd_push file
static struct timespec g_hd_push_start;

typedef struct {
    unsigned char *data;
    uint32_t data_size;
} hd_frame_data;

static void start_push_timeout();

static void stop_push_timeout();

static void hd_camera_ota_consume(uint8_t data);

static void on_hd_push_delete_and_reply(int result);

static void hd_camera_ota_reset(const char *tag);

static void start_push_timeout() {

}

static void stop_push_timeout() {

}

static void hd_camera_ota_timeout_handler(int sig) {
    hd_camera_ota_reset("time_out");
}


static void hd_camera_ota_reset_internal(const char *dir_path, const char *end) {
    DIR *dir;
    struct dirent *entry;
    char filepath[1024];

    dir = opendir(dir_path);
    if (dir == NULL) {
        LOGW("hd_camera_ota_reset_internal 无法打开目录\n");
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        // 检查文件后缀
        char *ext = strrchr(entry->d_name, '.');
        if (ext != NULL && strcmp(ext, end) == 0) {
            // 构建完整文件路径
            snprintf(filepath, sizeof(filepath), "%s/%s", dir_path, entry->d_name);

            // 删除文件
            if (remove(filepath)) {
                LOGW("删除文件失败");
            } else {
                LOGI("已删除: %s\n", filepath);
            }
        }
    }

    closedir(dir);
}

int hd_camera_ota_version(
        uint8_t property_id_out,
        const unsigned char *payload_data, uint32_t payload_data_size,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
) {
    int ret;
    LOGD("[%d获取模型名称]%02x\n", g_addr, property_id_out);
    // 查找模型
    char version[1024];
    ret = hd_find_model_name(MODEL_DIR_PATH, version, MODEL_PREFIX);
    if (ret) {
        return ret;
    }
    LOGD("模型名称: %s\n", version);
    size_t len = strlen(version);
    unsigned char *result = (unsigned char *) malloc(len);
    if (!result) {
        return 1;
    }
    memcpy(result, version, len);
//            result[len] = '\0';
    ret = hd_slave_property_get_encode(protocol_data_out, protocol_data_size_out,
                                       g_addr, property_id_out, 0, result, len);
    free(result);
    return ret;
}


void hd_camera_ota_model_recv(uint8_t str) {
    size_t total_size = g_hd_push_mode_file_size;
    if (total_size <= 0) {
        return;
    }
    hd_queue_put_uint8(g_hd_push_frame_queue, str);
}

/**
 * 见 handle_hd_push_file
 */
static void hd_camera_ota_consume(uint8_t data) {
    stop_push_timeout();
    if (g_hd_push_mode_file == NULL) {
        LOGD("[升级模型]g_hd_push_mode_file == null \n");
        return;
    }
    if (g_hd_push_recv_count < g_hd_push_mode_file_size) {
        size_t written = fwrite(&data, sizeof(uint8_t), 1, g_hd_push_mode_file);
        if (written != 1) {
            LOGW("[升级模型] Failed to write byte\n");
            hd_camera_ota_reset("fwrite");
            on_hd_push_delete_and_reply(1);

            return;
        }
    }

    g_hd_push_recv_count++;

    if (g_hd_push_recv_count == g_hd_push_mode_file_size) {
        g_hd_push_mode_file_size = 0;
        g_hd_push_recv_count = 0;
        stop_push_timeout();
        usleep(100);
        if (g_hd_push_mode_file == NULL) {
            LOGD("[升级模型]g_hd_push_mode_file == null \n");
            return;
        }
        if (g_hd_push_mode_file != NULL) {
            fflush(g_hd_push_mode_file);  // 确保所有缓冲数据写入文件
            fclose(g_hd_push_mode_file);
            g_hd_push_mode_file = NULL;
        }

        usleep(10 * 1000);
        printf("\n");
        LOGD("[升级模型]接受完毕！文件：%s ，大小：%d\n", g_hd_push_mode_file_path_downloading, g_hd_push_mode_file_size);
        hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);
        LOGD("[升级模型]开始校验 ... <%s>\n", g_hd_push_mode_file_path_downloading);
        // 校验md5
        unsigned char result[16];
        int ret = hd_md5(g_hd_push_mode_file_path_downloading, result);
        if (ret) {
            LOGW("[升级模型]md5生成 fail :%d\n", ret);
            on_hd_push_delete_and_reply(4);
            memset(g_hd_push_mode_file_path, 0, sizeof(g_hd_push_mode_file_path));
            memset(g_hd_push_mode_file_path_downloading, 0, sizeof(g_hd_push_mode_file_path_downloading));
            memset(g_hd_push_mode_file_md5, 0, sizeof(g_hd_push_mode_file_md5));
            return;
        }
        hd_printf_buff(result, 16, "md5", 0);
        if (hd_array_cmp(result, 16, g_hd_push_mode_file_md5, 16) == 0) {
            LOGD("[升级模型]修改名称！\n");
            ret = rename(g_hd_push_mode_file_path_downloading, g_hd_push_mode_file_path);
            if (ret == 0) {
                LOGD("[升级模型] hd push model success！\n");
                on_hd_push_delete_and_reply(0);
            } else {
                LOGD("[升级模型] hd push model fail！\n");
                on_hd_push_delete_and_reply(5);
            }
        } else {
            LOGW("[升级模型]md5不同\n");
            on_hd_push_delete_and_reply(4);
        }
        memset(g_hd_push_mode_file_path, 0, sizeof(g_hd_push_mode_file_path));
        memset(g_hd_push_mode_file_path_downloading, 0, sizeof(g_hd_push_mode_file_path_downloading));
        memset(g_hd_push_mode_file_md5, 0, sizeof(g_hd_push_mode_file_md5));


        double time_used;
        struct timespec g_hd_push_end;
        clock_gettime(CLOCK_MONOTONIC, &g_hd_push_end);
        time_used = (g_hd_push_end.tv_sec - g_hd_push_start.tv_sec) * 1e9;  // 秒转纳秒
        time_used += (g_hd_push_end.tv_nsec - g_hd_push_start.tv_nsec);     // 加上纳秒部分
        time_used /= 1e6;                               // 转换为毫秒

        printf("[升级模型]耗时: %f 毫秒\n", time_used);

        LOGW("[升级模型]准备重启！\n");
        sleep(1);
        system("reboot");
    } else if (g_hd_push_recv_count > g_hd_push_mode_file_size) {
        printf("\n");
        LOGW("[升级模型]file_size不同 %d\n", g_hd_push_recv_count);
        hd_camera_ota_reset("size_error");
        g_hd_push_recv_count = 0;
        on_hd_push_delete_and_reply(3);
    } else {
        start_push_timeout();
    }
}


static void hd_camera_ota_reset(const char *tag) {
    LOGD("resetHDPushMode -> %s\n", tag);
    if (g_hd_push_mode_file != NULL) {
        fclose(g_hd_push_mode_file);
        g_hd_push_mode_file = NULL;
    }
//    memset(g_hd_push_mode_file_path_downloading, 0, sizeof(g_hd_push_mode_file_path_downloading));
//    memset(g_hd_push_mode_file_path, 0, sizeof(g_hd_push_mode_file_path));
//    memset(g_hd_push_mode_file_md5, 0, sizeof(g_hd_push_mode_file_md5));
    g_hd_push_mode_file_size = 0;
    hd_camera_change_serial_mode(HD_SERIAL_NORMAL_MODE);


}


// ************************************************************************************************************************
// ************************************************************************************************************************
// ************************************************************************************************************************

static void *hd_camera_ota_progress_thread(void *arg) {
    LOGD("hd_camera_ota_progress_thread start...\n");
    while (g_running && g_hd_push_mode_file_size > 0) {
        sleep(3);
        if (g_hd_push_mode_file_size != 0) {
            double progress;
            if (g_hd_push_recv_count >= g_hd_push_mode_file_size) {
                progress = 100.0;
            } else {
                progress = 100.0 * (1.0 * ((double) g_hd_push_recv_count) / ((double) g_hd_push_mode_file_size));
            }
            printf("更新模型进度：[%3d%%] (%zu/%zu)\n", (int) progress, g_hd_push_recv_count, g_hd_push_mode_file_size);
        }
    }
    LOGD("hd_camera_ota_thread end...\n");

    return NULL;
}

static void *hd_camera_ota_thread(void *arg) {
    LOGD("hd_camera_ota_thread start...\n");
    while (g_running) {
        uint8_t item = hd_queue_take_uint8(g_hd_push_frame_queue);
        hd_camera_ota_consume(item);
    }
    LOGD("hd_camera_ota_thread end.\n");
    return NULL;
}


static void on_hd_push_delete_and_reply(int result) {
    unsigned char *out_protocol_data = NULL;
    uint32_t out_protocol_data_size;
    uint8_t in_type = 0x01;
    uint32_t offset = 0;
    int ret = hd_slave_file_encode(&out_protocol_data, &out_protocol_data_size,
                                   g_addr, in_type, result, offset);

    if (ret) {
        LOGW("on_hd_push_delete_and_reply error = %d\n", ret);
        return;
    }
    hd_camera_uart_write(out_protocol_data, out_protocol_data_size);
    free(out_protocol_data);
}

int hd_camera_ota_model_handle_cmd(
        unsigned char *payload_data,
        uint32_t payload_data_size
) {
    // 解析
    LOGD("[hd_camera_ota]\n");
    uint8_t type;
    uint32_t file_size;
    static unsigned char file_md5[16] = {0};
    static char file_name[2048] = {0};
    int ret;
    ret = hd_slave_file_decode_payload(&type, &file_size, file_md5, file_name, payload_data, payload_data_size);
    if (ret) {
        LOGW("[hd_camera_ota] hd_slave_file_decode_payload error = %d\n", ret);
        return 1;
    }
    switch (type) {
        case 0x01: {
            if (g_hd_push_mode_file != NULL) {
                LOGW("[hd_camera_ota] doing ....\n");
                break;
            }
            usleep(100);
            hd_camera_ota_reset("init");
            if (file_size <= 0) {
                LOGW("[hd_camera_ota] hd_slave_file_decode_payload file_size error\n");
                break;
            }

            if (strlen(file_name) <= 0) {
                LOGW("[hd_camera_ota] hd_slave_file_decode_payload file_name error\n");
                break;
            }

            LOGD("============ 准备接受文件的信息a ============ \n");
            LOGD("type           :           %02x \n", type);
            LOGD("file_size      :           %02x(%d) \n", file_size, file_size);
            LOGD("file_md5       :           ");
            hd_printf_buff(file_md5, 16, "md5", 0);
            LOGD("file_name      :           %s \n", file_name);

            LOGD("============ 准备接受文件的信息z ============\n");
            g_hd_push_mode_file_size = file_size;
            g_hd_push_recv_count = 0;
            memcpy(g_hd_push_mode_file_md5, file_md5, 16);
            snprintf(g_hd_push_mode_file_path, sizeof(g_hd_push_mode_file_path), "%s/%s%s", MODEL_DEST_PATH, file_name,
                     MODEL_PREFIX);
            snprintf(g_hd_push_mode_file_path_downloading, sizeof(g_hd_push_mode_file_path_downloading), "%s%s",
                     g_hd_push_mode_file_path, MODEL_PREFIX_DOWNLOADING);
            // 创建文件夹，准备接受数据
            LOGD("[hd_camera_ota] g_hd_push_mode_file_path_downloading      :           %s \n",
                 g_hd_push_mode_file_path_downloading);
            ret = create_directory_if_not_exists(g_hd_push_mode_file_path_downloading);
            if (ret) {
                LOGW("[hd_camera_ota] create_directory_if_not_exists fail.\n");
                break;
            }
            delete_file_if_exists(g_hd_push_mode_file_path_downloading);
            usleep(10);
            g_hd_push_mode_file = fopen(g_hd_push_mode_file_path_downloading, "wb");  // 二进制写入模式
            if (!g_hd_push_mode_file) {
                LOGD("<<<<打开文件失败 g_hd_push_mode_file ==  null>>>>\n");
                break;
            }
            LOGD("<<<<打开文件成功 准备接受数据>>>>\n");
            hd_camera_change_serial_mode(HD_SERIAL_HD_PUSH_MODE);
            usleep(1 * 1000);
            // 回复
            unsigned char *out_payload;
            uint32_t out_payload_size;
            uint8_t int_type = 0x01;
            uint8_t int_result = 0;
            ret = hd_slave_file_encode_payload(&out_payload, &out_payload_size, int_type, int_result, 0);
            if (ret) {
                LOGW("[hd_camera_ota] hd_slave_file_encode_payload error = %d\n", ret);
                hd_camera_ota_reset("hd_slave_file_encode_payload");
                break;
            }
            unsigned char *out_p;
            uint32_t out_p_size;
            ret = hd_camera_protocol_encode(&out_p, &out_p_size, g_addr, CMD_HD_PUSH_FILE, out_payload_size,
                                            out_payload);
            free(out_payload);
            if (ret) {
                LOGW("[hd_camera_ota] hd_camera_protocol_encode error = %d\n", ret);
                hd_camera_ota_reset("hd_camera_protocol_encode");
                break;
            }
            ret = hd_camera_uart_write(out_p, out_p_size);
            free(out_p);
            if (ret) {
                LOGW("[hd_camera_ota] hd_camera_uart_write error = %d\n", ret);
                hd_camera_ota_reset("hd_camera_uart_write");
                break;
            }
            LOGD("[hd_camera_ota] <%s>(%d)等待上传至<%s> ...\n", file_name, file_size, g_hd_push_mode_file_path_downloading);
            // 准备接受数据 do_uart_recv_with_hd_push
            pthread_create(&g_hd_push_progress_pthread_t, NULL, hd_camera_ota_progress_thread, NULL);
            start_push_timeout();
            clock_gettime(CLOCK_MONOTONIC, &g_hd_push_start);
            break;
        }

        default:
            break;
    }
    return 0;
}

int hd_camera_ota_init(uint8_t addr) {
//    signal(SIGALRM, timeout_handler);

    hd_camera_ota_reset_internal(MODEL_DEST_PATH, MODEL_PREFIX_DOWNLOADING);
    g_addr = addr;
    g_hd_push_frame_queue = hd_queue_create_uint8(10 * 1024);
    if (g_hd_push_frame_queue == NULL) {
        LOGE("createQueue g_hd_push_frame_queue error!");
        return -1;
    }
    g_running = 1;
    pthread_create(&g_hd_push_t, NULL, hd_camera_ota_thread, NULL);
    return 0;
}


void hd_camera_ota_deinit() {
    g_running = 0;
    g_addr = -1;
    if (g_hd_push_t) {
        pthread_join(g_hd_push_t, NULL);
    }

    if (g_hd_push_progress_pthread_t) {
        pthread_join(g_hd_push_progress_pthread_t, NULL);
    }
    hd_queue_destroy_uint8(g_hd_push_frame_queue);
    g_hd_push_frame_queue = NULL;
}
