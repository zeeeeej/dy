
#include "hd_c_log.h"

//<editor-fold desc="拍摄">
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include "photo.h"
#include "hd_camera_protocol.h"
#include "mpu6887p.h"
#include "hd_queue.h"
#include "hd_utils.h"
#include <errno.h>

typedef struct {
    uint8_t pic_id;
    uint32_t timestamp;
    char *file_name;
} SnapshotItem;

typedef struct {
    uint32_t action_id_timestamp;
    uint8_t action_id_index;
    pthread_t snap_t;
    int running;
    int triggerAngel;
    int cameraType;
    SnapshotItem **pics;
    int pic_count;
    char *name;
    uint8_t trigger_type;
} SnapshotTask;

typedef struct {
    uint32_t action_id_timestamp;
    uint8_t action_id_index;
    int triggerAngel;
    uint8_t triggerType;
    int cameraType;
    SnapshotItem **pics;
    int pic_count;
    char *action_id_name;
} SnapshotResource;

void SnapshotItem_free(SnapshotItem *item) {
    if (!item)return;
    if (item->file_name) {
        free(item->file_name);
        item->file_name = NULL;
    }
}

void SnapshotTask_free(SnapshotTask *task) {
    if (!task)return;
    if (task->pics) {
        for (int i = 0; i < task->pic_count; ++i) {
            free(task->pics[i]);
            task->pics[i] = NULL;
        }
        free(task->pics);
        task->pics = NULL;
        if (task->name) {
            free(task->name);
            task->name = NULL;

        }
    }
    free(task);
    task = NULL;
}

void SnapshotResource_free(SnapshotResource *res) {
    if (!res)return;
    if (res->pics) {
        for (int i = 0; i < res->pic_count; ++i) {
            free(res->pics[i]);
            res->pics[i] = NULL;
        }
        free(res->pics);
        res->pics = NULL;
    }
    if (res->action_id_name) {
        free(res->action_id_name);
        res->action_id_name = NULL;
    }
}

#define CAMERA_TYPE_S       0       // 静态类型摄像头
#define CAMERA_TYPE_D       1       // 动态类型摄像头
#define MAX_PICS            100     // 单次开门事件图片最大数量
#define SNAPSHOT_COUNT      3      // 每秒拍多少张
#define SNAP_SRC            "src"

static int (*g_callback)(char *, char **, int) = NULL;

static int (*g_transform_pic)(const char *, char *) = NULL;

static HDBlockingQueue *g_queue;
static volatile uint8_t g_running = 0;
static uint8_t g_addr = 0;
static uint8_t debug = 1;
static char *tag = "hd_c_log.c";
static pthread_mutex_t g_lock;
static SnapshotTask *current_task;
static uint16_t *g_pic_id;
static uint8_t *snap_pic_id;
static char g_src_path[1024];
static char g_dst_path[1024];
static char g_demo_path[1024];

static pthread_mutex_t my_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t my_cond = PTHREAD_COND_INITIALIZER;
static int my_ready = 0;  // 条件变量

static uint16_t sem_pic_id;

// TODO 合并hd_uart_parser.c里的do_action_id_2_str
static int do_action_id_2_str(char *str, size_t str_size, uint32_t action_id_timestamps, uint8_t action_id_index) {
    snprintf(str, str_size, "%d%03d", action_id_timestamps, action_id_index);
    return 0;
}

static int mkdir2(const char *dir_path) {
    // 检查目录是否存在（F_OK 检查文件是否存在）
    if (access(dir_path, F_OK)) {
        // 目录不存在，尝试创建
        if (mkdir(dir_path, 0755) == 0) {
            printf("Directory created: %s\n", dir_path);
        } else {
            perror("mkdir failed");
            return 1;
        }
    } else {
        printf("Directory already exists: %s\n", dir_path);
    }
    return 0;
}


/**
 * 获取文件大小(字节数)
 * @param file_path 文件路径
 * @return 文件大小(字节)，出错返回-1
 */
static long get_file_size(const char *file_path) {
    struct stat st;
    printf("get_file_size文件：%s\n", file_path);
    if (stat(file_path, &st) == 0) {
        return st.st_size;
    }
    perror("获取文件大小失败");

    return -1;
}


static void mkdir_recursive(const char *path) {
    char tmp[256];
    char *p = NULL;
    size_t len;

    snprintf(tmp, sizeof(tmp), "%s", path);
    len = strlen(tmp);

    // 去掉末尾的 '/'
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = '\0';
    }

    // 逐级创建目录
    for (p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';  // 临时截断路径
            mkdir(tmp, 0755);  // 尝试创建
            *p = '/';   // 恢复路径
        }
    }

    // 创建最终目录
    mkdir(tmp, 0755);
}

//<editor-fold desc="debug">
// 复制文件函数
static int BUFFER_SIZE = 4096;
//</editor-fold>

static int copy_file(const char *src_path, const char *dest_path) {
    FILE *src_file = fopen(src_path, "rb");
    if (src_file == NULL) {
        perror("无法打开源文件");
        return -1;
    }

    create_directory_if_not_exists(dest_path);

    FILE *dest_file = fopen(dest_path, "wb");
    if (dest_file == NULL) {
        printf("error : %s\n", dest_path);
        perror("无法创建目标文件");
        fclose(src_file);
        return -1;
    }

    char buffer[BUFFER_SIZE];
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, BUFFER_SIZE, src_file)) > 0) {
        if (fwrite(buffer, 1, bytes_read, dest_file) != bytes_read) {
            perror("写入目标文件失败");
            fclose(src_file);
            fclose(dest_file);
            return -1;
        }
    }

    fclose(src_file);
    fclose(dest_file);
    return 0;
}


static void refresh_pic_id(int camera_type) {
    uint16_t cur = *g_pic_id;
    if (CAMERA_TYPE_S == camera_type) {
        if (cur >= 0x3000) {
            *g_pic_id = 0;
        } else {
            *g_pic_id = cur + 1;
        }
    } else if (CAMERA_TYPE_D == camera_type) {
        if (cur >= 0xEFFF || cur <= 0) {
            *g_pic_id = 0x3001;
        } else {
            *g_pic_id = cur + 1;
        }
    }
}

/** *** 拍摄照片线程 *** */
//static void *snap_thread_func_at_stop(void *arg) {
//    printf("[%s]snap_thread_func_at_stop \n", tag);
//    while (g_running && snap_count < total) {
//        if (current_task == NULL)break;
//        if (!current_task->running)break;
//        angel = get_angle();
//        angel = angel >= 0 ? angel : -angel;
//        print_fq++;
////        if (angel > 0 && print_fq > 3) {
////            print_fq = 0;
////            printf("[%s]snap_thread_func take_photo >>> %d angel=%d \n", tag, *snap_pic_id, angel);
////        }
//        if (current_task->trigger_type == 0 && angel < current_task->triggerAngel) {
//            continue;
//        }
//        // 重置时间和pic_id
//        snap_timestamp = time(NULL);
//        refresh_pic_id(current_task->cameraType);
//        // 生成file_name
//        snprintf(tmp_img_name, sizeof(tmp_img_name), "hd_%d_%03d.jpg", snap_timestamp, *snap_pic_id);
//        // 生成action_path
//        snprintf(action_id_src_path, sizeof(action_id_src_path), "%s/%s", g_src_path, current_task->name);
//        mkdir_recursive(action_id_src_path); // todo 判断结果
//        printf("[%s]snap_thread_func >>> 准备拍照 %s/%s \n", tag, action_id_src_path, tmp_img_name);
//        // 拍照
//        ret = qjy_take_photo(2, snap_pic_id, tmp_img_name, action_id_src_path);
//        if (ret) {
//            printf("[%s]snap error %d \n", tag, ret);
//            continue;
//        }
//        usleep(1000000 / SNAPSHOT_COUNT);
//        // 生成记录
//        SnapshotItem *item = malloc(sizeof(SnapshotItem));
//        if (item == NULL) {
//            continue;
//        }
//        item->pic_id = *snap_pic_id;
//        item->timestamp = snap_timestamp;
//        item->file_name = strdup(tmp_img_name);
//        current_task->pics[snap_count] = item;
//        *snap_pic_id = *snap_pic_id + 1;
//        snap_count++;
//    }
//    printf("[%s]snap_thread_func_at_stop end \n", tag);
//    return NULL;
//}

static void *snap_thread_func(void *arg) {
    printf("[%s]snap_thread_func \n", tag);

    uint32_t snap_timestamp;
    int ret;
    int angel;
    int snap_count = 0;
    int total = current_task->cameraType == CAMERA_TYPE_S ? 1 : MAX_PICS;
    char action_id_src_path[1024]; // src action_id 文件夹path
    char tmp_img_name[1024]; // timestamp_index.jpg
    int print_fq = 0;
    while (g_running) {
        if (current_task == NULL)break;
        if (!current_task->running)break;
        angel = get_angle();
        angel = angel >= 0 ? angel : -angel;
//        print_fq++;
        if (angel == 0)continue;
//        int can = 0;
//        if (last_angel < angel) {
//            last_angel = angel;
//        } else {
//            if (angel <= 20 && angel < last_angel) {
//                can = 1;
//            }
//        }

//        if (angel > 0 && print_fq > 3) {
//            print_fq = 0;
//            printf("[%s]snap_thread_func take_photo >>> %d angel=%d \n", tag, *snap_pic_id, angel);
//        }
        if (current_task->trigger_type == 0 && angel < current_task->triggerAngel) {
            continue;
        }
//        if (!can)continue;
        // 重置时间和pic_id
        snap_timestamp = time(NULL);
        refresh_pic_id(current_task->cameraType);
        // 生成file_name
        snprintf(tmp_img_name, sizeof(tmp_img_name), "hd_%d_%03d.jpg", snap_timestamp, *snap_pic_id);
        // 生成action_path
        snprintf(action_id_src_path, sizeof(action_id_src_path), "%s/%s", g_src_path, current_task->name);
        mkdir_recursive(action_id_src_path); // todo 判断结果
        printf("[%s]snap_thread_func >>> 准备拍照(%d) %s/%s \n", tag,angel ,action_id_src_path, tmp_img_name);
        // 拍照
        ret = qjy_take_photo(2, snap_pic_id, tmp_img_name, action_id_src_path);
        if (ret) {
            printf("[%s]snap error %d \n", tag, ret);
            continue;
        }
        usleep(1000000 / SNAPSHOT_COUNT);
        // 生成记录
        SnapshotItem *item = malloc(sizeof(SnapshotItem));
        if (item == NULL) {
            continue;
        }
        item->pic_id = *snap_pic_id;
        item->timestamp = snap_timestamp;
        item->file_name = strdup(tmp_img_name);
        current_task->pics[snap_count] = item;
        *snap_pic_id = *snap_pic_id + 1;
        snap_count++;
    }

    current_task->pic_count = snap_count;
    printf("[%s]snap_thread_func. end!  一共%d张\n", tag, snap_count);
    return NULL;
}

static void *handle_thread_func(void *arg) {
    printf("[%s]handle_thread_func \n", tag);
    int index_1 = 0;
    int index_2 = 0;
    char src_action_id_path[1024];      // src action_id 文件夹绝对地址

    char dest_action_id_path[1024];     // dst action_id 文件夹绝对地址
    char dest_file_name[1024];          // dst 文件名称
    char dest_file_path[1024];          // dst 文件绝对地址
    char src_file_path[1024];           // src 文件绝对地址
    char transform_pic_path[1024];     //
    unsigned char file_md5[16];         // 文件md5
    long file_size;                    // 文件大小

    int result_pic_size = 0;            // 所有文件大小

    while (g_running) {

        void **item = hd_queue_take(g_queue);
        if (item == NULL)continue;
        if (g_callback == NULL)continue;
        SnapshotResource *res = (SnapshotResource *) item;
        printf("[%s]handle_thread_func handle_photo >>> %d \n", tag, res->action_id_index);
//        char **result_pics = malloc(sizeof(char *) * MAX_PICS);        // 所有文件
        char *result_pics[MAX_PICS];
        // 每次处理sleep一会 让拍摄的照片存到本地
        usleep(1000000 / SNAPSHOT_COUNT);

        result_pic_size = 0;
        snprintf(src_action_id_path, sizeof(src_action_id_path), "%s/%s", g_src_path, res->action_id_name);
        snprintf(dest_action_id_path, sizeof(dest_action_id_path), "%s/%s", g_dst_path, res->action_id_name);

        for (int i = 0; i < res->pic_count; ++i) {

            if (res->cameraType == CAMERA_TYPE_S) {
                snprintf(src_file_path, sizeof(src_file_path), "%s/%s/%s", g_src_path, res->action_id_name,
                         res->pics[i]->file_name);

                // 获取文件信息：大小和md5
                file_size = get_file_size(src_file_path);
                if (file_size <= 0)continue; // 无法创建目标文件
                printf("[%s]handle_thread_func  大小:%ld\n", tag, file_size);
                int ret = hd_md5_file(src_file_path, file_md5);
                if (ret)continue;
                printf("[%s]handle_thread_func  md5:", tag);
                for (int j = 0; j < 16; ++j) {
                    printf("%02x", file_md5[j]);
                }
                printf("\n");
                // 生成文件名称。
                refresh_pic_id(res->cameraType);
                ret = hd_camera_protocol_pic_info_encode(dest_file_name, index_1++, file_md5, file_size, index_2++,
                                                         g_addr,
                                                         res->triggerAngel, res->triggerType, res->pics[i]->timestamp,
                                                         *g_pic_id);


                if (ret)continue;
                // 复制文件到dst

                snprintf(dest_file_path, sizeof(dest_file_path), "%s/%s", dest_action_id_path, dest_file_name);

                printf("[%s]handle_thread_func  静态处理：复制 %s => %s\n", tag, res->pics[i]->file_name, dest_file_path);

                result_pics[result_pic_size] = strdup(dest_file_path);
                copy_file(src_file_path, dest_file_path);
                result_pic_size++;
                SnapshotItem_free(res->pics[i]);
                if (res->triggerType == 1) { // action_id一致
                    // 通知
                    pthread_mutex_lock(&my_mutex);
                    my_ready = 1;
                    printf("[Notify Thread] Signaling condition...\n");
                    pthread_cond_signal(&my_cond);
                    pthread_mutex_unlock(&my_mutex);
                    sem_pic_id = *g_pic_id;
                }
            } else if (res->cameraType == CAMERA_TYPE_D) { // 动态图片
                if (res->pic_count - i > 5) {
                    SnapshotItem_free(res->pics[i]);
                    res->pics[i] = NULL;

                    continue;
                }


                if (g_transform_pic == NULL) {
                    // 直接用测试图
                    snprintf(transform_pic_path, sizeof(transform_pic_path), "%s", g_demo_path);
                } else {
                    int ret = g_transform_pic(src_file_path, transform_pic_path);
                    if (ret) {
                        SnapshotItem_free(res->pics[i]);
                        res->pics[i] = NULL;
                        continue;
                    }
                }

                // 获取文件信息：大小和md5
                file_size = get_file_size(transform_pic_path);
                if (file_size <= 0)continue; // 无法创建目标文件
                printf("[%s]handle_thread_func  大小:%ld\n", tag, file_size);
                int ret = hd_md5_file(transform_pic_path, file_md5);
                if (ret)continue;
                printf("[%s]handle_thread_func  md5:", tag);
                for (int j = 0; j < 16; ++j) {
                    printf("%02x", file_md5[j]);
                }
                printf("\n");
                // 生成文件名称。
                refresh_pic_id(res->cameraType);
                ret = hd_camera_protocol_pic_info_encode(dest_file_name, index_1++, file_md5, file_size, index_2++,
                                                         g_addr,
                                                         res->triggerAngel, 0, res->pics[i]->timestamp,
                                                         *g_pic_id);
                if (ret)continue;
                // 复制文件到dst
                snprintf(dest_file_path, sizeof(dest_file_path), "%s/%s/%s", g_dst_path, res->action_id_name,
                         dest_file_name);

                printf("[%s]handle_thread_func  动态处理：复制 %s => %s\n", tag, transform_pic_path, dest_file_path);
                result_pics[result_pic_size] = strdup(dest_file_path);
                copy_file(transform_pic_path, dest_file_path);
                if (g_transform_pic != NULL) {
                    delete_file_if_exists(transform_pic_path);
                }
                result_pic_size++;
                SnapshotItem_free(res->pics[i]);
            }
        }

        // 清理
        SnapshotResource_free(res);

        if (result_pic_size > 0) {
            g_callback(dest_action_id_path, result_pics, result_pic_size);
        }

        // 清理
        for (int i = 0; i < result_pic_size; ++i) {
            if (result_pics[i]) {
                free(result_pics[i]);
                result_pics[i] = NULL;
            }
        }

        // 删除action_id文件夹
        hd_delete_directory(src_action_id_path);

    }
    printf("[%s]handle_thread_func. end! \n", tag);
    return NULL;
}


static int do_snapshot_start(uint32_t action_id_timestamp, uint8_t action_id_index, uint8_t trigger_type) {
    printf("[%s]do_snapshot_start %d %d\n", tag, action_id_timestamp, action_id_index);
    // 1.创建任务
    pthread_mutex_lock(&g_lock);
    SnapshotTask *task = malloc(sizeof(SnapshotTask));
    if (task == NULL) {
        printf("[%s]do_snapshot_start malloc error\n", tag);
        pthread_mutex_unlock(&g_lock);
        return 1;
    }
    task->action_id_index = action_id_index;
    task->action_id_timestamp = action_id_timestamp;

    char tmp_name[1024];
    do_action_id_2_str(tmp_name, sizeof(tmp_name), action_id_timestamp, action_id_index);
    task->name = strdup(tmp_name);
    task->triggerAngel = g_addr == 1 ? 40 : 20;
    task->trigger_type = trigger_type;
    task->cameraType = g_addr == 1 ? CAMERA_TYPE_S : CAMERA_TYPE_D;
    SnapshotItem **pics = malloc(sizeof(SnapshotItem *) * MAX_PICS);
    if (NULL == pics) {
        printf("[%s]do_snapshot_start malloc error\n", tag);
        pthread_mutex_unlock(&g_lock);
        return 2;
    }
    task->pics = pics;
    task->running = 1;

    current_task = task;
    printf("[%s]do_snapshot_start pthread_create\n", tag);
    pthread_create(&current_task->snap_t, NULL, snap_thread_func, NULL);
    printf("[%s]do_snapshot_start pthread_create end\n", tag);
    pthread_mutex_unlock(&g_lock);
    printf("[%s]do_snapshot_start 8\n", tag);
    return 0;
}

static int do_snapshot_stop(uint32_t action_id_timestamp, uint8_t action_id_index) {
    printf("[%s]do_snapshot_stop \n", tag);
    pthread_mutex_lock(&g_lock);
    if (current_task == NULL) {
        pthread_mutex_unlock(&g_lock);
        return 0;
    }
    printf("[%s]do_snapshot_stop 222\n", tag);
    current_task->running = 0;
    // 添加到处理队列
    pthread_join(current_task->snap_t, NULL);

    // 交给图片处理线程
    if (g_queue) {
        SnapshotResource *res = malloc(sizeof(SnapshotResource));
        if (res) {
            res->action_id_timestamp = current_task->action_id_timestamp;
            res->action_id_index = current_task->action_id_index;
            res->triggerAngel = current_task->triggerAngel;
            res->cameraType = current_task->cameraType;
            res->action_id_name = strdup(current_task->name);
            res->triggerType = current_task->trigger_type;
            res->pic_count = (current_task->pic_count);
            res->pics = malloc(sizeof(SnapshotItem *) * current_task->pic_count);
            printf("生成资源文件\n");
            printf("action_id   %d   %d\n", action_id_timestamp, action_id_index);
            for (int i = 0; i < current_task->pic_count; ++i) {

                SnapshotItem *item = malloc(sizeof(SnapshotItem));
                item->file_name = strdup(current_task->pics[i]->file_name);
                item->timestamp = current_task->pics[i]->timestamp;
                item->pic_id = current_task->pics[i]->pic_id;
                res->pics[i] = item;
                printf("    >pic   %d   %s\n", item->pic_id, item->file_name);
            }


            hd_queue_put(g_queue, res);
        }
        printf("[%s]do_snapshot_stop 333\n", tag);
        SnapshotTask_free(current_task);
        current_task = NULL;
    }
    printf("[%s]do_snapshot_stop 444\n", tag);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

/**
 *
 * @param addr
 * @param path
 * @param demo_path
 * @param on_action_id_info_produce         图片产生 参数：action_id path，图片path列表 列表大小
 * @param transform_pic                     处理图片 参数：原图path，生成图path
 * @return
 */
int hd_camera_produce_init(uint8_t addr,
                           const char *path,
                           const char *demo_path,
                           int(*on_action_id_info_produce)(char *, char **, int),
                           int(*transform_pic)(const char *, char *)
) {
    g_callback = on_action_id_info_produce;
    g_addr = addr;
    g_transform_pic = transform_pic;

    snprintf(g_src_path, sizeof(g_src_path), "%s/%s", path, SNAP_SRC);
    snprintf(g_dst_path, sizeof(g_dst_path), "%s", path);
    snprintf(g_demo_path, sizeof(g_demo_path), "%s", demo_path);

    mkdir_recursive(g_src_path);
    if (0 != strcmp("/userdata/crop.jpg",g_dst_path)){
        mkdir_recursive(g_dst_path);
    }

    printf("[%s]hd_camera_produce_init %d %s %s %s \n", tag, g_addr, g_dst_path, g_src_path, g_demo_path);
    g_pic_id = malloc(sizeof(uint16_t *));
    snap_pic_id = malloc(sizeof(uint8_t *));
    *g_pic_id = 0;
    g_queue = hd_queue_create(100);
    if (g_queue == NULL)return 1;
    restore_sensor();
    g_running = 1;
    pthread_t handle_t;
    pthread_create(&handle_t, NULL, handle_thread_func, NULL);
    return 0;
}

int hd_camera_produce_take_photos_actively(uint16_t *pic_id) {
    printf("hd_camera_produce_take_photos_actively Waiting...\n");
    uint32_t action_id_timestamp = time(NULL);
    uint8_t action_id_index = 0xfe;
    hd_camera_produce_on_action_id_changed(action_id_timestamp, action_id_index, 1, 1);
    usleep(100 * 1000);
    hd_camera_produce_on_action_id_changed(action_id_timestamp, action_id_index + 1, 0, 1);

    pthread_mutex_lock(&my_mutex);
    printf("[Wait Thread] Waiting for condition (timeout=3s)...\n");

    // 设置超时时间（当前时间 + 3秒）
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 3;

    // 带超时的条件等待
    while (!my_ready) {
        int ret = pthread_cond_timedwait(&my_cond, &my_mutex, &ts);
        if (ret == ETIMEDOUT) {
            printf("[Wait Thread] Timeout! Condition not met.\n");
            break;
        }
    }
    int ret;
    if (my_ready) {
        printf("[Wait Thread] Condition met!\n");
        *pic_id = sem_pic_id;
        ret = 0;
    } else {
        ret = 1;
    }
    pthread_mutex_unlock(&my_mutex);


    printf("hd_camera_produce_take_photos_actively Received notification!\n");

    // 超时返回
    return ret;
}

int hd_camera_produce_on_action_id_changed(uint32_t action_id_timestamp, uint8_t action_id_index, uint8_t status,
                                           uint8_t trigger_type) {
    printf("[%s]hd_camera_produce_on_action_id_changed status %d\n", tag, status);
    int ret;
    ret = do_snapshot_stop(action_id_timestamp, action_id_index); // 关闭拍摄
    printf("[%s]do_snapshot_stop ret = %d\n", tag, ret);
    if (status == 0) {
        restore_sensor();
    } else {
        ret = do_snapshot_start(action_id_timestamp, action_id_index, trigger_type);
        printf("[%s]do_snapshot_start ret = %d\n", tag, ret);
    }

    return 0;
}

static void item_free(void **item) {
    if (item) {
        SnapshotResource *res = (SnapshotResource *) item;
        SnapshotResource_free(res);
    }
}

int hd_camera_produce_deinit(uint8_t addr) {
    g_running = 0;
    g_callback = NULL;
    g_transform_pic = NULL;
    g_addr = 0;
    if (g_queue) {
        hd_queue_destroy(g_queue, item_free);
    }
    if (g_pic_id) {
        free(g_pic_id);
        g_pic_id = NULL;
    }
    if (snap_pic_id) {
        free(snap_pic_id);
        snap_pic_id = NULL;
    }
    printf("[%s]hd_camera_produce_deinit\n", tag);
    return 0;

}
//</editor-fold>
