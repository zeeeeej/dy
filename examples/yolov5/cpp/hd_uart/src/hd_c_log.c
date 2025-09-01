
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
#include <stdio.h>

/************************* 双端队列 a **********************************/


// 双端队列节点结构
typedef struct Node {
    void *data;         // 存储任意类型的数据指针
    struct Node *prev;
    struct Node *next;
} Node;

// 定义foreach回调函数类型
typedef void (*deque_foreach_callback)(void *data, void *user_data, int index);

// 双端队列结构
typedef struct {
    Node *front;        // 队首指针
    Node *rear;         // 队尾指针
    int size;           // 当前队列大小
    int capacity;       // 队列容量
    pthread_mutex_t lock; // 互斥锁
    void (*free_data)(void *); // 数据释放函数
} Deque;

// 初始化双端队列
Deque *deque_init(int capacity, void (*free_data)(void *)) {
    if (capacity <= 0) {
        return NULL;
    }

    Deque *deque = (Deque *) malloc(sizeof(Deque));
    if (!deque) {
        return NULL;
    }

    deque->front = NULL;
    deque->rear = NULL;
    deque->size = 0;
    deque->capacity = capacity;
    deque->free_data = free_data;
    pthread_mutex_init(&deque->lock, NULL);

    return deque;
}

// 销毁双端队列
void deque_destroy(Deque *deque) {
    if (!deque) return;

    pthread_mutex_lock(&deque->lock);

    Node *current = deque->front;
    while (current) {
        Node *temp = current;
        current = current->next;

        // 如果有提供free_data函数，则释放数据
        if (deque->free_data) {
            deque->free_data(temp->data);
        }
        free(temp);
    }

    pthread_mutex_unlock(&deque->lock);
    pthread_mutex_destroy(&deque->lock);
    free(deque);
}

// 检查队列是否为空
int deque_is_empty(Deque *deque) {
    if (!deque) return 1;

    pthread_mutex_lock(&deque->lock);
    int empty = (deque->size == 0);
    pthread_mutex_unlock(&deque->lock);

    return empty;
}

// 检查队列是否已满
int deque_is_full(Deque *deque) {
    if (!deque) return 0;

    pthread_mutex_lock(&deque->lock);
    int full = (deque->size == deque->capacity);
    pthread_mutex_unlock(&deque->lock);

    return full;
}

// 获取队列当前大小
int deque_size(Deque *deque) {
    if (!deque) return 0;

    pthread_mutex_lock(&deque->lock);
    int size = deque->size;
    pthread_mutex_unlock(&deque->lock);

    return size;
}

// 从队首插入元素
int deque_push_front(Deque *deque, void *data) {
    if (!deque) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size >= deque->capacity) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列已满
    }

    Node *new_node = (Node *) malloc(sizeof(Node));
    if (!new_node) {
        pthread_mutex_unlock(&deque->lock);
        return -1;
    }

    new_node->data = data;
    new_node->prev = NULL;
    new_node->next = deque->front;

    if (deque->front) {
        deque->front->prev = new_node;
    } else {
        deque->rear = new_node; // 队列为空时，rear也指向新节点
    }

    deque->front = new_node;
    deque->size++;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 从队尾插入元素
int deque_push_rear(Deque *deque, void *data) {
    if (!deque) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size >= deque->capacity) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列已满
    }

    Node *new_node = (Node *) malloc(sizeof(Node));
    if (!new_node) {
        pthread_mutex_unlock(&deque->lock);
        return -1;
    }

    new_node->data = data;
    new_node->next = NULL;
    new_node->prev = deque->rear;

    if (deque->rear) {
        deque->rear->next = new_node;
    } else {
        deque->front = new_node; // 队列为空时，front也指向新节点
    }

    deque->rear = new_node;
    deque->size++;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 从队首移除元素
int deque_pop_front(Deque *deque, void **data) {
    if (!deque || !data) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size == 0) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列为空
    }

    Node *temp = deque->front;
    *data = temp->data;

    deque->front = temp->next;
    if (deque->front) {
        deque->front->prev = NULL;
    } else {
        deque->rear = NULL; // 队列为空时，rear也置为NULL
    }

    free(temp);
    deque->size--;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 从队尾移除元素
int deque_pop_rear(Deque *deque, void **data) {
    if (!deque || !data) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size == 0) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列为空
    }

    Node *temp = deque->rear;
    *data = temp->data;

    deque->rear = temp->prev;
    if (deque->rear) {
        deque->rear->next = NULL;
    } else {
        deque->front = NULL; // 队列为空时，front也置为NULL
    }

    free(temp);
    deque->size--;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 获取队首元素但不移除
int deque_peek_front(Deque *deque, void **data) {
    if (!deque || !data) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size == 0) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列为空
    }

    *data = deque->front->data;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 获取队尾元素但不移除
int deque_peek_rear(Deque *deque, void **data) {
    if (!deque || !data) return -1;

    pthread_mutex_lock(&deque->lock);

    if (deque->size == 0) {
        pthread_mutex_unlock(&deque->lock);
        return -1; // 队列为空
    }

    *data = deque->rear->data;

    pthread_mutex_unlock(&deque->lock);
    return 0;
}

// 线程安全的foreach功能
void deque_foreach(Deque *deque, deque_foreach_callback callback, void *user_data) {
    if (!deque || !callback) return;

    pthread_mutex_lock(&deque->lock);

    Node *current = deque->front;
    int index = 0;
    while (current) {
        callback(current->data, user_data, index);
        current = current->next;
        index++;
    }

    pthread_mutex_unlock(&deque->lock);
}

// 打印队列内容 (用于调试)
void deque_print(Deque *deque, void (*print_func)(void *)) {
    if (!deque) return;

    pthread_mutex_lock(&deque->lock);

    printf("Deque (size=%d/%d): [", deque->size, deque->capacity);

    Node *current = deque->front;
    while (current) {
        if (print_func) {
            print_func(current->data);
        } else {
            printf("%p", current->data);
        }
        if (current->next) {
            printf(", ");
        }
        current = current->next;
    }

    printf("]\n");

    pthread_mutex_unlock(&deque->lock);
}

//// 测试用例
//
//// 示例数据结构和相关函数
//typedef struct {
//    int id;
//    char name[32];
//} Person;
//
//void print_person(void* data) {
//    Person* p = (Person*)data;
//    printf("Person{id=%d, name='%s'}", p->id, p->name);
//}
//
//void free_person(void* data) {
//    free(data);
//}
//
//void sum_person_ids(void* data, void* user_data) {
//    Person* p = (Person*)data;
//    int* sum = (int*)user_data;
//    *sum += p->id;
//}
//
//int main() {
//    // 创建一个容量为3的双端队列，指定数据释放函数
//    Deque* deque = deque_init(3, free_person);
//
//    // 创建并添加一些Person对象
//    Person* p1 = malloc(sizeof(Person));
//    p1->id = 1; strcpy(p1->name, "Alice");
//    deque_push_rear(deque, p1);
//
//    Person* p2 = malloc(sizeof(Person));
//    p2->id = 2; strcpy(p2->name, "Bob");
//    deque_push_rear(deque, p2);
//
//    Person* p3 = malloc(sizeof(Person));
//    p3->id = 3; strcpy(p3->name, "Charlie");
//    deque_push_front(deque, p3);
//
//    // 打印队列内容
//    deque_print(deque, print_person);
//
//    // 使用foreach计算ID总和
//    int total_id = 0;
//    deque_foreach(deque, sum_person_ids, &total_id);
//    printf("Total ID sum: %d\n", total_id);
//
//    // 测试弹出元素
//    void* data;
//    if (deque_pop_front(deque, &data) == 0) {
//        Person* p = (Person*)data;
//        printf("Popped front: ");
//        print_person(p);
//        printf("\n");
//        free_person(p); // 手动释放，因为已经从队列中移除
//    }
//
//    deque_print(deque, print_person);
//
//    // 销毁队列(会自动释放剩余元素)
//    deque_destroy(deque);
//    return 0;
//}
/************************* 双端队列 z **********************************/

typedef struct {
    uint8_t snap_id;
    uint8_t snap_angle;
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
    Deque *pics;
    SnapshotItem **head_pics;
    int head_pics_count;
    char *name;
    uint8_t trigger_type;
    int snap_max;
} SnapshotTask;

typedef struct {
    uint32_t action_id_timestamp;
    uint8_t action_id_index;
    int triggerAngel;           // 拍照角度
    uint8_t triggerType;        // 主动抓图 or 正常拍摄
    int cameraType;             // 静态 or 动态
    SnapshotItem **pics;        // 照片数据 （pic_id 时间戳 名称）
    int pic_count;              // 照片数据大小
    char *action_id_name;       // action_id文件夹名称
    // array[n]                 // 前几张照片
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
        deque_destroy(task->pics);
        task->pics = NULL;
    }
    if (task->name) {
        free(task->name);
        task->name = NULL;

    }
    free(task);
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

#define CAMERA_TYPE_S               0       // 静态类型摄像头
#define CAMERA_TYPE_D               1       // 动态类型摄像头
#define MAX_PICS                    20      // 单次开门事件图片最大数量
#define SNAPSHOT_COUNT              3       // 每秒拍多少张
#define SNAPSHOT_CHECK_COUNT        5       // 检测角度频率
#define SNAP_SRC                    "src"   // 源文件文件夹名称
#define SNAP_OFFSET                 5       // 范围区间 根据实际测试调整
#define SNAP_OFFSET_MAX             5       // 范围区间 根据实际测试调整
#define CHECK_CLOSE_DOOR_COUNT_MAX  3       // 连续angel减少次数
#define FILE_NAME_LENGTH            512     // 文件path最大长度
#define DY_CROP_MAX                 10      // 动态图片生成最大数量
//#define HD_SNAP_TYPE_STATIC_DYNAMIC HD_SNAP_TYPE_STATIC_DYNAMIC      // 动销项目

static int (*g_callback)(char *, char **, int) = NULL;

static int (*g_transform_pic)(const char *, char *) = NULL;

static HDBlockingQueue *g_queue;
static volatile uint8_t g_running = 0;
static uint8_t g_addr = 0;
static uint8_t debug = 1;
static char *tag = "hd_c_log.c";
static pthread_mutex_t g_current_task_lock;
static SnapshotTask *current_task;
static uint16_t *g_pic_id;
static uint8_t *snap_pic_id;
static char g_src_path[FILE_NAME_LENGTH];
static char g_dst_path[FILE_NAME_LENGTH];
static char g_demo_path[FILE_NAME_LENGTH];

static pthread_mutex_t my_mutex = PTHREAD_MUTEX_INITIALIZER;    // 主动抓图
static pthread_cond_t my_cond = PTHREAD_COND_INITIALIZER;       // 主动抓图
static int my_ready = 0;                                        // 主动抓图条件变量

static uint16_t sem_pic_id;

// TODO 合并hd_uart_parser.c里的do_action_id_2_str
static int do_action_id_2_str(char *str, size_t str_size, uint32_t action_id_timestamps, uint8_t action_id_index) {
    snprintf(str, str_size, "%d%03d", action_id_timestamps, action_id_index);
    return 0;
}

//static int mkdir2(const char *dir_path) {
//    // 检查目录是否存在（F_OK 检查文件是否存在）
//    if (access(dir_path, F_OK)) {
//        // 目录不存在，尝试创建
//        if (mkdir(dir_path, 0755) == 0) {
//            printf("Directory created: %s\n", dir_path);
//        } else {
//            perror("mkdir failed");
//            return 1;
//        }
//    } else {
//        printf("Directory already exists: %s\n", dir_path);
//    }
//    return 0;
//}


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


static int mkdir_recursive(const char *path) {
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
    return mkdir(tmp, 0755);
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


static void refresh_snap_id(int camera_type) {
    uint8_t cur = *snap_pic_id;
    if (cur >= 0xff) {
        *snap_pic_id = 0;
    } else {
        *snap_pic_id = cur + 1;
    }
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

static int g_angle = 0;
static pthread_t get_angel_t;

static void *get_angle_func(void *arg) {
    printf("[%s]get_angle_func  获取陀螺仪\n", tag);
    while (current_task && current_task->running) {
        int angel = get_angle();
        if (angel != 0) {
            g_angle = angel >= 0 ? angel : -angel;
            //printf("[%s]get_angle_func  获取陀螺仪 %d\n", tag, angel);
        }
        usleep(20 * 1000);
    }
    printf("[%s]get_angle_func  获取陀螺仪 end\n", tag);
    return NULL;
}


static void snap_thread_func_dynamic() {
    uint32_t snap_timestamp;
    int ret;
    int angel = g_angle;
    int snap_count = 0;
    char action_id_src_path[FILE_NAME_LENGTH];  // src action_id 文件夹path
    char tmp_img_name[FILE_NAME_LENGTH];        // timestamp_index.jpg
    char delete_file_path[FILE_NAME_LENGTH];

    // 生成action_path
    snprintf(action_id_src_path, sizeof(action_id_src_path), "%s/%s", g_src_path, current_task->name);
    printf("action_id_src_path = %s %s \n", action_id_src_path, tmp_img_name);
    int result = mkdir_recursive(action_id_src_path); // todo 判断结果
    if (result) {
        printf("[%s]snap_thread_func mkdir_recursive fail! %d  \n", tag, result);
    }

    while (g_running && snap_count < current_task->snap_max) {
        if (!current_task->running) {
            printf("[%s]snap_thread_func  current_task->running = false\n", tag);
            usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            break;
        }


        angel = g_angle;

        // 不在范围内直接不考虑
        if (current_task->trigger_type == 0 /*主动抓图不考虑角度*/ && angel < (current_task->triggerAngel)) {
            printf("[%s]snap_thread_func  不在角度范围内 %d  < (%d) \n", tag, angel,current_task->triggerAngel);
            usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            continue;
        }

        // 重置时间和pic_id
        snap_timestamp = time(NULL);
//        refresh_snap_id(current_task->cameraType);
        // 生成file_name
        snprintf(tmp_img_name, sizeof(tmp_img_name), "hd_src_%d_%03d.jpg", snap_timestamp, *snap_pic_id);
        printf("[%s]snap_thread_func >>> 准备拍照(%d) %s/%s \n", tag, angel, action_id_src_path, tmp_img_name);
        // 拍照
        ret = qjy_take_photo(2, snap_pic_id, tmp_img_name, action_id_src_path);
        if (ret) {
            printf("[%s]snap error %d \n", tag, ret);
            continue;
        }

        usleep(1000000 / 5);

        // 生成记录，检查是否满了
        int full = deque_is_full(current_task->pics);
        if (full) {
            printf("[%s]snap error 满了 \n", tag);
            void *data;
            deque_pop_front(current_task->pics, &data);
            if (data) {
                SnapshotItem *item = data;
                // 删除文件
                snprintf(delete_file_path, sizeof(delete_file_path), "%s/%s/%s", g_src_path, current_task->name,
                         item->file_name);
                printf("delete_file_if_exists before 文件：%s ,file_name:%s\n",delete_file_path,item->file_name);
                delete_file_if_exists(delete_file_path);
                SnapshotItem_free(item);
            }
        }
        SnapshotItem *item = malloc(sizeof(SnapshotItem));
        if (item == NULL) {
            printf("[%s]snap error malloc SnapshotItem fail\n", tag);
            continue;
        }
        item->snap_id = *snap_pic_id;
        item->timestamp = snap_timestamp;
        item->snap_angle = angel;
        item->file_name = strdup(tmp_img_name);


        deque_push_rear(current_task->pics, item);
        refresh_snap_id(current_task->cameraType);
        snap_count++;
    }
    printf("[%s]动态拍照结束 一共拍照%d次 \n", tag, snap_count);
}

static void snap_thread_func_static() {
    uint32_t snap_timestamp;
    int ret;
    int angel = g_angle;
    int snap_count = 0;
    char action_id_src_path[FILE_NAME_LENGTH];  // src action_id 文件夹path
    char tmp_img_name[FILE_NAME_LENGTH];        // timestamp_index.jpg
    char delete_file_path[FILE_NAME_LENGTH];

    // 生成action_path
    snprintf(action_id_src_path, sizeof(action_id_src_path), "%s/%s", g_src_path, current_task->name);
    printf("action_id_src_path = %s %s \n", action_id_src_path, tmp_img_name);
    int result = mkdir_recursive(action_id_src_path); // todo 判断结果
    if (result) {
        printf("[%s]snap_thread_func mkdir_recursive fail! %d  \n", tag, result);
    }

#ifndef HD_SNAP_TYPE_STATIC_DYNAMIC
    int first = 1;                              //第一次不需要延迟

    int last_angel = 0;                         // 上一次角度
    int check_close_door = 0;                   // 连续3次下降，就代表关门
    int check_close_door_count = 0;             // 记录连续下降次数
#endif
    while (g_running && snap_count < current_task->snap_max) {
        if (!current_task->running) {
            printf("[%s]snap_thread_func  current_task->running = false\n", tag);
            usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            break;
        }


        angel = g_angle;
#ifdef  HD_SNAP_TYPE_STATIC_DYNAMIC
        if (current_task->trigger_type == 0 && angel < current_task->triggerAngel) {
            printf("[%s]snap_thread_func  不在角度范围内 %d < %d \n", tag, angel, current_task->triggerAngel);
            continue;
        }
#else
        // 不在范围内直接不考虑
        if (current_task->trigger_type == 0 &&
            (angel < (current_task->triggerAngel - SNAP_OFFSET) ||
             angel > (current_task->triggerAngel + SNAP_OFFSET_MAX))) {
            printf("[%s]snap_thread_func  不在角度范围内 %d not in (%d,%d) \n", tag, angel,
                   current_task->triggerAngel - SNAP_OFFSET,
                   current_task->triggerAngel + SNAP_OFFSET_MAX);
            last_angel = angel;
            if (first) {
                first = 0;
            } else {
                usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            }
            continue;
        }
        if (current_task->trigger_type == 0 && (angel != 0 && last_angel != 0 && angel > last_angel + 1)) {
            printf("[%s]snap_thread_func  角度趋势不对  %d > %d + 1\n", tag, angel, last_angel);
            check_close_door_count = 0; // 只要趋势不对 就重置
            check_close_door = 0;
            last_angel = angel;
            if (first) {
                first = 0;
            } else {
                usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            }
            continue;
        }
        check_close_door_count++;
        if (current_task->trigger_type == 0 &&
            check_close_door_count >= CHECK_CLOSE_DOOR_COUNT_MAX) {     // 连续大于CHECK_CLOSE_DOOR_COUNT_MAX，说明是关门
            printf("[%s]snap_thread_func  连续%d次角度下降\n", tag, CHECK_CLOSE_DOOR_COUNT_MAX);
            check_close_door = 1;
            check_close_door_count = 0; // 重置
        }
        if (current_task->trigger_type == 0 && !check_close_door) {
            // 不是关门
            printf("[%s]snap_thread_func  正在开门不拍照\n", tag);
            last_angel = angel;
            if (first) {
                first = 0;
            } else {
                usleep(1000 * 1000 / SNAPSHOT_CHECK_COUNT);
            }
            continue;
        }

        last_angel = angel;
#endif

        // 重置时间和pic_id
        snap_timestamp = time(NULL);
//        refresh_snap_id(current_task->cameraType);
        // 生成file_name
        snprintf(tmp_img_name, sizeof(tmp_img_name), "hd_src_%d_%03d.jpg", snap_timestamp, *snap_pic_id);

        printf("[%s]snap_thread_func >>> 准备拍照(%d) %s/%s \n", tag, angel, action_id_src_path, tmp_img_name);
        // 拍照
        ret = qjy_take_photo(2, snap_pic_id, tmp_img_name, action_id_src_path);
        if (ret) {
            printf("[%s]snap error %d \n", tag, ret);
            continue;
        }

        usleep(1000000 / SNAPSHOT_COUNT);

        // 生成记录，检查是否满了
        int full = deque_is_full(current_task->pics);
        if (full) {
            printf("[%s]snap error 满了 \n", tag);
            void *data;
            deque_pop_front(current_task->pics, &data);
            if (data) {
                SnapshotItem *item = data;
                // 删除文件
                snprintf(delete_file_path, sizeof(delete_file_path), "%s/%s/%s", g_src_path, current_task->name,
                         item->file_name);
                delete_file_if_exists(delete_file_path);
                SnapshotItem_free(item);
            }
        }
        SnapshotItem *item = malloc(sizeof(SnapshotItem));
        if (item == NULL) {
            printf("[%s]snap error malloc SnapshotItem fail\n", tag);
            continue;
        }
        item->snap_id = *snap_pic_id;
        item->timestamp = snap_timestamp;
        item->file_name = strdup(tmp_img_name);
        item->snap_angle = angel;

        deque_push_rear(current_task->pics, item);
        refresh_snap_id(current_task->cameraType);
        snap_count++;


        if (current_task->trigger_type == 1) {
            printf("[%s]snap error 抓图结束 \n", tag);
            break;
        }
    }

    printf("[%s]静态拍照结束 一共拍照%d次 \n", tag, snap_count);
}

static void *snap_thread_func(void *arg) {
    printf("[%s]snap_thread_func  开始拍照\n", tag);
    if (current_task->cameraType == CAMERA_TYPE_S) {
        snap_thread_func_static();
    } else if (current_task->cameraType == CAMERA_TYPE_D) {
        snap_thread_func_dynamic();
    }
    printf("[%s]snap_thread_func. 结束拍照!\n", tag);
    return NULL;
}

static void *handle_thread_func(void *arg) {
    printf("[%s]handle_thread_func \n", tag);
    static int index_1 = 0;
    static int index_2 = 0;
    static char src_action_id_path[FILE_NAME_LENGTH];       // src action_id 文件夹绝对地址
    static char dest_action_id_path[FILE_NAME_LENGTH];      // dst action_id 文件夹绝对地址
    static char dest_file_name[FILE_NAME_LENGTH];           // dst 文件名称
    static char dest_file_path[FILE_NAME_LENGTH];           // dst 文件绝对地址
    static char src_file_path[FILE_NAME_LENGTH];            // src 文件绝对地址
    static char transform_pic_path[FILE_NAME_LENGTH];       //
    unsigned char file_md5[16];                             // 文件md5
    long file_size;                                         // 文件大小
    int result_pic_size = 0;                                // 所有文件大小

    while (g_running) {

        void **item = hd_queue_take(g_queue);
        if (item == NULL)continue;
        if (g_callback == NULL)continue;
        SnapshotResource *res = (SnapshotResource *) item;
        printf("[%s]handle_thread_func handle_photo >>> %d \n", tag, res->action_id_index);
//        char **result_pics = malloc(sizeof(char *) * MAX_PICS);        // 所有文件
        char *result_pics[MAX_PICS];
        // 每次处理sleep一会 让拍摄的照片存到本地
        usleep(500000);

        result_pic_size = 0;
        snprintf(src_action_id_path, sizeof(src_action_id_path), "%s/%s", g_src_path, res->action_id_name);
        snprintf(dest_action_id_path, sizeof(dest_action_id_path), "%s/%s", g_dst_path, res->action_id_name);


        if (res->cameraType == CAMERA_TYPE_S) {
            printf("[%s]handle_thread_func handle_photo 处理静态照片.......期望：%d(张) \n", tag, res->pic_count);
//            int dest_count = res->pic_count > 5 ? 5 : res->pic_count; // 只取最后5张,
            for (int i = res->pic_count - 1; i >= 0; --i) {
//            for (int i =0; i <res->pic_count; ++i) {
                snprintf(src_file_path, sizeof(src_file_path), "%s/%s/%s", g_src_path, res->action_id_name,
                         res->pics[i]->file_name);

//                int retry = 0;
//                int access_result = 0;
//                while (retry < 20) {
//                    if (access(src_file_path, F_OK)) {
//                        access_result = 1;
//                        break;
//                    }
//                    retry++;
//                    usleep(200 * 1000);
//                }
//
//                if (!access_result) {
//                    printf("access file fail : %s\n", src_file_path);
//                    continue;
//                }
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
                                                         res->pics[i]->snap_angle, res->triggerType, res->pics[i]->timestamp,
                                                         *g_pic_id);


                if (ret)continue;
                // 复制文件到dst

                snprintf(dest_file_path, sizeof(dest_file_path), "%s/%s", dest_action_id_path, dest_file_name);

                printf("[%s]handle_thread_func  静态处理：复制 %s => %s\n", tag, res->pics[i]->file_name, dest_file_path);

                result_pics[result_pic_size] = strdup(dest_file_path);
                copy_file(src_file_path, dest_file_path);
                result_pic_size++;
                SnapshotItem_free(res->pics[i]);
                if (res->triggerType == 1) {
                    // 通知
                    pthread_mutex_lock(&my_mutex);
                    my_ready = 1;
                    sem_pic_id = *g_pic_id;
                    printf("[Notify Thread] Signaling condition...\n");
                    pthread_cond_signal(&my_cond);
                    pthread_mutex_unlock(&my_mutex);
                    printf("静态抓拍成功\n");
                    break;
                }
                printf("静态只要一张\n");
                break;
            }
            printf("静态处理完毕！！！\n");
        } else if (res->cameraType == CAMERA_TYPE_D) { // 动态图片
            printf("[%s]handle_thread_func handle_photo 处理动态照片.......期望：%d(张) \n", tag, res->pic_count);
            for (int i = res->pic_count - 1; i >= 0; --i) {

                if (result_pic_size > DY_CROP_MAX) {
                    printf("[%s]handle_thread_func handle_photo 处理动态照片 已满%d张 \n", tag,DY_CROP_MAX);
                    SnapshotItem_free(res->pics[i]);
                    res->pics[i] = NULL;
                    continue;
                }

                if (g_transform_pic == NULL) {
                    // 直接用测试图
                    snprintf(transform_pic_path, sizeof(transform_pic_path), "%s", g_demo_path);
                } else {

                    // 原始图片地址
                    snprintf(src_file_path, sizeof(src_file_path), "%s/%s/%s", g_src_path, res->action_id_name,
                             res->pics[i]->file_name);

                    // 目标图片地址
                    snprintf(transform_pic_path, sizeof(transform_pic_path), "%s/%s/%s_%d_%s", g_src_path, res->action_id_name,
                             "crop_",i,res->pics[i]->file_name);


                    printf("g_transform_pic 处理图片 原始地址：%s -> 目标地址：%s \n", src_file_path, transform_pic_path);
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
                                                         res->pics[i]->snap_angle, res->triggerType, res->pics[i]->timestamp,
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
                res->pics[i] = NULL;
            }
            printf("动态处理完毕！！！\n");
        }



        // 清理
        SnapshotResource_free(res);

        if (result_pic_size > 0 && g_callback != NULL) {
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
        printf("一次任务处理完毕！！！\n");
    }
    printf("[%s]handle_thread_func. end! \n", tag);
    return NULL;
}

void free_data_func(void *item) {
    if (item) {
        SnapshotItem *snapshotItem = item;
        SnapshotItem_free(snapshotItem);
    }
}


static int do_snapshot_start(uint32_t action_id_timestamp, uint8_t action_id_index, uint8_t trigger_type) {
    printf("[%s]do_snapshot_start %d %d\n", tag, action_id_timestamp, action_id_index);
    // 1.创建任务
    pthread_mutex_lock(&g_current_task_lock);
    if (current_task != NULL ){
        printf("[%s]do_snapshot_start 正在拍照中....\n", tag);
        pthread_mutex_unlock(&g_current_task_lock);
        return 2;
    }
    SnapshotTask *task = malloc(sizeof(SnapshotTask));
    if (task == NULL) {
        printf("[%s]do_snapshot_start malloc error\n", tag);
        pthread_mutex_unlock(&g_current_task_lock);
        return 1;
    }
    task->action_id_index = action_id_index;
    task->action_id_timestamp = action_id_timestamp;

    char tmp_name[FILE_NAME_LENGTH];
    do_action_id_2_str(tmp_name, sizeof(tmp_name), action_id_timestamp, action_id_index);
    task->name = strdup(tmp_name);
    task->triggerAngel = g_addr == PROTOCOL_SLAVE_STATIC ? 60 : 20;
    task->trigger_type = trigger_type;
    task->cameraType = g_addr == PROTOCOL_SLAVE_STATIC ? CAMERA_TYPE_S : CAMERA_TYPE_D;
    Deque *pics = deque_init(g_addr == PROTOCOL_SLAVE_STATIC?MAX_PICS:MAX_PICS*2, free_data_func);
    if (NULL == pics) {
        printf("[%s]do_snapshot_start malloc error\n", tag);
        pthread_mutex_unlock(&g_current_task_lock);
        return 2;
    }
    task->pics = pics;
    task->running = 1;
    task->snap_max = 1000000; // 暂时默认一个最大值，表示可以一直拍。
#ifdef HD_SNAP_TYPE_STATIC_DYNAMIC
    task->snap_max = 1; // 动销只拍一张
#endif
    task->head_pics = NULL;
    task->head_pics_count = 0;

    current_task = task;
    pthread_create(&get_angel_t, NULL, get_angle_func, NULL);
    pthread_create(&current_task->snap_t, NULL, snap_thread_func, NULL);
    pthread_mutex_unlock(&g_current_task_lock);
    printf("[%s]do_snapshot_start %d %d ok.\n", tag, action_id_timestamp, action_id_index);
    return 0;
}

static void snap_convert(void *data, void *res, int index) {
    SnapshotResource *resource = res;
    SnapshotItem *src = data;
    SnapshotItem *item = malloc(sizeof(SnapshotItem));
    if (!item) {
        printf("SnapshotItem mail fail");
        return;
    }
    item->file_name = strdup(src->file_name);
    item->timestamp = src->timestamp;
    item->snap_id = src->snap_id;
    resource->pics[index] = item;
    printf("    >snap_convert   %d   %s\n", item->snap_id, item->file_name);

}

static int do_snapshot_stop(uint32_t action_id_timestamp, uint8_t action_id_index) {
    printf("[%s]do_snapshot_stop \n", tag);
    pthread_mutex_lock(&g_current_task_lock);
    if (current_task == NULL) {
        pthread_mutex_unlock(&g_current_task_lock);
        return 1;
    }
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
            res->pic_count = (current_task->pics->size);
            res->pics = malloc(sizeof(SnapshotItem *) * current_task->pics->size);
            if (res->pics) {
                printf("do_snapshot_stop 生成资源文件,action_id   %d   %d\n", action_id_timestamp, action_id_index);
                deque_foreach(current_task->pics, snap_convert, res);
                hd_queue_put(g_queue, res);
            } else {
                SnapshotResource_free(res);
                res = NULL;
            }
        }
        printf("[%s]do_snapshot_stop 333\n", tag);
    }
    SnapshotTask_free(current_task); // SnapshotItem 在snap_convert已经释放
    current_task = NULL;
    pthread_mutex_unlock(&g_current_task_lock);
    printf("[%s]do_snapshot_stop ok.\n", tag);
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

#ifdef HD_SNAP_TYPE_STATIC_DYNAMIC
    printf("玄武项目动销项目\n");
#endif

    mkdir_recursive(g_src_path);
//    if (0 != strcmp("/userdata/crop.jpg", g_dst_path)) {
    mkdir_recursive(g_dst_path);
//    }

    printf("[%s]hd_camera_produce_init %d %s %s %s \n", tag, g_addr, g_dst_path, g_src_path, g_demo_path);
    g_pic_id = malloc(sizeof(uint16_t *));
    if (!g_pic_id) {
        return 1;
    }
    snap_pic_id = malloc(sizeof(uint8_t *));
    if (!snap_pic_id) {
        return 2;
    }
    * g_pic_id = 0;
    * snap_pic_id = 0;
    g_queue = hd_queue_create(100);
    if (g_queue == NULL)return 3;
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

    if (g_addr == PROTOCOL_SLAVE_DYNAMIC){
        usleep(10 * 1000 * 1000);
    }else if(g_addr == 0){
        usleep(100 * 1000);
    }

    hd_camera_produce_on_action_id_changed(action_id_timestamp, action_id_index + 1, 0, 1);

    pthread_mutex_lock(&my_mutex);
    printf("[Wait Thread] Waiting for condition (timeout=3s)...\n");

    // 设置超时时间（当前时间 + 3秒）
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 1;

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
    if (status == 0) {
        ret = do_snapshot_stop(action_id_timestamp, action_id_index); // 关闭拍摄
        printf("[%s]do_snapshot_stop ret = %d\n", tag, ret);
        restore_sensor();
    } else {
        ret = do_snapshot_start(action_id_timestamp, action_id_index, trigger_type);
        printf("[%s]do_snapshot_start ret = %d\n", tag, ret);
    }

    return ret;
}

static void item_free(void **item) {
    if (item) {
        SnapshotResource *res = (SnapshotResource *) item;
        SnapshotResource_free(res);
        res = NULL;
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
