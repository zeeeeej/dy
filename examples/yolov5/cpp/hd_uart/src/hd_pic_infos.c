#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include "hd_pic_infos.h"

#define HD_PIC_INFOS_TAG   "hd_pic_infos.c"
#define HD_PIC_INFOS_DEBUG  1

//<editor-fold desc="集合">

// 集合节点
typedef struct CollectionNode {
    HD_ACTION_ID_INFO *action_info;
    struct CollectionNode *prev;
    struct CollectionNode *next;
} CollectionNode;

// 集合结构
typedef struct {
    CollectionNode *head;
    CollectionNode *tail;
    int size;
    int max_size;
    int keep_ordered;  // 是否保持有序的标志
    pthread_mutex_t lock;
} OrderedCollection;

/* 辅助函数声明 */
static int compare_action_info(HD_ACTION_ID_INFO *a, HD_ACTION_ID_INFO *b);

static void
insert_node(OrderedCollection *col, CollectionNode *new_node, int (*on_action_id_removed)(const HD_ACTION_ID_INFO *));

// 初始化集合
OrderedCollection *collection_init(int max_size, int keep_ordered) {
    OrderedCollection *col = (OrderedCollection *) malloc(sizeof(OrderedCollection));
    if (!col) return NULL;

    col->head = NULL;
    col->tail = NULL;
    col->size = 0;
    col->max_size = max_size <= 0 ? HD_ACTION_INFO_MAX : max_size;
    col->keep_ordered = keep_ordered;
    pthread_mutex_init(&col->lock, NULL);

    return col;
}

// 释放集合
void collection_free(OrderedCollection *col) {
    if (!col) return;

    pthread_mutex_lock(&col->lock);

    CollectionNode *current = col->head;
    while (current) {
        CollectionNode *next = current->next;

        // 释放HD_ACTION_ID_INFO及其包含的HD_PIC_INFO
        if (current->action_info) {
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i]) {
                    free(current->action_info->pics[i]->path);
                    free(current->action_info->pics[i]);
                }
            }
            free(current->action_info->path);
            free(current->action_info);
        }

        free(current);
        current = next;
    }

    pthread_mutex_unlock(&col->lock);
    pthread_mutex_destroy(&col->lock);
    free(col);
}

// 比较两个action info
static int compare_action_info(HD_ACTION_ID_INFO *a, HD_ACTION_ID_INFO *b) {
    if (a->sort != b->sort) {
        return a->sort - b->sort;
    }
    if (a->action_id_timestamps != b->action_id_timestamps) {
        return a->action_id_timestamps - b->action_id_timestamps;
    }
    return a->action_id_index - b->action_id_index;
}

// 插入节点（内部函数）
static void
insert_node(OrderedCollection *col, CollectionNode *new_node, int (*on_action_id_removed)(const HD_ACTION_ID_INFO *)) {
    if (!col || !new_node || !new_node->action_info) return;

    // 1. 首先检查是否存在相同action_id的节点
    CollectionNode *current = col->head;
    CollectionNode *to_remove = NULL;

    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == new_node->action_info->action_id_timestamps &&
            current->action_info->action_id_index == new_node->action_info->action_id_index) {
            to_remove = current;
            break;
        }
        current = current->next;
    }

    // 2. 如果找到重复节点，先移除它
    if (to_remove) {
        if (on_action_id_removed != NULL) {
            on_action_id_removed(to_remove->action_info);
        }

        // 从链表中解除链接
        if (to_remove->prev) {
            to_remove->prev->next = to_remove->next;
        } else {
            col->head = to_remove->next;
        }

        if (to_remove->next) {
            to_remove->next->prev = to_remove->prev;
        } else {
            col->tail = to_remove->prev;
        }

        // 释放资源（但不释放pics，因为它们将被新节点接管）
        free(to_remove->action_info->path);
        free(to_remove->action_info);
        free(to_remove);
        col->size--;
    }

    // 3. 插入新节点
    if (col->keep_ordered) {
        // 有序插入：找到正确位置插入
        current = col->head;
        CollectionNode *prev = NULL;

        while (current && compare_action_info(new_node->action_info, current->action_info) > 0) {
            prev = current;
            current = current->next;
        }

        // 插入节点
        if (prev) {
            prev->next = new_node;
            new_node->prev = prev;
        } else {
            col->head = new_node;
        }

        new_node->next = current;
        if (current) {
            current->prev = new_node;
        } else {
            col->tail = new_node;
        }
    } else {
        // 无序插入：直接插入到链表尾部
        if (col->tail) {
            col->tail->next = new_node;
            new_node->prev = col->tail;
            col->tail = new_node;
        } else {
            col->head = col->tail = new_node;
        }
    }

    col->size++;
}

// 添加action到集合
int collection_add_action(OrderedCollection *col, HD_ACTION_ID_INFO *action_info,
                          int (*on_action_id_removed)(const HD_ACTION_ID_INFO *)) {
    if (!col || !action_info) return -1;

    pthread_mutex_lock(&col->lock);

    // 检查容量并移除最老的（如果有必要）
    if (col->size >= col->max_size) {
#if(HD_PIC_INFOS_DEBUG == 1)
        printf("collection_add_action over max : %d %d \n", col->size, col->max_size);
#endif
        CollectionNode *oldest = col->head;
        if (oldest) {
            col->head = oldest->next;
            if (col->head) {
                col->head->prev = NULL;
            } else {
                col->tail = NULL;
            }

            if (on_action_id_removed != NULL) {
                on_action_id_removed(oldest->action_info); // 通知删除文件
            }

            // 释放资源
            if (oldest->action_info) {
                for (int i = 0; i < oldest->action_info->pic_count; i++) {
                    if (oldest->action_info->pics[i]) {
                        free(oldest->action_info->pics[i]->path);
                        free(oldest->action_info->pics[i]);
                    }
                }
                free(oldest->action_info->path);
                free(oldest->action_info);
            }

            free(oldest);
            col->size--;

        }
    }

    // 创建新节点
    CollectionNode *new_node = (CollectionNode *) malloc(sizeof(CollectionNode));
    if (!new_node) {
        pthread_mutex_unlock(&col->lock);
        return -1;
    }
    new_node->action_info = action_info;
    new_node->prev = NULL;
    new_node->next = NULL;

    // 插入节点
    insert_node(col, new_node, on_action_id_removed);

    pthread_mutex_unlock(&col->lock);
    return 0;
}

// 根据action_id查找action
HD_ACTION_ID_INFO *collection_find_action(OrderedCollection *col,
                                          uint32_t action_id_timestamps,
                                          uint8_t action_id_index) {
    if (!col) return NULL;

    pthread_mutex_lock(&col->lock);

    CollectionNode *current = col->head;
    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == action_id_timestamps &&
            current->action_info->action_id_index == action_id_index) {
            pthread_mutex_unlock(&col->lock);
            return current->action_info;
        }
        current = current->next;
    }

    pthread_mutex_unlock(&col->lock);
    return NULL;
}

// 删除action
int collection_remove_action(OrderedCollection *col,
                             uint32_t action_id_timestamps,
                             uint8_t action_id_index) {
    if (!col) return -1;

    pthread_mutex_lock(&col->lock);

    CollectionNode *current = col->head;
    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == action_id_timestamps &&
            current->action_info->action_id_index == action_id_index) {

            // 从链表中移除节点
            if (current->prev) {
                current->prev->next = current->next;
            } else {
                col->head = current->next;
            }

            if (current->next) {
                current->next->prev = current->prev;
            } else {
                col->tail = current->prev;
            }

            // 释放资源
            if (current->action_info) {
                for (int i = 0; i < current->action_info->pic_count; i++) {
                    if (current->action_info->pics[i]) {
                        free(current->action_info->pics[i]->path);
                        free(current->action_info->pics[i]);
                    }
                }
                free(current->action_info->path);
                free(current->action_info);
            }
            free(current);
            col->size--;

            pthread_mutex_unlock(&col->lock);
            return 0;
        }
        current = current->next;
    }

    pthread_mutex_unlock(&col->lock);
    return -1;
}

//// 添加pic到指定的action（如果没有则创建）
//int collection_add_pic(OrderedCollection *col,
//                       uint32_t action_id_timestamps,
//                       uint8_t action_id_index,
//                       HD_PIC_INFO *pic_info) {
//    if (!col || !pic_info) return -1;
//
//    pthread_mutex_lock(&col->lock);
//
//    // 1. 首先尝试查找现有的action
//    CollectionNode *current = col->head;
//    while (current) {
//        if (!current->action_info->empty &&
//            current->action_info->action_id_timestamps == action_id_timestamps &&
//            current->action_info->action_id_index == action_id_index) {
//
//            // 检查是否达到最大pic数量
//            if (current->action_info->pic_count >= 50) {
//                pthread_mutex_unlock(&col->lock);
//                return -1;
//            }
//
//            // 添加pic到现有action
//            current->action_info->pics[current->action_info->pic_count++] = pic_info;
//            pthread_mutex_unlock(&col->lock);
//            return 0;
//        }
//        current = current->next;
//    }
//
//    // 2. 没有找到现有action，创建新的action
//
//    // 检查集合容量
//    if (col->size >= col->max_size) {
//        CollectionNode *oldest = col->head;
//        if (oldest) {
//            col->head = oldest->next;
//            if (col->head) {
//                col->head->prev = NULL;
//            } else {
//                col->tail = NULL;
//            }
//
//            // 释放资源
//            if (oldest->action_info) {
//                for (int i = 0; i < oldest->action_info->pic_count; i++) {
//                    if (oldest->action_info->pics[i]) {
//                        free(oldest->action_info->pics[i]->path);
//                        free(oldest->action_info->pics[i]);
//                    }
//                }
//                free(oldest->action_info->path);
//                free(oldest->action_info);
//            }
//            free(oldest);
//            col->size--;
//        }
//    }
//
//    // 创建新的action
//    HD_ACTION_ID_INFO *new_action = (HD_ACTION_ID_INFO *) malloc(sizeof(HD_ACTION_ID_INFO));
//    if (!new_action) {
//        pthread_mutex_unlock(&col->lock);
//        return -1;
//    }
//
//    // 初始化新的action
//    memset(new_action, 0, sizeof(HD_ACTION_ID_INFO));
//    new_action->action_id_timestamps = action_id_timestamps;
//    new_action->action_id_index = action_id_index;
//    new_action->sort = action_id_timestamps; // 默认使用时间戳作为排序值
//    new_action->empty = 0;
//    new_action->pic_count = 0;
//    new_action->path = NULL;
//
//    // 添加pic到新action
//    new_action->pics[new_action->pic_count++] = pic_info;
//
//    // 创建新节点
//    CollectionNode *new_node = (CollectionNode *) malloc(sizeof(CollectionNode));
//    if (!new_node) {
//        free(new_action);
//        pthread_mutex_unlock(&col->lock);
//        return -1;
//    }
//    new_node->action_info = new_action;
//    new_node->prev = NULL;
//    new_node->next = NULL;
//
//    // 插入节点
//    insert_node(col, new_node);
//
//    pthread_mutex_unlock(&col->lock);
//    return 0;
//}

HD_PIC_INFO *collection_find_pic_malloc(OrderedCollection *col,
                                        uint32_t action_id_timestamps,
                                        uint8_t action_id_index,
                                        uint16_t pic_id) {
    if (!col) return NULL;

    pthread_mutex_lock(&col->lock);

    HD_PIC_INFO *found_pic = NULL;
    HD_PIC_INFO *new_pic = NULL;

    // 查找对应的action
    CollectionNode *current = col->head;
    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == action_id_timestamps &&
            current->action_info->action_id_index == action_id_index) {

            // 在action中查找pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {
                    found_pic = current->action_info->pics[i];
                    break;
                }
            }
            break;
        } else {
            // 没有action_id 则遍历所有pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {
                    found_pic = current->action_info->pics[i];
                    break;
                }
            }
        }
        current = current->next;
    }

    if (found_pic) {
        // 分配新内存
        new_pic = (HD_PIC_INFO *) malloc(sizeof(HD_PIC_INFO));
        if (new_pic) {
            // 复制结构体内容
            memcpy(new_pic, found_pic, sizeof(HD_PIC_INFO));

            // 深度复制path字符串
            if (found_pic->path) {
                new_pic->path = strdup(found_pic->path);
                if (!new_pic->path) {
                    free(new_pic);
                    new_pic = NULL;
                }
            } else {
                new_pic->path = NULL;
            }
        }
    }

    pthread_mutex_unlock(&col->lock);
    return new_pic;
}

// 从指定的action中查找pic
HD_PIC_INFO *collection_find_pic(OrderedCollection *col,
                                 uint32_t action_id_timestamps,
                                 uint8_t action_id_index,
                                 uint16_t pic_id) {
    if (!col) return NULL;

    pthread_mutex_lock(&col->lock);

    // 查找对应的action
    CollectionNode *current = col->head;
    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == action_id_timestamps &&
            current->action_info->action_id_index == action_id_index) {

            // 在action中查找pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {
                    pthread_mutex_unlock(&col->lock);
                    return current->action_info->pics[i];
                }
            }
            break;
        } else {
            // 没有action_id 则遍历所有pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {
                    pthread_mutex_unlock(&col->lock);
                    return current->action_info->pics[i];
                }
            }
        }
        current = current->next;
    }


    pthread_mutex_unlock(&col->lock);
    return NULL;
}


/**
 * 将集合中所有HD_PIC_INFO提取到数组中
 * @param infos 输出参数，指向HD_PIC_INFO指针数组的指针
 * @param size 输出参数，返回数组大小
 * @return 成功返回0，失败返回-1
 */
static int collection_to_array(OrderedCollection *col, HD_PIC_INFO ***infos, size_t *size) {
    if (!col || !infos || !size) return -1;

    pthread_mutex_lock(&col->lock);

    // 首先计算总图片数量
    size_t total_pics = 0;
    CollectionNode *current = col->head;
    while (current) {
        total_pics += current->action_info->pic_count;
        current = current->next;
    }

    if (total_pics == 0) {
        *infos = NULL;
        *size = 0;
        pthread_mutex_unlock(&col->lock);
        return 0;
    }
    // 分配数组内存
    HD_PIC_INFO **pic_array = (HD_PIC_INFO **) malloc(total_pics * sizeof(HD_PIC_INFO *));
    if (!pic_array) {
        pthread_mutex_unlock(&col->lock);
        return -1;
    }

    // 填充数组
    size_t index = 0;
    current = col->head;
    while (current) {
        for (int i = 0; i < current->action_info->pic_count; i++) {
            HD_PIC_INFO *tmp = HD_PIC_INFO_free_deep_copy(current->action_info->pics[i]);
            if (tmp) {
                pic_array[index++] = tmp;
            }
        }
        current = current->next;
    }
    *infos = pic_array;
    *size = total_pics;

    pthread_mutex_unlock(&col->lock);
    return 0;
}

//// 使用示例：
//void example_usage(OrderedCollection *col) {
//    HD_PIC_INFO **pic_array = NULL;
//    size_t array_size = 0;
//
//    if (collection_to_array(col, &pic_array, &array_size) == 0) {
//        printf("Found %zu pictures:\n", array_size);
//        for (size_t i = 0; i < array_size; i++) {
//            printf("  Pic %zu: id=%u, path=%s\n",
//                   i, pic_array[i]->id, pic_array[i]->path);
//        }
//
//        // 注意：这里只是借用指针，不需要释放pic_array中的元素
//        free(pic_array); // 只需要释放数组本身
//    }
//}

// 从指定的action中删除pic
int collection_remove_pic(OrderedCollection *col,
                          uint32_t action_id_timestamps,
                          uint8_t action_id_index,
                          uint16_t pic_id, int do_free,
                          int (*on_pic_removed)(const HD_PIC_INFO *)
) {
    if (!col) return -1;

    pthread_mutex_lock(&col->lock);

    // 查找对应的action
    CollectionNode *current = col->head;
    while (current) {
        if (!current->action_info->empty &&
            current->action_info->action_id_timestamps == action_id_timestamps &&
            current->action_info->action_id_index == action_id_index) {

            // 在action中查找并删除pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {
                    if (on_pic_removed!=NULL){
                        on_pic_removed(current->action_info->pics[i]);
                    }
                    if (do_free) {
                        // 释放pic资源
                        free(current->action_info->pics[i]->path);
                        free(current->action_info->pics[i]);
                    }
                    // 将数组后面的元素前移
                    for (int j = i; j < current->action_info->pic_count - 1; j++) {
                        current->action_info->pics[j] = current->action_info->pics[j + 1];
                    }
                    current->action_info->pics[current->action_info->pic_count - 1] = NULL;
                    current->action_info->pic_count--;

                    pthread_mutex_unlock(&col->lock);
                    return 0;
                }
            }
            break;
        } else {
            // 没有action_id 则遍历所有pic
            for (int i = 0; i < current->action_info->pic_count; i++) {
                if (current->action_info->pics[i] &&
                    current->action_info->pics[i]->id == pic_id) {

                    if (on_pic_removed!=NULL){
                        on_pic_removed(current->action_info->pics[i]);
                    }

                    if (do_free) {
                        // 释放pic资源
                        free(current->action_info->pics[i]->path);
                        free(current->action_info->pics[i]);
                    }

                    // 将数组后面的元素前移
                    for (int j = i; j < current->action_info->pic_count - 1; j++) {
                        current->action_info->pics[j] = current->action_info->pics[j + 1];
                    }
                    current->action_info->pics[current->action_info->pic_count - 1] = NULL;
                    current->action_info->pic_count--;

                    pthread_mutex_unlock(&col->lock);
                    return 0;
                }
            }
        }
        current = current->next;
    }

    pthread_mutex_unlock(&col->lock);
    return -1;
}

// 设置有序开关
void collection_set_ordered(OrderedCollection *col, int keep_ordered) {
    if (!col) return;

    pthread_mutex_lock(&col->lock);
    col->keep_ordered = keep_ordered;
    pthread_mutex_unlock(&col->lock);
}

// 打印集合内容（用于调试）
void collection_print(OrderedCollection *col) {
    if (!col) return;

    pthread_mutex_lock(&col->lock);
#if(HD_PIC_INFOS_DEBUG == 1)
    printf("Collection (size=%d, ordered=%d):\n", col->size, col->keep_ordered);
#endif
    CollectionNode *current = col->head;
    while (current) {
        HD_ACTION_ID_INFO *action = current->action_info;
#if(HD_PIC_INFOS_DEBUG == 1)
        printf("  Action: ts=%u, idx=%u, sort=%u, empty=%d, pic_count=%d\n",
               action->action_id_timestamps, action->action_id_index,
               action->sort, action->empty, action->pic_count);
#endif
        for (int i = 0; i < action->pic_count; i++) {
            HD_PIC_INFO *pic = action->pics[i];
            printf("    Pic: id=%u, path=%s\n", pic->id, pic->path);
        }

        current = current->next;
    }

    pthread_mutex_unlock(&col->lock);
}

/* 示例用法 */
//int main() {
//    // 初始化集合，最大容量10，保持有序
//    OrderedCollection *col = collection_init(10, 1);
//
//    // 创建一个action和pic
//    HD_ACTION_ID_INFO *action1 = (HD_ACTION_ID_INFO*)malloc(sizeof(HD_ACTION_ID_INFO));
//    memset(action1, 0, sizeof(HD_ACTION_ID_INFO));
//    action1->action_id_timestamps = 123456789;
//    action1->action_id_index = 1;
//    action1->sort = 100;
//
//    HD_PIC_INFO *pic1 = (HD_PIC_INFO*)malloc(sizeof(HD_PIC_INFO));
//    memset(pic1, 0, sizeof(HD_PIC_INFO));
//    pic1->id = 1;
//    pic1->path = strdup("/path/to/pic1.jpg");
//
//    // 添加action到集合
//    collection_add_action(col, action1);
//
//    // 添加pic到action
//    collection_add_pic(col, 123456789, 1, pic1);
//
//    // 查找pic
//    HD_PIC_INFO *found_pic = collection_find_pic(col, 123456789, 1, 1);
//    if (found_pic) {
//        printf("Found pic: %s\n", found_pic->path);
//    }
//
//    // 打印集合内容
//    collection_print(col);
//
//    // 关闭有序模式
//    collection_set_ordered(col, 0);
//    printf("\nAfter turning off ordering:\n");
//    collection_print(col);
//
//    // 清理
//    collection_free(col);
//    return 0;
//}
//</editor-fold>

/***** 业务代码 *****/

#define KEY_MAX_SIZE                            512
#define MAX_FILE_SIZE                           (512*1024)      // 最大图片传输大小
#define PRINT_PIC_INFO_LEVEL                    0
#define PRINT_PIC_INFO_ITEM_LEVEL               0

pthread_mutex_t HD_PIC_INFOS_MUTEX;
static OrderedCollection *g_all_pic_infos;
static int g_max = HD_PIC_INFO_MAX;
static HD_PIC_INFO *g_doing_info = NULL;
static unsigned char g_file_buffer[MAX_FILE_SIZE];              // 图片文件缓冲区
static size_t g_file_buffer_size = -1;                          // 当前上传文件大小

static int load_file_to_buffer(const char *filename, unsigned char *data, size_t data_size, size_t *loaded_size) {
    if (filename == NULL) {
        printf("错误：文件名不能为NULL\n");
        return -1;
    }

    FILE *file = fopen(filename, "rb");
    if (file == NULL) {
        printf("无法打开文件 %s\n", filename);
        return -2;
    }
    size_t bytes_read = fread(data, 1, data_size, file);

    // 检查文件大小是否合适
    if (bytes_read < 0) {
        fprintf(stderr, "获取文件大小失败\n");
        fclose(file);
        return -1;
    }

    if ((size_t) bytes_read > data_size) {
        fprintf(stderr, "文件太大 (%ld字节)，最大支持 %zu字节\n",
                bytes_read, sizeof(data));
        fclose(file);
        return -1;
    }
    *loaded_size = bytes_read;
    fclose(file);
    printf("成功加载[%s] => <%zu>字节数据到缓冲区\n", filename, bytes_read);
    return 0;
}

// void HD_PIC_INFO_free(HD_PIC_INFO *info) {
//    if (info != NULL) {
//        if (info->path != NULL) {
//            free(info->path);
//            info->path = NULL;
//        }
//        free(info);
//    }
//
//}

//
//static int info_key_to_str(uint8_t pic_id, uint32_t pic_snapshot_timestamps, uint32_t action_id_timestamps,
//                           uint8_t action_id_index, char key[KEY_MAX_SIZE]) {
//    if (key == NULL)return 1;
//    // 参考：do_action_id_2_str
//    //   snprintf(str, str_size, "%d%03d", action_id_timestamps, action_id_index);
//    snprintf(key, KEY_MAX_SIZE, "%d_%03d_%d_%03d", action_id_timestamps, action_id_index,
//             pic_snapshot_timestamps, pic_id);
//    return 0;
//}
//
//static void free__pic_info(void *info) {
//    if (info == NULL)return;
//    HD_PIC_INFO *free_info = (HD_PIC_INFO *) info;
//    HD_PIC_INFO_free(free_info);
//}
//
static void print__pic_info(void *info) {
    if (info == NULL)return;

    HD_PIC_INFO *print_info = (HD_PIC_INFO *) info;
//    if (PRINT_PIC_INFO_ITEM_LEVEL == 0) {
//        printf("%s\n", print_info->path);
//        return;
//    }
    printf("[%s]id                      :        %d\n", HD_PIC_INFOS_TAG, print_info->id);
    printf("[%s]action_id_timestamps    :        %d\n", HD_PIC_INFOS_TAG, print_info->action_id_timestamps);
    printf("[%s]action_id_index         :        %d\n", HD_PIC_INFOS_TAG, print_info->action_id_index);
    printf("[%s]trigger_type            :        %d\n", HD_PIC_INFOS_TAG, print_info->trigger_type);
    printf("[%s]trigger_angel           :        %d\n", HD_PIC_INFOS_TAG, print_info->trigger_angel);
    printf("[%s]snapshot_timestamps     :        %d\n", HD_PIC_INFOS_TAG, print_info->snapshot_timestamps);
    printf("[%s]size                    :        %d\n", HD_PIC_INFOS_TAG, print_info->size);
    printf("[%s]sort                    :        %d\n", HD_PIC_INFOS_TAG, print_info->sort);
    printf("[%s]path                    :        %s\n", HD_PIC_INFOS_TAG, print_info->path);
    printf("[%s]md5                     :        [", HD_PIC_INFOS_TAG);
    for (int i = 0; i < 16; ++i) {
        printf("%02x ", print_info->md5[i]);
    }
    printf("]\n");
    printf("---\n");
}

static void hd_pic_infos_print(const char *tag) {
    collection_print(g_all_pic_infos);
//    printf("[%s]%s>>>>>>>>打印集合。集合大小:%zu<<<<<<<<<<\n", HD_PIC_INFOS_TAG, tag, g_all_pic_infos->size);
//    if (PRINT_PIC_INFO_LEVEL == 0) {
//
//    } else {
//        list_for_each(&g_all_pic_infos, print__pic_info);
//    }


}

int hd_pic_infos_init(int max) {
    printf("[%s]hd_pic_infos_init %d\n", HD_PIC_INFOS_TAG, max);
    g_doing_info = NULL;
    g_file_buffer_size = -1;
    OrderedCollection *col = collection_init(max, 0);
    if (col == NULL) {
        printf("[%s]hd_queue_create error  \n", HD_PIC_INFOS_TAG);
        return 1;
    }
    g_all_pic_infos = col;
    hd_pic_infos_print("hd_pic_infos_init");
    return 0;
}


int hd_pic_infos_add(HD_ACTION_ID_INFO *info, int (*on_action_id_removed)(const HD_ACTION_ID_INFO *)) {
    printf("[%s]添加图片信息\n", HD_PIC_INFOS_TAG);
    if (info == NULL)return 1;
    int ret = collection_add_action(g_all_pic_infos, info, on_action_id_removed);
    hd_pic_infos_print("hd_pic_infos_add");
    return ret;
}


//static void hd_pic_infos_delete_foreach(void * value){
//    int ret = 0;
//    char key[KEY_MAX_SIZE];
//        HD_PIC_INFO *info = (HD_PIC_INFO *) value;
//        if (info != NULL) {
//            if (info->id == id) {
//                ret = info_key_to_str(id, -1, action_id_timestamps, action_id_index, key);
//            }
//        }
//    map_iterator_end(&it);
//}


static int compare_pic_info(uint16_t pic_id,
                            uint32_t action_id_timestamps,
                            uint8_t action_id_index,
                            HD_PIC_INFO *target, int empty) {
    printf("------------------开始比较------------------\n");
    if ((empty != 0) || (action_id_timestamps < 0) ||
        (action_id_index < 0)) {
        printf("[compare_pic_info]只对比pic_info     :   %d %d \n", target->id, pic_id);
        if (target->id == (pic_id)) {
            printf("[compare_pic_info]匹配成功     :   %d %d \n", target->id, pic_id);
            return 0;
        }
    } else {
        printf("[compare_pic_info]完整对比pic_id     :   %d %d \n", target->id, pic_id);
        if ((action_id_timestamps == target->action_id_timestamps) &&
            (action_id_index == target->action_id_index) && pic_id == (target->id)) {
            printf("[compare_pic_info]匹配成功     :   %d %d \n", target->id, pic_id);
            return 0;
        }
    }
    return -1;
}


/**
 *
 * 会出现action_id_timestamps=-1，action_id_index=-1，即只传了pic_id的情况。此时为温控器通过协议下发，如果存在pic_id相同的都会被删除
 *
 * @param pic_id                    pic_id 一个action_id     下必定唯一
 * @param action_id_timestamps  action_id_timestamps    可能为-1 代表不传入
 * @param action_id_index       action_id_index         可能为-1 代表不传入
 * @return
 */
int hd_pic_infos_delete(
        uint16_t pic_id,
        uint32_t action_id_timestamps,
        uint8_t action_id_index,
        int only_delete_by_pic_id,
        int (*on_pic_removed)(const HD_PIC_INFO *)
) {
    printf("[%s]删除图片信息 pic_id:%02x,action_id_timestamps:%02x,action_id_index:%02x\n", HD_PIC_INFOS_TAG, pic_id,
           action_id_timestamps, action_id_index);
    int ret = collection_remove_pic(g_all_pic_infos, action_id_timestamps, action_id_index, pic_id, 1, on_pic_removed);
    hd_pic_infos_print("hd_pic_infos_delete");
    return ret;

}

//int hd_pic_infos_get(
//        uint16_t pic_id,
//        uint32_t action_id_timestamps,
//        uint8_t action_id_index,
//        HD_PIC_INFO **info) {
//    printf("[%s]hd_pic_infos_get\n", HD_PIC_INFOS_TAG);
//    HD_PIC_INFO *found = collection_find_pic_malloc(g_all_pic_infos, action_id_timestamps, action_id_index, pic_id);
//    if (found == NULL) {
//        *info = NULL;
//        return 1;
//    }
//    *info = found;
//    return 0;
//}

int hd_pic_infos_get_all(HD_PIC_INFO ***infos, size_t *size) {
    printf("[%s]hd_pic_infos_get_all\n", HD_PIC_INFOS_TAG);
    if (infos == NULL)return 2;
    int ret = collection_to_array(g_all_pic_infos, infos, size);
    return ret;
}

static void reset_file_buffer() {
    g_file_buffer_size = -1;
    if (g_doing_info != NULL) {
        HD_PIC_INFO_free(g_doing_info);
        g_doing_info = NULL;
    }
}

void hd_pic_infos_deinit() {
    printf("[%s]hd_pic_infos_deinit\n", HD_PIC_INFOS_TAG);
    reset_file_buffer();
    collection_free(g_all_pic_infos);
}


void HD_PIC_INFO_free(HD_PIC_INFO *info) {
    if (info == NULL)return;
    if (info->path != NULL) {
        free(info->path);
    }
    free(info);
}

void HD_ACTION_ID_INFOs_free(HD_ACTION_ID_INFO **infos, size_t size) {
    if (infos == NULL) {
        return;
    }
    for (int i = 0; i < size; ++i) {
        HD_ACTION_ID_INFO_free(infos[i]);
    }
    free(infos);
}

void HD_ACTION_ID_INFO_free(HD_ACTION_ID_INFO *info) {
    if (info == NULL)return;
    if (info->path != NULL) {
        free(info->path);
    }
    HD_PIC_INFOs_free(info->pics, info->pic_count);
    free(info);
}

void HD_PIC_INFOs_free(HD_PIC_INFO **infos, size_t size) {
    if (infos == NULL) {
        return;
    }
    if (size > 0) {
        for (int i = 0; i < size; ++i) {
            HD_PIC_INFO_free(infos[i]);
        }
    }

    free(infos);
}

HD_PIC_INFO *HD_PIC_INFO_free_deep_copy(HD_PIC_INFO *info) {
    if (info == NULL)return NULL;
    HD_PIC_INFO *copy = malloc(sizeof(HD_PIC_INFO));
    if (!copy)return NULL;
    copy->id = info->id;
    copy->action_id_timestamps = info->action_id_timestamps;
    copy->action_id_index = info->action_id_index;
    copy->trigger_type = info->trigger_type;
    copy->trigger_angel = info->trigger_angel;
    copy->snapshot_timestamps = info->snapshot_timestamps;
    copy->size = info->size;
    for (int i = 0; i < 16; ++i) {
        copy->md5[i] = info->md5[i];
    }
    copy->sort = info->sort;
    copy->path = strdup (info->path);
    return copy;
}

static int hd_pic_infos_load_file_get(unsigned char *result, size_t in_size,
                                      size_t in_offset, size_t *result_size) {
//    if (HD_UART_PARSER_DEBUG) {
//        LOGI("[read_from_buffer] g_file_buffer_size = %d\n", g_file_buffer_size);
//    }
    // 参数检查
    if (result == NULL) {
        fprintf(stderr, "错误：目标数组不能为NULL\n");
        return 1;
    }
    // 检查读取长度是否合理
    if (in_size == 0) {
        fprintf(stderr, "警告：请求读取0字节\n");
        *result_size = 0;
        return 2;
    }

    if (g_doing_info == NULL) {
        fprintf(stderr, "g_doing_info is NULL\n");
        return 3;
    }

    // 检查请求是否超出缓冲区范围
    if (in_offset >= g_file_buffer_size) {
        return -2;
    }
    size_t real_read_size = in_size;
//    if (HD_UART_PARSER_DEBUG) {
//        LOGI("[read_from_buffer] in_offset = %d \n", in_offset);
//        LOGI("[read_from_buffer] in_size = %d \n", in_size);
//        LOGI("[read_from_buffer] g_file_buffer_size = %d \n", g_file_buffer_size);
//        LOGI("[read_from_buffer] in_offset + in_size - g_file_buffer_size = %d \n", g_file_buffer_size - in_offset);
//    }
    if (g_file_buffer_size - in_offset < in_size) {
        real_read_size = g_file_buffer_size - in_offset;
    }

//    if (HD_UART_PARSER_DEBUG) {
    printf("[read_from_buffer] real_read_size = %zu \n", real_read_size);
//    }
    if (real_read_size <= 0) {
        printf("real_read_size==0,没有数据可读了\n");
        return 1;
    }
    *result_size = real_read_size;
    memcpy(result, g_file_buffer + in_offset, real_read_size);

    return 0;
}

int hd_pic_infos_pull(uint16_t pic_id,
                      uint32_t action_id_timestamps,
                      uint8_t action_id_index,
                      unsigned char *result, size_t in_size,
                      size_t in_offset, size_t *result_size,
                      int (*on_pic_removed)(const HD_PIC_INFO *)

) {
    printf("[%s]拉取图片信息 \n", HD_PIC_INFOS_TAG);
    // 1.加锁
    pthread_mutex_lock(&HD_PIC_INFOS_MUTEX);

    int error = 0;
    while (1) {

        // 1.检查是否在上传
        printf("1.检查是否在上传... ...\n");
        int loaded = 0;
        if (g_doing_info != NULL && g_file_buffer_size > 0) {
            printf("正在上传的图片... ...\n");
            print__pic_info(g_doing_info);
            if (compare_pic_info(pic_id, action_id_timestamps, action_id_index, g_doing_info, 1) == 0) {
                printf("正在上传的图片匹配成功\n");
                loaded = 1;
            } else {
                printf("正在上传的图片匹配失败,加载新的图片\n");
                reset_file_buffer();
                loaded = 0;
            }
        }

        // 2.预加载图片
        printf("2.预加载图片... ...\n");
        if (loaded) {
            error = 0;
        } else {
            printf("没有正在上传的图片,查找图片中...\n");
            HD_PIC_INFO *found = collection_find_pic_malloc(g_all_pic_infos, action_id_timestamps,
                                                            action_id_index, pic_id); // 复制一个新的保证不能其他线程free掉当前上传的指针
            if (found == NULL) {
                printf("图片信息不存在\n");
                error = 2;
                break;
            }
            print__pic_info(found);
            printf("没有正在上传的图片,加载图片数据中...\n");
            int ret = load_file_to_buffer(found->path, g_file_buffer, sizeof(g_file_buffer), &g_file_buffer_size);
            if (ret) {
                error = 3;
                reset_file_buffer();
                break;
            }
            g_doing_info = found;

            printf("没有正在上传的图片,加载图片数据成功！删除图片信息。\n");
            ret = collection_remove_pic(g_all_pic_infos, action_id_timestamps, action_id_index, pic_id, 1,
                                        on_pic_removed);
            if (ret) {
                printf("没有正在上传的图片,加载图片数据成功！删除图片信息 失败 %d\n", ret);
                error = 4;
            } else {
                error = 0;
            }
            break;
        }
        break; // 保底break
    }

    // 3.检查完毕,读取数据
    printf("3.检查完毕,读取数据... ...\n");
    int ret;
    while (1) {
        if (error) {
            printf("检查完毕 error = %d\n", error);
            ret = error;
            break;
        }
        printf("分段加载图片数据... \n");
        ret = hd_pic_infos_load_file_get(result, in_size, in_offset, result_size);
        if (ret) {
            break;
        }

        if (*result_size < in_offset) {
            printf("分段加载图片数据,完整拉取了！ \n");
            // TODO 需要清除吗？还是等温控器发命令来，否则数据就没了 。
        }
        break;
    }

    // 3.释放锁
    pthread_mutex_unlock(&HD_PIC_INFOS_MUTEX);
    return ret;
}
