#ifndef __HD_QUEUE_1__
#define __HD_QUEUE_1__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    void **items;        // 使用void*指针数组存储任意类型数据
    int capacity;       // 队列容量
    int size;           // 当前元素数量
    int front;          // 队列头指针
    int rear;           // 队列尾指针
    pthread_mutex_t mutex;          // 互斥锁
    pthread_cond_t not_empty;       // 非空条件变量
    pthread_cond_t not_full;        // 非满条件变量
} HDBlockingQueue;

// 初始化队列
HDBlockingQueue* hd_queue_create(int capacity);

// 销毁队列
void hd_queue_destroy(HDBlockingQueue *queue) ;

// 向队列添加元素（阻塞直到有空间）
void hd_queue_put(HDBlockingQueue *queue, void *item) ;

// 从队列取出元素（阻塞直到有元素）
void* hd_queue_take(HDBlockingQueue *queue);

// 获取队列当前大小
int hd_queue_size(HDBlockingQueue *queue) ;
#ifdef __cplusplus
}
#endif

#endif // __HD_QUEUE_1__
