#ifndef H__HD_QUEUE_1__H
#define H__HD_QUEUE_1__H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        void **items;        // 使用void*指针数组存储任意类型数据
        int capacity;       // 队列容量
        int size;           // 当前元素数量
        int front;          // 队列头指针
        int rear;           // 队列尾指针
        pthread_mutex_t mutex;          // 互斥锁
        pthread_cond_t not_empty;       // 非空条件变量
        pthread_cond_t not_full;        // 非满条件变量
        int terminated; // 添加终止标志
    } HDBlockingQueue;

    // 初始化队列
    HDBlockingQueue* hd_queue_create(int capacity);

    // 销毁队列
    void hd_queue_destroy(HDBlockingQueue *queue,void(*item)(void**));

    // 向队列添加元素（阻塞直到有空间）
    int hd_queue_put(HDBlockingQueue *queue, void *item) ;

    // 从队列取出元素（阻塞直到有元素）
    void* hd_queue_take(HDBlockingQueue *queue);

    // 获取队列当前大小
    int hd_queue_size(HDBlockingQueue *queue) ;

    int hd_queue_get_all(HDBlockingQueue *queue,void **items,int items_size);
    void * hd_queue_get(void * item);
    int hd_queue_delete(void * item);



// ###

typedef struct {
    uint8_t *items;        // 使用void*指针数组存储任意类型数据
    int capacity;       // 队列容量
    int size;           // 当前元素数量
    int front;          // 队列头指针
    int rear;           // 队列尾指针
    pthread_mutex_t mutex;          // 互斥锁
    pthread_cond_t not_empty;       // 非空条件变量
    pthread_cond_t not_full;        // 非满条件变量
    int terminated; // 添加终止标志
} HDBlockingQueueUint8;

// 初始化队列
HDBlockingQueueUint8* hd_queue_create_uint8(int capacity);

// 销毁队列
void hd_queue_destroy_uint8(HDBlockingQueueUint8 *queue) ;

int hd_queue_offer_uint8(HDBlockingQueueUint8 *queue, uint8_t item);

// 向队列添加元素（阻塞直到有空间）
void hd_queue_put_uint8(HDBlockingQueueUint8 *queue, uint8_t item) ;

// 从队列取出元素（阻塞直到有元素）
uint8_t hd_queue_take_uint8(HDBlockingQueueUint8 *queue);

// 获取队列当前大小
int hd_queue_size_uint8(HDBlockingQueueUint8 *queue) ;


#ifdef __cplusplus
}
#endif

#endif // H__HD_QUEUE_1__H
