#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "hd_queue.h"

// 初始化队列
HDBlockingQueue* hd_queue_create(int capacity) {
    HDBlockingQueue *queue = (HDBlockingQueue*)malloc(sizeof(HDBlockingQueue));
    if (!queue) {
        perror("Failed to allocate memory for queue");
        return NULL;
    }

    queue->items = (void**)malloc(sizeof(void*) * capacity);
    if (!queue->items) {
        perror("Failed to allocate memory for items");
        free(queue);
        return NULL;
    }

    queue->capacity = capacity;
    queue->size = 0;
    queue->front = 0;
    queue->rear = -1;

    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);

    return queue;
}

// 销毁队列
void hd_queue_destroy(HDBlockingQueue *queue) {
    if (queue) {
        pthread_mutex_destroy(&queue->mutex);
        pthread_cond_destroy(&queue->not_empty);
        pthread_cond_destroy(&queue->not_full);
        free(queue->items);
        free(queue);
    }
}

// 向队列添加元素（阻塞直到有空间）
void hd_queue_put(HDBlockingQueue *queue, void *item) {
//    printf("[queue] hd_queue_put (%d)%p \n",queue->size,item);
    pthread_mutex_lock(&queue->mutex);

    // 如果队列已满，等待直到有空间
    while (queue->size == queue->capacity) {
        printf("[queue] hd_queue_put full.\n");
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    // 添加元素
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->items[queue->rear] = item;
    queue->size++;

    // 通知可能正在等待的消费者
    pthread_cond_signal(&queue->not_empty);

    pthread_mutex_unlock(&queue->mutex);
}

// 从队列取出元素（阻塞直到有元素）
void* hd_queue_take(HDBlockingQueue *queue) {
//    printf("[queue] hd_queue_take (%d) \n",queue->size);
    pthread_mutex_lock(&queue->mutex);

    // 如果队列为空，等待直到有元素
    while (queue->size == 0) {
//        printf("[queue] hd_queue_take empty.\n");
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    // 取出元素
    void *item = queue->items[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size--;

    // 通知可能正在等待的生产者
    pthread_cond_signal(&queue->not_full);

    pthread_mutex_unlock(&queue->mutex);

    return item;
}

// 获取队列当前大小
int hd_queue_size(HDBlockingQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}

//// 测试代码 - 自定义数据类型
//typedef struct {
//    int id;
//    char name[20];
//    double value;
//} CustomData;
//
//void* producer(void *arg) {
//    BlockingQueue *queue = (BlockingQueue*)arg;
//    for (int i = 0; i < 10; i++) {
//        // 创建自定义数据
//        CustomData *data = malloc(sizeof(CustomData));
//        data->id = i;
//        snprintf(data->name, sizeof(data->name), "Item-%d", i);
//        data->value = i * 3.14;
//
//        put(queue, data);
//        printf("Produced: ID=%d, Name=%s, Value=%.2f, Queue size: %d\n",
//               data->id, data->name, data->value, queue_size(queue));
//        usleep(150000); // 模拟生产耗时
//    }
//    return NULL;
//}
//
//void* consumer(void *arg) {
//    BlockingQueue *queue = (BlockingQueue*)arg;
//    for (int i = 0; i < 10; i++) {
//        CustomData *data = (CustomData*)take(queue);
//        printf("Consumed: ID=%d, Name=%s, Value=%.2f, Queue size: %d\n",
//               data->id, data->name, data->value, queue_size(queue));
//        free(data); // 释放内存
//        usleep(250000); // 模拟消费耗时
//    }
//    return NULL;
//}

//int main() {
//    BlockingQueue *queue = create_queue(3); // 小容量队列更容易观察阻塞行为
//    if (!queue) {
//        return 1;
//    }
//
//    pthread_t producer_thread, consumer_thread;
//    pthread_create(&producer_thread, NULL, producer, queue);
//    pthread_create(&consumer_thread, NULL, consumer, queue);
//
//    pthread_join(producer_thread, NULL);
//    pthread_join(consumer_thread, NULL);
//
//    destroy_queue(queue);
//    return 0;
//}


// ###

// 初始化队列
HDBlockingQueueUint8* hd_queue_create_uint8(int capacity) {
    HDBlockingQueueUint8 *queue = (HDBlockingQueueUint8*)malloc(sizeof(HDBlockingQueueUint8));
    if (!queue) {
        perror("Failed to allocate memory for queue");
        return NULL;
    }

    queue->items = (uint8_t *)malloc(sizeof(uint8_t) * capacity);
    if (!queue->items) {
        perror("Failed to allocate memory for items");
        free(queue);
        return NULL;
    }

    queue->capacity = capacity;
    queue->size = 0;
    queue->front = 0;
    queue->rear = -1;

    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);

    return queue;
}

// 销毁队列
void hd_queue_destroy_uint8(HDBlockingQueueUint8 *queue) {
    if (queue) {
        pthread_mutex_destroy(&queue->mutex);
        pthread_cond_destroy(&queue->not_empty);
        pthread_cond_destroy(&queue->not_full);
        free(queue->items);
        free(queue);
    }
}

// 向队列添加元素（阻塞直到有空间）
void hd_queue_put_uint8(HDBlockingQueueUint8 *queue, uint8_t item) {
//    printf("[queue] hd_queue_put (%d)%p \n",queue->size,item);
    pthread_mutex_lock(&queue->mutex);

    // 如果队列已满，等待直到有空间
    while (queue->size == queue->capacity) {
        printf("[queue] hd_queue_put full.\n");
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    // 添加元素
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->items[queue->rear] = item;
    queue->size++;

    // 通知可能正在等待的消费者
    pthread_cond_signal(&queue->not_empty);

    pthread_mutex_unlock(&queue->mutex);
}

// 从队列取出元素（阻塞直到有元素）
uint8_t  hd_queue_take_uint8(HDBlockingQueueUint8 *queue) {
//    printf("[queue] hd_queue_take (%d) \n",queue->size);
    pthread_mutex_lock(&queue->mutex);

    // 如果队列为空，等待直到有元素
    while (queue->size == 0) {
//        printf("[queue] hd_queue_take empty.\n");
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    // 取出元素
    uint8_t item = queue->items[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size--;

    // 通知可能正在等待的生产者
    pthread_cond_signal(&queue->not_full);

    pthread_mutex_unlock(&queue->mutex);

    return item;
}

// 获取队列当前大小
int hd_queue_size_uint8(HDBlockingQueueUint8 *queue) {
    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);
    return size;
}