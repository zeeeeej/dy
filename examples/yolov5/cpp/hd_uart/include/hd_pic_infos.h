#ifndef H__HD_PIC_INFOS__H
#define H__HD_PIC_INFOS__H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HD_PIC_INFO_MAX             50
#define HD_ACTION_INFO_MAX          50


/**
 * 图片信息流程
 *
 * 1。收到温控器开门事件,发送action_id给app，app开启拍摄任务，开启拍摄。
 * 2。收到温控器关门事件，发送action_id给app，app停止拍摄，处理图片，生成目标图片。
 * 3。每次生成图片，通知lib，lib添加图片信息。添加时，如果超过50张，则删除最老的那一张。
 * 4。收到温控器删除命令，lib从图片信息里找到对应的信息，找出path，给app删除命令，app删除完毕，返回给lib，lib再删除此图片信息。
 * 5。收到温控器拉取图片信息命令
 * （1）没有加载中。lib从图片信息里找到对应的信息，并删除掉，找出path，加载图片。
 * （2）加载中，从加载中直接拉取。
 */

/**
 *  用c语言设计一个集合，可以使用链表，也可以使用map，但是必须有序。
 *  1。线程安全
 *  2。数据时HD_ACTION_ID_INFO，里面有多个HD_PIC_INFO最大50个，可用用map或者数组
 *  3。添加时判断最大容量，移除最老的。
 *  4。支持查询HD_PIC_INFO 删除HD_PIC_INFO 删除HD_ACTION_ID_INFO 添加HD_PIC_INFO等常规方法
 */
typedef struct {
    /** pic_id 2BYTE */
    uint16_t id;
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
    /** 排序 */
    uint32_t sort;
    /** 路径 */
    char *path;
} HD_PIC_INFO;

typedef struct {
    uint32_t action_id_timestamps;
    uint8_t action_id_index;
    uint32_t sort;
    char *path;
    /* 空的action_id 此时action_id_timestamps、action_id_index无效 */
    int empty;
    HD_PIC_INFO *pics[HD_PIC_INFO_MAX]; // 使用map
    int pic_count;
} HD_ACTION_ID_INFO;

/**
 * 初始化
 *
 * @param max       action_id上限
 * @return          0：成功 1：失败
 */
int hd_pic_infos_init(int max);

/**
 * 添加图片
 *
 * @param info                      action_id信息
 * @param on_action_id_removed      action_id删除回调
 * @return                          0：成功 1：失败
 */
int hd_pic_infos_add(HD_ACTION_ID_INFO *info, int (*on_action_id_removed)(const HD_ACTION_ID_INFO *));

/**
 * 删除图片
 *
 * @param pic_id                        图片id
 * @param action_id_timestamps          action_id时间戳
 * @param action_id_index               action_id序号
 * @param only_delete_by_pic_id         是否只通过图片id删除，1：是；0：否。
 * @param on_pic_removed                图片删除回调
 * @return                              0：成功 1：失败
 */
int hd_pic_infos_delete(uint16_t pic_id,
                        uint32_t action_id_timestamps,
                        uint8_t action_id_index,
                        int only_delete_by_pic_id,
                        int (*on_pic_removed)(const HD_PIC_INFO *)
);

/**
 * 获取所有图片信息
 *
 * @param infos         图片信息【malloc 需要释放】
 * @param size          图片信息大小
 * @return              0：成功 1：失败
 */
int hd_pic_infos_get_all(HD_PIC_INFO ***infos, size_t *size);

/**
 * 拉取图片
 *
 * @param pic_id                    图片id
 * @param action_id_timestamps      action_id时间戳
 * @param action_id_index           action_id序号
 * @param result                    拉取结果
 * @param in_size                   需要拉取size
 * @param in_offset                 需要拉取offset
 * @param result_size               拉取结果大小
 * @param on_pic_removed            图片删除回调 0：成功 1：失败
 * @return                          0：成功 1：失败
 */
int hd_pic_infos_pull(uint16_t pic_id,
                      uint32_t action_id_timestamps,
                      uint8_t action_id_index,
                      unsigned char *result, size_t in_size,
                      size_t in_offset, size_t *result_size,
                      int (*on_pic_removed)(const HD_PIC_INFO *)
);

/**
 * 反初始化
 */
void hd_pic_infos_deinit();

void HD_PIC_INFO_free(HD_PIC_INFO * info);

void HD_PIC_INFOs_free(HD_PIC_INFO ** infos,size_t size);

void HD_ACTION_ID_INFOs_free(HD_ACTION_ID_INFO** infos,size_t size);

void HD_ACTION_ID_INFO_free(HD_ACTION_ID_INFO * info);

HD_PIC_INFO * HD_PIC_INFO_free_deep_copy(HD_PIC_INFO * info);

#ifdef __cplusplus
}
#endif

#endif // H__HD_PIC_INFOS__H
