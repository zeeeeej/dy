#ifndef H__HD_UART_PARSER__H
#define H__HD_UART_PARSER__H

#ifdef __cplusplus
extern "C" {
#endif

#define  EVENT_HD_SNAPSHOT                  0x01    // 主动抓图。
#define  EVENT_HD_DELETE_ALL_FILE           0x02    // 删除所有图片。
#define  EVENT_HD_DELETE_PIC                0x03    // 删除图片，参数：图片path 字符串长度
#define  EVENT_HD_DELETE_ACTION_ID          0x04    // 删除图片，参数：action_id_path 字符串长度

/**
 * 收到3.17 广播门开事件（0x1E）
 * @param status 0:关闭；1：开启。
 * @param action_id_str action_id（时间戳（4字节）+序号（1字节））转成的字符串格式：时间戳（单位：秒）+序号（3位的10进制，范围0～255）
 */
typedef void (*hd_on_action_id_changed)(uint8_t status, const char *action_id_str);

typedef void *(*hd_on_event)(int event_id, void *event_value, size_t event_value_size);

/**
 * 初始化
 *
 * 初始化例子：
 * uint8_t addr = 0x01;
 * 1.初始化qjy_uart_init
 *      recv_callback_func func = {qjy_uart_parser, hd_uart_recv};
 *      qjy_uart_init(&func, addr);
 * 2.初始化hd_uart_init
 *      hd_uart_init(addr,"/userdata/jpeg",on_action_id_changed,on_event);
 *
 * @param addr                  从机地址。01:静态摄像头；02：动态摄像头。
 * @param pic_dir_path          图片保存的地址
 * @param callback              action_id回调
 * @return                      0：成功；1：失败。
 */
int hd_uart_init(
        uint8_t addr,
        const char *pic_dir_path,
        const char *version,
        hd_on_action_id_changed on_action_id_changed,
        hd_on_event on_event
//        ,int(*transform_pic)(const char *, char *)
);

/**
 * 解析函数
 * @param byte
 */
void hd_uart_recv(uint8_t byte);

/**
 * 反初始化
 */
void hd_uart_deinit();

/**
 * 图片新增
 *
 * app生成一个完整图片后，调用此方法将此图片添加到【图片信息缓存】中。如果超过最大限制，则需要从图片信息缓存中按指定的【删除策略】发出删除事
 * 件【EVENT_HD_DELETE_ACTION_ID】。app收到该事件后，删除图片，并返回结果1：删除失败；0：删除成功。收到结果将该图片信息从【图片信息缓存】中删除。
 *【删除策略】不在使用中的最老的图片。
 *
 * @param action_id_str 新增图片的文件夹名称，即：action_id_str。注意是外部malloc，需要free.
 * @param array         新增图片数组。存储的是每一个新增图片的绝对路径；图片名称包含了图片信息，例如：文件md5，文件大小，拍照时间，角度等等。注意是外部malloc，需要free
 * @param array_size    新增图片数组大小。
 * @return 1：失败 0：成功
 */
int hd_uart_on_pic_add(char *action_id_str, char **array, int array_size);
// int hd_uart_on_pic_add(const char *action_id_str, char * const *array, int array_size);

char *hd_uart_version();

#ifdef __cplusplus
}
#endif

#endif // H__HD_UART_PARSER__H