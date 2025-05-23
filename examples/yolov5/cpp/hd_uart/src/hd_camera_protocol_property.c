#include <stdint.h>
#include "hd_camera_protocol.h"
#include "hd_camera_protocol_cmd.h"

// 属性ID TODO
static  struct {

    uint8_t id;
    /**
     * 数据类型：
     * 0:uint8_t
     * 1:string
     * 2:uint32_t
     * 3:struct
     */
    uint8_t type;
    /**
     * 数据长度
     * -1 : n
     */
    u_int8_t len;
    /**
     * 读写
     * 0b11
     * bit0:读
     * bit1:写
     */
    u_int8_t rw;
} HDCameraProperty;


static  void test(){
    struct  HDCameraProperty *p
            ;


}
