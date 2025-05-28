#ifndef __DATA_H__
#define __DATA_H__
#include <stdint.h>
#include <semaphore.h>

#define JPEG_VENC_CHN 0

#define bswap_16(A) (( ((uint16_t)(A) & 0xff00) >> 8) | (( (uint16_t)(A) & 0x00ff) << 8))  
 
#define bswap_32(A) ((( (uint32_t)(A) & 0xff000000) >> 24) | \
                          (( (uint32_t)(A) & 0x00ff0000) >> 8)  | \
                          (( (uint32_t)(A) & 0x0000ff00) << 8)  | \
                          (( (uint32_t)(A) & 0x000000ff) << 24))


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t pic_id;
    uint8_t type;
    uint8_t angle;
    uint32_t time;
    uint32_t size;
    uint8_t md5[16];
}PICTURE_INFO;

const static char __APP_VERSION__[] = "0.4.6";

sem_t image_sem, uart_sem;

uint16_t crc_16(uint8_t *data, uint32_t len);
int pthread_sem_init();
int pthread_sem_deinit();
uint8_t get_pic_info(uint8_t *data);
int get_pic_id();
int get_pic_from_id(uint8_t id, uint8_t *data);
uint8_t del_pic_from_id(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif
