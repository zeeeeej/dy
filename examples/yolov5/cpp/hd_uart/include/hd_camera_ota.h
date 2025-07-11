
#ifndef HD_CAMERA_OTA_MODEL_H
#define HD_CAMERA_OTA_MODEL_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "hd_camera.h"

int hd_camera_ota_init(uint8_t addr);

void hd_camera_ota_deinit(void);

int hd_camera_ota_model_handle_cmd(
        unsigned char *payload_data,
        uint32_t payload_data_size
);

void hd_camera_ota_model_recv(uint8_t str);

// extra

int hd_camera_ota_version(
        uint8_t property_id_out,
        const unsigned char *payload_data, uint32_t payload_data_size,
        unsigned char **protocol_data_out,
        uint32_t *protocol_data_size_out
);

#ifdef __cplusplus
}
#endif

#endif // HD_CAMERA_OTA_MODEL_H
