/**
 * Copyright (c) 2020 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef H_HD_LOG_H
#define H_HD_LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

//<editor-fold desc="拍摄">
#include <stdint.h>

int hd_camera_produce_init(uint8_t addr,
                           const char *path,
                           const char *demo_path,
                           int(*on_action_id_info_produce)(char *, char **, int),
                           int(*transform_pic)(const char *, char *)
                           );

int hd_camera_produce_on_action_id_changed(uint32_t action_id_timestamp, uint8_t action_id_index, uint8_t status,uint8_t trigger_type);

int hd_camera_produce_take_photos_actively(uint16_t* pic_id);

int hd_camera_produce_deinit(uint8_t addr);
//</editor-fold>

#ifdef __cplusplus
}
#endif

#endif
