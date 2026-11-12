/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#pragma once
#include <stddef.h>
#include <stdint.h>
struct cam_dummy_image {
    uint16_t width,height;
    uint32_t length,crc32;
    const uint8_t *data;
};
const struct cam_dummy_image *cam_dummy_find(uint16_t width,uint16_t height);
