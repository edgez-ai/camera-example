/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#pragma once
#include <stddef.h>
#include <stdint.h>
int vc0706_capture(uint16_t width,uint16_t height,uint32_t *length);
int vc0706_version(uint8_t *version,size_t capacity,uint16_t *length);
int vc0706_read(uint32_t offset,uint8_t *bytes,size_t length);
void vc0706_off(void);
