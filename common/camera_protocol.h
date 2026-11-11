/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CAM_SSID "edgez-camera"
#define CAM_COUNTRY "US"
#define CAM_FREQUENCY_HZ 914500000U
#define CAM_BANDWIDTH_MHZ 1U
#define CAM_DTIM_PERIOD 10U
#define CAM_BLOCK_SIZE 768U
#define CAM_MAX_JPEG (512U * 1024U)
#define CAM_HEADER_SIZE 28U
#define CAM_PACKET_MAX (CAM_HEADER_SIZE + CAM_BLOCK_SIZE)
enum cam_type { CAM_HELLO=1, CAM_HELLO_ACK, CAM_CAPTURE, CAM_CAPTURED,
                CAM_READ, CAM_DATA, CAM_FINISH, CAM_FINISHED, CAM_ERROR };
enum cam_error { CAM_ERR_CAMERA=1, CAM_ERR_BUSY, CAM_ERR_SESSION,
                 CAM_ERR_RANGE, CAM_ERR_TIMEOUT, CAM_ERR_RADIO };
struct cam_packet {
    uint8_t type;
    uint32_t session, offset, total;
    uint16_t size;
    const uint8_t *data;
};
struct cam_event {
    uint8_t source[6];
    uint16_t length;
    uint8_t bytes[CAM_PACKET_MAX];
};
uint32_t cam_crc32(const uint8_t *data, size_t length);
size_t cam_encode(uint8_t *out, size_t capacity, const struct cam_packet *packet);
bool cam_decode(const uint8_t *bytes, size_t length, struct cam_packet *packet);
/* Host and MCU agree on exactly aligned, bounded reads. */
bool cam_valid_block(uint32_t total, uint32_t offset, uint16_t size);

struct cam_image {
    uint8_t *bytes;
    uint32_t session,total,received;
};
enum cam_image_result { CAM_IMAGE_IGNORED=0,CAM_IMAGE_PROGRESS=1,
                        CAM_IMAGE_COMPLETE=2,CAM_IMAGE_INVALID=-1 };
bool cam_image_start(struct cam_image *image,uint8_t *bytes,uint32_t capacity,
                     uint32_t session,uint32_t total);
enum cam_image_result cam_image_accept(struct cam_image *image,const struct cam_packet *packet);
