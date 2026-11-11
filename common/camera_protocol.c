/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#include "camera_protocol.h"
#include <string.h>
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static void put32(uint8_t *p, uint32_t x) {
    for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(x>>(8*i));
}
uint32_t cam_crc32(const uint8_t *data, size_t length) {
    uint32_t crc=~0U;
    for(size_t i=0;i<length;i++) {
        crc ^= data[i];
        for(unsigned b=0;b<8;b++) crc=(crc>>1)^((0U-(crc&1U))&0xedb88320U);
    }
    return ~crc;
}
size_t cam_encode(uint8_t *out,size_t capacity,const struct cam_packet *p) {
    if(!out || !p || p->size>CAM_BLOCK_SIZE || (p->size && !p->data) ||
       capacity<CAM_HEADER_SIZE+p->size || p->type<CAM_HELLO || p->type>CAM_VERSION_ACK) return 0;
    memset(out,0,CAM_HEADER_SIZE);
    memcpy(out,"EZCA",4); out[4]=1; out[5]=p->type;
    out[6]=p->size; out[7]=p->size>>8;
    put32(out+8,p->session); put32(out+12,p->offset); put32(out+16,p->total);
    if(p->size) memcpy(out+CAM_HEADER_SIZE,p->data,p->size);
    put32(out+20,cam_crc32(out+CAM_HEADER_SIZE,p->size));
    put32(out+24,cam_crc32(out,24));
    return CAM_HEADER_SIZE+p->size;
}
bool cam_decode(const uint8_t *in,size_t length,struct cam_packet *p) {
    if(!in || !p || length<CAM_HEADER_SIZE || memcmp(in,"EZCA",4) ||
       in[4]!=1 || in[5]<CAM_HELLO || in[5]>CAM_VERSION_ACK) return false;
    uint16_t size=(uint16_t)in[6] | (uint16_t)in[7]<<8;
    if(size>CAM_BLOCK_SIZE || length!=CAM_HEADER_SIZE+size ||
       get32(in+24)!=cam_crc32(in,24) ||
       get32(in+20)!=cam_crc32(in+CAM_HEADER_SIZE,size)) return false;
    *p=(struct cam_packet){.type=in[5],.session=get32(in+8),.offset=get32(in+12),
        .total=get32(in+16),.size=size,.data=in+CAM_HEADER_SIZE};
    return true;
}
bool cam_valid_block(uint32_t total,uint32_t offset,uint16_t size) {
    if(total<4 || total>CAM_MAX_JPEG || offset>=total || offset%CAM_BLOCK_SIZE) return false;
    uint32_t remaining=total-offset;
    return size==(remaining>CAM_BLOCK_SIZE ? CAM_BLOCK_SIZE : remaining);
}

bool cam_image_start(struct cam_image *image,uint8_t *bytes,uint32_t capacity,
                     uint32_t session,uint32_t total) {
    if(!image || !bytes || !session || total<4 || total>CAM_MAX_JPEG || total>capacity) return false;
    *image=(struct cam_image){.bytes=bytes,.session=session,.total=total};
    return true;
}
enum cam_image_result cam_image_accept(struct cam_image *image,const struct cam_packet *packet) {
    if(!image || !image->bytes || !packet) return CAM_IMAGE_INVALID;
    if(packet->type!=CAM_DATA || packet->session!=image->session ||
       packet->total!=image->total || packet->offset!=image->received) return CAM_IMAGE_IGNORED;
    if(!packet->data || !cam_valid_block(packet->total,packet->offset,packet->size)) return CAM_IMAGE_INVALID;
    memcpy(image->bytes+packet->offset,packet->data,packet->size);
    image->received+=packet->size;
    if(image->received!=image->total) return CAM_IMAGE_PROGRESS;
    return image->bytes[0]==0xff && image->bytes[1]==0xd8 &&
        image->bytes[image->total-2]==0xff && image->bytes[image->total-1]==0xd9 ?
        CAM_IMAGE_COMPLETE : CAM_IMAGE_INVALID;
}

const struct cam_resolution *cam_resolution_find(uint16_t width,uint16_t height) {
    /* Putal modes from offgrid-tracker's working camera_resolutions.h. */
    static const struct cam_resolution modes[]={
        {160,120,4,0x22},{320,240,4,0x11},{640,480,4,0x00},
        {1024,768,5,0x33},{1280,720,5,0x44},{1280,960,5,0x55},
        {1600,1200,5,0x77},{1920,1080,5,0x66},{2304,1296,5,0x88},
    };
    for(size_t i=0;i<sizeof(modes)/sizeof(modes[0]);i++)
        if(modes[i].width==width && modes[i].height==height) return &modes[i];
    return NULL;
}
bool cam_capture_dimensions(const struct cam_packet *packet,uint16_t *width,uint16_t *height) {
    if(!packet || !width || !height || packet->type!=CAM_CAPTURE) return false;
    uint16_t w=640,h=480;
    if(packet->size) {
        if(packet->size!=4 || !packet->data) return false;
        w=(uint16_t)packet->data[0] | (uint16_t)packet->data[1]<<8;
        h=(uint16_t)packet->data[2] | (uint16_t)packet->data[3]<<8;
    }
    if(!cam_resolution_find(w,h)) return false;
    *width=w;*height=h;return true;
}

bool cam_jpeg_dimensions(const uint8_t *jpeg,size_t length,uint16_t *width,uint16_t *height) {
    if(!jpeg || !width || !height || length<4 || jpeg[0]!=0xff || jpeg[1]!=0xd8) return false;
    size_t at=2;
    while(at<length) {
        if(jpeg[at++]!=0xff) return false;
        while(at<length && jpeg[at]==0xff) at++;
        if(at>=length) return false;
        uint8_t marker=jpeg[at++];
        if(marker==0xda || marker==0xd9 || !marker) return false;
        if(marker==0x01 || (marker>=0xd0 && marker<=0xd8)) continue;
        if(length-at<2) return false;
        size_t n=(size_t)jpeg[at]<<8 | jpeg[at+1];
        if(n<2 || n>length-at) return false;
        if(marker>=0xc0 && marker<=0xcf && marker!=0xc4 && marker!=0xc8 && marker!=0xcc) {
            if(n<8) return false;
            uint16_t h=(uint16_t)jpeg[at+3]<<8 | jpeg[at+4];
            uint16_t w=(uint16_t)jpeg[at+5]<<8 | jpeg[at+6];
            if(!w || !h) return false;
            *width=w;*height=h;return true;
        }
        at+=n;
    }
    return false;
}
