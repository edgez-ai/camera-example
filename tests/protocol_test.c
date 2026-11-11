/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#include "camera_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void codec(void) {
    assert(cam_crc32((const uint8_t *)"123456789",9)==0xcbf43926U);
    uint8_t data[CAM_BLOCK_SIZE],wire[CAM_PACKET_MAX];
    for(unsigned i=0;i<sizeof(data);i++) data[i]=(uint8_t)i;
    struct cam_packet p={.type=CAM_DATA,.session=0xf1234567,.offset=768,.total=9000,.size=sizeof(data),.data=data},q;
    size_t n=cam_encode(wire,sizeof(wire),&p);
    assert(n==CAM_PACKET_MAX && cam_decode(wire,n,&q));
    assert(q.type==p.type && q.session==p.session && q.offset==p.offset && q.total==p.total && q.size==p.size);
    assert(!memcmp(q.data,data,sizeof(data)));
    /* Every single-bit corruption, including routing metadata and CRCs. */
    for(size_t i=0;i<n;i++) for(unsigned b=0;b<8;b++) {
        wire[i]^=1U<<b;assert(!cam_decode(wire,n,&q));wire[i]^=1U<<b;
    }
    for(size_t i=0;i<n;i++) assert(!cam_decode(wire,i,&q));
    assert(!cam_encode(wire,CAM_HEADER_SIZE-1,&p));
    p.size=CAM_BLOCK_SIZE+1;assert(!cam_encode(wire,sizeof(wire),&p));
    p.size=1;p.data=NULL;assert(!cam_encode(wire,sizeof(wire),&p));
    for(unsigned type=CAM_HELLO;type<=CAM_ERROR;type++) {
        p=(struct cam_packet){.type=type,.session=42};
        n=cam_encode(wire,sizeof(wire),&p);assert(n==CAM_HEADER_SIZE && cam_decode(wire,n,&q));
        assert(q.type==type && q.session==42 && q.size==0);
    }
}
static void transfer(uint32_t length) {
    uint8_t *source=malloc(length),*output=malloc(length+16);
    assert(source && output);memset(output,0xa5,length+16);
    for(uint32_t i=0;i<length;i++) source[i]=(uint8_t)(i*71U+13U);
    source[0]=255;source[1]=216;source[length-2]=255;source[length-1]=217;
    struct cam_image image;
    assert(cam_image_start(&image,output,length,1234,length));
    uint8_t wire[CAM_PACKET_MAX];uint32_t delivered=0;
    for(uint32_t offset=0;offset<length;) {
        uint16_t size=(length-offset)>CAM_BLOCK_SIZE ? CAM_BLOCK_SIZE : length-offset;
        struct cam_packet p={.type=CAM_DATA,.session=1234,.total=length,.offset=offset,.size=size,.data=source+offset},q;
        size_t n=cam_encode(wire,sizeof(wire),&p);assert(cam_decode(wire,n,&q));
        /* Wrong session, future offset and damaged packets cannot advance. */
        q.session++;assert(cam_image_accept(&image,&q)==CAM_IMAGE_IGNORED);q.session--;
        q.offset+=CAM_BLOCK_SIZE;assert(cam_image_accept(&image,&q)==CAM_IMAGE_IGNORED);q.offset-=CAM_BLOCK_SIZE;
        wire[n-1]^=1;assert(!cam_decode(wire,n,&q));wire[n-1]^=1;assert(cam_decode(wire,n,&q));
        /* Simulate lost responses and retransmission of the same frozen block. */
        assert(image.received==offset);
        int result=cam_image_accept(&image,&q);
        offset+=size;delivered++;
        assert(result==(offset==length ? CAM_IMAGE_COMPLETE : CAM_IMAGE_PROGRESS));
        assert(cam_image_accept(&image,&q)==CAM_IMAGE_IGNORED); /* late duplicate */
    }
    assert(delivered==(length+CAM_BLOCK_SIZE-1)/CAM_BLOCK_SIZE);
    assert(image.received==length && !memcmp(source,output,length));
    for(unsigned i=0;i<16;i++) assert(output[length+i]==0xa5);
    free(source);free(output);
}
static void bounds_and_bad_jpeg(void) {
    assert(!cam_valid_block(10,UINT32_MAX,1));
    assert(!cam_valid_block(100,0,101));
    assert(!cam_valid_block(CAM_MAX_JPEG+1,0,CAM_BLOCK_SIZE));
    assert(!cam_valid_block(100,1,99));
    uint8_t bytes[4],bad[]={1,2,3,4};struct cam_image image;
    assert(!cam_image_start(&image,bytes,sizeof(bytes),1,5));
    assert(!cam_image_start(&image,bytes,sizeof(bytes),0,4));
    assert(cam_image_start(&image,bytes,sizeof(bytes),1,4));
    struct cam_packet p={.type=CAM_DATA,.session=1,.total=4,.size=4,.data=bad};
    assert(cam_image_accept(&image,&p)==CAM_IMAGE_INVALID);
}
int main(void) {
    codec();bounds_and_bad_jpeg();
    const uint32_t sizes[]={4,767,768,769,1535,1536,1537,32768,65536,CAM_MAX_JPEG};
    for(unsigned i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++) transfer(sizes[i]);
    puts("PASS: CRC/codec corruption, truncation, bounds, stale/duplicate blocks, invalid JPEG, 10 image sizes through 512 KiB");
}
