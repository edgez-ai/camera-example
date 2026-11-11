/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#pragma once
#include "camera_protocol.h"
#include "edgez_pawr_protocol.h"
#include "edgez_rf_api.h"
#include "edgez_umac.h"
#include <string.h>
static const uint8_t cam_broadcast[6]={255,255,255,255,255,255};
static inline uint32_t cam_cssid(void) {
    return edgez_pawr_compress_ssid((const uint8_t *)CAM_SSID,sizeof(CAM_SSID)-1);
}
/* All transmit calls are serialized in the application worker. */
static inline bool cam_radio_send(struct edgez_umac *radio,const uint8_t local[6],
                                  const uint8_t peer[6],uint16_t *sequence,
                                  const struct cam_packet *packet) {
    uint8_t payload[CAM_PACKET_MAX],wire[CAM_PACKET_MAX+EDGEZ_PAWR_HEADER_LENGTH];
    uint8_t frame[sizeof(wire)+EDGEZ_PAWR_ACTION_HEADER_LENGTH];
    size_t n=cam_encode(payload,sizeof(payload),packet),wn,fn;
    if(!n || !edgez_pawr_encode_typed_request(wire,sizeof(wire),
        EDGEZ_PAWR_REQUEST_BULK_STREAM,packet->session,0,cam_cssid(),payload,n,&wn) ||
       !edgez_pawr_encode_action_frame(frame,sizeof(frame),(*sequence)++,local,peer,wire,wn,&fn)) return false;
    struct edgez_umac_tx_request tx={.token=*sequence,.frame=frame,.frame_length=fn,
        .frame_type=EDGEZ_UMAC_TX_MANAGEMENT,.mcs=0,.attempts=1,.bandwidth_mhz=1,
        .no_ack=true,.immediate_status=true};
    return edgez_umac_transmit(radio,&tx)==EDGEZ_UMAC_OK;
}
static inline bool cam_radio_event(const struct edgez_umac_rx_frame *frame,
                                    const uint8_t local[6],struct cam_event *out) {
    struct edgez_pawr_action_view action;
    struct edgez_pawr_request_view request;
    struct cam_packet packet;
    if(!frame || !edgez_pawr_decode_action_frame(frame->data,frame->length,&action) ||
       (action.source[0]&1) || !memcmp(action.source,"\0\0\0\0\0\0",6) ||
       (memcmp(action.destination,local,6) && memcmp(action.destination,cam_broadcast,6)) ||
       !edgez_pawr_decode_request(action.payload,action.payload_length,&request) ||
       request.type!=EDGEZ_PAWR_REQUEST_BULK_STREAM || request.compressed_ssid!=cam_cssid() ||
       request.payload_length>sizeof(out->bytes) ||
       !cam_decode(request.payload,request.payload_length,&packet)) return false;
    memcpy(out->source,action.source,6);
    out->length=request.payload_length;
    memcpy(out->bytes,request.payload,out->length);
    return true;
}
