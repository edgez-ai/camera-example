/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#include "camera_radio.h"
#include "vc0706.h"
#include "camera_dummy.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include "common/morse_commands.h"
#include "common/morse_command_utils.h"
#include "mmwlan.h"
extern enum mmwlan_status mmdrv_execute_command(uint8_t *,uint8_t *,uint32_t *);
extern enum mmwlan_status mmdrv_set_wake_enabled(bool enabled);
extern enum mmwlan_status mmdrv_set_dynamic_ps_timeout(uint32_t timeout_ms);
LOG_MODULE_REGISTER(camera,LOG_LEVEL_INF);
K_MSGQ_DEFINE(rx_queue,sizeof(struct cam_event),12,4);
static struct edgez_umac radio;
static struct edgez_umac_radio_info info;
static uint16_t sequence;
static atomic_t rx_total,rx_valid;
static uint8_t peer[6];
static bool paired,ps_ready,image_open,transaction_awake;
K_SEM_DEFINE(tx_done,0,1);
static atomic_t tx_armed,tx_failed;
static uintptr_t tx_expected;
static uint32_t session,total,boot_nonce,last_error;
static const struct cam_dummy_image *dummy_image;
static int64_t last_activity,led_until;
static uint32_t led_session,version_session;
static uint8_t version_data[CAM_VERSION_MAX];
static uint16_t version_length;
static int version_result;
static uint8_t block[CAM_BLOCK_SIZE];
static const struct gpio_dt_spec radio_power=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),halow_power_en_gpios);
static const struct gpio_dt_spec led=GPIO_DT_SPEC_GET(DT_ALIAS(led0),gpios);
static void receive(void *ctx,const struct edgez_umac_rx_frame *frame) {
    ARG_UNUSED(ctx);
    struct cam_event event;
    atomic_inc(&rx_total);
    if(cam_radio_event(frame,info.mac_addr,&event)) { atomic_inc(&rx_valid);(void)k_msgq_put(&rx_queue,&event,K_NO_WAIT); }
}
static void radio_fault(void *ctx) {
    ARG_UNUSED(ctx);
    LOG_ERR("Radio requested restart; reset nRF to recover");
}
static void transmit_done(void *ctx,const struct edgez_umac_tx_status *status) {
    ARG_UNUSED(ctx);
    if(atomic_get(&tx_armed) && status->token==tx_expected) {
        atomic_set(&tx_failed,!!(status->status_flags & EDGEZ_RF_TX_STATUS_FAILED));
        k_sem_give(&tx_done);
    }
}
static const struct edgez_umac_event_ops events={.rx_frame=receive,.tx_status=transmit_done,.hardware_restart_required=radio_fault};
static bool checked(void *cmd,uint16_t id,uint16_t vif) {
    struct { struct morse_cmd_resp response; uint8_t extra[128]; } reply={0};
    uint32_t len=sizeof(reply);
    enum mmwlan_status rc=mmdrv_execute_command(cmd,(uint8_t *)&reply,&len);
    bool ok=rc==MMWLAN_SUCCESS && len>=sizeof(reply.response) && len<=sizeof(reply) &&
        reply.response.hdr.message_id==id && reply.response.hdr.vif_id==vif && !reply.response.status;
    if(!ok) LOG_ERR("Radio command 0x%x failed sdk=%d status=%d length=%u",id,rc,(int)reply.response.status,len);
    return ok;
}
static int add_station(void *ctx,const uint8_t *mac,uint16_t *vif) {
    ARG_UNUSED(ctx);
    struct morse_cmd_req_add_interface cmd=MORSE_COMMAND_INIT(cmd,MORSE_CMD_ID_ADD_INTERFACE,UINT16_MAX,
        .interface_type=MORSE_CMD_INTERFACE_TYPE_STA);
    memcpy(cmd.addr.octet,mac,6);
    struct morse_cmd_resp_add_interface reply={0}; uint32_t n=sizeof(reply);
    int rc=mmdrv_execute_command((uint8_t *)&cmd,(uint8_t *)&reply,&n);
    if(rc!=MMWLAN_SUCCESS || n!=sizeof(reply) || reply.status ||
       reply.hdr.message_id!=MORSE_CMD_ID_ADD_INTERFACE) return MMWLAN_ERROR;
    *vif=reply.hdr.vif_id; return MMWLAN_SUCCESS;
}
static bool param(uint32_t id,uint32_t value) {
    struct morse_cmd_req_get_set_generic_param cmd=MORSE_COMMAND_INIT(cmd,
        MORSE_CMD_ID_GET_SET_GENERIC_PARAM,radio.vif_id,
        .param_id=id,.action=MORSE_CMD_PARAM_ACTION_SET,.value=value);
    return checked(&cmd,MORSE_CMD_ID_GET_SET_GENERIC_PARAM,radio.vif_id);
}
static bool configure_sleep(void) {
    if(!IS_ENABLED(CONFIG_CAMERA_DTIM_SLEEP)) return true;
    if(mmdrv_set_wake_enabled(true)!=MMWLAN_SUCCESS) return false;
    struct morse_cmd_req_set_long_sleep_config ls=MORSE_COMMAND_INIT(ls,
        MORSE_CMD_ID_SET_LONG_SLEEP_CONFIG,radio.vif_id,.enabled=false);
    if(!checked(&ls,MORSE_CMD_ID_SET_LONG_SLEEP_CONFIG,radio.vif_id)) return false;
    if(radio.driver->configure_bss(radio.driver_ctx,radio.vif_id,100,0,0)) return false;
    struct __packed { struct morse_cmd_header hdr; uint8_t bssid[6]; } bssid=
        MORSE_COMMAND_INIT(bssid,0x0052,radio.vif_id);
    memcpy(bssid.bssid,peer,6);
    if(!checked(&bssid,0x0052,radio.vif_id)) return false;
    struct morse_cmd_req_li_sleep li=MORSE_COMMAND_INIT(li,MORSE_CMD_ID_LI_SLEEP,radio.vif_id,
        .listen_interval=CAM_DTIM_PERIOD);
    if(!checked(&li,MORSE_CMD_ID_LI_SLEEP,radio.vif_id)) return false;
    for(unsigned state=1;state<=4;state++) {
        struct morse_cmd_req_set_sta_state sta=MORSE_COMMAND_INIT(sta,
            MORSE_CMD_ID_SET_STA_STATE,radio.vif_id,.aid=1,.state=state);
        memcpy(sta.sta_addr,peer,6);
        if(!checked(&sta,MORSE_CMD_ID_SET_STA_STATE,radio.vif_id)) return false;
        if(state==3 && !param(MORSE_CMD_PARAM_ID_NON_TIM_MODE,0)) return false;
    }
    k_msleep(300);
    struct morse_cmd_req_config_ps ps=MORSE_COMMAND_INIT(ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id,
        .enabled=true,.dynamic_ps_offload=true);
    if(!checked(&ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id) ||
       mmdrv_set_dynamic_ps_timeout(13)!=MMWLAN_SUCCESS) return false;
    struct morse_cmd_req_get_set_generic_param read=MORSE_COMMAND_INIT(read,
        MORSE_CMD_ID_GET_SET_GENERIC_PARAM,UINT16_MAX,
        .param_id=MORSE_CMD_PARAM_ID_DYNAMIC_PS_TIMEOUT_MS,.action=MORSE_CMD_PARAM_ACTION_GET);
    struct morse_cmd_resp_get_set_generic_param value={0}; uint32_t n=sizeof(value);
    if(mmdrv_execute_command((uint8_t *)&read,(uint8_t *)&value,&n)!=MMWLAN_SUCCESS ||
       n!=sizeof(value) || value.hdr.message_id!=MORSE_CMD_ID_GET_SET_GENERIC_PARAM ||
       value.status || value.value!=13) return false;
    ps_ready=true;
    LOG_INF("DTIM10 ready: beacon=100 TU, timeout=13 ms, native TWT=0, association frames=0");
    return true;
}
static bool send_packet(uint8_t type,uint32_t id,uint32_t offset,uint32_t length,
                        const uint8_t *data,uint16_t size,const uint8_t *dest) {
    struct cam_packet p={.type=type,.session=id,.offset=offset,.total=length,.data=data,.size=size};
    k_sem_reset(&tx_done);atomic_clear(&tx_failed);
    tx_expected=(uint16_t)(sequence+1U);atomic_set(&tx_armed,1);
    bool queued=cam_radio_send(&radio,info.mac_addr,dest,true,&sequence,&p);
    bool completed=queued && !k_sem_take(&tx_done,K_MSEC(250));
    atomic_clear(&tx_armed);
    bool ok=completed && !atomic_get(&tx_failed);
    if(!ok) LOG_WRN("Radio TX type=%u offset=%u queued=%u complete=%u failed=%ld",type,offset,queued,completed,(long)atomic_get(&tx_failed));
    return ok;
}
static void error(uint32_t id,uint32_t code,const uint8_t *dest) {
    (void)send_packet(CAM_ERROR,id,code,0,NULL,0,dest);
}
static bool send_captured(void) {
    uint8_t metadata[5]={1,0,0,0,0};
    if(dummy_image) {
        for(unsigned i=0;i<4;i++) metadata[1+i]=(uint8_t)(dummy_image->crc32>>(8*i));
        return send_packet(CAM_CAPTURED,session,0,total,metadata,sizeof(metadata),peer);
    }
    return send_packet(CAM_CAPTURED,session,0,total,NULL,0,peer);
}
static bool camera_radio_awake(void) {
    if(!ps_ready || transaction_awake) return true;
    struct morse_cmd_req_config_ps ps=MORSE_COMMAND_INIT(ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id,
        .enabled=false,.dynamic_ps_offload=false);
    if(!checked(&ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id)) return false;
    transaction_awake=true;
    LOG_INF("Camera transaction: firmware PS disabled, radio RX held awake");return true;
}
static void sleep_idle(void) {
    vc0706_off();
    gpio_pin_set_dt(&led,0);
    if(ps_ready) {
        if(transaction_awake) {
            struct morse_cmd_req_config_ps ps=MORSE_COMMAND_INIT(ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id,
                .enabled=true,.dynamic_ps_offload=true);
            if(!checked(&ps,MORSE_CMD_ID_CONFIG_PS,radio.vif_id) ||
               mmdrv_set_dynamic_ps_timeout(13)!=MMWLAN_SUCCESS) {
                LOG_ERR("DTIM restore failed; host wake retained");return;
            }
            transaction_awake=false;LOG_INF("Camera transaction ended; DTIM10/13ms restored");
        }
        (void)mmdrv_set_wake_enabled(false);
    }
}
static void handle(const struct cam_event *event,const struct cam_packet *p) {
    if(p->type==CAM_HELLO) {
        /* Host restart: the stable radio MAC can rediscover a sleeping node. */
        if(!paired || !memcmp(peer,event->source,6))
            (void)send_packet(CAM_HELLO_ACK,p->session,0,CAM_MAX_JPEG,NULL,0,event->source);
        return;
    }
    if(!paired || memcmp(peer,event->source,6)) return;
    (void)mmdrv_set_wake_enabled(true);
    last_activity=k_uptime_get();
    if(p->type==CAM_LED && p->session) {
        if(image_open) { error(p->session,CAM_ERR_BUSY,peer); return; }
        if(p->session!=led_session) {
            led_session=p->session;led_until=k_uptime_get()+5000;
            gpio_pin_set_dt(&led,1);
            LOG_INF("LED ON session=%u duration_ms=5000 camera_power=off",led_session);
        }
        (void)send_packet(CAM_LED_ACK,p->session,0,5000,NULL,0,peer);
        k_msleep(30);
        if(ps_ready) (void)mmdrv_set_wake_enabled(false);
        return;
    }
    if(p->type==CAM_VERSION && p->session) {
        if(image_open) { error(p->session,CAM_ERR_BUSY,peer);return; }
        if(version_session!=p->session) {
            version_session=p->session;
            if(!camera_radio_awake()) {version_result=-EIO;error(p->session,CAM_ERR_RADIO,peer);sleep_idle();return;}
            version_result=vc0706_version(version_data,sizeof(version_data),&version_length);
        }
        if(version_result) error(p->session,CAM_ERR_CAMERA,peer);
        else (void)send_packet(CAM_VERSION_ACK,p->session,0,0,version_data,version_length,peer);
        k_msleep(30);sleep_idle();return;
    }
    if(p->type==CAM_CAPTURE && p->session) {
        led_until=0;
        if(p->session==session) {
            if(last_error) error(session,last_error,peer);
            else if(image_open) (void)send_captured();
            else error(session,CAM_ERR_SESSION,peer);
            if(!image_open) { k_msleep(30);sleep_idle(); }
            return;
        }
        if(image_open) { error(p->session,CAM_ERR_BUSY,peer); return; }
        uint16_t width,height;
        if(!cam_capture_dimensions(p,&width,&height)) {
            error(p->session,CAM_ERR_RANGE,peer);k_msleep(30);sleep_idle();return;
        }
        if(!camera_radio_awake()) {error(p->session,CAM_ERR_RADIO,peer);sleep_idle();return;}
        session=p->session; last_error=0; total=0;
        gpio_pin_set_dt(&led,1);
        int rc=0;dummy_image=NULL;
        if(IS_ENABLED(CONFIG_CAMERA_DUMMY_IMAGE)) {
            vc0706_off();dummy_image=cam_dummy_find(width,height);
            if(!dummy_image) rc=-EINVAL;
            else {
                total=dummy_image->length;
                LOG_INF("DUMMY_CAPTURE %ux%u bytes=%u crc32=%08x camera_power=off",width,height,total,dummy_image->crc32);
            }
        } else rc=vc0706_capture(width,height,&total);
        if(rc) {
            LOG_ERR("Camera capture failed: %d",rc);
            last_error=CAM_ERR_CAMERA; error(session,last_error,peer); sleep_idle();
        } else {
            image_open=true;
            LOG_INF("Captured session=%u JPEG=%u bytes",session,total);
            (void)send_captured();
        }
    } else if(p->type==CAM_READ && p->session==session && image_open) {
        uint16_t n=(uint16_t)MIN(CAM_BLOCK_SIZE,total>p->offset ? total-p->offset : 0);
        if(p->total!=total || !cam_valid_block(total,p->offset,n)) { error(session,CAM_ERR_RANGE,peer); return; }
        if(!p->offset) LOG_INF("Camera block READ offset=0 size=%u",n);
        int rc=0;
        if(dummy_image) memcpy(block,dummy_image->data+p->offset,n);
        else rc=vc0706_read(p->offset,block,n);
        if(rc) { LOG_ERR("Camera read offset=%u failed=%d",p->offset,rc); error(session,CAM_ERR_CAMERA,peer); return; }
        bool sent=send_packet(CAM_DATA,session,p->offset,total,block,n,peer);
        if(!p->offset || p->offset+n==total) LOG_INF("Camera block DATA offset=%u size=%u sent=%u",p->offset,n,sent);
    } else if(p->type==CAM_FINISH && p->session==session) {
        (void)send_packet(CAM_FINISHED,session,0,total,NULL,0,peer);
        image_open=false;dummy_image=NULL;
        k_msleep(30); sleep_idle();
        LOG_INF("Session=%u finished; camera off",session);
    } else if(p->type==CAM_READ) error(p->session,CAM_ERR_SESSION,peer);
    if(!image_open) { k_msleep(30); sleep_idle(); }
}
int main(void) {
    if(!gpio_is_ready_dt(&radio_power) || !gpio_is_ready_dt(&led)) return -ENODEV;
    gpio_pin_configure_dt(&radio_power,GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led,GPIO_OUTPUT_INACTIVE);
    vc0706_off(); k_msleep(20);
    /* Zephyr initializes the GPIO/SPI Morse platform devices before main. */
    static struct edgez_rf_backend_ops ops;
    ops=*edgez_rf_driver_ops(); ops.add_mesh_interface=add_station;
    const struct edgez_umac_config config={.country_code=CAM_COUNTRY,
        .channel={.operating_frequency_hz=CAM_FREQUENCY_HZ,.operating_bw_mhz=1,.primary_bw_mhz=1},
        .beacon_interval_tu=100,.dtim_period=1,.compressed_ssid=cam_cssid(),
        .rate_control={.initial_mcs=0,.maximum_mcs=0},.enable_beaconing=false};
    if(edgez_umac_bind(&radio,&ops,NULL) || edgez_umac_set_event_ops(&radio,&events,NULL) ||
       edgez_rf_bind_umac(&radio) || edgez_umac_start(&radio,&config,&info)) {
        LOG_ERR("HaLow start failed driver=%d",radio.last_driver_status); return -EIO;
    }
    if(mmdrv_set_wake_enabled(true)!=MMWLAN_SUCCESS) { LOG_ERR("Initial host wake failed"); return -EIO; }
    struct morse_cmd_req_config_ps awake=MORSE_COMMAND_INIT(awake,MORSE_CMD_ID_CONFIG_PS,radio.vif_id,
        .enabled=false,.dynamic_ps_offload=false);
    if(!checked(&awake,MORSE_CMD_ID_CONFIG_PS,radio.vif_id)) return -EIO;
    boot_nonce=sys_rand32_get()|1U;
    struct cam_event event;
    struct cam_packet p;
    LOG_INF("Camera node ready; discovering HC33 on 914.5 MHz/1 MHz");
    unsigned hello_count=0;
    while(!paired) {
        bool ok=send_packet(CAM_HELLO,boot_nonce,0,CAM_MAX_JPEG,NULL,0,cam_broadcast);
        if(!(hello_count++%10)) LOG_INF("HELLO sent=%u rx=%ld valid=%ld mac=%02x:%02x:%02x:%02x:%02x:%02x",ok,
            (long)atomic_get(&rx_total),(long)atomic_get(&rx_valid),info.mac_addr[0],info.mac_addr[1],info.mac_addr[2],info.mac_addr[3],info.mac_addr[4],info.mac_addr[5]);
        int64_t end=k_uptime_get()+1000;
        while(k_uptime_get()<end && !k_msgq_get(&rx_queue,&event,K_MSEC(100))) {
            if(cam_decode(event.bytes,event.length,&p) && p.type==CAM_HELLO_ACK && p.session==boot_nonce) {
                memcpy(peer,event.source,6); paired=true; break;
            }
        }
    }
    if(!configure_sleep()) {
        LOG_ERR("DTIM setup failed; stopping instead of claiming low-power readiness");
        gpio_pin_set_dt(&led,1); return -EIO;
    }
    sleep_idle();
    for(;;) {
        int rc=k_msgq_get(&rx_queue,&event,led_until ? K_MSEC(30) : image_open ? K_MSEC(1000) : K_FOREVER);
        if(led_until && k_uptime_get()>=led_until) {
            led_until=0;
            if(!image_open) { gpio_pin_set_dt(&led,0);LOG_INF("LED OFF session=%u",led_session); }
        }
        if(!rc && cam_decode(event.bytes,event.length,&p)) handle(&event,&p);
        if(image_open && k_uptime_get()-last_activity>90000) {
            LOG_WRN("Abandoned capture expired"); image_open=false; last_error=CAM_ERR_TIMEOUT; sleep_idle();
        }
    }
}
