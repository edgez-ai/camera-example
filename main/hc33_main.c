/* Copyright 2026 EdgeZeron AI AB. See LICENSE. */
#include "camera_radio.h"
#include "web_page.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "edgez_mesh_s1g.h"
#include "common/morse_commands.h"
#include "common/morse_command_utils.h"
#include "mmpkt.h"
extern enum mmwlan_status mmdrv_execute_command(uint8_t *,uint8_t *,uint32_t *);
static const char *TAG="camera_gateway";
static struct edgez_umac radio;
static struct edgez_umac_radio_info info;
static uint16_t sequence;
static uint8_t peer[6];
static bool peer_known;
static QueueHandle_t rx_queue;
static SemaphoreHandle_t lock;
static atomic_bool pending,dtim_pulse,capture_requested;
static bool requested_photo;
static struct cam_image assembly;
static atomic_uint dropped_rx,rx_total,rx_valid;
static unsigned tx_ok,tx_bad;
enum phase { IDLE,QUEUED,DISCOVERING,CAPTURING,RECEIVING,FINISHING,COMPLETE,WAKING,LED_COMPLETE,FAILED };
static const char *const phases[]={"idle","queued","discovering","capturing","receiving","finishing","complete","waking","led-complete","error"};
static struct {
    enum phase phase;
    uint32_t id,total,received,retries;
    uint8_t *image;
    unsigned readers;
    bool busy,peer,photo;
    char error[96];
} state;
static int64_t started,stage_started,last_send,burst_until;
static unsigned attempts;
static void take(void) { xSemaphoreTake(lock,portMAX_DELAY); }
static void give(void) { xSemaphoreGive(lock); }

/* Keep the tested S1G TIM layout: AID 1 pending bit, countdown in firmware's
 * actual beacon builder. Never issue a blocking transmit from this callback. */
extern struct mmpkt *__real_mmdrv_host_get_beacon(void);
struct mmpkt *__wrap_mmdrv_host_get_beacon(void) {
    struct mmpkt *packet=__real_mmdrv_host_get_beacon();
    if(!packet) return packet;
    static uint8_t count=CAM_DTIM_PERIOD-1;
    struct mmpktview *view=mmpkt_open(packet);
    uint8_t *data=mmpkt_get_data_start(view);
    size_t length=mmpkt_get_data_length(view);
    if(length>=15 && (data[0]&0xfc)==0x1c) for(size_t at=15;at+2<=length;) {
        size_t end=at+2+data[at+1]; if(end>length) break;
        if(data[at]==5 && data[at+1]==6) {
            bool ready=atomic_load(&pending);
            data[at+2]=count;data[at+3]=CAM_DTIM_PERIOD;
            data[at+4]=0x3e|(ready && !count ? 1 : 0);
            data[at+5]=0;data[at+6]=1;data[at+7]=ready?2:0;
            if(!count) atomic_store(&dtim_pulse,true);
            count=count ? count-1 : CAM_DTIM_PERIOD-1;
            break;
        }
        at=end;
    }
    mmpkt_close(&view); return packet;
}
static void receive(void *ctx,const struct edgez_umac_rx_frame *frame) {
    (void)ctx; struct cam_event event;
    atomic_fetch_add(&rx_total,1);
    if(cam_radio_event(frame,info.mac_addr,&event)) {
        atomic_fetch_add(&rx_valid,1);
        if(xQueueSend(rx_queue,&event,0)!=pdPASS) atomic_fetch_add(&dropped_rx,1);
    }
}
static void radio_fault(void *ctx) { (void)ctx; ESP_LOGE(TAG,"Radio restart requested; reboot HC33 to recover"); }
static const struct edgez_umac_event_ops events={.rx_frame=receive,.hardware_restart_required=radio_fault};
static int add_ap(void *ctx,const uint8_t *mac,uint16_t *vif) {
    (void)ctx;
    struct morse_cmd_req_add_interface cmd=MORSE_COMMAND_INIT(cmd,MORSE_CMD_ID_ADD_INTERFACE,UINT16_MAX,
        .interface_type=MORSE_CMD_INTERFACE_TYPE_AP);
    memcpy(cmd.addr.octet,mac,6);
    struct morse_cmd_resp_add_interface reply={0};uint32_t n=sizeof(reply);
    int rc=mmdrv_execute_command((uint8_t *)&cmd,(uint8_t *)&reply,&n);
    if(rc!=MMWLAN_SUCCESS || n!=sizeof(reply) || reply.status ||
       reply.hdr.message_id!=MORSE_CMD_ID_ADD_INTERFACE) return MMWLAN_ERROR;
    *vif=reply.hdr.vif_id; return MMWLAN_SUCCESS;
}
static void halow_start(void) {
    static struct edgez_rf_backend_ops ops;
    ESP_ERROR_CHECK(edgez_rf_platform_init()==0?ESP_OK:ESP_FAIL);
    ops=*edgez_rf_driver_ops();ops.add_mesh_interface=add_ap;
    const struct edgez_umac_config config={.country_code=CAM_COUNTRY,
        .channel={.operating_frequency_hz=CAM_FREQUENCY_HZ,.operating_bw_mhz=1,.primary_bw_mhz=1},
        .beacon_interval_tu=100,.dtim_period=CAM_DTIM_PERIOD,.compressed_ssid=0,
        .rate_control={.initial_mcs=0,.maximum_mcs=0},.enable_beaconing=false};
    ESP_ERROR_CHECK(edgez_umac_bind(&radio,&ops,NULL)==0?ESP_OK:ESP_FAIL);
    ESP_ERROR_CHECK(edgez_umac_set_event_ops(&radio,&events,NULL)==0?ESP_OK:ESP_FAIL);
    ESP_ERROR_CHECK(edgez_rf_bind_umac(&radio)==0?ESP_OK:ESP_FAIL);
    ESP_ERROR_CHECK(edgez_umac_start(&radio,&config,&info)==0?ESP_OK:ESP_FAIL);
    uint8_t frame[128]={0x1c,0};memcpy(frame+4,info.mac_addr,6);size_t n=15;
    const uint8_t timing[]={213,8,1,0,100,0,0,0,0,0,5,6,0,CAM_DTIM_PERIOD,0x3e,0,1,0};
    memcpy(frame+n,timing,sizeof(timing));n+=sizeof(timing);
    memcpy(frame+n,edgez_mesh_s1g_default_capabilities_ie,EDGEZ_MESH_S1G_CAPABILITIES_IE_LEN);
    n+=EDGEZ_MESH_S1G_CAPABILITIES_IE_LEN;
    const uint8_t operation[]={232,6,1,1,25,25,0xfc,0xff,214,2,100,0};
    memcpy(frame+n,operation,sizeof(operation));n+=sizeof(operation);
    frame[n++]=0;frame[n++]=sizeof(CAM_SSID)-1;
    memcpy(frame+n,CAM_SSID,sizeof(CAM_SSID)-1);n+=sizeof(CAM_SSID)-1;
    ESP_ERROR_CHECK(radio.driver->configure_bss(radio.driver_ctx,radio.vif_id,100,CAM_DTIM_PERIOD,cam_cssid())==0?ESP_OK:ESP_FAIL);
    const struct edgez_rf_beacon_template beacon={.frame=frame,.frame_length=n,.mcs=0,.bandwidth_mhz=1};
    ESP_ERROR_CHECK(edgez_umac_enable_beaconing(&radio,&beacon)==0?ESP_OK:ESP_FAIL);
    ESP_LOGI(TAG,"HaLow DTIM10 AP ready on US 914.5 MHz/1 MHz mac=%02x:%02x:%02x:%02x:%02x:%02x",info.mac_addr[0],info.mac_addr[1],info.mac_addr[2],info.mac_addr[3],info.mac_addr[4],info.mac_addr[5]);
}
static bool send_packet(uint8_t type,uint32_t id,uint32_t offset,uint32_t total,const uint8_t *dest) {
    const struct cam_packet p={.type=type,.session=id,.offset=offset,.total=total};
    bool ok=cam_radio_send(&radio,info.mac_addr,dest,&sequence,&p);
    if(ok) tx_ok++; else tx_bad++;
    return ok;
}
static void phase(enum phase phase) {
    take();state.phase=phase;give();
    stage_started=esp_timer_get_time();last_send=0;attempts=0;
}
static void fail(const char *message) {
    (void)send_packet(CAM_FINISH,state.id,0,state.total,peer);
    take();snprintf(state.error,sizeof(state.error),"%s",message);state.phase=FAILED;state.busy=false;give();
    atomic_store(&pending,false);ESP_LOGE(TAG,"Remote action failed: %s",message);
}
static bool start_action(bool photo) {
    take();
    if(state.busy || state.readers || atomic_load(&capture_requested)) { give();return false; }
    requested_photo=photo;
    atomic_store(&capture_requested,true);
    give();return true;
}
/* Only the radio worker changes transfer state or frees the image. */
static void begin_capture(void) {
    take();
    if(state.busy || state.readers) { give();return; }
    free(state.image);state.image=NULL;
    state.id++;if(!state.id)state.id=1;
    state.total=state.received=state.retries=0;state.error[0]=0;
    state.busy=true;state.phase=QUEUED;state.photo=requested_photo;
    give();
}
static void handle(const struct cam_event *event,const struct cam_packet *p) {
    if(p->type==CAM_HELLO || p->type==CAM_HELLO_ACK) {
        if(peer_known && memcmp(peer,event->source,6)) return;
        if(p->type==CAM_HELLO) (void)send_packet(CAM_HELLO_ACK,p->session,0,CAM_MAX_JPEG,event->source);
        if(p->type==CAM_HELLO_ACK && (!state.busy || state.phase!=DISCOVERING || p->session!=state.id)) return;
        memcpy(peer,event->source,6);peer_known=true;
        take();state.peer=true;give();
        if(state.busy && state.phase==DISCOVERING) phase(state.photo?CAPTURING:WAKING);
        return;
    }
    if(!state.busy || !peer_known || memcmp(peer,event->source,6) || p->session!=state.id) return;
    if(state.phase==WAKING && p->type==CAM_LED_ACK) {
        take();state.phase=LED_COMPLETE;state.busy=false;give();atomic_store(&pending,false);
        ESP_LOGI(TAG,"LED acknowledged id=%"PRIu32" on_ms=%"PRIu32" camera_requested=0",state.id,p->total);
        return;
    }
    if(p->type==CAM_ERROR) {
        const char *message=p->offset==CAM_ERR_CAMERA?"Camera UART/capture failed. Check camera power and wiring.":
            p->offset==CAM_ERR_BUSY?"Remote camera is busy. Retry after the old capture expires.":
            p->offset==CAM_ERR_SESSION?"Remote capture expired or the nRF restarted. Take another photo.":"Remote camera reported a transfer error.";
        fail(message);return;
    }
    if(state.phase==CAPTURING && p->type==CAM_CAPTURED) {
        if(p->total<4 || p->total>CAM_MAX_JPEG) { fail("Remote image length is outside the supported limit.");return; }
        uint8_t *image=heap_caps_malloc(p->total,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        if(!image) image=heap_caps_malloc(p->total,MALLOC_CAP_8BIT);
        if(!image) { fail("HC33 has insufficient RAM for this JPEG.");return; }
        if(!cam_image_start(&assembly,image,p->total,state.id,p->total)) { free(image);fail("Invalid image transfer metadata.");return; }
        take();state.total=p->total;state.image=image;give();phase(RECEIVING);
        ESP_LOGI(TAG,"Capture id=%"PRIu32" JPEG=%"PRIu32" bytes",state.id,state.total);
    } else if(state.phase==RECEIVING && p->type==CAM_DATA) {
        enum cam_image_result result=cam_image_accept(&assembly,p);
        if(result==CAM_IMAGE_IGNORED) return;
        if(result==CAM_IMAGE_INVALID) { fail("Invalid JPEG or block; the image was not published.");return; }
        take();state.received=assembly.received;give();
        attempts=0;last_send=0;stage_started=esp_timer_get_time();
        if(result==CAM_IMAGE_COMPLETE) {
            phase(FINISHING);
        }
    } else if(state.phase==FINISHING && p->type==CAM_FINISHED) {
        take();state.phase=COMPLETE;state.busy=false;give();atomic_store(&pending,false);
        ESP_LOGI(TAG,"Photo id=%"PRIu32" ready bytes=%"PRIu32" retries=%"PRIu32" crc32=%08"PRIx32,state.id,state.total,state.retries,cam_crc32(state.image,state.total));
    }
}
static void worker(void *arg) {
    (void)arg;int previous_button=1;int64_t button_at=0;
    for(;;) {
        int64_t now=esp_timer_get_time();
        uint8_t command;
        if(uart_read_bytes(UART_NUM_0,&command,1,0)==1) {
            if(command=='w') (void)start_action(false);
            if(command=='c') (void)start_action(true);
            if(command=='s') ESP_LOGI(TAG,"STATUS state=%s peer=%u id=%"PRIu32" received=%"PRIu32" total=%"PRIu32" retries=%"PRIu32,
                phases[state.phase],peer_known,state.id,state.received,state.total,state.retries);
            if(command=='s') ESP_LOGI(TAG,"RADIO rx=%u valid=%u tx_ok=%u tx_bad=%u stack_free=%u",
                atomic_load(&rx_total),atomic_load(&rx_valid),tx_ok,tx_bad,uxTaskGetStackHighWaterMark(NULL));
        }
        int button=gpio_get_level(GPIO_NUM_0);
        if(previous_button && !button && now-button_at>300000) { (void)start_action(false);button_at=now; }
        previous_button=button;
        if(atomic_exchange(&dtim_pulse,false)) burst_until=now+80000;
        if(atomic_load(&capture_requested)) {
            begin_capture();
            if(state.phase==QUEUED) atomic_store(&capture_requested,false);
        }
        struct cam_event event;
        for(unsigned i=0;i<12 && xQueueReceive(rx_queue,&event,0)==pdPASS;i++) {
            struct cam_packet p;
            if(cam_decode(event.bytes,event.length,&p)) handle(&event,&p);
        }
        if(state.phase==QUEUED) {
            started=now;atomic_store(&pending,true);
            phase(peer_known?(state.photo?CAPTURING:WAKING):DISCOVERING);
        }
        if(state.busy) {
            if(now-started>240000000) fail("Capture timed out; check the remote camera and HaLow link.");
            else if(state.phase==DISCOVERING || state.phase==CAPTURING || state.phase==WAKING) {
                if(now-stage_started>40000000) fail("Remote camera did not respond. Check that the nRF is powered.");
                else if(now<burst_until && now-last_send>10000) {
                    (void)send_packet(state.phase==DISCOVERING?CAM_HELLO:state.photo?CAM_CAPTURE:CAM_LED,state.id,0,0,peer_known?peer:cam_broadcast);
                    last_send=now;
                }
            } else if((state.phase==RECEIVING || state.phase==FINISHING) && (!last_send || now-last_send>350000)) {
                if(attempts>=12) fail("Block delivery timed out. Move closer and retry.");
                else {
                    if(attempts) { take();state.retries++;give(); }
                    attempts++;last_send=now;
                    (void)send_packet(state.phase==RECEIVING?CAM_READ:CAM_FINISH,state.id,state.received,state.total,peer);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
static esp_err_t index_get(httpd_req_t *req) {
    httpd_resp_set_type(req,"text/html");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,camera_page,HTTPD_RESP_USE_STRLEN);
}
static esp_err_t status_get(httpd_req_t *req) {
    char json[384];take();
    snprintf(json,sizeof(json),"{\"state\":\"%s\",\"busy\":%s,\"peer\":%s,\"id\":%"PRIu32",\"total\":%"PRIu32",\"received\":%"PRIu32",\"retries\":%"PRIu32",\"error\":\"%s\",\"dropped_rx\":%u}",
        phases[state.phase],(state.busy || atomic_load(&capture_requested))?"true":"false",state.peer?"true":"false",state.id,state.total,state.received,state.retries,state.error,atomic_load(&dropped_rx));
    give();httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,json,HTTPD_RESP_USE_STRLEN);
}
static esp_err_t action_post(httpd_req_t *req) {
    httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    if(!start_action(strcmp(req->uri,"/api/led")!=0)) {httpd_resp_set_status(req,"409 Conflict");return httpd_resp_sendstr(req,"{\"error\":\"A capture or photo download is already in progress.\"}");}
    httpd_resp_set_status(req,"202 Accepted");return httpd_resp_sendstr(req,"{\"accepted\":true}");
}
static esp_err_t photo_get(httpd_req_t *req) {
    char query[64],id[16];
    if(httpd_req_get_url_query_str(req,query,sizeof(query))!=ESP_OK ||
       httpd_query_key_value(query,"id",id,sizeof(id))!=ESP_OK) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Missing image id");
    char *end;unsigned long wanted=strtoul(id,&end,10);
    if(!id[0] || *end || wanted>UINT32_MAX) return httpd_resp_send_err(req,HTTPD_400_BAD_REQUEST,"Invalid image id");
    take();
    if(state.phase!=COMPLETE || atomic_load(&capture_requested) || state.id!=wanted || !state.image) { give();return httpd_resp_send_err(req,HTTPD_404_NOT_FOUND,"Photo is not available"); }
    state.readers++;uint8_t *image=state.image;uint32_t length=state.total;give();
    httpd_resp_set_type(req,"image/jpeg");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    esp_err_t rc=httpd_resp_send(req,(const char *)image,length);
    take();state.readers--;give();return rc;
}
static void wifi_web_start(void) {
    esp_err_t rc=nvs_flash_init();
    if(rc==ESP_ERR_NVS_NO_FREE_PAGES || rc==ESP_ERR_NVS_NEW_VERSION_FOUND) {ESP_ERROR_CHECK(nvs_flash_erase());rc=nvs_flash_init();}
    ESP_ERROR_CHECK(rc);ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_ap()?ESP_OK:ESP_FAIL);
    wifi_init_config_t init=WIFI_INIT_CONFIG_DEFAULT();ESP_ERROR_CHECK(esp_wifi_init(&init));
    const char *password=CONFIG_CAMERA_AP_PASSWORD;
    ESP_ERROR_CHECK(strlen(password)>=8 && strlen(password)<=63 ? ESP_OK:ESP_ERR_INVALID_ARG);
    wifi_config_t wifi={.ap={.ssid=CONFIG_CAMERA_AP_SSID,.password=CONFIG_CAMERA_AP_PASSWORD,
        .channel=6,.max_connection=4,.authmode=WIFI_AUTH_WPA2_PSK,.pmf_cfg={.required=false}}};
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&wifi));ESP_ERROR_CHECK(esp_wifi_start());
    httpd_config_t config=HTTPD_DEFAULT_CONFIG();config.stack_size=6144;config.lru_purge_enable=true;
    httpd_handle_t server=NULL;ESP_ERROR_CHECK(httpd_start(&server,&config));
    const httpd_uri_t routes[]={
        {.uri="/",.method=HTTP_GET,.handler=index_get},
        {.uri="/api/status",.method=HTTP_GET,.handler=status_get},
        {.uri="/api/capture",.method=HTTP_POST,.handler=action_post},
        {.uri="/api/led",.method=HTTP_POST,.handler=action_post},
        {.uri="/photo.jpg",.method=HTTP_GET,.handler=photo_get},
    };
    for(unsigned i=0;i<sizeof(routes)/sizeof(routes[0]);i++) ESP_ERROR_CHECK(httpd_register_uri_handler(server,&routes[i]));
    ESP_LOGI(TAG,"Connect to Wi-Fi %s and open http://192.168.4.1",CONFIG_CAMERA_AP_SSID);
}
void app_main(void) {
    lock=xSemaphoreCreateMutex();rx_queue=xQueueCreate(12,sizeof(struct cam_event));
    if(!lock || !rx_queue) abort();
    state.id=esp_random()|1U;
    halow_start();
    const gpio_config_t button={.pin_bit_mask=1ULL<<0,.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_ERROR_CHECK(gpio_config(&button));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,256,0,0,NULL,0));
    if(xTaskCreate(worker,"camera_radio",16384,NULL,5,NULL)!=pdPASS) abort();
    wifi_web_start();
}
