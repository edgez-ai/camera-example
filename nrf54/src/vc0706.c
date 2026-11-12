/* Copyright 2026 EdgeZeron AI AB. See LICENSE.
 * Adapted from template-offgrid-tracker 498f883 camera driver (DMA RX).
 * Uses that board's 921600-baud Putal/VC0706 command dialect and pin mapping.
 */
#include "vc0706.h"
#include "camera_protocol.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(vc0706,LOG_LEVEL_INF);

static const struct device *const uart=DEVICE_DT_GET(DT_ALIAS(camera0));
static const struct gpio_dt_spec power=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),camera_power_gpios);
static uint8_t ring[2048];
static volatile unsigned head,tail,overruns;
static bool running;
static uint8_t cached_version[CAM_VERSION_MAX];
static uint16_t cached_version_length;
static volatile bool rx_enabled;
static volatile int rx_error;
static uint8_t dma[2][512] __aligned(4);
static uint8_t dma_next;
static volatile uint32_t rx_bytes;
K_SEM_DEFINE(rx_disabled,0,1);
/* JPEG blocks are separate from the DMA byte ring. Only the producer issues
 * camera READ commands; a full queue applies backpressure to the frozen frame.
 * The consumer retains its last block independently for radio retransmission. */
struct jpeg_block { uint32_t offset; uint16_t length; uint8_t bytes[CAM_BLOCK_SIZE]; };
K_MSGQ_DEFINE(jpeg_blocks,sizeof(struct jpeg_block),8,4);
K_SEM_DEFINE(stream_start,0,1);
K_SEM_DEFINE(stream_idle,0,1);
static atomic_t stream_active,stream_cancel,stream_error,stream_done;
static uint32_t stream_length,stream_delivered,stream_produced,stream_replays;
static struct jpeg_block replay;
static int read_camera_block(uint32_t offset,uint8_t *out,size_t length);
static void stop_stream(void) {
    if(!atomic_get(&stream_active)) return;
    atomic_set(&stream_cancel,1);
    /* Producer checks cancellation during UART waits and full-queue waits. */
    (void)k_sem_take(&stream_idle,K_FOREVER);
    atomic_clear(&stream_active);
    k_msgq_purge(&jpeg_blocks);
    LOG_INF("Camera stream stopped produced=%u delivered=%u replays=%u rx_bytes=%u overruns=%u error=%ld",
        stream_produced,stream_delivered,stream_replays,rx_bytes,overruns,(long)atomic_get(&stream_error));
    replay.length=0;
}
static void camera_reader(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    for(;;) {
        (void)k_sem_take(&stream_start,K_FOREVER);
        struct jpeg_block next;
        uint8_t previous_byte=0;
        for(uint32_t offset=0;offset<stream_length && !atomic_get(&stream_cancel);) {
            next.offset=offset;next.length=MIN(CAM_BLOCK_SIZE,stream_length-offset);
            int rc=read_camera_block(offset,next.bytes,next.length);
            if(!rc && ((!offset && (next.bytes[0]!=0xff || next.bytes[1]!=0xd8)) ||
                (offset+next.length==stream_length &&
                 ((next.length>1?next.bytes[next.length-2]:previous_byte)!=0xff ||
                  next.bytes[next.length-1]!=0xd9)))) rc=-EBADMSG;
            if(rc) {
                if(!atomic_get(&stream_cancel)) {
                    atomic_set(&stream_error,rc);
                    LOG_ERR("Camera producer offset=%u length=%u failed=%d rx_bytes=%u overruns=%u",
                        offset,next.length,rc,rx_bytes,overruns);
                }
                break;
            }
            while(k_msgq_put(&jpeg_blocks,&next,K_MSEC(20)))
                if(atomic_get(&stream_cancel)) break;
            if(atomic_get(&stream_cancel)) break;
            previous_byte=next.bytes[next.length-1];
            offset+=next.length;stream_produced=offset;
        }
        atomic_set(&stream_done,1);
        k_sem_give(&stream_idle);
    }
}
K_THREAD_DEFINE(camera_reader_id,3072,camera_reader,NULL,NULL,NULL,15,0,0);
static void start_stream(uint32_t length) {
    k_msgq_purge(&jpeg_blocks);k_sem_reset(&stream_idle);
    stream_length=length;stream_delivered=stream_produced=stream_replays=0;replay.length=0;
    atomic_clear(&stream_cancel);atomic_clear(&stream_error);atomic_clear(&stream_done);
    atomic_set(&stream_active,1);k_sem_give(&stream_start);
    LOG_INF("Camera JPEG ring started slots=8 block=%u total=%u",CAM_BLOCK_SIZE,length);
}
static void receive(const struct device *dev,struct uart_event *event,void *arg) {
    ARG_UNUSED(arg);
    switch(event->type) {
    case UART_RX_RDY:
        for(size_t i=0;i<event->data.rx.len;i++) {
            unsigned next=(head+1)%sizeof(ring);
            if(next==tail) { overruns++;continue; }
            ring[head]=event->data.rx.buf[event->data.rx.offset+i];head=next;rx_bytes++;
        }
        break;
    case UART_RX_BUF_REQUEST:
        if(uart_rx_buf_rsp(dev,dma[dma_next],sizeof(dma[0]))) rx_error=-EIO;
        dma_next^=1;
        break;
    case UART_RX_STOPPED:
        if(rx_enabled) rx_error=-EIO;
        break;
    case UART_RX_DISABLED:
        if(rx_enabled) rx_error=-EIO;
        k_sem_give(&rx_disabled);
        break;
    default:break;
    }
}
static bool pop(uint8_t *out) {
    unsigned key=irq_lock();
    bool ok=head!=tail;
    if(ok) { *out=ring[tail]; tail=(tail+1)%sizeof(ring); }
    irq_unlock(key);
    return ok;
}
static int exact(uint8_t *out,size_t n,int timeout) {
    int64_t deadline=k_uptime_get()+timeout;
    unsigned errors=overruns;
    while(n) {
        if(atomic_get(&stream_active) && atomic_get(&stream_cancel)) return -ECANCELED;
        if(rx_error) return rx_error;
        if(overruns!=errors) return -EOVERFLOW;
        if(pop(out)) { out++; n--; }
        else if(k_uptime_get()>=deadline) return -ETIMEDOUT;
        else k_msleep(1);
    }
    return 0;
}
static void write_bytes(const uint8_t *bytes,size_t n) {
    for(size_t i=0;i<n;i++) uart_poll_out(uart,bytes[i]);
}
static void drain(void) {
    uint8_t b;
    int64_t end=k_uptime_get()+500,quiet=k_uptime_get()+20;
    while(k_uptime_get()<quiet && k_uptime_get()<end) {
        if(pop(&b)) quiet=k_uptime_get()+20;
        else k_msleep(1);
    }
}
static int command(const uint8_t *bytes,size_t n,uint8_t op,int timeout) {
    uint8_t reply[5];
    drain(); write_bytes(bytes,n);
    int rc=exact(reply,sizeof(reply),timeout);
    if(rc) return rc;
    const uint8_t expected[]={0x76,0,op,0,0};
    bool clone_resolution_ack=op==0x31 && reply[0]==0x76 && !reply[1] &&
        reply[2]==op && reply[3]==1 && !reply[4];
    if(clone_resolution_ack) {
        LOG_WRN("Resolution ACK status=1; receiver must verify JPEG dimensions");return 0;
    }
    if(memcmp(reply,expected,sizeof(reply))) {
        LOG_ERR("Command 0x%02x unexpected ACK %02x %02x %02x %02x %02x",op,reply[0],reply[1],reply[2],reply[3],reply[4]);
        return -EBADMSG;
    }
    return 0;
}
static int reset(void) {
    const uint8_t cmd[]={0x56,0,0x26,0},prefix[]={0x76,0,0x26,0};
    drain(); write_bytes(cmd,sizeof(cmd));
    unsigned match=0;
    int64_t deadline=k_uptime_get()+2000;
    while(match<sizeof(prefix) && k_uptime_get()<deadline) {
        uint8_t b;
        if(!pop(&b)) { k_msleep(1); continue; }
        match=b==prefix[match] ? match+1 : b==prefix[0] ? 1 : 0;
    }
    return match==sizeof(prefix) ? 0 : -ETIMEDOUT;
}
void vc0706_off(void) {
    stop_stream();
    if(device_is_ready(uart)) {
        if(rx_enabled) {
            rx_enabled=false;
            k_sem_reset(&rx_disabled);
            if(!uart_rx_disable(uart)) (void)k_sem_take(&rx_disabled,K_MSEC(100));
        }
        (void)pm_device_action_run(uart,PM_DEVICE_ACTION_SUSPEND);
        running=false;
    }
    if(gpio_is_ready_dt(&power)) (void)gpio_pin_configure_dt(&power,GPIO_OUTPUT_INACTIVE);
}
static int prepare_camera(void) {
    if(!device_is_ready(uart) || !gpio_is_ready_dt(&power)) return -ENODEV;
    cached_version_length=0;
    int rc=gpio_pin_configure_dt(&power,GPIO_OUTPUT_ACTIVE);
    if(rc) return rc;
    const char *stage="uart-resume";
    k_msleep(4000);
    rc=pm_device_action_run(uart,PM_DEVICE_ACTION_RESUME);
    if(rc && rc!=-EALREADY && rc!=-ENOSYS && rc!=-ENOTSUP) goto fail;
    running=true;
    struct uart_config config={.baudrate=CONFIG_CAMERA_UART_BAUD,
        .parity=UART_CFG_PARITY_NONE,.stop_bits=UART_CFG_STOP_BITS_1,
        .data_bits=UART_CFG_DATA_BITS_8,.flow_ctrl=UART_CFG_FLOW_CTRL_NONE};
    stage="uart-configure";
    rc=uart_configure(uart,&config);
    if(rc) goto fail;
    head=tail=overruns=rx_bytes=0;rx_error=0;dma_next=1;
    stage="dma-rx-start";
    rc=uart_callback_set(uart,receive,NULL);
    if(rc) goto fail;
    rx_enabled=true;
    rc=uart_rx_enable(uart,dma[0],sizeof(dma[0]),1000);
    if(rc) { rx_enabled=false;goto fail; }
    LOG_INF("Camera UART ready baud=%u DMA=2x512 TX=P1.12 RX=P1.11 power=P1.09",CONFIG_CAMERA_UART_BAUD);
    k_msleep(50);
    stage="reset-ack";
    rc=reset(); if(rc) goto fail;
    LOG_INF("Camera reset acknowledged");
    k_msleep(4000);
    stage="version";
    const uint8_t version_cmd[]={0x56,0,0x11,0};
    uint8_t version_header[4];
    drain();write_bytes(version_cmd,sizeof(version_cmd));
    rc=exact(version_header,sizeof(version_header),2000);if(rc) goto fail;
    const uint8_t version_prefix[]={0x76,0,0x11,0};
    if(memcmp(version_header,version_prefix,sizeof(version_prefix))) { rc=-EBADMSG;goto fail; }
    /* Consume the optional version body before sending another command. */
    uint8_t version_length,version[64];
    rc=exact(&version_length,1,250);
    if(!rc) {
        if(!version_length || version_length>sizeof(version)) { rc=-EBADMSG;goto fail; }
        rc=exact(version,version_length,2000);if(rc) goto fail;
        memcpy(cached_version,version,version_length);cached_version_length=version_length;
        LOG_INF("Camera version received bytes=%u",version_length);
    } else if(rc!=-ETIMEDOUT) goto fail;
    drain();
    LOG_INF("Camera version prefix acknowledged");
    return 0;
fail:
    LOG_ERR("Camera stage=%s error=%d rx_bytes=%u overruns=%u rx_error=%d",stage,rc,rx_bytes,overruns,rx_error);
    vc0706_off();return rc;
}
int vc0706_version(uint8_t *version,size_t capacity,uint16_t *length) {
    if(!version || !length || capacity<CAM_VERSION_MAX) return -EINVAL;
    int rc=prepare_camera();
    if(!rc) { memcpy(version,cached_version,cached_version_length);*length=cached_version_length; }
    vc0706_off();return rc;
}
int vc0706_capture(uint16_t width,uint16_t height,uint32_t *length) {
    const struct cam_resolution *mode=cam_resolution_find(width,height);
    if(!mode || !length) return -EINVAL;
    int rc=prepare_camera();if(rc) return rc;
    const char *stage="release-frame";
    const uint8_t release[]={0x56,0,0x36,1,3};
    rc=command(release,sizeof(release),0x36,2000);if(rc) goto fail;
    /* Existing board example's verified 640x480 register write. */
    uint8_t resolution[]={0x56,0,0x31,5,mode->field,1,0,0x19,mode->code};
    LOG_INF("Setting camera resolution %ux%u",width,height);
    stage="resolution";
    rc=command(resolution,sizeof(resolution),0x31,2000); if(rc) goto fail;
    k_msleep(250);stage="capture-ack";
    const uint8_t capture[]={0x56,0,0x36,1,0};
    rc=command(capture,sizeof(capture),0x36,10000); if(rc) goto fail;
    stage="jpeg-length";
    const uint8_t len_cmd[]={0x56,0,0x34,1,0};
    uint8_t reply[9];
    drain(); write_bytes(len_cmd,sizeof(len_cmd));
    rc=exact(reply,sizeof(reply),2000); if(rc) goto fail;
    const uint8_t prefix[]={0x76,0,0x34,0,4};
    if(memcmp(reply,prefix,sizeof(prefix))) { rc=-EBADMSG; goto fail; }
    *length=sys_get_be32(reply+5);
    if(*length<4 || *length>CAM_MAX_JPEG) { rc=-EMSGSIZE; goto fail; }
    start_stream(*length);
    return 0;
fail:
    LOG_ERR("Camera stage=%s error=%d rx_bytes=%u overruns=%u rx_error=%d",stage,rc,rx_bytes,overruns,rx_error);
    vc0706_off();
    return rc;
}
static int read_camera_block(uint32_t offset,uint8_t *out,size_t length) {
    if(!running || !out || length>CAM_BLOCK_SIZE) return -EINVAL;
    unsigned errors=overruns;
    while(length) {
        size_t n=MIN(length,256U);
        uint8_t cmd[]={0x56,0,0x32,12,0,0x0a,0,0,0,0,0,0,0,0,0,0xff};
        sys_put_be32(offset,cmd+6); sys_put_be32(n,cmd+10);
        uint8_t frame[5];
        const uint8_t expected[]={0x76,0,0x32,0,0};
        drain(); write_bytes(cmd,sizeof(cmd));
        int rc=exact(frame,sizeof(frame),2000);
        if(rc) return rc;
        if(memcmp(frame,expected,sizeof(frame))) return -EBADMSG;
        rc=exact(out,n,2000); if(rc) return rc;
        rc=exact(frame,sizeof(frame),2000); if(rc) return rc;
        if(memcmp(frame,expected,sizeof(frame)) || overruns!=errors) return -EBADMSG;
        offset+=n; out+=n; length-=n;
    }
    return 0;
}

int vc0706_read(uint32_t offset,uint8_t *out,size_t length) {
    if(!atomic_get(&stream_active) || !out || length>CAM_BLOCK_SIZE ||
       !cam_valid_block(stream_length,offset,length)) return -EINVAL;
    if(replay.length && offset==replay.offset && length==replay.length) {
        memcpy(out,replay.bytes,length);stream_replays++;return 0;
    }
    /* Ignore delayed requests already superseded by the receiver's next READ. */
    if(offset<stream_delivered) return -EALREADY;
    if(offset!=stream_delivered) return -ERANGE;
    int64_t deadline=k_uptime_get()+2500;
    while(k_msgq_get(&jpeg_blocks,&replay,K_MSEC(20))) {
        int rc=atomic_get(&stream_error);
        if(rc) return rc;
        if(atomic_get(&stream_done)) return -ENODATA;
        if(k_uptime_get()>=deadline) return -ETIMEDOUT;
    }
    if(replay.offset!=offset || replay.length!=length) return -EBADMSG;
    memcpy(out,replay.bytes,length);stream_delivered+=length;
    return 0;
}
