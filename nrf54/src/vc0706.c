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
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(vc0706,LOG_LEVEL_INF);

static const struct device *const uart=DEVICE_DT_GET(DT_ALIAS(camera0));
static const struct gpio_dt_spec power=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),camera_power_gpios);
static uint8_t ring[2048];
static volatile unsigned head,tail,overruns;
static bool running;
static volatile bool rx_enabled;
static volatile int rx_error;
static uint8_t dma[2][512] __aligned(4);
static uint8_t dma_next;
static volatile uint32_t rx_bytes;
K_SEM_DEFINE(rx_disabled,0,1);
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
int vc0706_capture(uint32_t *length) {
    if(!length || !device_is_ready(uart) || !gpio_is_ready_dt(&power)) return -ENODEV;
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
        LOG_INF("Camera version received bytes=%u",version_length);
    } else if(rc!=-ETIMEDOUT) goto fail;
    drain();
    LOG_INF("Camera version prefix acknowledged");
    stage="release-frame";
    const uint8_t release[]={0x56,0,0x36,1,3};
    rc=command(release,sizeof(release),0x36,2000);if(rc) goto fail;
    /* Existing board example's verified 640x480 register write. */
    const uint8_t resolution[]={0x56,0,0x31,5,4,1,0,0x19,0};
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
    return 0;
fail:
    LOG_ERR("Camera stage=%s error=%d rx_bytes=%u overruns=%u rx_error=%d",stage,rc,rx_bytes,overruns,rx_error);
    vc0706_off();
    return rc;
}
int vc0706_read(uint32_t offset,uint8_t *out,size_t length) {
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
