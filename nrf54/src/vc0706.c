/* Copyright 2026 EdgeZeron AI AB. See LICENSE.
 * Adapted from template-offgrid-tracker 39820d3 camera diagnostic.
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

static const struct device *const uart=DEVICE_DT_GET(DT_ALIAS(camera0));
static const struct gpio_dt_spec power=GPIO_DT_SPEC_GET(DT_PATH(zephyr_user),camera_power_gpios);
static uint8_t ring[2048];
static volatile unsigned head,tail,overruns;
static bool running;
static void receive(const struct device *dev,void *arg) {
    ARG_UNUSED(arg);
    while(uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t bytes[64];
        int n=uart_fifo_read(dev,bytes,sizeof(bytes));
        if(n<=0) break;
        for(int i=0;i<n;i++) {
            unsigned next=(head+1)%sizeof(ring);
            if(next==tail) { overruns++; continue; }
            ring[head]=bytes[i]; head=next;
        }
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
    int64_t end=k_uptime_get()+40,quiet=k_uptime_get()+2;
    while(k_uptime_get()<quiet && k_uptime_get()<end) {
        if(pop(&b)) quiet=k_uptime_get()+2;
        else k_msleep(1);
    }
}
static int command(const uint8_t *bytes,size_t n,uint8_t op,int timeout) {
    uint8_t reply[5];
    drain(); write_bytes(bytes,n);
    int rc=exact(reply,sizeof(reply),timeout);
    if(rc) return rc;
    const uint8_t expected[]={0x76,0,op,0,0};
    return memcmp(reply,expected,sizeof(reply)) ? -EBADMSG : 0;
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
        if(running) uart_irq_rx_disable(uart);
        (void)pm_device_action_run(uart,PM_DEVICE_ACTION_SUSPEND);
        running=false;
    }
    if(gpio_is_ready_dt(&power)) (void)gpio_pin_configure_dt(&power,GPIO_OUTPUT_INACTIVE);
}
int vc0706_capture(uint32_t *length) {
    if(!length || !device_is_ready(uart) || !gpio_is_ready_dt(&power)) return -ENODEV;
    int rc=gpio_pin_configure_dt(&power,GPIO_OUTPUT_ACTIVE);
    if(rc) return rc;
    rc=pm_device_action_run(uart,PM_DEVICE_ACTION_RESUME);
    if(rc && rc!=-EALREADY) goto fail;
    running=true;
    k_msleep(4000);
    struct uart_config config={.baudrate=CONFIG_CAMERA_UART_BAUD,
        .parity=UART_CFG_PARITY_NONE,.stop_bits=UART_CFG_STOP_BITS_1,
        .data_bits=UART_CFG_DATA_BITS_8,.flow_ctrl=UART_CFG_FLOW_CTRL_NONE};
    rc=uart_configure(uart,&config);
    if(rc) goto fail;
    head=tail=overruns=0;
    rc=uart_irq_callback_user_data_set(uart,receive,NULL);
    if(rc) goto fail;
    uart_irq_rx_enable(uart);
    k_msleep(50);
    rc=reset(); if(rc) goto fail;
    k_msleep(4000);
    /* Existing board example's verified 640x480 register write. */
    const uint8_t resolution[]={0x56,0,0x31,5,4,1,0,0x19,0};
    rc=command(resolution,sizeof(resolution),0x31,2000); if(rc) goto fail;
    const uint8_t capture[]={0x56,0,0x36,1,0};
    rc=command(capture,sizeof(capture),0x36,10000); if(rc) goto fail;
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
