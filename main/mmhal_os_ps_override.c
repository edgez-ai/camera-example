/*
 * ESP32-S3 Morse HAL initialization.
 *
 * The upstream implementation installs the global GPIO ISR service without
 * first clearing interrupt state retained across a software reset. An asserted
 * MM6108 IRQ can consequently starve CPU0 in gpio_isr_loop before the Morse
 * per-pin handlers exist.
 */

#include "mmhal_os.h"
#include "mmosal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"

static bool gpio_isr_service_installed;

void mmhal_init(void)
{
    gpio_hold_dis((gpio_num_t)CONFIG_MM_RESET_N);
    gpio_deep_sleep_hold_dis();
    vTaskDelay(pdMS_TO_TICKS(2));

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CONFIG_MM_RESET_N,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_MM_RESET_N, 0));

    /*
     * GPIO interrupt routing survives an ESP32 software reset. Disable every
     * GPIO source before first installing the MCU-global ISR service; limiting
     * this to IRQ/BUSY is insufficient when a previously flashed application
     * left another level interrupt enabled. Once the service is installed,
     * application handlers are live: a radio restart must preserve them.
     */
    if (!gpio_isr_service_installed) {
        for (gpio_num_t pin = GPIO_NUM_0; pin < GPIO_NUM_MAX; ++pin) {
            if (GPIO_IS_VALID_GPIO(pin)) {
                ESP_ERROR_CHECK(gpio_intr_disable(pin));
            }
        }
    }

    /* Configure both Morse sources while the MM6108 remains in reset. */
    io_conf.pin_bit_mask = (1ULL << CONFIG_MM_SPI_IRQ) |
                           (1ULL << CONFIG_MM_BUSY);
    io_conf.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_intr_disable((gpio_num_t)CONFIG_MM_SPI_IRQ));
    ESP_ERROR_CHECK(gpio_intr_disable((gpio_num_t)CONFIG_MM_BUSY));

    const uint64_t pending_mask = gpio_isr_service_installed ?
        ((UINT64_C(1) << CONFIG_MM_SPI_IRQ) | (UINT64_C(1) << CONFIG_MM_BUSY)) :
        UINT64_MAX;
    REG_WRITE(GPIO_STATUS_W1TC_REG, (uint32_t)pending_mask);
    REG_WRITE(GPIO_STATUS1_W1TC_REG, (uint32_t)(pending_mask >> 32));

    if (!gpio_isr_service_installed) {
        const esp_err_t err = gpio_install_isr_service(0);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            gpio_isr_service_installed = true;
        } else {
            ESP_ERROR_CHECK(err);
        }
    }
}

void mmhal_log_write(const uint8_t *data, size_t length)
{
    while (length-- != 0U) {
        putc(*data++, stdout);
    }
}

void mmhal_log_flush(void)
{
}

void mmhal_reset(void)
{
    esp_restart();
    for (;;) {
    }
}
