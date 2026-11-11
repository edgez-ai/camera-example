#if defined(CONFIG_OFFGRID_NRF54_SENSOR_SPI_SLEEP_PINS)
#include <zephyr/pm/device.h>
#include <hal/nrf_gpio.h>

/* Passive evidence: successful suspends, resumes, errors, suspended state.
 * All transitions run in the SDK thread, serialized by its transport lock. */
volatile uint32_t sensor_spi_sleep_state[4];
/* Last errors per stage, plus already-released context count. */
volatile int32_t sensor_spi_sleep_diagnostics[6];

int morsemicro_sensor_spi_resume(void)
{
	if (!sensor_spi_sleep_state[3]) {
		return 0;
	}
	int rc = pm_device_action_run(bus_config.spi.bus, PM_DEVICE_ACTION_RESUME);
	if (rc != 0 && rc != -EALREADY) {
		sensor_spi_sleep_diagnostics[0] = rc;
		++sensor_spi_sleep_state[2];
		LOG_ERR("Sensor SPI resume failed: %d", rc);
		return rc;
	}
	/* The SPIM PM callback restores the default SCK/MOSI/MISO pinctrl state. */
	rc = gpio_pin_configure_dt(&bus_config.spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		sensor_spi_sleep_diagnostics[1] = rc;
		++sensor_spi_sleep_state[2];
		return rc;
	}
	sensor_spi_sleep_state[3] = 0;
	++sensor_spi_sleep_state[1];
	return 0;
}

static void morsemicro_sensor_spi_suspend(void)
{
	if (sensor_spi_sleep_state[3]) {
		return;
	}
	/* SPI_LOCK_ON retains the controller context after a transfer. Release it
	 * while the SDK transport lock is held, before asking PM to suspend. */
	int rc = spi_release(bus_config.spi.bus, &bus_config.spi.config);
	if (rc == -EALREADY) {
		/* Zephyr's async SPI API reports this when no matching context is
		 * owned, including initial startup and after training releases it. */
		++sensor_spi_sleep_diagnostics[5];
	}
	if (rc != 0 && rc != -EINVAL && rc != -EALREADY) {
		sensor_spi_sleep_diagnostics[2] = rc;
		++sensor_spi_sleep_state[2];
		return;
	}
	rc = gpio_pin_configure_dt(&bus_config.spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		sensor_spi_sleep_diagnostics[3] = rc;
	}
	if (rc == 0) {
		rc = pm_device_action_run(bus_config.spi.bus, PM_DEVICE_ACTION_SUSPEND);
	}
	if (rc != 0 && rc != -EALREADY) {
		sensor_spi_sleep_diagnostics[4] = rc;
		++sensor_spi_sleep_state[2];
		LOG_ERR("Sensor SPI suspend failed: %d", rc);
		return;
	}
	/* Match the carrier's all-off benchmark. The radio stays powered, CS
	 * remains inactive, and BUSY retains its existing wake interrupt. */
	nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(2, 1));
	nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(2, 2));
	nrf_gpio_cfg_default(NRF_GPIO_PIN_MAP(2, 4));
	sensor_spi_sleep_state[3] = 1;
	++sensor_spi_sleep_state[0];
}
#endif
