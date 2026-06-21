/**
 * slave_example — Modbus RTU slave at address 1
 *
 * Wiring: same as master_example (mirror the RS485 bus)
 *
 * Data model:
 *   coils[0..3]         = driven by on_write callback (toggle GPIO outputs)
 *   discrete_inputs[0]  = GPIO input (button/sensor)
 *   holding_regs[0..3]  = R/W scratch registers
 *   input_regs[0]       = ADC reading, updated in background task
 *   input_regs[1]       = uptime in seconds
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "modbus_rtu.h"

#define TAG             "slave_example"

#define GPIO_COIL_0     GPIO_NUM_18
#define GPIO_COIL_1     GPIO_NUM_19
#define GPIO_DIN_0      GPIO_NUM_34   // input only on most ESP32s

static modbus_ctx_t mb;

// Called from modbus_slave_poll() whenever a write FC completes.
static void on_write(modbus_ctx_t *ctx, uint8_t fc, uint16_t start_addr, uint16_t count)
{
    ESP_LOGI(TAG, "write fc=0x%02X addr=%u count=%u", fc, start_addr, count);

    // Mirror coil[0] and coil[1] to GPIO outputs
    if (fc == MODBUS_FC_WRITE_SINGLE_COIL || fc == MODBUS_FC_WRITE_MULTIPLE_COILS) {
        gpio_set_level(GPIO_COIL_0, ctx->data.coils[0]);
        gpio_set_level(GPIO_COIL_1, ctx->data.coils[1]);
    }
}

// Background task: updates input registers from hardware.
static void sensor_task(void *arg)
{
    uint32_t uptime = 0;
    while (1) {
        // Fake ADC read — replace with adc_oneshot_read() for real use
        mb.data.input_regs[0] = (uint16_t)(esp_random() & 0x0FFF);
        mb.data.input_regs[1] = (uint16_t)(uptime & 0xFFFF);

        // Sample discrete input from GPIO
        mb.data.discrete_inputs[0] = gpio_get_level(GPIO_DIN_0);

        uptime++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Slave poll task — blocks waiting for requests.
static void slave_task(void *arg)
{
    while (1) {
        modbus_err_t err = modbus_slave_poll(&mb);
        if (err != MODBUS_OK && err != MODBUS_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "poll: %s", modbus_strerror(err));
        }
    }
}

void app_main(void)
{
    // GPIO setup
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << GPIO_COIL_0) | (1ULL << GPIO_COIL_1),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << GPIO_DIN_0),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&in_cfg);

    // Modbus init
    modbus_config_t cfg = {
        .uart_port           = UART_NUM_1,
        .tx_pin              = 17,
        .rx_pin              = 16,
        .rts_pin             = 4,
        .baud_rate           = 9600,
        .mode                = MODBUS_MODE_SLAVE,
        .slave_addr          = 1,
        .response_timeout_ms = 500,
    };

    modbus_err_t err = modbus_init(&mb, &cfg);
    if (err != MODBUS_OK) {
        ESP_LOGE(TAG, "init failed: %s", modbus_strerror(err));
        return;
    }

    // Pre-populate some holding registers
    mb.data.holding_regs[0] = 0xDEAD;
    mb.data.holding_regs[1] = 0xBEEF;

    mb.on_write = on_write;

    xTaskCreate(sensor_task, "sensor", 2048, NULL, 5, NULL);
    xTaskCreate(slave_task,  "slave",  4096, NULL, 10, NULL);
}
