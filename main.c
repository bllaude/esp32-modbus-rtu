#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "modbus_rtu.h"

static const char *TAG = "modbus_example";

// Define pins for UART communication
#define UART_NUM UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16
#define RTS_PIN 5

// Define GPIO pins for coil control
#define COIL_GPIO_0 18
#define COIL_GPIO_1 19
#define COIL_GPIO_2 21
#define COIL_GPIO_3 22

// Modbus slave address
#define SLAVE_ADDR 0x01

// Context for Modbus communication
modbus_context_t modbus_ctx;

// Write callback for coils - controls GPIOs
void coil_write_callback(uint16_t addr, uint16_t value) {
    bool state = (value == 0xFF00) ? true : false;
    
    switch(addr) {
        case 0:
            gpio_set_level(COIL_GPIO_0, state);
            ESP_LOGI(TAG, "Coil 0 -> GPIO %d", state);
            break;
        case 1:
            gpio_set_level(COIL_GPIO_1, state);
            ESP_LOGI(TAG, "Coil 1 -> GPIO %d", state);
            break;
        case 2:
            gpio_set_level(COIL_GPIO_2, state);
            ESP_LOGI(TAG, "Coil 2 -> GPIO %d", state);
            break;
        case 3:
            gpio_set_level(COIL_GPIO_3, state);
            ESP_LOGI(TAG, "Coil 3 -> GPIO %d", state);
            break;
        default:
            ESP_LOGW(TAG, "Unknown coil address: %d", addr);
            break;
    }
}

// Write callback for registers - just logs the change
void register_write_callback(uint16_t addr, uint16_t value) {
    ESP_LOGI(TAG, "Register %d written with value 0x%04X", addr, value);
}

// Task to update input registers with simulated sensor data
void sensor_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while(1) {
        // Simulate sensor readings by incrementing values
        static uint16_t temp_reading = 250;   // Temperature * 10 (25.0°C)
        static uint16_t hum_reading = 600;    // Humidity * 10 (60.0%)
        
        temp_reading += 1;
        if(temp_reading > 350) temp_reading = 200;  // Reset after reaching max
        
        hum_reading += 2;
        if(hum_reading > 800) hum_reading = 400;    // Reset after reaching max
        
        // Update input registers
        modbus_set_input_register(&modbus_ctx, 0, temp_reading);
        modbus_set_input_register(&modbus_ctx, 1, hum_reading);
        
        ESP_LOGI(TAG, "Sensor readings updated - Temp: %.1f°C, Humidity: %.1f%%", 
                 temp_reading / 10.0, hum_reading / 10.0);
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000));  // Update every 5 seconds
    }
}

// Main Modbus slave task
void modbus_slave_task(void *pvParameters) {
    while(1) {
        // Poll for incoming Modbus requests
        int result = modbus_slave_poll(&modbus_ctx);
        if(result > 0) {
            ESP_LOGI(TAG, "Processed Modbus frame, length: %d", result);
        }
        
        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() {
    ESP_LOGI(TAG, "Starting Modbus RTU example...");
    
    // Initialize GPIOs for coil control
    gpio_reset_pin(COIL_GPIO_0);
    gpio_set_direction(COIL_GPIO_0, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_GPIO_0, 0);
    
    gpio_reset_pin(COIL_GPIO_1);
    gpio_set_direction(COIL_GPIO_1, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_GPIO_1, 0);
    
    gpio_reset_pin(COIL_GPIO_2);
    gpio_set_direction(COIL_GPIO_2, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_GPIO_2, 0);
    
    gpio_reset_pin(COIL_GPIO_3);
    gpio_set_direction(COIL_GPIO_3, GPIO_MODE_OUTPUT);
    gpio_set_level(COIL_GPIO_3, 0);
    
    // Initialize Modbus context
    int ret = modbus_init(&modbus_ctx, UART_NUM, TX_PIN, RX_PIN, RTS_PIN);
    if(ret != 0) {
        ESP_LOGE(TAG, "Failed to initialize Modbus");
        return;
    }
    
    // Set our slave address
    modbus_ctx.address = SLAVE_ADDR;
    
    // Register callbacks
    modbus_ctx.coil_write_callback = coil_write_callback;
    modbus_ctx.register_write_callback = register_write_callback;
    
    // Initialize some holding registers with default values
    modbus_set_holding_register(&modbus_ctx, 0, 1234);
    modbus_set_holding_register(&modbus_ctx, 1, 5678);
    
    // Initialize discrete inputs
    modbus_set_discrete_input(&modbus_ctx, 0, 1);
    modbus_set_discrete_input(&modbus_ctx, 1, 0);
    
    ESP_LOGI(TAG, "Modbus RTU slave initialized successfully");
    ESP_LOGI(TAG, "Slave address: 0x%02X", SLAVE_ADDR);
    ESP_LOGI(TAG, "UART: %d, TX: %d, RX: %d, RTS: %d", UART_NUM, TX_PIN, RX_PIN, RTS_PIN);
    
    // Create tasks
    xTaskCreate(modbus_slave_task, "modbus_slave", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_update", 2048, NULL, 3, NULL);
}