# ESP-IDF Modbus RTU Library

A complete implementation of Modbus RTU protocol for ESP-IDF framework, supporting both master and slave modes with RS485 hardware control.

**Supported Chips:** ESP32, ESP32-S2, ESP32-S3, ESP32-C3
**ESP-IDF Version:** >= 4.4 (tested on v5.x)

## Features

- Full Modbus RTU protocol implementation
- Support for both master and slave modes
- RS485 hardware flow control via RTS pin
- All standard function codes supported:
  - 01: Read Coils
  - 02: Read Discrete Inputs
  - 03: Read Holding Registers
  - 04: Read Input Registers
  - 05: Write Single Coil
  - 06: Write Single Register
  - 0F: Write Multiple Coils
  - 10: Write Multiple Registers
- Broadcast message support (function codes that don't require responses)
- Configurable data model sizes
- CRC-16/MODBUS checksum verification
- Write callbacks for handling register/coil updates
- Buffer overflow protection
- Input validation

## Installation

### Via ESP-IDF Component Manager (Recommended)

Add the following to your project's `idf_component.yml`:

```yaml
dependencies:
  <namespace>/esp32-modbus-rtu: "^1.0.0"
```

Replace `<namespace>` with the actual component registry namespace.

### Manual Installation

1. Clone this repository to your ESP-IDF project components directory
2. Include the header in your source files: `#include "modbus_rtu.h"`
3. Register the component in your CMakeLists.txt file

## Usage

### Slave Mode Example

```c
#include "modbus_rtu.h"

modbus_context_t modbus_ctx;

void app_main() {
    // Initialize Modbus
    modbus_init(&modbus_ctx, UART_NUM_2, 17, 16, 5);
    modbus_ctx.address = 0x01;  // Set slave address

    // Set callbacks for write operations
    modbus_ctx.coil_write_callback = my_coil_callback;
    modbus_ctx.register_write_callback = my_reg_callback;

    while(1) {
        modbus_slave_poll(&modbus_ctx);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### Master Mode Example

```c
#include "modbus_rtu.h"

modbus_context_t modbus_ctx;
uint16_t holding_registers[10];
uint8_t coils[16];

void app_main() {
    // Initialize Modbus as master
    modbus_init(&modbus_ctx, UART_NUM_2, 17, 16, 5);

    while(1) {
        // Read holding registers from slave device at address 0x01
        modbus_error_t err = modbus_master_read_holding_registers(
            &modbus_ctx,
            0x01,           // Slave address
            0x0000,         // Starting register address
            10,             // Number of registers to read
            holding_registers
        );

        if (err == MODBUS_OK) {
            // Successfully read registers
        }

        // Write a single register to slave device at address 0x01
        err = modbus_master_write_single_register(
            &modbus_ctx,
            0x01,           // Slave address
            0x0001,         // Register address
            0x1234          // Value to write
        );

        // Read coils from slave device
        err = modbus_master_read_coils(
            &modbus_ctx,
            0x01,           // Slave address
            0x0000,         // Starting coil address
            16,             // Number of coils to read
            coils
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```
