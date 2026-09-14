# ESP-IDF Modbus RTU Library

A complete implementation of Modbus RTU protocol for ESP-IDF framework, supporting both master and slave modes with RS485 hardware control.

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