# esp32-modbus-rtu

Modbus RTU master/slave library for ESP-IDF.

- Full master and slave roles in one component
- Function codes: `FC01` `FC02` `FC03` `FC04` `FC05` `FC06` `FC0F` `FC10`
- CRC16 with hardware-validated test vectors
- RS485 DE/RE direction control via configurable GPIO
- Slave data model: coils, discrete inputs, holding registers, input registers
- Write callback on slave for GPIO/peripheral side-effects
- All three hardware UARTs supported (UART0/1/2)

## Hardware

Requires an RS485 transceiver (MAX485, SP3485, or equivalent) between the ESP32 UART pins and the RS485 bus.

```
ESP32 GPIO17 (TX) ──► DI  ┐
ESP32 GPIO16 (RX) ◄── RO  │ MAX485
ESP32 GPIO4  (DE) ──► DE  │
                   ──► RE  ┘
                      A/B ──── RS485 bus
```

DE and RE are typically tied together and driven by a single GPIO (configured as `rts_pin`). Set `rts_pin = -1` for direct UART-to-UART wiring without a transceiver.

## Building

Requires ESP-IDF v5.x.

```bash
# clone and enter an example
git clone https://github.com/youruser/esp32-modbus-rtu
cd esp32-modbus-rtu/examples/master_example

# set target and build
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Usage

### master

```c
#include "modbus_rtu.h"

modbus_ctx_t mb;

modbus_config_t cfg = {
    .uart_port           = UART_NUM_1,
    .tx_pin              = 17,
    .rx_pin              = 16,
    .rts_pin             = 4,
    .baud_rate           = 9600,
    .mode                = MODBUS_MODE_MASTER,
    .response_timeout_ms = 500,
};
modbus_init(&mb, &cfg);

// read 4 holding registers from slave 1
uint16_t regs[4];
modbus_err_t err = modbus_read_holding_regs(&mb, 1, 0, 4, regs);

// write to single register
modbus_write_single_reg(&mb, 1, 10, 1234);
```

### slave

```c
modbus_config_t cfg = {
    .uart_port  = UART_NUM_1,
    .tx_pin     = 17,
    .rx_pin     = 16,
    .rts_pin    = 4,
    .baud_rate  = 9600,
    .mode       = MODBUS_MODE_SLAVE,
    .slave_addr = 1,
};
modbus_init(&mb, &cfg);

// populate data model
mb.data.holding_regs[0] = 0x1234;
mb.data.input_regs[0]   = adc_read();

// optional write callback
mb.on_write = my_write_handler;

// poll in a dedicated task
while (1) modbus_slave_poll(&mb);
```

## API

### init

| Function | Description |
|---|---|
| `modbus_init(ctx, cfg)` | Initialize UART and internal state |
| `modbus_deinit(ctx)` | Release UART driver |

### master

| Function | FC |
|---|---|
| `modbus_read_coils` | 0x01 |
| `modbus_read_discrete_inputs` | 0x02 |
| `modbus_read_holding_regs` | 0x03 |
| `modbus_read_input_regs` | 0x04 |
| `modbus_write_single_coil` | 0x05 |
| `modbus_write_single_reg` | 0x06 |
| `modbus_write_multiple_coils` | 0x0F |
| `modbus_write_multiple_regs` | 0x10 |

### slave

| Function | Description |
|---|---|
| `modbus_slave_poll(ctx)` | Block until request received, process, respond |

### Error codes

```c
MODBUS_OK
MODBUS_ERR_TIMEOUT
MODBUS_ERR_CRC
MODBUS_ERR_INVALID_ADDR
MODBUS_ERR_INVALID_FC
MODBUS_ERR_INVALID_DATA
MODBUS_ERR_EXCEPTION
MODBUS_ERR_BUFFER_OVERFLOW
MODBUS_ERR_UART
```

Use `modbus_strerror(err)` for human-readable strings.

## Tested on

- ESP32-WROOM-32 (ESP-IDF v5.2)
- MAX485 transceiver
- Modpoll (Linux master tool) as external master
- QModMaster (Windows) as external master/slave simulator

## License

MIT
