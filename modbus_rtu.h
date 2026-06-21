#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─── Constants ───────────────────────────────────────────────────────────────

#define MODBUS_MAX_PDU_SIZE       253
#define MODBUS_MAX_ADU_SIZE       256
#define MODBUS_BROADCAST_ADDR     0x00
#define MODBUS_MAX_SLAVE_ADDR     247

#define MODBUS_FC_READ_COILS            0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS  0x02
#define MODBUS_FC_READ_HOLDING_REGS     0x03
#define MODBUS_FC_READ_INPUT_REGS       0x04
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05
#define MODBUS_FC_WRITE_SINGLE_REG      0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS  0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10

#define MODBUS_EXCEPTION_FLAG           0x80

// Exception codes
#define MODBUS_EX_ILLEGAL_FUNCTION      0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS  0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE    0x03
#define MODBUS_EX_SERVER_DEVICE_FAILURE 0x04

// ─── Types ────────────────────────────────────────────────────────────────────

typedef enum {
    MODBUS_MODE_MASTER,
    MODBUS_MODE_SLAVE,
} modbus_mode_t;

typedef enum {
    MODBUS_OK                   =  0,
    MODBUS_ERR_TIMEOUT          = -1,
    MODBUS_ERR_CRC              = -2,
    MODBUS_ERR_INVALID_ADDR     = -3,
    MODBUS_ERR_INVALID_FC       = -4,
    MODBUS_ERR_INVALID_DATA     = -5,
    MODBUS_ERR_EXCEPTION        = -6,
    MODBUS_ERR_BUFFER_OVERFLOW  = -7,
    MODBUS_ERR_UART             = -8,
} modbus_err_t;

typedef struct {
    uart_port_t     uart_port;
    int             tx_pin;
    int             rx_pin;
    int             rts_pin;        // DE/RE pin for RS485 transceiver; -1 if unused
    uint32_t        baud_rate;
    modbus_mode_t   mode;
    uint8_t         slave_addr;     // ignored in master mode
    uint32_t        response_timeout_ms;
} modbus_config_t;

// Slave data model — flat arrays sized for typical embedded use
typedef struct {
    uint8_t  coils[256];            // FC01/FC05/FC0F  (bit per index)
    uint8_t  discrete_inputs[256];  // FC02            (bit per index, read-only)
    uint16_t holding_regs[256];     // FC03/FC06/FC10
    uint16_t input_regs[256];       // FC04            (read-only)
} modbus_data_model_t;

typedef struct modbus_ctx modbus_ctx_t;

// Callback invoked on the slave after a write completes.
// fc: function code that triggered the write
typedef void (*modbus_write_cb_t)(modbus_ctx_t *ctx, uint8_t fc, uint16_t start_addr, uint16_t count);

struct modbus_ctx {
    modbus_config_t     cfg;
    modbus_data_model_t data;           // slave only
    modbus_write_cb_t   on_write;       // slave only; may be NULL
    uint8_t             _rx_buf[MODBUS_MAX_ADU_SIZE];
    uint8_t             _tx_buf[MODBUS_MAX_ADU_SIZE];
};

// ─── Init / Deinit ───────────────────────────────────────────────────────────

/**
 * Initialize UART and internal state. Must be called before any other API.
 * For slave mode, populate ctx->data before calling modbus_slave_poll().
 */
modbus_err_t modbus_init(modbus_ctx_t *ctx, const modbus_config_t *cfg);
void         modbus_deinit(modbus_ctx_t *ctx);

// ─── Master API ──────────────────────────────────────────────────────────────

modbus_err_t modbus_read_coils(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, uint8_t *out);
modbus_err_t modbus_read_discrete_inputs(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, uint8_t *out);
modbus_err_t modbus_read_holding_regs(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, uint16_t *out);
modbus_err_t modbus_read_input_regs(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, uint16_t *out);
modbus_err_t modbus_write_single_coil(modbus_ctx_t *ctx, uint8_t slave, uint16_t addr, bool value);
modbus_err_t modbus_write_single_reg(modbus_ctx_t *ctx, uint8_t slave, uint16_t addr, uint16_t value);
modbus_err_t modbus_write_multiple_coils(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, const uint8_t *values);
modbus_err_t modbus_write_multiple_regs(modbus_ctx_t *ctx, uint8_t slave, uint16_t start, uint16_t count, const uint16_t *values);

// ─── Slave API ───────────────────────────────────────────────────────────────

/**
 * Block until a valid request arrives, process it, and send a response.
 * Call in a dedicated FreeRTOS task loop.
 */
modbus_err_t modbus_slave_poll(modbus_ctx_t *ctx);

// ─── Utilities ───────────────────────────────────────────────────────────────

uint16_t    modbus_crc16(const uint8_t *buf, uint16_t len);
const char *modbus_strerror(modbus_err_t err);

#ifdef __cplusplus
}
#endif
