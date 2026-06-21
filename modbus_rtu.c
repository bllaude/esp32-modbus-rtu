#include "modbus_rtu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "modbus_rtu";

// ─── crc16 ───────────────────────────────────────────────────────────────────

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// ─── uart helpers ─────────────────────────────────────────────────────────────

static void rs485_tx_mode(modbus_ctx_t *ctx)
{
    if (ctx->cfg.rts_pin >= 0)
        gpio_set_level(ctx->cfg.rts_pin, 1);
}

static void rs485_rx_mode(modbus_ctx_t *ctx)
{
    if (ctx->cfg.rts_pin >= 0)
        gpio_set_level(ctx->cfg.rts_pin, 0);
}

static void uart_flush(modbus_ctx_t *ctx)
{
    uart_flush_input(ctx->cfg.uart_port);
}

static esp_err_t uart_send(modbus_ctx_t *ctx, const uint8_t *data, size_t len)
{
    rs485_tx_mode(ctx);
    int written = uart_write_bytes(ctx->cfg.uart_port, (const char *)data, len);
    uart_wait_tx_done(ctx->cfg.uart_port, pdMS_TO_TICKS(100));
    rs485_rx_mode(ctx);
    return (written == (int)len) ? ESP_OK : ESP_FAIL;
}

// read exactly `len` bytes within timeout, returns bytes read.
static int uart_recv(modbus_ctx_t *ctx, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    int total = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (total < (int)len) {
        TickType_t now = xTaskGetTickCount();
        if (now >= deadline) break;
        int remaining_ms = pdTICKS_TO_MS(deadline - now);
        int n = uart_read_bytes(ctx->cfg.uart_port,
                                buf + total,
                                len - total,
                                pdMS_TO_TICKS(remaining_ms));
        if (n <= 0) break;
        total += n;
    }
    return total;
}

// ─── init / deinit ───────────────────────────────────────────────────────────

modbus_err_t modbus_init(modbus_ctx_t *ctx, const modbus_config_t *cfg)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = *cfg;

    uart_config_t uart_cfg = {
        .baud_rate  = cfg->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(cfg->uart_port, &uart_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        return MODBUS_ERR_UART;
    }
    if (uart_set_pin(cfg->uart_port, cfg->tx_pin, cfg->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return MODBUS_ERR_UART;
    }
    if (uart_driver_install(cfg->uart_port, MODBUS_MAX_ADU_SIZE * 2,
                            MODBUS_MAX_ADU_SIZE * 2, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return MODBUS_ERR_UART;
    }

    if (cfg->rts_pin >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->rts_pin),
            .mode         = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        rs485_rx_mode(ctx);
    }

    ESP_LOGI(TAG, "init: port=%d baud=%lu mode=%s addr=%d",
             cfg->uart_port, cfg->baud_rate,
             cfg->mode == MODBUS_MODE_MASTER ? "MASTER" : "SLAVE",
             cfg->slave_addr);

    return MODBUS_OK;
}

void modbus_deinit(modbus_ctx_t *ctx)
{
    uart_driver_delete(ctx->cfg.uart_port);
}

// ─── frame build/parse helpers ───────────────────────────────────────────────

static uint16_t build_request(uint8_t *buf, uint8_t slave, uint8_t fc,
                               uint16_t addr, uint16_t count)
{
    buf[0] = slave;
    buf[1] = fc;
    buf[2] = addr >> 8;
    buf[3] = addr & 0xFF;
    buf[4] = count >> 8;
    buf[5] = count & 0xFF;
    uint16_t crc = modbus_crc16(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = crc >> 8;
    return 8;
}

static modbus_err_t check_response_crc(const uint8_t *buf, uint16_t len)
{
    if (len < 4) return MODBUS_ERR_CRC;
    uint16_t calc = modbus_crc16(buf, len - 2);
    uint16_t recv = buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    return (calc == recv) ? MODBUS_OK : MODBUS_ERR_CRC;
}

// ─── master: generic read ─────────────────────────────────────────────────────

static modbus_err_t master_read(modbus_ctx_t *ctx, uint8_t slave, uint8_t fc,
                                 uint16_t start, uint16_t count, uint8_t *raw_out)
{
    uart_flush(ctx);
    uint16_t req_len = build_request(ctx->_tx_buf, slave, fc, start, count);
    if (uart_send(ctx, ctx->_tx_buf, req_len) != ESP_OK)
        return MODBUS_ERR_UART;

    // expected response size: [slave][fc][byte_count][data...][crc_lo][crc_hi]
    uint8_t *buf = ctx->_rx_buf;
    int header = uart_recv(ctx, buf, 3, ctx->cfg.response_timeout_ms);
    if (header < 3) return MODBUS_ERR_TIMEOUT;
    if (buf[0] != slave) return MODBUS_ERR_INVALID_ADDR;
    if (buf[1] & MODBUS_EXCEPTION_FLAG) {
        uart_recv(ctx, buf + 3, 2, 50);
        ESP_LOGW(TAG, "exception 0x%02X from slave %d fc 0x%02X", buf[2], slave, fc);
        return MODBUS_ERR_EXCEPTION;
    }
    if (buf[1] != fc) return MODBUS_ERR_INVALID_FC;

    uint8_t byte_count = buf[2];
    int data_bytes = uart_recv(ctx, buf + 3, byte_count + 2, 50);
    if (data_bytes < byte_count + 2) return MODBUS_ERR_TIMEOUT;

    modbus_err_t crc_err = check_response_crc(buf, 3 + byte_count + 2);
    if (crc_err != MODBUS_OK) return crc_err;

    memcpy(raw_out, buf + 3, byte_count);
    return MODBUS_OK;
}

// ─── master: generic write (single/multiple) ─────────────────────────────────

static modbus_err_t master_write_single(modbus_ctx_t *ctx, uint8_t slave,
                                         uint8_t fc, uint16_t addr, uint16_t value)
{
    uart_flush(ctx);
    uint8_t *buf = ctx->_tx_buf;
    buf[0] = slave;
    buf[1] = fc;
    buf[2] = addr >> 8;
    buf[3] = addr & 0xFF;
    buf[4] = value >> 8;
    buf[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = crc >> 8;

    if (uart_send(ctx, buf, 8) != ESP_OK) return MODBUS_ERR_UART;

    uint8_t *rbuf = ctx->_rx_buf;
    int n = uart_recv(ctx, rbuf, 8, ctx->cfg.response_timeout_ms);
    if (n < 8) return MODBUS_ERR_TIMEOUT;
    if (rbuf[0] != slave) return MODBUS_ERR_INVALID_ADDR;
    if (rbuf[1] & MODBUS_EXCEPTION_FLAG) return MODBUS_ERR_EXCEPTION;
    return check_response_crc(rbuf, 8);
}

static modbus_err_t master_write_multiple(modbus_ctx_t *ctx, uint8_t slave,
                                           uint8_t fc, uint16_t start, uint16_t count,
                                           const uint8_t *payload, uint8_t payload_len)
{
    uart_flush(ctx);
    uint8_t *buf = ctx->_tx_buf;
    buf[0] = slave;
    buf[1] = fc;
    buf[2] = start >> 8;
    buf[3] = start & 0xFF;
    buf[4] = count >> 8;
    buf[5] = count & 0xFF;
    buf[6] = payload_len;
    memcpy(buf + 7, payload, payload_len);
    uint16_t crc = modbus_crc16(buf, 7 + payload_len);
    buf[7 + payload_len]     = crc & 0xFF;
    buf[7 + payload_len + 1] = crc >> 8;
    uint16_t total = 7 + payload_len + 2;

    if (uart_send(ctx, buf, total) != ESP_OK) return MODBUS_ERR_UART;

    uint8_t *rbuf = ctx->_rx_buf;
    int n = uart_recv(ctx, rbuf, 8, ctx->cfg.response_timeout_ms);
    if (n < 8) return MODBUS_ERR_TIMEOUT;
    if (rbuf[0] != slave) return MODBUS_ERR_INVALID_ADDR;
    if (rbuf[1] & MODBUS_EXCEPTION_FLAG) return MODBUS_ERR_EXCEPTION;
    return check_response_crc(rbuf, 8);
}

// ─── master public api ───────────────────────────────────────────────────────

modbus_err_t modbus_read_coils(modbus_ctx_t *ctx, uint8_t slave,
                                uint16_t start, uint16_t count, uint8_t *out)
{
    uint8_t raw[32] = {0};
    modbus_err_t err = master_read(ctx, slave, MODBUS_FC_READ_COILS, start, count, raw);
    if (err == MODBUS_OK) memcpy(out, raw, (count + 7) / 8);
    return err;
}

modbus_err_t modbus_read_discrete_inputs(modbus_ctx_t *ctx, uint8_t slave,
                                          uint16_t start, uint16_t count, uint8_t *out)
{
    uint8_t raw[32] = {0};
    modbus_err_t err = master_read(ctx, slave, MODBUS_FC_READ_DISCRETE_INPUTS, start, count, raw);
    if (err == MODBUS_OK) memcpy(out, raw, (count + 7) / 8);
    return err;
}

modbus_err_t modbus_read_holding_regs(modbus_ctx_t *ctx, uint8_t slave,
                                       uint16_t start, uint16_t count, uint16_t *out)
{
    uint8_t raw[128] = {0};
    modbus_err_t err = master_read(ctx, slave, MODBUS_FC_READ_HOLDING_REGS, start, count, raw);
    if (err == MODBUS_OK) {
        for (int i = 0; i < count; i++)
            out[i] = ((uint16_t)raw[i * 2] << 8) | raw[i * 2 + 1];
    }
    return err;
}

modbus_err_t modbus_read_input_regs(modbus_ctx_t *ctx, uint8_t slave,
                                     uint16_t start, uint16_t count, uint16_t *out)
{
    uint8_t raw[128] = {0};
    modbus_err_t err = master_read(ctx, slave, MODBUS_FC_READ_INPUT_REGS, start, count, raw);
    if (err == MODBUS_OK) {
        for (int i = 0; i < count; i++)
            out[i] = ((uint16_t)raw[i * 2] << 8) | raw[i * 2 + 1];
    }
    return err;
}

modbus_err_t modbus_write_single_coil(modbus_ctx_t *ctx, uint8_t slave,
                                       uint16_t addr, bool value)
{
    return master_write_single(ctx, slave, MODBUS_FC_WRITE_SINGLE_COIL,
                               addr, value ? 0xFF00 : 0x0000);
}

modbus_err_t modbus_write_single_reg(modbus_ctx_t *ctx, uint8_t slave,
                                      uint16_t addr, uint16_t value)
{
    return master_write_single(ctx, slave, MODBUS_FC_WRITE_SINGLE_REG, addr, value);
}

modbus_err_t modbus_write_multiple_coils(modbus_ctx_t *ctx, uint8_t slave,
                                          uint16_t start, uint16_t count,
                                          const uint8_t *values)
{
    uint8_t payload[32] = {0};
    uint8_t byte_count = (count + 7) / 8;
    for (int i = 0; i < count; i++) {
        if (values[i]) payload[i / 8] |= (1 << (i % 8));
    }
    return master_write_multiple(ctx, slave, MODBUS_FC_WRITE_MULTIPLE_COILS,
                                 start, count, payload, byte_count);
}

modbus_err_t modbus_write_multiple_regs(modbus_ctx_t *ctx, uint8_t slave,
                                         uint16_t start, uint16_t count,
                                         const uint16_t *values)
{
    uint8_t payload[128];
    for (int i = 0; i < count; i++) {
        payload[i * 2]     = values[i] >> 8;
        payload[i * 2 + 1] = values[i] & 0xFF;
    }
    return master_write_multiple(ctx, slave, MODBUS_FC_WRITE_MULTIPLE_REGS,
                                 start, count, payload, count * 2);
}

// ─── slave ───────────────────────────────────────────────────────────────────

static void slave_send_exception(modbus_ctx_t *ctx, uint8_t fc, uint8_t ex_code)
{
    uint8_t *buf = ctx->_tx_buf;
    buf[0] = ctx->cfg.slave_addr;
    buf[1] = fc | MODBUS_EXCEPTION_FLAG;
    buf[2] = ex_code;
    uint16_t crc = modbus_crc16(buf, 3);
    buf[3] = crc & 0xFF;
    buf[4] = crc >> 8;
    uart_send(ctx, buf, 5);
}

static void slave_send(modbus_ctx_t *ctx, uint8_t *frame, uint16_t len)
{
    uint16_t crc = modbus_crc16(frame, len);
    frame[len]     = crc & 0xFF;
    frame[len + 1] = crc >> 8;
    uart_send(ctx, frame, len + 2);
}

modbus_err_t modbus_slave_poll(modbus_ctx_t *ctx)
{
    uint8_t *buf = ctx->_rx_buf;

    // receive header: [addr][fc][hi][lo][...][crc_lo][crc_hi]
    // min frame: 4 bytes data + 2 crc = 8 for most FCs
    int n = uart_recv(ctx, buf, 8, portMAX_DELAY);
    if (n < 8) return MODBUS_ERR_TIMEOUT;

    // drain any remaining bytes (fc0f/fc10 with payload)
    // For write-multiple fcs the frame is longer; read remaining bytes
    uint8_t fc = buf[1];
    if (fc == MODBUS_FC_WRITE_MULTIPLE_COILS || fc == MODBUS_FC_WRITE_MULTIPLE_REGS) {
        int extra = uart_recv(ctx, buf + 8, MODBUS_MAX_ADU_SIZE - 8, 20);
        n += extra;
    }

    // filter by address
    if (buf[0] != ctx->cfg.slave_addr && buf[0] != MODBUS_BROADCAST_ADDR)
        return MODBUS_OK; // silently ignore

    // crc check
    if (check_response_crc(buf, n) != MODBUS_OK)
        return MODBUS_ERR_CRC;

    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];
    modbus_data_model_t *dm = &ctx->data;
    uint8_t *out = ctx->_tx_buf;
    uint16_t out_len;

    switch (fc) {
        case MODBUS_FC_READ_COILS: {
            uint8_t byte_count = (count + 7) / 8;
            out[0] = ctx->cfg.slave_addr;
            out[1] = fc;
            out[2] = byte_count;
            memset(out + 3, 0, byte_count);
            for (int i = 0; i < count; i++) {
                if (dm->coils[addr + i]) out[3 + i / 8] |= (1 << (i % 8));
            }
            slave_send(ctx, out, 3 + byte_count);
            break;
        }
        case MODBUS_FC_READ_DISCRETE_INPUTS: {
            uint8_t byte_count = (count + 7) / 8;
            out[0] = ctx->cfg.slave_addr;
            out[1] = fc;
            out[2] = byte_count;
            memset(out + 3, 0, byte_count);
            for (int i = 0; i < count; i++) {
                if (dm->discrete_inputs[addr + i]) out[3 + i / 8] |= (1 << (i % 8));
            }
            slave_send(ctx, out, 3 + byte_count);
            break;
        }
        case MODBUS_FC_READ_HOLDING_REGS: {
            out[0] = ctx->cfg.slave_addr;
            out[1] = fc;
            out[2] = count * 2;
            for (int i = 0; i < count; i++) {
                out[3 + i * 2]     = dm->holding_regs[addr + i] >> 8;
                out[3 + i * 2 + 1] = dm->holding_regs[addr + i] & 0xFF;
            }
            slave_send(ctx, out, 3 + count * 2);
            break;
        }
        case MODBUS_FC_READ_INPUT_REGS: {
            out[0] = ctx->cfg.slave_addr;
            out[1] = fc;
            out[2] = count * 2;
            for (int i = 0; i < count; i++) {
                out[3 + i * 2]     = dm->input_regs[addr + i] >> 8;
                out[3 + i * 2 + 1] = dm->input_regs[addr + i] & 0xFF;
            }
            slave_send(ctx, out, 3 + count * 2);
            break;
        }
        case MODBUS_FC_WRITE_SINGLE_COIL: {
            uint16_t val = ((uint16_t)buf[4] << 8) | buf[5];
            if (val != 0xFF00 && val != 0x0000) {
                slave_send_exception(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            dm->coils[addr] = (val == 0xFF00) ? 1 : 0;
            if (ctx->on_write) ctx->on_write(ctx, fc, addr, 1);
            memcpy(out, buf, 8); // echo
            uart_send(ctx, out, 8);
            break;
        }
        case MODBUS_FC_WRITE_SINGLE_REG: {
            dm->holding_regs[addr] = ((uint16_t)buf[4] << 8) | buf[5];
            if (ctx->on_write) ctx->on_write(ctx, fc, addr, 1);
            memcpy(out, buf, 8);
            uart_send(ctx, out, 8);
            break;
        }
        case MODBUS_FC_WRITE_MULTIPLE_COILS: {
            uint8_t byte_count = buf[6];
            for (int i = 0; i < count; i++) {
                dm->coils[addr + i] = (buf[7 + i / 8] >> (i % 8)) & 0x01;
            }
            if (ctx->on_write) ctx->on_write(ctx, fc, addr, count);
            out[0] = ctx->cfg.slave_addr; out[1] = fc;
            out[2] = buf[2]; out[3] = buf[3];
            out[4] = buf[4]; out[5] = buf[5];
            slave_send(ctx, out, 6);
            (void)byte_count;
            break;
        }
        case MODBUS_FC_WRITE_MULTIPLE_REGS: {
            for (int i = 0; i < count; i++) {
                dm->holding_regs[addr + i] =
                    ((uint16_t)buf[7 + i * 2] << 8) | buf[7 + i * 2 + 1];
            }
            if (ctx->on_write) ctx->on_write(ctx, fc, addr, count);
            out[0] = ctx->cfg.slave_addr; out[1] = fc;
            out[2] = buf[2]; out[3] = buf[3];
            out[4] = buf[4]; out[5] = buf[5];
            slave_send(ctx, out, 6);
            break;
        }
        default:
            slave_send_exception(ctx, fc, MODBUS_EX_ILLEGAL_FUNCTION);
            break;
    }

    return MODBUS_OK;
}

// ─── utils ───────────────────────────────────────────────────────────────

const char *modbus_strerror(modbus_err_t err)
{
    switch (err) {
        case MODBUS_OK:                  return "OK";
        case MODBUS_ERR_TIMEOUT:         return "timeout";
        case MODBUS_ERR_CRC:             return "CRC mismatch";
        case MODBUS_ERR_INVALID_ADDR:    return "invalid address";
        case MODBUS_ERR_INVALID_FC:      return "invalid function code";
        case MODBUS_ERR_INVALID_DATA:    return "invalid data";
        case MODBUS_ERR_EXCEPTION:       return "modbus exception";
        case MODBUS_ERR_BUFFER_OVERFLOW: return "buffer overflow";
        case MODBUS_ERR_UART:            return "uart error";
        default:                         return "unknown error";
    }
}
