#include "modbus_rtu.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "modbus_rtu";

// CRC16 lookup table
static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// Calculate CRC16
static uint16_t calculate_crc(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t table_index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crc_table[table_index];
    }
    return crc;
}

// Set RS485 direction pin for transmission
static void set_rs485_tx(int rts_pin) {
    if (rts_pin >= 0) {
        gpio_set_level(rts_pin, 1);  // High for transmit
    }
}

// Set RS485 direction pin for reception
static void set_rs485_rx(int rts_pin) {
    if (rts_pin >= 0) {
        gpio_set_level(rts_pin, 0);  // Low for receive
    }
}

// Initialize Modbus context
int modbus_init(modbus_context_t *ctx, int uart_num, int tx_pin, int rx_pin, int rts_pin) {
    if (!ctx) return -1;
    
    // Clear context
    memset(ctx, 0, sizeof(modbus_context_t));
    
    // Store configuration
    ctx->uart_num = uart_num;
    ctx->tx_pin = tx_pin;
    ctx->rx_pin = rx_pin;
    ctx->rts_pin = rts_pin;
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num, MODBUS_MAX_FRAME_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, rts_pin, UART_PIN_NO_CHANGE));
    
    // Configure RTS pin for RS485 direction control
    if (rts_pin >= 0) {
        gpio_reset_pin(rts_pin);
        gpio_set_direction(rts_pin, GPIO_MODE_OUTPUT);
        set_rs485_rx(rts_pin);  // Start in receive mode
    }
    
    return 0;
}

// Get coil value from coils array
bool modbus_get_coil(modbus_context_t *ctx, uint16_t addr) {
    if (addr >= MODBUS_COILS_SIZE) return false;
    
    uint16_t byte_idx = addr / 8;
    uint8_t bit_idx = addr % 8;
    
    return (ctx->coils[byte_idx] >> bit_idx) & 0x01;
}

// Set coil value in coils array
void modbus_set_coil(modbus_context_t *ctx, uint16_t addr, bool value) {
    if (addr >= MODBUS_COILS_SIZE) return;
    
    uint16_t byte_idx = addr / 8;
    uint8_t bit_idx = addr % 8;
    
    if (value) {
        ctx->coils[byte_idx] |= (1 << bit_idx);
    } else {
        ctx->coils[byte_idx] &= ~(1 << bit_idx);
    }
}

// Get holding register value
uint16_t modbus_get_holding_register(modbus_context_t *ctx, uint16_t addr) {
    if (addr >= MODBUS_HOLDING_REGS_SIZE) return 0;
    return ctx->holding_registers[addr];
}

// Set holding register value
void modbus_set_holding_register(modbus_context_t *ctx, uint16_t addr, uint16_t value) {
    if (addr >= MODBUS_HOLDING_REGS_SIZE) return;
    ctx->holding_registers[addr] = value;
}

// Get input register value
uint16_t modbus_get_input_register(modbus_context_t *ctx, uint16_t addr) {
    if (addr >= MODBUS_INPUT_REGS_SIZE) return 0;
    return ctx->input_registers[addr];
}

// Set input register value
void modbus_set_input_register(modbus_context_t *ctx, uint16_t addr, uint16_t value) {
    if (addr >= MODBUS_INPUT_REGS_SIZE) return;
    ctx->input_registers[addr] = value;
}

// Get discrete input value
uint8_t modbus_get_discrete_input(modbus_context_t *ctx, uint16_t addr) {
    if (addr >= MODBUS_DISCRETE_INPUTS_SIZE) return 0;
    
    uint16_t byte_idx = addr / 8;
    uint8_t bit_idx = addr % 8;
    
    return (ctx->discrete_inputs[byte_idx] >> bit_idx) & 0x01;
}

// Set discrete input value
void modbus_set_discrete_input(modbus_context_t *ctx, uint16_t addr, bool value) {
    if (addr >= MODBUS_DISCRETE_INPUTS_SIZE) return;
    
    uint16_t byte_idx = addr / 8;
    uint8_t bit_idx = addr % 8;
    
    if (value) {
        ctx->discrete_inputs[byte_idx] |= (1 << bit_idx);
    } else {
        ctx->discrete_inputs[byte_idx] &= ~(1 << bit_idx);
    }
}

// Validate address range for coils
static bool validate_coil_range(uint16_t start_addr, uint16_t count) {
    return (start_addr + count <= MODBUS_COILS_SIZE) && (count > 0) && (count <= 2000);
}

// Validate address range for discrete inputs
static bool validate_discrete_input_range(uint16_t start_addr, uint16_t count) {
    return (start_addr + count <= MODBUS_DISCRETE_INPUTS_SIZE) && (count > 0) && (count <= 2000);
}

// Validate address range for registers
static bool validate_register_range(uint16_t start_addr, uint16_t count) {
    return (start_addr + count <= MODBUS_HOLDING_REGS_SIZE) && (count > 0) && (count <= 125);
}

// Process read coils request (FC 01)
static int process_read_coils(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_coils = (request[4] << 8) | request[5];
    
    if (!validate_coil_range(start_addr, num_coils)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = (num_coils + 7) / 8;  // Byte count
    
    uint16_t byte_idx = 3;
    for (uint16_t i = 0; i < num_coils; i++) {
        if (i % 8 == 0 && i > 0) {
            byte_idx++;
        }
        bool coil_val = modbus_get_coil(ctx, start_addr + i);
        if (coil_val) {
            response[byte_idx] |= (1 << (i % 8));
        } else {
            response[byte_idx] &= ~(1 << (i % 8));
        }
    }
    
    *resp_len = 3 + response[2];  // Fixed header + byte count
    return 0;
}

// Process read discrete inputs request (FC 02)
static int process_read_discrete_inputs(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_inputs = (request[4] << 8) | request[5];
    
    if (!validate_discrete_input_range(start_addr, num_inputs)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = (num_inputs + 7) / 8;  // Byte count
    
    uint16_t byte_idx = 3;
    for (uint16_t i = 0; i < num_inputs; i++) {
        if (i % 8 == 0 && i > 0) {
            byte_idx++;
        }
        bool input_val = modbus_get_discrete_input(ctx, start_addr + i);
        if (input_val) {
            response[byte_idx] |= (1 << (i % 8));
        } else {
            response[byte_idx] &= ~(1 << (i % 8));
        }
    }
    
    *resp_len = 3 + response[2];  // Fixed header + byte count
    return 0;
}

// Process read holding registers request (FC 03)
static int process_read_holding_registers(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_regs = (request[4] << 8) | request[5];
    
    if (!validate_register_range(start_addr, num_regs)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = num_regs * 2;  // Byte count
    
    uint16_t byte_idx = 3;
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t reg_val = modbus_get_holding_register(ctx, start_addr + i);
        response[byte_idx++] = (reg_val >> 8) & 0xFF;  // MSB first
        response[byte_idx++] = reg_val & 0xFF;         // LSB
    }
    
    *resp_len = 3 + response[2];  // Fixed header + byte count
    return 0;
}

// Process read input registers request (FC 04)
static int process_read_input_registers(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_regs = (request[4] << 8) | request[5];
    
    if (!validate_register_range(start_addr, num_regs)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = num_regs * 2;  // Byte count
    
    uint16_t byte_idx = 3;
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t reg_val = modbus_get_input_register(ctx, start_addr + i);
        response[byte_idx++] = (reg_val >> 8) & 0xFF;  // MSB first
        response[byte_idx++] = reg_val & 0xFF;         // LSB
    }
    
    *resp_len = 3 + response[2];  // Fixed header + byte count
    return 0;
}

// Process write single coil request (FC 05)
static int process_write_single_coil(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t coil_addr = (request[2] << 8) | request[3];
    uint16_t value = (request[4] << 8) | request[5];
    
    if (coil_addr >= MODBUS_COILS_SIZE) {
        return MB_EX_DATA_ADDRESS;
    }
    
    bool coil_value = false;
    if (value == 0xFF00) {
        coil_value = true;
    } else if (value == 0x0000) {
        coil_value = false;
    } else {
        return MB_EX_DATA_VALUE;
    }
    
    modbus_set_coil(ctx, coil_addr, coil_value);
    
    // Call write callback if registered
    if (ctx->coil_write_callback) {
        ctx->coil_write_callback(coil_addr, value);
    }
    
    // Echo the request in the response
    memcpy(response, request, 6);
    *resp_len = 6;
    return 0;
}

// Process write single register request (FC 06)
static int process_write_single_register(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len != 6) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t reg_addr = (request[2] << 8) | request[3];
    uint16_t value = (request[4] << 8) | request[5];
    
    if (reg_addr >= MODBUS_HOLDING_REGS_SIZE) {
        return MB_EX_DATA_ADDRESS;
    }
    
    modbus_set_holding_register(ctx, reg_addr, value);
    
    // Call write callback if registered
    if (ctx->register_write_callback) {
        ctx->register_write_callback(reg_addr, value);
    }
    
    // Echo the request in the response
    memcpy(response, request, 6);
    *resp_len = 6;
    return 0;
}

// Process write multiple coils request (FC 0F)
static int process_write_multiple_coils(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len < 7) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_coils = (request[4] << 8) | request[5];
    uint8_t byte_count = request[6];
    
    if (req_len != 7 + byte_count) return MB_EX_ILLEGAL_FUNCTION;
    
    if (!validate_coil_range(start_addr, num_coils)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    if (byte_count != (num_coils + 7) / 8) {
        return MB_EX_DATA_VALUE;
    }
    
    // Write the coils
    for (uint16_t i = 0; i < num_coils; i++) {
        uint16_t byte_idx = 7 + (i / 8);
        uint8_t bit_idx = i % 8;
        bool coil_val = (request[byte_idx] >> bit_idx) & 0x01;
        modbus_set_coil(ctx, start_addr + i, coil_val);
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = request[2];  // Start address high
    response[3] = request[3];  // Start address low
    response[4] = request[4];  // Quantity high
    response[5] = request[5];  // Quantity low
    
    *resp_len = 6;
    return 0;
}

// Process write multiple registers request (FC 10)
static int process_write_multiple_registers(modbus_context_t *ctx, uint8_t *request, uint16_t req_len, uint8_t *response, uint16_t *resp_len) {
    if (req_len < 7) return MB_EX_ILLEGAL_FUNCTION;
    
    uint16_t start_addr = (request[2] << 8) | request[3];
    uint16_t num_regs = (request[4] << 8) | request[5];
    uint8_t byte_count = request[6];
    
    if (req_len != 7 + byte_count) return MB_EX_ILLEGAL_FUNCTION;
    
    if (!validate_register_range(start_addr, num_regs)) {
        return MB_EX_DATA_ADDRESS;
    }
    
    if (byte_count != num_regs * 2) {
        return MB_EX_DATA_VALUE;
    }
    
    // Write the registers
    for (uint16_t i = 0; i < num_regs; i++) {
        uint16_t byte_idx = 7 + (i * 2);
        uint16_t reg_val = (request[byte_idx] << 8) | request[byte_idx + 1];
        modbus_set_holding_register(ctx, start_addr + i, reg_val);
        
        // Call write callback if registered
        if (ctx->register_write_callback) {
            ctx->register_write_callback(start_addr + i, reg_val);
        }
    }
    
    // Build response
    response[0] = request[0];  // Slave address
    response[1] = request[1];  // Function code
    response[2] = request[2];  // Start address high
    response[3] = request[3];  // Start address low
    response[4] = request[4];  // Quantity high
    response[5] = request[5];  // Quantity low
    
    *resp_len = 6;
    return 0;
}

// Handle broadcast messages (no response)
static void handle_broadcast_message(modbus_context_t *ctx, uint8_t *frame, uint16_t frame_len) {
    // Only process broadcast messages for write functions
    uint8_t func_code = frame[1];
    if (func_code == FC_WRITE_SINGLE_COIL || 
        func_code == FC_WRITE_SINGLE_REGISTER ||
        func_code == FC_WRITE_MULTIPLE_COILS ||
        func_code == FC_WRITE_MULTIPLE_REGISTERS) {
        
        // Process the write command but don't send a response
        switch (func_code) {
            case FC_WRITE_SINGLE_COIL:
                process_write_single_coil(ctx, frame, frame_len, NULL, NULL);
                break;
            case FC_WRITE_SINGLE_REGISTER:
                process_write_single_register(ctx, frame, frame_len, NULL, NULL);
                break;
            case FC_WRITE_MULTIPLE_COILS:
                process_write_multiple_coils(ctx, frame, frame_len, NULL, NULL);
                break;
            case FC_WRITE_MULTIPLE_REGISTERS:
                process_write_multiple_registers(ctx, frame, frame_len, NULL, NULL);
                break;
        }
    }
}

// Main slave poll function
int modbus_slave_poll(modbus_context_t *ctx) {
    if (!ctx) return -1;
    
    // Read available data from UART
    uint8_t buffer[MODBUS_MAX_FRAME_SIZE];
    int len = uart_read_bytes(ctx->uart_num, buffer, sizeof(buffer), 0);
    
    if (len < 4) return 0;  // Minimum frame size is 4 bytes (address + FC + CRC)
    
    // Validate CRC
    uint16_t received_crc = (buffer[len - 1] << 8) | buffer[len - 2];
    uint16_t calculated_crc = calculate_crc(buffer, len - 2);
    
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "CRC mismatch: received=0x%04X, calculated=0x%04X", received_crc, calculated_crc);
        return 0;
    }
    
    uint8_t slave_addr = buffer[0];
    uint8_t func_code = buffer[1];
    
    // Check if this message is addressed to us (or broadcast)
    if (slave_addr != ctx->address && slave_addr != 0x00) {
        return 0;  // Not addressed to us
    }
    
    // Handle broadcast message (slave_addr == 0x00)
    if (slave_addr == 0x00) {
        handle_broadcast_message(ctx, buffer, len);
        return 0;
    }
    
    // Process the request based on function code
    uint8_t response[MODBUS_MAX_FRAME_SIZE];
    uint16_t resp_len = 0;
    int result = 0;
    
    switch (func_code) {
        case FC_READ_COILS:
            result = process_read_coils(ctx, buffer, len, response, &resp_len);
            break;
        case FC_READ_DISCRETE_INPUTS:
            result = process_read_discrete_inputs(ctx, buffer, len, response, &resp_len);
            break;
        case FC_READ_HOLDING_REGISTERS:
            result = process_read_holding_registers(ctx, buffer, len, response, &resp_len);
            break;
        case FC_READ_INPUT_REGISTERS:
            result = process_read_input_registers(ctx, buffer, len, response, &resp_len);
            break;
        case FC_WRITE_SINGLE_COIL:
            result = process_write_single_coil(ctx, buffer, len, response, &resp_len);
            break;
        case FC_WRITE_SINGLE_REGISTER:
            result = process_write_single_register(ctx, buffer, len, response, &resp_len);
            break;
        case FC_WRITE_MULTIPLE_COILS:
            result = process_write_multiple_coils(ctx, buffer, len, response, &resp_len);
            break;
        case FC_WRITE_MULTIPLE_REGISTERS:
            result = process_write_multiple_registers(ctx, buffer, len, response, &resp_len);
            break;
        default:
            result = MB_EX_ILLEGAL_FUNCTION;
            break;
    }
    
    // If there was an error, construct an exception response
    if (result != 0) {
        response[0] = slave_addr;
        response[1] = func_code | MODBUS_EXCEPTION_FLAG;
        response[2] = result;
        resp_len = 3;
    }
    
    // Add CRC to response
    uint16_t response_crc = calculate_crc(response, resp_len);
    response[resp_len++] = response_crc & 0xFF;
    response[resp_len++] = (response_crc >> 8) & 0xFF;
    
    // Send response
    set_rs485_tx(ctx->rts_pin);
    uart_write_bytes(ctx->uart_num, (const char *)response, resp_len);
    uart_wait_tx_done(ctx->uart_num, 1000 / portTICK_PERIOD_MS);
    set_rs485_rx(ctx->rts_pin);
    
    return resp_len;
}

// These are simplified stubs for now
int modbus_master_read_coils(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_coils, uint8_t *data) {
    // Implementation for master reading coils from slave
    return 0;
}

int modbus_master_read_discrete_inputs(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_inputs, uint8_t *data) {
    // Implementation for master reading discrete inputs from slave
    return 0;
}

int modbus_master_read_holding_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *data) {
    // Implementation for master reading holding registers from slave
    return 0;
}

int modbus_master_read_input_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *data) {
    // Implementation for master reading input registers from slave
    return 0;
}

int modbus_master_write_single_coil(modbus_context_t *ctx, uint8_t slave_addr, uint16_t coil_addr, bool value) {
    // Implementation for master writing single coil to slave
    return 0;
}

int modbus_master_write_single_register(modbus_context_t *ctx, uint8_t slave_addr, uint16_t reg_addr, uint16_t value) {
    // Implementation for master writing single register to slave
    return 0;
}

int modbus_master_write_multiple_coils(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_coils, uint8_t *values) {
    // Implementation for master writing multiple coils to slave
    return 0;
}

int modbus_master_write_multiple_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *values) {
    // Implementation for master writing multiple registers to slave
    return 0;
}