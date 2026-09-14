#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>

// Configuration defines
#define MODBUS_MAX_FRAME_SIZE 256
#define MODBUS_COILS_SIZE 1024
#define MODBUS_HOLDING_REGS_SIZE 128
#define MODBUS_INPUT_REGS_SIZE 128
#define MODBUS_DISCRETE_INPUTS_SIZE 128

// Function codes
#define FC_READ_COILS 0x01
#define FC_READ_DISCRETE_INPUTS 0x02
#define FC_READ_HOLDING_REGISTERS 0x03
#define FC_READ_INPUT_REGISTERS 0x04
#define FC_WRITE_SINGLE_COIL 0x05
#define FC_WRITE_SINGLE_REGISTER 0x06
#define FC_WRITE_MULTIPLE_COILS 0x0F
#define FC_WRITE_MULTIPLE_REGISTERS 0x10

// Error codes
#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_DATA_ADDRESS 0x02
#define MB_EX_DATA_VALUE 0x03
#define MB_EX_SLAVE_DEVICE_FAILURE 0x04

// Modbus exception responses
#define MODBUS_EXCEPTION_FLAG 0x80

// Data structures
typedef struct {
    uint8_t address;
    uint8_t function_code;
    uint8_t *data;
    uint16_t length;
} modbus_frame_t;

// Function pointer type for write callbacks
typedef void (*write_callback_t)(uint16_t addr, uint16_t value);

// Modbus context structure
typedef struct {
    // Data model arrays
    uint8_t coils[MODBUS_COILS_SIZE / 8];
    uint16_t holding_registers[MODBUS_HOLDING_REGS_SIZE];
    uint16_t input_registers[MODBUS_INPUT_REGS_SIZE];
    uint8_t discrete_inputs[MODBUS_DISCRETE_INPUTS_SIZE / 8];
    
    // Callbacks
    write_callback_t coil_write_callback;
    write_callback_t register_write_callback;
    
    // UART configuration
    int uart_num;
    int tx_pin;
    int rx_pin;
    int rts_pin;  // For RS485 direction control
} modbus_context_t;

// API Functions
int modbus_init(modbus_context_t *ctx, int uart_num, int tx_pin, int rx_pin, int rts_pin);
int modbus_slave_poll(modbus_context_t *ctx);
int modbus_master_read_coils(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_coils, uint8_t *data);
int modbus_master_read_discrete_inputs(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_inputs, uint8_t *data);
int modbus_master_read_holding_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *data);
int modbus_master_read_input_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *data);
int modbus_master_write_single_coil(modbus_context_t *ctx, uint8_t slave_addr, uint16_t coil_addr, bool value);
int modbus_master_write_single_register(modbus_context_t *ctx, uint8_t slave_addr, uint16_t reg_addr, uint16_t value);
int modbus_master_write_multiple_coils(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_coils, uint8_t *values);
int modbus_master_write_multiple_registers(modbus_context_t *ctx, uint8_t slave_addr, uint16_t start_addr, uint16_t num_regs, uint16_t *values);

// Helper functions
bool modbus_get_coil(modbus_context_t *ctx, uint16_t addr);
void modbus_set_coil(modbus_context_t *ctx, uint16_t addr, bool value);
uint16_t modbus_get_holding_register(modbus_context_t *ctx, uint16_t addr);
void modbus_set_holding_register(modbus_context_t *ctx, uint16_t addr, uint16_t value);
uint16_t modbus_get_input_register(modbus_context_t *ctx, uint16_t addr);
void modbus_set_input_register(modbus_context_t *ctx, uint16_t addr, uint16_t value);
uint8_t modbus_get_discrete_input(modbus_context_t *ctx, uint16_t addr);
void modbus_set_discrete_input(modbus_context_t *ctx, uint16_t addr, bool value);

#endif // MODBUS_RTU_H