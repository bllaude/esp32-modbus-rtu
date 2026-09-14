# Modbus RTU Hardware Wiring Guide
This document provides instructions for connecting your ESP32 to an RS485 transceiver module for Modbus RTU communication.

## Required Components
- ESP32 development board
- `MAX485` or similar `RS485` transceiver module
- Power supply (typically `5V` or `3.3V` depending on module)
- Twisted pair cable for Modbus network

### Pin Connections
| RS485 Module Pin | RS485 Module Pin | Description |
|---|---|---|
| GPIO16 | RO (Receive) | UART RX - Receive data from bus |
| GPIO17 | DI (Driver) | UART TX - Transmit data to bus |
| GPIO5 | DE/RE (Enable) | Direction control (high=transmit, low=receive) |
| GND | GND | Ground common reference |
| 3.3V/5V | VCC | Power supply |

## Network Topology
- Use twisted pair cable (shielded recommended)
- Terminate the bus with `120Ω` resistors at both ends
- Keep stub lengths short (less than 1m if possible)
- Maximum cable length: `~1200m` at `9600` baud

### Daisy Chain Connection
For multiple devices on the same bus:
- Connect `A` line to `A` on all devices
- Connect `B` line to `B` on all devices
- Ensure common ground between all devices
- Each device needs its own unique Modbus address

### Notes
- The DE/RE pins on `MAX485` modules are often tied together since we want transmit/receive enable to be synchronized
- Use proper termination resistors for longer runs
- Shield should be grounded at only one end to avoid ground loops
- For short distances, you may be able to omit termination resistors