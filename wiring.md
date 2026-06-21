# Wiring

## RS485 Transceiver (MAX485 / SP3485)

```
ESP32               MAX485              RS485 Bus
────────            ──────              ─────────
GPIO17 (TX) ──────► DI
GPIO16 (RX) ◄────── RO
GPIO4       ──────► DE ─┐
                    RE ◄─┘ (tie DE and RE together)
                         A ────────────── A
                         B ────────────── B
GND ─────────────── GND
3.3V ────────────── VCC  (MAX485 runs on 3.3V or 5V; check datasheet)
```

**Note:** Add 120Ω termination resistors at both ends of the RS485 bus for runs over ~1m.

## Direct UART (no transceiver, short range only)

Set `rts_pin = -1` in config. Cross TX/RX between two ESP32 boards:

```
Board A GPIO17 (TX) ──── Board B GPIO16 (RX)
Board A GPIO16 (RX) ──── Board B GPIO17 (TX)
Board A GND         ──── Board B GND
```

## Baud Rate vs Cable Length

| Baud    | Max Recommended Length |
|---------|------------------------|
| 9600    | 1200m                  |
| 19200   | 600m                   |
| 38400   | 300m                   |
| 115200  | 100m                   |

Values are approximate and depend on cable quality and number of nodes.
