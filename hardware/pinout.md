# Pinout notes (prototype)

> Verify against your board revision and `sketch.ino` defines before wiring.

| Function | Typical pin (prototype) | Notes |
|----------|-------------------------|--------|
| Relay Critical | D9 | Highest priority load |
| Relay High | D8 | |
| Relay Low | D7 | |
| Relay Auxiliary | D6 | First shed candidate |
| E-STOP (NC) | D5 | Forces all relays OFF |
| RS-485 DI (soft UART TX) | D2 | TX-only status |
| RS-485 DE | D4 | Driver enable |
| PZEM UART | D0/D1 (or as configured) | Primary meter |
| HMI | I²C | Local display |

Power: keep 5 V relay coil supply and logic grounds consistent with module requirements. Use appropriate isolation and fusing for any AC load side.
