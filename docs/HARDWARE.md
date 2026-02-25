# Hardware Reference

## Pin Assignments

| Signal | GPIO |
|---|---|
| UART TX → RoboClaw | 8 |
| UART RX ← RoboClaw | 9 |
| I2C SDA (AS5600) | 10 |
| I2C SCL (AS5600) | 11 |
| Status LED | 25 |

## Communication Interfaces

| Interface | Device | Speed |
|---|---|---|
| SPI | WIZnet W5500 | — |
| UART | RoboClaw | 38400 baud, 8N1 |
| I2C | AS5600 | 400kHz |

## RoboClaw Setup

RoboClaw must be configured for **Simple Serial mode** before use.  
Baud rate: **38400**.  
This is set via the RoboClaw's onboard configuration — not in firmware.

## Notes

- Pull-up resistors on SDA/SCL are enabled in firmware via `gpio_pull_up()`
- W5500 SPI pins are handled by the WIZnet ioLibrary — see `w5x00_spi.h`
- System clock is overclocked to 133MHz at boot
