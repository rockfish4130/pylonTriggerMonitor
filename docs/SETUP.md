# Setup

## Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- USB-C data cable (not charge-only)
- WEMOS/LOLIN S2 Pico board

## Credential File

Before building, create `src/wifi_credentials.h`. This file is gitignored and must be created locally. See [WIFI.md](WIFI.md) for the template and explanation.

## Build

```bash
pio run
```

## Upload

```bash
pio run -t upload
```

Upload port is set to `COM16` in `platformio.ini`. Change this to match your system.

If the upload fails, the board may need to be manually put into bootloader mode:

1. Hold the `0`/BOOT button
2. Tap `RST`
3. Release `0`
4. Run the upload command

After flashing, tap `RST` once to reboot into firmware. The USB CDC port re-enumerates on reboot — if the port disappears, wait a few seconds and try again.

## Serial Monitor

```bash
pio device monitor
```

Baud rate: `115200` (set in `platformio.ini`).

USB CDC is used (no external UART chip). Requires `ARDUINO_USB_CDC_ON_BOOT=1` and `ARDUINO_USB_MODE=0` build flags, which are already set in `platformio.ini`.

## Dependencies

Declared in `platformio.ini`. PlatformIO installs automatically:

| Library | Purpose |
|---------|---------|
| `Adafruit GFX Library` | OLED graphics primitives |
| `Adafruit SSD1306` | OLED driver |
| `ESP32Ping` | ICMP ping to RPIBOOSH |
| `CNMAT/OSC` | OSC packet parsing |
| `DNSServer` | Captive portal DNS (bundled with ESP32 Arduino core) |

## Dev-Board Button

On the WEMOS S2 Pico, the `0`/BOOT button doubles as a local boosh trigger:

- Press → boosh ON
- Release → boosh OFF

This is equivalent to sending an OSC `/pylon/BooshMain` message.
