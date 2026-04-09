# Setup

This project uses PlatformIO with the Arduino framework targeting the WEMOS/LOLIN S2 Pico (ESP32-S2).

Official board reference:
- https://www.wemos.cc/en/latest/s2/s2_pico.html

## Install
1. Install PlatformIO (VS Code extension or CLI).
2. Connect the board via USB-C.

## Build
```bash
pio run
```

## Upload
```bash
pio run -t upload
```

## Serial Monitor
```bash
pio device monitor
```

Default baud is `115200` (see `platformio.ini`).

## Dependencies
PlatformIO will auto-install:
- `Adafruit GFX Library`
- `Adafruit SSD1306`

These are declared in `platformio.ini`.

## Example OSC Payloads
Hex payloads for `/rpiboosh/BooshMain` with 3 float args:

Start (ON):
```text
2F 72 70 69 62 6F 6F 73 68 2F 42 6F 6F 73 68 4D 61 69 6E 00 2C 66 66 66 00 00 00 00 3F 80 00 00 00 00 00 00 00 00 00 00
```

Stop (OFF):
```text
2F 72 70 69 62 6F 6F 73 68 2F 42 6F 6F 73 68 4D 61 69 6E 00 2C 66 66 66 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

## Dev-Board Button
On the WEMOS S2 Pico dev board, the `0`/BOOT button is also mapped as a local BooshMain trigger:
- Press: ON (`[1.0, 0.0, 0.0]`)
- Release: OFF (`[0.0, 0.0, 0.0]`)
