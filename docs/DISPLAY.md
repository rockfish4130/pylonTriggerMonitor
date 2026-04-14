# Display

## Hardware

- Controller: SSD1306
- Resolution: 128×32
- I2C address: `0x3C`
- SDA: GPIO8, SCL: GPIO9, Reset: GPIO18

## Initialization

```cpp
Wire.begin(8, 9);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
```

Reset pin is set to GPIO18 before `begin()`.

## Boot Messages

During setup the OLED shows progress messages: OLED ready, Wi-Fi scan, connecting, IP address on success, or failure.

## Runtime Pages

Pages cycle every ~3 seconds (`kDisplayCycleMs`). The pylon rotates through 5 pages:

### Page 1 — Ping

```
RPIBOOSH
OK  last:7ms
min:2 max:17
since: 00:05:23
```

- Status: `OK`, `FAIL`, or `TIMEOUT` (no successful ping within `kPingTimeoutMs`)
- Shows last/min/max RTT and elapsed time since last successful ping

### Page 2 — Wi-Fi A

```
SSID: MyNetwork
RSSI: -58 dBm
IP: 192.168.1.70
UP: 02:32:03
```

### Page 3 — Wi-Fi B

```
RSN: 0 (none)
SSID: MyNetwork
RSSI: -58
IP: 192.168.1.70
```

`RSN` shows the last Wi-Fi disconnect reason code and a short label.

### Page 4 — Node

```
PYLON5
Triggers: 12
v0.0.1
```

### Page 5 — Firmware

```
v0.0.1
Apr 13 2026
12:00:00
```

## Boosh Indicator

When boosh is active (solenoid ON), the entire display inverts (white-on-black → black-on-white). This is the primary visual confirmation of a fire trigger.

Trigger sources that cause inversion: OSC, HTTP API, web UI, dev-board button.
