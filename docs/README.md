# Pylons

ESP32-S2 (WEMOS/LOLIN S2 Pico) Arduino project with OLED status and prioritized Wi-Fi connection.

Reference board:
- WEMOS S2 Pico: https://www.wemos.cc/en/latest/s2/s2_pico.html

## What It Does
At boot the firmware:
1. Initializes the built-in SSD1306 128x32 OLED on I2C.
2. Scans for the Lava production SSID (`BOOSH_WIFI_SSID_LL`).
3. Connects to Lava production if found, otherwise falls back to home test/development Wi-Fi (`BOOSH_WIFI_SSID_MW`).
4. Shows connection status and IP address on the OLED and serial.

At runtime the firmware:
1. Listens for OSC on UDP port `8000` at `/rpiboosh/BooshMain`.
2. Treats the dev-board `0`/BOOT button (`GPIO0`) as a local BooshMain trigger.
3. Uses OLED inversion as a prototype proxy for the boosher solenoid state.
4. Inverted display means solenoid open / fire ON.
5. Normal display means solenoid closed / fire OFF.
6. Pings `RPIBOOSH` once per second and shows ping status/stats on the OLED.
7. Applies a failsafe: if ON is received and OFF is not seen within 5 seconds, it forces OFF.
8. Cycles OLED pages with Wi-Fi debug metrics and ping timeout status.
9. Tracks Wi-Fi disconnect reason and uptime since last connect for field debugging.
10. Announces this node to RPIBOOSH PYLON registry API and sends periodic heartbeats.

## Requirements
- PlatformIO (VS Code extension or CLI)
- WEMOS/LOLIN S2 Pico (ESP32-S2)
- USB-C cable (data-capable)

## Quick Start
1. Create `src/wifi_credentials.h` with your credentials and these exact defines:
   `BOOSH_WIFI_SSID_MW`, `BOOSH_WIFI_PASS_MW`, `BOOSH_WIFI_SSID_LL`, `BOOSH_WIFI_PASS_LL`
   (see `docs/WIFI.md` for a copy/paste template).
2. Build and upload:

```bash
pio run -t upload
```

3. Monitor serial output:

```bash
pio device monitor
```

## Files
- `platformio.ini`: PlatformIO config and dependencies.
- `src/main.cpp`: Firmware entry point.
- `src/wifi_credentials.h`: Wi-Fi SSIDs/passwords (not for commit).

## OSC Control
The device listens for OSC messages on `/rpiboosh/BooshMain` with 3 float arguments.
- ON: `[1.0, 0.0, 0.0]`
- OFF: `[0.0, 0.0, 0.0]`

Local dev-board control on the WEMOS S2 Pico matches the OSC behavior:
- Press the `0`/BOOT button: ON (`[1.0, 0.0, 0.0]`)
- Release the `0`/BOOT button: OFF (`[0.0, 0.0, 0.0]`)

Prototype proxy behavior:
- Inverted display = solenoid open / fire ON
- Normal display = solenoid closed / fire OFF

If ON is received and OFF does not arrive within 5 seconds, the device forces OFF and logs a failsafe note to Serial and the OLED.

## PYLON Registry API
The device posts presence metadata to RPIBOOSH:
- `POST /api/pylons/announce` after Wi-Fi connect/reconnect
- `POST /api/pylons/heartbeat` every 10 seconds

Default metadata includes:
- `pylon_id` (stable string)
- `description` (human-readable string)
- `hostname` (`<mdns>.local`)
- `ip`
- `osc_port` (`8000`)
- `osc_paths` (`/rpiboosh/BooshMain`)
- `roles` (`boosh_main`)
- `fw_version`
- `ttl_sec` (`30`)

See:
- `docs/PYLON_REGISTRY.md`

## Docs
- `docs/SETUP.md`: Toolchain, build, and flashing.
- `docs/DISPLAY.md`: OLED details and usage.
- `docs/PYLON_REGISTRY.md`: Registry API integration details.
