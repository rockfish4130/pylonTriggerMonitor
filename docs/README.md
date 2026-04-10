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
3. Serves an HTTP control/status page on port `80`.
4. Exposes REST APIs for telemetry, console log access, solenoid hold control, and node config.
5. Uses OLED inversion as a prototype proxy for the boosher solenoid state.
6. Inverted display means solenoid open / fire ON.
7. Normal display means solenoid closed / fire OFF.
8. Counts total trigger events for the current boot session.
9. Pings `RPIBOOSH` once per second and shows ping status/stats on the OLED.
10. Applies a failsafe: if ON is received and OFF is not seen within 5 seconds, it forces OFF.
11. Cycles OLED pages with Wi-Fi debug metrics, node stats, firmware version, and ping timeout status.
12. Tracks Wi-Fi disconnect reason and uptime since last connect for field debugging.
13. Announces this node to RPIBOOSH PYLON registry API and sends periodic heartbeats.

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
The device listens for OSC messages on `/rpiboosh/BooshMain` with 1 float argument.
- ON: `[1.0]`
- OFF: `[0.0]`

Local dev-board control on the WEMOS S2 Pico matches the OSC behavior:
- Press the `0`/BOOT button: ON (`[1.0]`)
- Release the `0`/BOOT button: OFF (`[0.0]`)

Prototype proxy behavior:
- Inverted display = solenoid open / fire ON
- Normal display = solenoid closed / fire OFF

If ON is received and OFF does not arrive within 5 seconds, the device forces OFF and logs a failsafe note to Serial and the OLED.

Web control behavior:
- `POST /api/solenoid/on`: open solenoid immediately
- `POST /api/solenoid/off`: close solenoid immediately
- `POST /api/solenoid/trigger`: compatibility alias for `on`
- The browser UI button is press-and-hold. Loss of focus, page hide, pointer cancel, or release is treated as OFF.

## HTTP Interface
The device exposes a small dark-theme web UI on `http://<ip>/` and `http://<mdns>.local/`.

Endpoints:
- `GET /`: control/status web UI
- `GET /api/telemetry`: current telemetry payload plus OLED page text
- `GET /api/logs`: mirrored serial console text buffer
- `GET /api/config`: current persisted node config (`id`, `host`, `hostname`, `description`)
- `POST /api/config`: update any subset of `id`, `host`, `description`, or `node`
- `POST /api/config/id`: set `id` via `value`
- `POST /api/config/host`: set `host` via `value`
- `POST /api/config/desc`: set `description` via `value`
- `POST /api/config/node`: set both `id` and `host` via `value`
- `POST /api/solenoid/on`: solenoid ON
- `POST /api/solenoid/off`: solenoid OFF
- `POST /api/solenoid/trigger`: compatibility alias for ON

The web UI main page includes:
- Live telemetry and console view
- Press-and-hold solenoid control
- Editable node config fields for `id`, `host`, `description`, and `node` alias

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
- `firmware_version` / `version` / `fw_semver`
- `ttl_sec` (`30`)
- `telemetry.ipv4` / `telemetry.mdns_hostname` legacy compatibility aliases
- `telemetry.temperature` / `telemetry.temperature_f` / `telemetry.temperature_c`
- `telemetry.battery_voltage` / `telemetry.battery_voltage_v`
- `telemetry.battery_charge` / `telemetry.battery_charge_pct`
- `telemetry.uptime` / `telemetry.uptime_hms`
- `telemetry.trigger_event_count` / `telemetry.solenoid_active`
- `telemetry.ping.target/sent/recv/lost/last_ms/min_ms/max_ms/avg_ms/count/last_ok`
- `telemetry.ping.since_ok` / `telemetry.ping.since_ok_s`

Compatibility note:
- The current announce/heartbeat payload is the legacy baseline for deployed nodes.
- The telemetry object is emitted in the same compatibility shape used by existing High Striker nodes so `boosh_box_pi/rpi_python_control` can ingest it without receiver changes.
- Future firmware may append optional telemetry fields, and receivers should ignore unknown fields.
- Absence of an explicit schema/protocol marker should be interpreted as the current legacy baseline documented in `docs/PYLON_REGISTRY.md`.

See:
- `docs/PYLON_REGISTRY.md`

## Docs
- `docs/SETUP.md`: Toolchain, build, and flashing.
- `docs/DISPLAY.md`: OLED details and usage.
- `docs/PYLON_REGISTRY.md`: Registry API integration details.

## Firmware Version
Current firmware version format:
- `0.0.1 <DATE> <TIME>`
