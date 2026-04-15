# Pylons — Project Overview

## What Is This?

Pylons are autonomous ESP32-S2 nodes deployed at Lava Lounge fire-effect installations. Each pylon:

- Receives fire-trigger commands from the central RPIBOOSH controller via OSC (UDP) or HTTP
- Drives a solenoid output and visual indicator LEDs
- Monitors battery voltage and enclosure temperature
- Reports telemetry (ping, sensors, identity) back to RPIBOOSH
- Can operate standalone via a captive-portal Wi-Fi AP if no venue network is available

The central controller (`RPIBOOSH`, a Raspberry Pi) discovers pylons via the registry API and orchestrates synchronized fire effects across multiple nodes.

---

## Repository Layout

```
pylons/
├── src/
│   ├── main.cpp              # All firmware logic (single file)
│   └── wifi_credentials.h   # NOT committed — local credentials only
├── platformio.ini            # PlatformIO build config
├── schematic_electrical/     # KiCad schematics
│   ├── ESP32_MCU.kicad_sch
│   ├── POWER.kicad_sch
│   └── CONNECTORS.kicad_sch
├── docs/                     # This documentation
└── README.md
```

---

## System Architecture

```
RPIBOOSH (Raspberry Pi)
    │
    ├─ OSC UDP :8000 ──────────► Pylon (ESP32-S2)
    │                                 │
    ├─ HTTP registry API ◄────────────┤  announces + heartbeats
    │                                 │
    └─ HTTP solenoid API ◄────────────┘  web UI / REST
```

Multiple pylons can be on the same network. RPIBOOSH maintains a registry of active nodes via announce/heartbeat and targets individual nodes by ID.

---

## Boot Sequence

1. USB CDC + Serial initialized
2. Task watchdog disabled for `loopTask` (blocking calls are intentional)
3. NVS config loaded (node ID, host, description, AP flag, user WiFi creds)
4. GPIO and ADC pins configured; LEDC PWM channels started for status LEDs
5. I2C + OLED initialized
6. Wi-Fi connection attempted in priority order:
   - `BOOSH_WIFI_SSID_LL` (Lava production, if visible in scan)
   - `BOOSH_WIFI_SSID_MW` (dev/bench network)
   - User-defined fallback SSID (if configured in NVS)
   - If all fail → AP mode auto-enabled and saved to NVS
7. If connected: mDNS, OSC UDP, web server started; pylon announced to registry
8. If AP enabled: SoftAP started on `PYLON_{id}` with captive DNS at `10.1.2.3`

---

## Main Loop

Each iteration (no hard delay except in disconnected/AP-only state):

- `PollBlinkLeds()` — drive status LED square wave (IO12) and sine-wave brightness (IO13/14/15)
- `PollSensors()` — read battery ADC and thermistor every 5 s
- `PollSerialCli()` — process serial CLI input
- `PollDevBoardButton()` — handle dev board button (GPIO0)
- `webServer.handleClient()` — serve HTTP requests
- `dnsServer.processNextRequest()` — captive portal DNS (if AP active)
- Apply live AP enable/disable from config changes
- If Wi-Fi disconnected and not in AP mode: show status and return
- `PollOsc()` — receive and process OSC packets
- `HandleRegistry()` — announce / heartbeat
- Boosh failsafe timeout check
- mDNS host resolution + ping once per second
- OLED page cycle (5 pages, 3 s each)

---

## Features

### Boosh (Fire Trigger)

The primary function. When triggered ON:
- OLED inverts (visual indicator)
- `IO11` goes HIGH (external signal: boosh active)
- `IO12` white LED continues its 1 Hz blink
- Failsafe: if ON is received and OFF is not seen within **5 seconds**, firmware forces OFF

Trigger sources (all equivalent):
- OSC message `/pylon/BooshMain` with float arg `1.0` (ON) or `0.0` (OFF)
- `POST /api/solenoid/on` and `POST /api/solenoid/off`
- Dev-board GPIO0/BOOT button press and release
- Web UI press-and-hold button

### Status LEDs

| Pin | Color | Behavior |
|-----|-------|----------|
| IO12 | White | 1 Hz square wave — boosh trigger indicator |
| IO13 | Yellow | Sine-wave brightness, ~1.2 Hz, 0–33% |
| IO14 | Blue | Sine-wave brightness, ~0.8 Hz, 0–33% |
| IO15 | Green | Sine-wave brightness, ~0.4 Hz, 0–33% |
| IO11 | — | HIGH when boosh is active |
| IO38 | — | HIGH when Wi-Fi STA connected |

### Battery Sensing (IO3)

Voltage divider: `BATTERY_SENSE_12V → R5(100 kΩ) → R8(22 kΩ) → GND`; junction feeds IO3 via R4(10 kΩ) protection resistor.

- Scale factor: `(100k + 22k) / 22k ≈ 5.545×`
- 100% = 12.7 V (SLA fully charged)
- 0% = 10.5 V (SLA discharged)
- Discharge rate estimated from a rolling 20-minute voltage history (1-min samples). Time-remaining shown in telemetry once 5+ samples are collected.

### Thermistor (IO4)

NTC thermistor in voltage divider with R12 (10 kΩ pull-down), output through R13 (10 kΩ) to IO4.

Steinhart-Hart equation with empirically calibrated coefficients:
- `c1 = 1.274219988e-03`
- `c2 = 2.171368266e-04`
- `c3 = 1.119659695e-07`
- Manual offset: `+11.17 °F` (validated against reference `boosh_box_esp32_remote_thermo` project)

Temperature reported in °F and °C in telemetry.

### Wi-Fi AP Mode

When enabled, pylon broadcasts its own open Wi-Fi network:
- SSID: `PYLON_{id}` (e.g. `PYLON_PYLON5`)
- No password
- Fixed IP: `10.1.2.3`
- DNS server answers all queries with `10.1.2.3` (captive portal)
- OS captive portal detection paths handled: `/generate_204`, `/hotspot-detect.html`, `/ncsi.txt`, `/connecttest.txt`
- Connecting clients are presented with the pylon control page

AP mode is **auto-enabled** if all STA networks fail to connect at boot (and saved to NVS so it persists).

### PYLON Registry

After connecting to Wi-Fi, the pylon announces itself to RPIBOOSH and sends heartbeats every 10 s. See [PYLON_REGISTRY.md](PYLON_REGISTRY.md) for the full payload contract.

---

## See Also

- [HARDWARE.md](HARDWARE.md) — GPIO pinout, schematic notes, ADC circuits
- [SETUP.md](SETUP.md) — build and flash instructions
- [WIFI.md](WIFI.md) — credential file, connection priority, AP mode
- [API.md](API.md) — HTTP REST and OSC reference
- [CONFIG.md](CONFIG.md) — node identity, NVS, CLI, HTTP config
- [PYLON_REGISTRY.md](PYLON_REGISTRY.md) — registry payload contract
- [DISPLAY.md](DISPLAY.md) — OLED page layout
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
