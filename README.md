# Pylons

ESP32-S2 firmware for Lava Lounge fire-effect pylon nodes. Each pylon is a networked device that receives fire-trigger commands via OSC or HTTP, drives solenoids and status LEDs, and reports telemetry back to the RPIBOOSH control system.

## Documentation

| Document | Contents |
|---|---|
| [docs/README.md](docs/README.md) | Full project overview and feature reference |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Board, schematic, GPIO pinout, sensors |
| [docs/SETUP.md](docs/SETUP.md) | Build, flash, serial monitor |
| [docs/WIFI.md](docs/WIFI.md) | Wi-Fi configuration, AP mode, fallback networks |
| [docs/API.md](docs/API.md) | HTTP REST API and OSC reference |
| [docs/CONFIG.md](docs/CONFIG.md) | Node config: NVS, serial CLI, HTTP config API |
| [docs/PYLON_REGISTRY.md](docs/PYLON_REGISTRY.md) | Registry announce/heartbeat payload contract |
| [docs/DISPLAY.md](docs/DISPLAY.md) | OLED page layout and behavior |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Common problems and fixes |

## Quick Start

```bash
# 1. Create src/wifi_credentials.h  (see docs/WIFI.md)
# 2. Build and flash
pio run -t upload
# 3. Monitor
pio device monitor
```
