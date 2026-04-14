# Configuration

Node identity and runtime settings are persisted in ESP32 NVS (non-volatile storage) using the `Preferences` library. Changes survive reboots.

## Configurable Fields

| Field | NVS key | Default | Description |
|-------|---------|---------|-------------|
| Node ID | `id` | `PYLON{MAC}` | Unique node identifier (e.g. `PYLON5`) |
| mDNS host | `host` | same as ID | Hostname for `.local` resolution |
| Description | `desc` | `""` | Human-readable label |
| AP mode | `ap_en` | `false` | Enable Wi-Fi access point |
| User SSID | `usr_ssid` | `""` | 3rd-priority Wi-Fi fallback SSID |
| User password | `usr_pass` | `""` | Password for user fallback SSID |

The default node ID is generated from the last two bytes of the MAC address on first boot. Once set explicitly it persists.

---

## Serial CLI

Connect via `pio device monitor` (115200 baud) or any serial terminal.

Commands:

```
help                       — print command list
show                       — print current config values
set id <value>             — set node ID
set host <value>           — set mDNS hostname
set desc <value>           — set node description
set node <value>           — set both id and host to the same value
set ap true|false          — enable or disable Wi-Fi AP mode
set wifi_ssid <value>      — set user fallback Wi-Fi SSID
set wifi_pass <value>      — set user fallback Wi-Fi password
clear nvs                  — erase all saved config (resets to defaults on next boot)
```

All `set` commands save to NVS immediately. AP mode change takes effect at the next main loop iteration (no reboot required).

---

## HTTP Config API

See [API.md](API.md) for full endpoint reference.

Quick reference:

```bash
# Read current config
curl http://<pylon-ip>/api/config

# Set node identity
curl -X POST http://<pylon-ip>/api/config/node -d "value=PYLON5"

# Set description
curl -X POST http://<pylon-ip>/api/config/desc -d "value=Center striker"

# Enable AP mode
curl -X POST http://<pylon-ip>/api/config/ap -d "value=true"

# Set user WiFi fallback
curl -X POST http://<pylon-ip>/api/config -d "wifi_ssid=MyNetwork&wifi_pass=MyPassword"
```

---

## Web UI

The web UI at `http://<pylon-ip>/` exposes the same config fields via a form panel. Changes are sent via the REST API and take effect immediately.

---

## Compile-Time Constants

These are set in `src/main.cpp` and require a rebuild to change:

| Constant | Default | Description |
|----------|---------|-------------|
| `kPylonIdDefaultPrefix` | `"PYLON"` | Prefix for auto-generated node ID |
| `kRegistryBaseUrlPrimary` | `"http://rpiboosh.local:5000"` | RPIBOOSH registry URL (mDNS) |
| `kRegistryBaseUrlFallback` | `""` | Fixed-IP fallback registry URL |
| `kRegistryTtlSec` | `30` | Registry TTL in seconds |
| `kRegistryHeartbeatIntervalMs` | `10000` | Heartbeat interval |
| `kRegistryHttpTimeoutMs` | `2500` | HTTP timeout for registry posts |
| `kOscPort` | `8000` | UDP OSC port |
| `kBooshFailsafeTimeoutMs` | `5000` | Max boosh ON duration before forced OFF |
| `kFirmwareVersion` | `"0.0.1 DATE TIME"` | Build identity string |
