# Wi-Fi

## Credential File

Create `src/wifi_credentials.h` locally. This file is gitignored — never commit credentials.

Define exactly these symbols:

```cpp
#pragma once

#define BOOSH_WIFI_SSID_MW "your_dev_ssid"
#define BOOSH_WIFI_PASS_MW "your_dev_password"

#define BOOSH_WIFI_SSID_LL "your_lava_production_ssid"
#define BOOSH_WIFI_PASS_LL "your_lava_production_password"
```

- `LL` — Lava Lounge production network
- `MW` — development/bench network

## Connection Priority

At boot, the firmware scans for visible networks and attempts to connect in priority order. Each attempt has a 15-second timeout before trying the next.

1. **Lava production** (`BOOSH_WIFI_SSID_LL`) — if visible in scan
2. **Dev/bench network** (`BOOSH_WIFI_SSID_MW`) — otherwise
3. **User-defined fallback** — SSID/password stored in NVS (if set, see [CONFIG.md](CONFIG.md))

If all three fail, the pylon automatically enables AP mode and saves that setting to NVS so it persists across reboots.

## AP Mode

When AP mode is active, the pylon broadcasts its own open Wi-Fi network:

- **SSID**: `PYLON_{id}` (e.g. `PYLON_PYLON5`)
- **Password**: none
- **Fixed IP**: `10.1.2.3`
- **DNS**: all queries resolved to `10.1.2.3` (captive portal)

Connecting clients are redirected to the pylon control page. OS captive portal detection paths are handled automatically:

| Path | OS |
|------|----|
| `/generate_204` | Android |
| `/hotspot-detect.html` | Apple |
| `/ncsi.txt` | Windows |
| `/connecttest.txt` | Windows |

AP mode can be enabled or disabled at runtime via the web UI, serial CLI (`set ap true`/`set ap false`), or REST API (`POST /api/config/ap`). The setting persists in NVS.

When both STA and AP are active simultaneously, the board uses `WIFI_AP_STA` mode — it remains connected to the upstream network while also serving the AP.

## Status Indicators

- **OLED**: shows connected SSID and IP, or AP status
- **IO38**: HIGH when Wi-Fi STA is connected
- **Serial**: logs SSID, IP, and disconnect reasons

## Notes

- ESP32-S2 supports 2.4 GHz only; 5 GHz networks are not visible.
- mDNS hostname resolves as `{pylon_mdns_host}.local` after STA connect.
- After STA connect, the firmware resolves `RPIBOOSH.local` and pings it once per second.
