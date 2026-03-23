# Wi-Fi

The firmware connects as a Wi-Fi client in station mode.

## Credential File
Create `src/wifi_credentials.h` locally with your own SSIDs and passwords.

Terminology:
- `MW` / `primary`: home test/development Wi-Fi credentials.
- `LL`: Lava production Wi-Fi credentials.

Define exactly these symbols:
- `BOOSH_WIFI_SSID_MW`
- `BOOSH_WIFI_PASS_MW`
- `BOOSH_WIFI_SSID_LL`
- `BOOSH_WIFI_PASS_LL`

Template:

```cpp
#pragma once

#define BOOSH_WIFI_SSID_MW "your_primary_ssid"
#define BOOSH_WIFI_PASS_MW "your_primary_password"

#define BOOSH_WIFI_SSID_LL "your_lava_production_ssid"
#define BOOSH_WIFI_PASS_LL "your_lava_production_password"
```

Do not commit credentials to git.

## Connection Priority
At boot the device scans for available networks:
1. If `BOOSH_WIFI_SSID_LL` (Lava production) is found, it connects to that network.
2. Otherwise it attempts to connect to `BOOSH_WIFI_SSID_MW` (home test/development).

## Status
Wi-Fi status is shown:
- On the OLED (LL/Lava if found, otherwise MW/home test-dev, connected, or failure).
- In the serial monitor (SSID and IP address).

## Timeouts
Connection attempts time out after ~20 seconds and will show a failure status.

## Hostname Resolution
After Wi-Fi connects, the firmware resolves `RPIBOOSH` (and `RPIBOOSH.local` via mDNS) and pings it once per second.
