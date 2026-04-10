# Display

The WEMOS/LOLIN S2 Pico includes a built-in SSD1306 OLED.

## Hardware
- Controller: SSD1306
- Resolution: 128x32
- I2C address: `0x3C`
- I2C pins: SDA `GPIO8`, SCL `GPIO9`
- Reset pin: `GPIO18`

## Initialization
In `src/main.cpp`, the display is initialized with:
- `Wire.begin(8, 9)`
- `display.begin(SSD1306_SWITCHCAPVCC, 0x3C)`

## Output
The display shows:
- Boot status
- Wi-Fi scan/connection
- IP address on success
- Ping status for `RPIBOOSH` (OK/FAIL/TIMEOUT), last/min/max/avg, and time since last successful ping
- Failsafe notices when BooshMain times out
- Node trigger count
- Firmware version / build timestamp

## Runtime Pages
The OLED cycles pages every ~3 seconds (see `kDisplayCycleMs` in `src/main.cpp`):

Ping page:
- Host name
- Status: `OK`, `FAIL`, or `TIMEOUT` (no successful ping within `kPingTimeoutMs`)
- `last` latency, `min/max` latency, and `since` in `HH:MM:SS`

Wi-Fi debug page A:
- `SSID`
- `RSSI` (signal strength)
- `IP` (DHCP success indicator)
- `UP` in `HH:MM:SS`

Wi-Fi debug page B:
- `RSN` (last disconnect reason code + short label)
- `SSID`
- `RSSI`
- `IP`

Node page:
- Node ID
- Trigger count
- Firmware semantic version

Firmware page:
- Semantic version
- Build date
- Build time

## Recent Changes
- Ping page now indicates `TIMEOUT` when no successful ping has occurred within `kPingTimeoutMs`.
- Wi-Fi pages include `UP` and ping `since` in `HH:MM:SS`.
- Node page includes total trigger count.
- Firmware page shows `0.0.1 <DATE> <TIME>` build identity.

## BooshMain Visuals
Display inversion is a prototype proxy for triggering the boosher solenoid.

When OSC `/rpiboosh/BooshMain` is ON (`[1]`), the OLED is inverted.
- Inverted display = solenoid open / fire ON

When OFF (`[0]`), the OLED returns to normal.
- Normal display = solenoid closed / fire OFF

The same inversion behavior is used when the solenoid is driven from:
- OSC
- The dev-board `0`/BOOT button
- The web UI / REST API
