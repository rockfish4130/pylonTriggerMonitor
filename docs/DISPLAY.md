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

## Runtime Pages
The OLED cycles pages every ~3 seconds (see `kDisplayCycleMs` in `src/main.cpp`):

Ping page:
- Host name
- Status: `OK`, `FAIL`, or `TIMEOUT` (no successful ping within `kPingTimeoutMs`)
- `last` latency, `min/max` latency, and `since ok` seconds

Wi-Fi debug page A:
- `SSID`
- `RSSI` (signal strength)
- `IP` (DHCP success indicator)
- `UP` (seconds since connected)

Wi-Fi debug page B:
- `RSN` (last disconnect reason code + short label)
- `SSID`
- `RSSI`
- `IP`

## Recent Changes
- Ping page now indicates `TIMEOUT` when no successful ping has occurred within `kPingTimeoutMs`.
- Wi-Fi pages include `UP` (seconds since connected) and `RSN` (last disconnect reason).

## BooshMain Visuals
When OSC `/rpiboosh/BooshMain` is ON (`[1, 0, 0]`), the OLED is inverted.
When OFF (`[0, 0, 0]`), the OLED returns to normal.
