# Troubleshooting

## Upload Fails / Port Not Found

- Use a data-capable USB-C cable (not charge-only).
- Check that the board appears in your device list.
- On normal boot, the firmware enumerates a USB CDC serial port automatically.
- If auto-reset fails, manually enter bootloader: hold `0`/BOOT, tap `RST`, release `0`, then start upload.
- After flashing, tap `RST` once to reboot. The COM port re-enumerates after a few seconds.
- If PlatformIO serial monitor was open, close it before uploading — it may hold the port.

## OLED Is Blank

- Confirm board is WEMOS/LOLIN S2 Pico (has built-in 128×32 SSD1306).
- I2C address must be `0x3C`.
- Reset pin must be `GPIO18`.
- Check `Wire.begin(8, 9)` is called before `display.begin()`.

## Serial Output Missing

- Confirm baud rate is `115200`.
- Ensure build flags `ARDUINO_USB_CDC_ON_BOOT=1` and `ARDUINO_USB_MODE=0` are set in `platformio.ini`.
- Watch for boot markers: `Boot: OLED ready`, `Boot: WiFi STA mode`, `WiFi scan...`.

## Wi-Fi Does Not Connect

- Verify SSIDs and passwords in `src/wifi_credentials.h`.
- ESP32-S2 supports 2.4 GHz only — 5 GHz networks are invisible.
- Each network attempt has a 15-second timeout before moving to the next.
- If all three networks fail, AP mode is auto-enabled.
- Connection order: Lava (`LL`) → dev (`MW`) → user-defined fallback.

## AP Mode Enabled Unexpectedly

If AP mode was auto-enabled because all STA networks failed, it is saved to NVS and persists on reboot. To disable:

```bash
# Via serial CLI
set ap false

# Via HTTP (when connected)
curl -X POST http://<pylon-ip>/api/config/ap -d "value=false"
```

Or use the web UI toggle.

## Cannot Connect to AP

- SSID is `PYLON_{id}` (e.g. `PYLON_PYLON5`) — open network, no password.
- Fixed IP is `10.1.2.3`.
- If captive portal does not appear, navigate directly to `http://10.1.2.3/`.

## OSC Not Responding

- Confirm UDP port `8000` is open on the network.
- OSC address is `/pylon/BooshMain` (not `/rpiboosh/BooshMain` — that was a legacy path).
- Message must have exactly one float argument: `1.0` (ON) or `0.0` (OFF).
- Test locally: press the `0`/BOOT button on the dev board (ON on press, OFF on release).

## Boosh Turns Off Unexpectedly

The 5-second failsafe forces OFF if no OFF message arrives within 5 s of ON. Serial log: `Failsafe: BooshMain timeout -> forcing OFF.`

This is intentional. Ensure the OFF message is sent promptly after ON.

## Battery / Temperature Reads Null

- Sensor values show `null` for the first poll cycle (~5 seconds after boot).
- If they remain null, check ADC wiring against [HARDWARE.md](HARDWARE.md).
- Battery ADC on IO3 reads 0 or 4095 if the divider is open or shorted — these are rejected.
- Thermistor ADC on IO4 reads 0 or 4095 if the NTC is open or shorted — these are also rejected.

## Battery Time Remaining Shows Null

Time remaining is estimated from a rolling 20-minute voltage history using 1-minute samples. It shows `null` until at least 5 samples are collected (~5 minutes after boot) and the battery is actively discharging.

## Registry Announce / Heartbeat Failing

- Check serial logs for `[REG]` prefixed lines.
- Confirm RPIBOOSH is reachable: `http://rpiboosh.local:5000`.
- If mDNS does not resolve `rpiboosh.local`, set `kRegistryBaseUrlFallback` to a fixed IP URL in `src/main.cpp`.
- Verify the registry service paths: `POST /api/pylons/announce`, `POST /api/pylons/heartbeat`.

## Device Resets After a Few Minutes (Watchdog)

Watchdog resets were caused by blocking HTTP and ping calls stalling the `loopTask` beyond the TWDT threshold. This is fixed in current firmware via `esp_task_wdt_delete(NULL)` in `setup()`, which removes `loopTask` from watchdog monitoring. If you see watchdog resets, confirm `#include "esp_task_wdt.h"` and the `esp_task_wdt_delete(NULL)` call are present in `src/main.cpp`.
