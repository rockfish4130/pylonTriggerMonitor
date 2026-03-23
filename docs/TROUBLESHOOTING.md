# Troubleshooting

## Upload Fails / Port Not Found
- Use a known data-capable USB-C cable.
- Try a different USB port.
- Check that the board appears in your device list.
- If auto-reset fails, manually enter bootloader:
  - Hold `0`, tap `RST`, release `0`, then start upload.

## OLED Is Blank
- Confirm the board is the WEMOS/LOLIN S2 Pico (128x32 SSD1306).
- Ensure the OLED address is `0x3C`.
- Verify the reset pin is set to `GPIO18`.

## Wi-Fi Does Not Connect
- Verify SSIDs and passwords in `src/wifi_credentials.h`.
- Ensure the AP is 2.4 GHz (ESP32-S2 does not support 5 GHz).
- Move closer to the AP to improve signal.

## Serial Output Missing
- Confirm baud is `115200`.
- Ensure `ARDUINO_USB_CDC_ON_BOOT=1` and `ARDUINO_USB_MODE=1` are set in `platformio.ini`.

## OSC Not Responding
- Confirm UDP port `8000` is open and the target IP is the device IP.
- Verify address is `/rpiboosh/BooshMain` with 3 float args.
- ON is `[1, 0, 0]`, OFF is `[0, 0, 0]`.

## BooshMain Turns Off Unexpectedly
- The firmware has a 5 second failsafe after ON if OFF is not received.
- Look for the Serial log: `Failsafe: BooshMain timeout -> forcing OFF.`

## Registry Announce / Heartbeat Failing
- Look for serial logs prefixed with `[REG]`.
- Confirm RPI endpoint is reachable: `http://rpiboosh.local:5000`.
- If mDNS does not resolve `rpiboosh.local`, set `kRegistryBaseUrlFallback` in `src/main.cpp` to a fixed IP URL.
- Confirm RPI service path availability:
  - `POST /api/pylons/announce`
  - `POST /api/pylons/heartbeat`
- Verify Wi-Fi remains connected long enough for retries.
