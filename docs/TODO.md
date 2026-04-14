# TODO

## Open

- [ ] Replace hardcoded `osc_paths` legacy string in registry payload with dynamic `kOscAddress` — done in firmware, but verify RPIBOOSH receiver accepts `/pylon/BooshMain` path.
- [ ] Validate Steinhart-Hart thermistor calibration on installed hardware (current coefficients are from `boosh_box_esp32_remote_thermo` reference project).
- [ ] Reduce LED resistors from 1 kΩ to 330 Ω on next board revision for better visibility (current ~2 mA; 330 Ω gives ~6 mA, still within GPIO 40 mA limit).
- [ ] Add OTA firmware update support.
- [ ] Add NTP time sync for timestamped telemetry.
- [ ] Multi-pylon synchronized fire choreography (timing protocol between RPIBOOSH and pylons).

## Done

- [x] OSC boosh trigger (`/pylon/BooshMain`)
- [x] HTTP REST API (solenoid on/off, telemetry, config)
- [x] Web UI with hold-to-fire button, live telemetry, config panel, log console
- [x] Status LEDs: IO12 white 1 Hz, IO13 yellow sine 1.2 Hz, IO14 blue sine 0.8 Hz, IO15 green sine 0.4 Hz
- [x] IO11 = HIGH when boosh active; IO38 = HIGH when Wi-Fi STA connected
- [x] Boosh failsafe (5-second forced OFF)
- [x] RPIBOOSH registry: announce + heartbeat with exponential backoff
- [x] Battery voltage sensing and % on IO3
- [x] Battery time-remaining estimation (rolling 20-min history)
- [x] Thermistor temperature on IO4 (Steinhart-Hart, calibrated)
- [x] Wi-Fi AP mode with captive portal (DNSServer, OS detection paths)
- [x] User-defined Wi-Fi fallback (SSID/password in NVS)
- [x] NVS config persistence (id, host, desc, ap_en, user WiFi)
- [x] Serial CLI for config
- [x] OLED 5-page display cycle
- [x] TWDT watchdog disabled for loopTask (prevents resets from blocking calls)
- [x] Web UI log scroll lock (only auto-scrolls when already at bottom)
- [x] Full documentation suite
