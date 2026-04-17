# PYLON Firmware — Claude Code Context

## Project Overview
ESP32-S2 (Wemos S2 Pico) firmware for fire-effect pylons. PlatformIO/Arduino framework. Main source: `src/main.cpp`.

## Top Priority
**OSC message latency is the #1 concern.** Nothing may block the main loop (Core 1). Blocking work (HTTP, DNS, ping) runs on Core 0 via `PingTask`. If a proposed change risks blocking the main loop, warn before proceeding.

## Production Deployments
- **BOOSHMAN** is deployed in production at commit `03302685` (tagged `BOOSHMAN-deployed`). Maintain backward compatibility with its OSC addresses, NVS key names, and API response fields. Flag any breaking change.

## Hardware
- **LED pins (as built):** IO13 = blue, IO14 = yellow, IO15 = green (blue/yellow are swapped vs. original schematic)
- **Battery:** 12V LiFePO4 4S — not SLA. Voltage thresholds in code reflect this.
- **Solenoid:** IO11. One channel. Failsafe timeout is NVS-persisted and web-configurable.
- **Bar Mode hardware** (boards with `"BARMODE ENABLED"` in description): 4 buttons on IO1/IO2/IO5/IO6 (INPUT_PULLDOWN, active HIGH); lamp PWM outputs TBD.

## System Architecture
- `rpiboosh.local` — Raspberry Pi running `rpi_python_control`; registry server, OSC sender
- PYLONs announce/heartbeat to `rpiboosh.local:5000`
- OSC received on UDP port 8000
- mDNS, web UI on port 80, OTA via `POST /api/ota`

## Key Nodes
- **testnodex.local** — stress-test board; now safe to include in normal OTA deployments
- **barbar.local** — Bar Mode development board

## OSC Addresses
- `/pylon/BooshMain` — raw solenoid open/close (1.0/0.0)
- `/pylon/BooshPulseSingle` — single 50ms pulse (1.0)
- `/pylon/BooshPulseTrain` — 5× 50ms pulses (1.0)
- `/pylon/BooshSteam` — steam sequence start/stop (1.0/0.0)
