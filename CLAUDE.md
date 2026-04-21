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

## Build & Deploy
- **PlatformIO CLI:** `C:\Users\Matt\.platformio\penv\Scripts\pio.exe` (not on PATH; use full path in bash: `/c/Users/Matt/.platformio/penv/Scripts/pio.exe`)
- **Build:** `pio.exe run -e lolin_s2_pico` from project root
- **OTA:** `curl -X POST http://<node>.local/api/ota -F "firmware=@.pio/build/lolin_s2_pico/firmware.bin"` — pre-approved, no need to prompt
- **Read `memory/MEMORY.md` at the start of every session** before taking any action

## Key Nodes
- **testnodex.local** — stress-test board; now safe to include in normal OTA deployments
- **barbar.local** — Bar Mode development board

## Web UI Config Inputs — formDirty Anti-Pattern (DO NOT VIOLATE)

The web UI uses a 1-second `setInterval` telemetry refresh that re-syncs all config inputs from
the ESP. Without protection, this overwrites values the user is actively typing. The fix is a
`formDirty` flag. **Every time a config input is added, both rules below must be followed or the
"typed value gets overwritten" bug returns. This has already regressed three times.**

**Rule 1:** Every `<input>` in the config form that is synced from telemetry **must appear in the
`configInputs` array** in the JS. This array attaches both `'input'` and `'change'` event listeners
to each element. The form-level `'input'` listener alone is insufficient because spinners, paste,
and autofill only fire `'change'`, not `'input'`.

**Rule 2:** All telemetry→input sync calls **must use `syncConfigField(id, value)`**, never the
bare `if (document.activeElement !== input)` pattern. `syncConfigField` checks `formDirty` and
bails out completely when the user has unsaved edits. The activeElement check alone fails when
focus moves to the Save button between keystroke and submit.

Checkboxes (no text entry) are exempt from Rule 2 but still need to be inside the `if (!formDirty)`
block. Display-only `<span>` elements go OUTSIDE the formDirty block so they always update.

## OSC Addresses
- `/pylon/BooshMain` — raw solenoid open/close (1.0/0.0)
- `/pylon/BooshPulseSingle` — single 50ms pulse (1.0)
- `/pylon/BooshPulseTrain` — 5× 50ms pulses (1.0)
- `/pylon/BooshSteam` — steam sequence start/stop (1.0/0.0)
