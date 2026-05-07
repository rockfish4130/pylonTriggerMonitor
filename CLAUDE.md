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
- **testnodex.local** — stress-test board; safe to include in normal OTA deployments
- **barbar.local** — Bar Mode development board (AP always-on, pylon_index=1)
- **tiki0.local** — MEGA TIKI Zero, pylon_index=3
- **tiki1.local** — MEGA TIKI One, pylon_index=2
- **testredled.local** — unspecified, pylon_index=6
- **pylon5d90.local** — unspecified, pylon_index=7
- **booshstriker.local** — High Striker Boosh node (different firmware project)

## Web UI Config Inputs — formDirty Anti-Pattern (DO NOT VIOLATE)

The web UI uses a 1-second `setInterval` telemetry refresh that re-syncs all config inputs from
the ESP. Without protection, this overwrites values the user is actively typing. The fix is a
`formDirty` flag. **Every time a config input is added, both rules below must be followed or the
"typed value gets overwritten" bug returns. This has already regressed three times.**

**Rule 1:** Every `<input>`, `<select>`, or `<input type="checkbox">` that is synced from telemetry
**must appear in the `configInputs` array** in the JS — no exceptions. This array attaches both
`'input'` and `'change'` event listeners. The form-level listener alone is insufficient. **This bug
has regressed 5+ times, including on a `<select>` element.** A large warning banner comment sits
directly above the `configInputs` array in the source — heed it.

**Rule 2:** All telemetry→input sync calls **must use `syncConfigField(id, value)`**, never the
bare `.value = x` or `if (document.activeElement !== input)` patterns. Applies to text inputs and
`<select>` elements. `syncConfigField` checks `formDirty` and bails out completely when the user
has unsaved edits.

Checkboxes are exempt from Rule 2 but **are NOT exempt from Rule 1**. Sync checkboxes inside the
`if (!formDirty)` block using `document.activeElement !== box`. Must still be in `configInputs`.

Display-only `<span>` elements go OUTSIDE the formDirty block so they always update.

## AP Soft-Access-Point

- AP SSID: `FIRE_PYLON_<pylon_id>` (e.g. `FIRE_PYLON_BARBAR`). Max 21 chars for current node names, well under the 32-char WiFi limit.
- AP IP: `10.1.2.3`. Captive portal DNS active when in AP mode.
- AP is auto-enabled when WiFi fails at boot (saved to NVS); barbar has AP permanently on.

**AP channel management (three-part rule — do not simplify):**

1. **AP+STA mode** (STA connected to router): the single radio is locked to the router's channel. `SetupApMode()` reads `WiFi.channel()` before the mode switch and passes that to `softAP()`. Passing any other channel silently fails, leaving the default `ESP_Fxxxxxx` SSID.

2. **AP-only mode** (no router): `softAP()` is called with `cfg_mesh_ch`. The default regulatory domain allows ch 1–11 only; ch 12–13 fail silently. A `softAP()` return-value check retries on ch 1 if the mesh channel is rejected.

3. **STA-loss transition**: when the STA drops and the AP was running on the router's channel (not `cfg_mesh_ch`), the `wasConnected→false` block in the main loop immediately calls `StopApMode()` + `SetupApMode()` to restart the AP on `cfg_mesh_ch`. This ensures all nodes converge on the same channel within seconds of WAP loss.

**Deployed mesh channel: 11.** Ch 11 is within the 1–11 regulatory range, so `softAP()` always succeeds on ch 11 without the ch-1 fallback. Do not change `cfg_mesh_ch` to 12 or 13 — `softAP()` will fail silently, restoring the `ESP_Fxxxxxx` SSID regression.

**OLED channel badge:** `Ch:11` when hardware matches config. `C6*11` means hardware is on ch 6 (router's channel) while config is 11 — expected in AP+STA mode, resolves automatically when the router drops.

## ESP-NOW Mesh Protocol

All nodes participate in a peer-to-peer ESP-NOW mesh on a configurable channel (NVS key `mesh_ch`, **deployed value: 11**). Key constants and structs are all in `src/main.cpp`.

**Constants:**
- `kMeshMagic = 0x4D455348UL` ("MESH") — first 4 bytes of every packet; quick sanity check
- `kMeshVersion = 3` — bump whenever a packet struct layout changes; mismatched nodes are dropped
- `kMeshPktBeacon = 1`, `kMeshPktCommand = 2`, `kMeshPktChanChange = 3`
- Beacon interval: 2000 ms. Peer timeout: 8000 ms. Max peers: 10.
- Quality metric: 16-slot rolling bitmap, one slot per beacon interval (32s window); `qual_pct = popcount(qual_bits) * 100 / 16`

**Packet structs** (all `__attribute__((packed))`):
- `MeshBeaconPkt` — node announcement: node_id, pylon_index, role, uptime_s, batt_v, batt_pct, temp_f, fw_ver[32]
- `MeshCommandPkt` — OSC relay: seq (dedup), osc_addr[32], osc_arg (float)
- `MeshChanChangePkt` — coordinated channel switch: new_ch (1-11), apply_ms (countdown from receipt)

**Channel management:**
- In STA mode (WiFi connected): radio channel is controlled by the AP. `peer.channel = 0` so ESP-NOW follows the current WiFi channel automatically.
- In AP-only mode: radio is on the AP channel (`cfg_mesh_ch`, or ch 1 fallback). `peer.channel = 0` follows the AP channel.
- Do NOT call `esp_wifi_set_channel()` while AP is active — the AP interface owns the channel; the call is a no-op or corrupts state.
- Coordinated channel change: broadcast `MeshChanChangePkt` with a 5s countdown from BARBAR's web UI. All receiving nodes arm a timer; PingTask (Core 0) applies the change when it fires. Saves to NVS. Only use channels 1–11.

**ESP-NOW interface selection — critical:**
Both `MeshInit()` (broadcast peer) and `MeshUpsertPeer()` (unicast peers) select the interface at runtime:
```cpp
peer.ifidx = ap_active ? WIFI_IF_AP : WIFI_IF_STA;
```
In AP-only mode `WIFI_IF_STA` is unassociated — ESP-NOW through it is silently dropped (M:0). In AP+STA mode either interface works since they share the radio channel. **Never hardcode `WIFI_IF_STA`.**

**WiFi reconnect behavior:**
- `setAutoReconnect(false)` when mesh is enabled (reconnect scans disrupt ESP-NOW channel).
- Main loop watchdog: `WiFi.reconnect()` every 30 s when no live peers; every **10 minutes** when live peers are present (brief ~5–10 s ESP-NOW disruption is acceptable vs. permanent WiFi loss).
- After 10 min offline with AP inactive: hard reboot. AP-active nodes (barbar) never auto-reboot.

**Receive callback** `MeshOnRecv` runs in WiFi task (Core 0). OSC commands are queued via `mesh_osc_queue` (FreeRTOS queue) for processing on Core 1. Do not block in the callback.

**Peer registration:** `esp_now_add_peer()` must be called before unicast. Broadcast peer uses MAC `FF:FF:FF:FF:FF:FF` with `channel=0`. Done in `MeshUpsertPeer` and mesh init.

**Dedup:** `MeshDedupEntry[16]` with 500ms window prevents duplicate OSC fires from retried broadcasts.

## OSC Addresses
- `/pylon/BooshMain` — raw solenoid open/close (1.0/0.0)
- `/pylon/BooshPulseSingle` — single 50ms pulse (1.0)
- `/pylon/BooshPulseTrain` — 5× 50ms pulses (1.0)
- `/pylon/BooshSteam` — steam sequence start/stop (1.0/0.0)
