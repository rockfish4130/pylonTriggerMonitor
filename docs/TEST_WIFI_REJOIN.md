# WiFi Rejoin Test — Procedure, Results, and Capabilities

## Purpose

Verify that PYLONs running the WiFi-probe firmware correctly rejoin the WAP (LavaLounge) after
a WAP outage, even when auto-AP mode was activated at boot.

---

## Background / What Is Being Tested

Prior to commit `838d8cd`, a PYLON that booted while the WAP was unavailable would:
1. Enable auto-AP (`ap_auto_enabled = true`) on `cfg_mesh_ch` (ch 11)
2. Never rejoin WiFi — the active AP pins the radio to ch 11, so `WiFi.reconnect()` cannot
   scan other channels, and the 10-min auto-reboot is suppressed by `ap_active`

The fix (`838d8cd`) adds a **probe sequence**:
- When a non-barmode PYLON has been offline long enough to trigger a reconnect attempt and
  `ap_auto_enabled` is true, it temporarily stops the AP, calls `WiFi.begin()`, and waits up to
  15 s for a connection
- On success: `ap_auto_enabled` is cleared; node stays on WiFi
- On timeout: AP is restored on `cfg_mesh_ch`; node remains mesh-only until next attempt

---

## Test Performed — 2026-05-10

### Firmware Under Test

| Commit | Description |
|---|---|
| `838d8cd` | WiFi probe introduced (core fix) |
| `63514f4` | Mesh panel hw/cfg channel display |
| `8120680` | Barmode probe guard (BARBAR AP preservation) |

All nodes were OTA'd to `8120680` before/after the test. The test run itself used `838d8cd`
(barmode guard not yet present — BARBAR AP regression found during this test, see below).

### Nodes Participating

| Node | Action during test | WiFi at test start |
|---|---|---|
| FIRE-PYLON-BARBAR | Reset while WAP down | LavaLounge |
| FIRE-PYLON-REDTEST | Reset while WAP down | LavaLounge |
| FIRE-PYLON-YELLOWTEST | Reset while WAP down | LavaLounge |
| FIRE-PYLON-TIKI0 | Reset while WAP down | LavaLounge |
| FIRE-PYLON-TIKI1 | Not reset (old firmware, excluded) | LavaLounge |

### Procedure

1. All probe-capable nodes confirmed online and on LavaLounge via registry
2. WAP (LavaLounge) powered off
3. BARBAR, REDTEST, YELLOWTEST, TIKI0 manually reset (power cycle) while WAP was down
   — nodes boot, fail WiFi, enter auto-AP + mesh-only mode
4. WAP powered back on (~2 min startup)
5. Claude monitored registry via background polling (2-min interval) and node logs via curl

### Results

- All 4 rebooted nodes showed the probe sequence in `/api/logs` at **~583 s offline**
  (matches the 10-min reconnect interval minus mesh-peer-live suppression offset)
- All 4 connected to LavaLounge successfully; `ap_auto_enabled` cleared
- `ap=false` in registry heartbeat confirmed auto-AP correctly disabled post-rejoin

**PASS** for the core probe mechanism.

### Bug Found During Test

BARBAR's always-on AP was disabled by the probe code and never restored after rejoining WiFi.
Root cause: `ap_auto_enabled = true` is set unconditionally at boot on WiFi failure — the same
flag the probe checks — even for barmode nodes whose AP is user-requested, not auto.

Fix: added `&& !barmode_active` guard to probe condition (`8120680`).

---

## Test Capabilities — What Claude Can Do

During a structured hardware/firmware test, Claude can perform the following without prompting:

| Capability | How |
|---|---|
| Poll registry for node presence / heartbeat fields | `curl http://rpiboosh.local:5000/api/nodes` |
| Poll individual node telemetry | `curl http://<ip>/api/telemetry` |
| Fetch node console log | `curl http://<ip>/api/logs` |
| Scan subnet for live nodes | curl sweep of 192.168.4.1–60 on port 80 |
| Background polling loop | Bash background task writing timestamped output |
| Build firmware | `/c/Users/Matt/.platformio/penv/Scripts/pio.exe run -e lolin_s2_pico` |
| OTA firmware to any/all nodes in parallel | `curl -X POST http://<ip>/api/ota -F firmware=@...` |
| Reconnect this PC to LavaLounge WiFi | `netsh wlan connect name=LavaLounge` |
| Reboot a node via API | `curl -X POST http://<ip>/api/reboot` |
| Push NVS config to a node | `curl -X POST http://<ip>/api/config -d ...` |
| Commit and push after test | `git add / commit / push` |
| Evaluate pass/fail from log patterns | grep probe/connect strings in `/api/logs` output |

Claude **cannot**:
- Power hardware on or off
- Press physical reset buttons
- Observe LEDs or OLED display
- Know exact timing of physical events without being told

---

## Manual Steps — What the User Provides

| Step | Notes |
|---|---|
| Power WAP on / off | Tell Claude immediately when done — timing matters |
| Reset / power-cycle individual nodes | Specify which nodes and approximately when |
| Confirm WAP startup complete | WAP typically takes ~2 min to be joinable |
| Observe OLED / LED state | Describe or photograph if relevant to the test |
| Report unexpected physical observations | e.g. a node's LED not responding, OLED frozen |

**Timing protocol:** Tell Claude the moment each manual action occurs (WAP off, node reset, WAP
on). Claude logs receipt time as a proxy for event time and uses it to interpret probe intervals
in node logs.

---

## Re-running This Test

Recommended sequence:

1. Confirm all nodes online and on LavaLounge: Claude polls registry
2. Claude starts background polling task (2-min interval)
3. Power WAP off → tell Claude
4. Reset target nodes (not BARBAR unless specifically testing barmode) → tell Claude which ones
5. Wait ~5 min (probe fires at the reconnect interval, ~10 min from last live-peer contact)
6. Power WAP on → tell Claude
7. Claude polls logs on each node to confirm probe sequence and rejoin
8. Claude reports pass/fail per node

For a faster iteration (skip the 10-min wait): reduce `cfg_wifi_reconnect_s` to 60 s via web UI
before the test, restore to 600 s after.
