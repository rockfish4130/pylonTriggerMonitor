# PYLON Registry Integration

This firmware integrates with the RPIBOOSH PYLON registry API so multiple ESP32 PYLON nodes can be discovered dynamically.

## Endpoints

Base URL defaults:
- Primary: `http://rpiboosh.local:5000`
- Optional fallback: compile-time configurable fixed-IP URL in `src/main.cpp`

Paths:
- `POST /api/pylons/announce`
- `POST /api/pylons/heartbeat`

## Runtime Behavior

1. On Wi-Fi connect (boot or reconnect), firmware posts an `announce`.
2. After a successful announce, firmware posts `heartbeat` every 10 seconds.
3. If ping to `RPIBOOSH` fails and later recovers, firmware posts a fresh `announce`.
4. If a post fails, firmware retries with exponential backoff capped to 30 seconds.
5. Posting uses short HTTP timeout to avoid disrupting OSC receive loop.

## Payload

This payload shape is the compatibility baseline for deployed legacy nodes.

Compatibility rules:
- Treat payloads with no explicit schema/protocol field as `registry.telemetry.v1` (legacy baseline).
- Newer firmware may add optional top-level fields or optional nested fields under `telemetry`.
- Receivers must ignore unknown fields.
- Existing field names, meanings, and types in the baseline payload must remain stable for compatibility.
- Any future breaking change should use an explicit schema/protocol marker instead of silently changing baseline fields.

Every announce/heartbeat in the current baseline includes:

```json
{
  "pylon_id": "PYLON0",
  "description": "Center striker",
  "hostname": "PYLON0.local",
  "ip": "192.168.1.70",
  "osc_port": 8000,
  "osc_paths": ["/rpiboosh/BooshMain"],
  "roles": ["boosh_main"],
  "fw_version": "pylons ...",
  "ttl_sec": 30,
  "telemetry": {
    "ipv4": "192.168.1.70",
    "mdns_hostname": "PYLON0.local",
    "fw_version": "pylons ...",
    "temperature": "N/A",
    "temperature_f": "N/A",
    "temperature_c": "N/A",
    "battery_voltage": "N/A",
    "battery_voltage_v": "N/A",
    "battery_charge": "N/A",
    "battery_charge_pct": "N/A",
    "wifi_rssi_dbm": -58,
    "uptime_s": 9123,
    "ping_target": "RPIBOOSH",
    "ping": {
      "target": "RPIBOOSH",
      "sent": 151,
      "recv": 143,
      "lost": 8,
      "last_ms": 7,
      "min_ms": 2,
      "max_ms": 17,
      "avg_ms": 6,
      "count": 143,
      "last_ok": true
    }
  }
}
```

## Baseline Field Contract

Legacy-compatible consumers should assume these fields exist and keep their current meaning:
- `pylon_id`: stable node identifier string
- `description`: human-readable node description string
- `hostname`: mDNS hostname string ending in `.local`
- `ip`: current IPv4 string
- `osc_port`: integer OSC UDP port
- `osc_paths`: array of supported OSC path strings
- `roles`: array of node role strings
- `fw_version`: firmware build/version string
- `ttl_sec`: integer registry TTL in seconds
- `telemetry.ipv4`: current IPv4 string duplicated under telemetry for legacy compatibility
- `telemetry.mdns_hostname`: current `.local` hostname duplicated under telemetry for legacy compatibility
- `telemetry.fw_version`: firmware build/version string duplicated under telemetry for legacy compatibility
- `telemetry.temperature`: generic temperature placeholder/value for receiver compatibility with newer High Striker-style consumers
- `telemetry.temperature_f`: string placeholder used by older consumers when no temperature sensor is present
- `telemetry.temperature_c`: optional Celsius alias kept as a string placeholder/value for forward compatibility
- `telemetry.battery_voltage`: generic battery-voltage placeholder/value for receiver compatibility
- `telemetry.battery_voltage_v`: voltage alias kept for explicit-unit consumers
- `telemetry.battery_charge`: generic battery charge placeholder/value for receiver compatibility
- `telemetry.battery_charge_pct`: percent alias kept for explicit-unit consumers
- `telemetry.wifi_rssi_dbm`: integer RSSI in dBm
- `telemetry.ping.target`: ping target string for older consumers that read nested ping metadata
- `telemetry.ping.sent`: number of ping attempts since boot
- `telemetry.ping.recv`: number of successful ping replies since boot
- `telemetry.ping.lost`: number of failed ping attempts since boot
- `telemetry.ping_target`: ping target hostname string
- `telemetry.ping.last_ms`: last measured round-trip time in ms, `0` before samples exist
- `telemetry.ping.min_ms`: minimum measured round-trip time in ms, `0` before samples exist
- `telemetry.ping.max_ms`: maximum measured round-trip time in ms, `0` before samples exist
- `telemetry.ping.avg_ms`: average measured round-trip time in ms, `0` before samples exist
- `telemetry.ping.count`: number of ping samples included in the aggregate
- `telemetry.ping.last_ok`: `true` when the most recent ping succeeded
- `telemetry.uptime_s`: device uptime in seconds

## Forward Compatibility

If newer firmware reports additional telemetry:
- Add fields, do not rename or repurpose baseline fields.
- Keep new fields optional from the receiver's point of view.
- Prefer nesting new measurements inside `telemetry` so older consumers can ignore them cleanly.
- If receivers need feature-gating, use `fw_version` for heuristics only as a temporary bridge; add an explicit schema/protocol field for any non-additive evolution.

## Config Knobs (`src/main.cpp`)

- `kPylonIdDefaultPrefix`
- `kPylonDescriptionDefault`
- `kRegistryBaseUrlPrimary`
- `kRegistryBaseUrlFallback`
- `kRegistryAnnouncePath`
- `kRegistryHeartbeatPath`
- `kRegistryTtlSec`
- `kRegistryHeartbeatIntervalMs`
- `kRegistryHttpTimeoutMs`
- `kFirmwareVersion`

Runtime identity (`id`, `host`, `desc`) is persisted in NVS and can be changed via serial CLI:
- `set id <value>`
- `set host <value>`
- `set desc <value>`
- `set node <value>`
- `clear nvs`

The same persisted config can also be queried or updated over HTTP:
- `GET /api/config`
- `POST /api/config`
- `POST /api/config/id`
- `POST /api/config/host`
- `POST /api/config/desc`
- `POST /api/config/node`

## Validation

Serial output should show:
- `[REG] announce ... -> 2xx` shortly after Wi-Fi connect
- `[REG] heartbeat ... -> 2xx` roughly every 10s
- `[REG] post failed, retry in ...` when endpoint is unavailable
