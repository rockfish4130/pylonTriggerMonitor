# PYLON Registry Integration

The pylon announces itself to RPIBOOSH on Wi-Fi connect and sends periodic heartbeats. RPIBOOSH uses these to build and maintain a live registry of active nodes.

## Endpoints

Base URL: `http://rpiboosh.local:5000` (compile-time `kRegistryBaseUrlPrimary`)  
Optional fixed-IP fallback: `kRegistryBaseUrlFallback` in `src/main.cpp`

| Method | Path | When |
|--------|------|------|
| `POST` | `/api/pylons/announce` | On Wi-Fi connect or reconnect |
| `POST` | `/api/pylons/heartbeat` | Every 10 seconds after announce |

## Runtime Behavior

1. On Wi-Fi connect, firmware posts an `announce`.
2. After a successful announce, posts `heartbeat` every `kRegistryHeartbeatIntervalMs` (10 s).
3. If ping to RPIBOOSH fails and later recovers, a fresh `announce` is posted.
4. On post failure, retries with exponential backoff capped to 30 s.
5. HTTP timeout: `kRegistryHttpTimeoutMs` (2.5 s) to avoid blocking the main loop.

Serial log prefixes: `[REG] announce`, `[REG] heartbeat`, `[REG] post failed`.

## Payload

Both `announce` and `heartbeat` send the same JSON body:

```json
{
  "pylon_id": "PYLON5",
  "description": "Center striker",
  "hostname": "PYLON5.local",
  "ip": "192.168.1.70",
  "osc_port": 8000,
  "osc_paths": ["/pylon/BooshMain"],
  "roles": ["boosh_main"],
  "fw_version": "0.0.1 Apr 13 2026 12:00:00",
  "firmware_version": "0.0.1 Apr 13 2026 12:00:00",
  "version": "0.0.1 Apr 13 2026 12:00:00",
  "ttl_sec": 30,
  "telemetry": {
    "ipv4": "192.168.1.70",
    "mdns_hostname": "PYLON5.local",
    "fw_version": "0.0.1 Apr 13 2026 12:00:00",
    "fw_semver": "0.0.1",
    "fw_build_date": "Apr 13 2026",
    "fw_build_time": "12:00:00",
    "temperature": 72.5,
    "temperature_f": 72.5,
    "temperature_c": 22.5,
    "battery_voltage": 12.4,
    "battery_voltage_v": 12.4,
    "battery_charge": 86.4,
    "battery_charge_pct": 86.4,
    "battery_time_remaining_h": 4.2,
    "wifi_rssi_dbm": -58,
    "uptime_s": 9123,
    "uptime": "02:32:03",
    "uptime_hms": "02:32:03",
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
      "last_ok": true,
      "since_ok_s": 1,
      "since_ok": "00:00:01"
    }
  }
}
```

Sensor fields (`temperature`, `battery_voltage`, `battery_charge`, `battery_time_remaining_h`) are `null` JSON when the sensor has not yet returned a valid reading (e.g. shortly after boot, or if the ADC returns an out-of-range value).

## Field Contract

Receivers must ignore unknown fields. Field meanings are stable — do not rename or repurpose existing fields.

| Field | Type | Notes |
|-------|------|-------|
| `pylon_id` | string | Stable node identifier |
| `description` | string | Human-readable label |
| `hostname` | string | mDNS hostname ending in `.local` |
| `ip` | string | Current IPv4 |
| `osc_port` | int | UDP OSC port (8000) |
| `osc_paths` | string[] | Accepted OSC addresses |
| `roles` | string[] | Node capability roles |
| `fw_version` | string | Full build string |
| `ttl_sec` | int | Registry TTL |
| `telemetry.temperature` | float\|null | Temperature in °F |
| `telemetry.temperature_f` | float\|null | Temperature in °F |
| `telemetry.temperature_c` | float\|null | Temperature in °C |
| `telemetry.battery_voltage` | float\|null | Battery voltage (V) |
| `telemetry.battery_voltage_v` | float\|null | Alias |
| `telemetry.battery_charge` | float\|null | Battery charge (%) |
| `telemetry.battery_charge_pct` | float\|null | Alias |
| `telemetry.battery_time_remaining_h` | float\|null | Hours remaining (null until 5+ history samples) |
| `telemetry.wifi_rssi_dbm` | int | Wi-Fi RSSI |
| `telemetry.uptime_s` | int | Uptime in seconds |
| `telemetry.ping_target` | string | Ping target hostname |
| `telemetry.ping.sent` | int | Total ping attempts since boot |
| `telemetry.ping.recv` | int | Successful ping replies |
| `telemetry.ping.lost` | int | Failed attempts |
| `telemetry.ping.last_ms` | int | Last RTT in ms |
| `telemetry.ping.min_ms` | int | Min RTT in ms |
| `telemetry.ping.max_ms` | int | Max RTT in ms |
| `telemetry.ping.avg_ms` | int | Average RTT in ms |
| `telemetry.ping.last_ok` | bool | `true` if most recent ping succeeded |

## Forward Compatibility

- Add fields; do not rename or repurpose existing ones.
- New fields should be optional from the receiver's perspective.
- Nest new measurements under `telemetry` so older consumers ignore them cleanly.
- For non-additive evolution, add an explicit `schema` or `protocol` marker rather than silently changing baseline fields.
