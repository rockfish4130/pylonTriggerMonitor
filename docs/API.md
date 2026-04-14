# API Reference

## OSC

**Transport:** UDP  
**Port:** 8000  
**Address:** `/pylon/BooshMain`

| Argument | Type | Meaning |
|----------|------|---------|
| `1.0` | float | Boosh ON |
| `0.0` | float | Boosh OFF |

The message must have exactly one float argument. Any other shape is ignored.

**Example** (raw hex, ON):
```
2F 70 79 6C 6F 6E 2F 42 6F 6F 73 68 4D 61 69 6E 00 00 2C 66 00 00 00 00 3F 80 00 00
```

**Example** (raw hex, OFF):
```
2F 70 79 6C 6F 6E 2F 42 6F 6F 73 68 4D 61 69 6E 00 00 2C 66 00 00 00 00 00 00 00 00
```

---

## HTTP REST

All endpoints are on port 80. Responses are JSON unless noted.

### `GET /`

Returns the web UI (HTML).

---

### `GET /api/telemetry`

Returns full device telemetry.

**Response fields:**

| Field | Type | Description |
|-------|------|-------------|
| `pylon_id` | string | Node identifier |
| `description` | string | Human-readable node description |
| `hostname` | string | mDNS hostname (`.local`) |
| `ip` | string | Current IPv4 address |
| `fw_version` | string | Firmware build string |
| `fw_semver` | string | Semantic version (e.g. `0.0.1`) |
| `fw_build_date` | string | Compile date |
| `fw_build_time` | string | Compile time |
| `uptime` | string | Uptime as `HH:MM:SS` |
| `uptime_hms` | string | Alias for `uptime` |
| `solenoid_active` | bool | `true` when boosh is firing |
| `trigger_event_count` | int | Total boosh trigger count since boot |
| `ap_enabled` | bool | AP mode enabled in config |
| `ap_active` | bool | AP currently running |
| `target_ip` | string | Resolved IP of RPIBOOSH |
| `battery_voltage_v` | float\|null | Battery voltage in volts |
| `battery_charge_pct` | float\|null | Battery charge 0–100% |
| `battery_time_remaining_h` | float\|null | Estimated hours remaining (null until 5+ samples) |
| `temperature_f` | float\|null | Enclosure temperature in °F |
| `telemetry` | object | Nested telemetry for registry compatibility (see [PYLON_REGISTRY.md](PYLON_REGISTRY.md)) |

---

### `GET /api/logs`

Returns recent serial log output as seen in the web UI console.

**Response:**
```json
{ "log": "...\n...\n" }
```

---

### `GET /api/config`

Returns current node configuration.

**Response fields:**

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | Node ID (e.g. `PYLON5`) |
| `host` | string | mDNS host (e.g. `PYLON5`) |
| `hostname` | string | Full hostname (e.g. `PYLON5.local`) |
| `description` | string | Node description |
| `ap_enabled` | bool | AP mode enabled |
| `ap_active` | bool | AP currently active |
| `wifi_ssid` | string | User-defined fallback SSID (empty if not set) |

---

### `POST /api/config`

Updates one or more config fields. Accepts `application/x-www-form-urlencoded` or JSON form args.

**Form args (all optional):**

| Arg | Description |
|-----|-------------|
| `node` | Sets both `id` and `host` to the same value |
| `id` | Node ID |
| `host` | mDNS host |
| `description` or `desc` | Node description |
| `wifi_ssid` | User fallback SSID |
| `wifi_pass` | User fallback password |
| `ap_enabled` | `true`/`false`/`1`/`0` |

Changes are persisted to NVS immediately.

---

### `POST /api/config/id`

Sets node ID from form arg `value`.

### `POST /api/config/host`

Sets mDNS host from form arg `value`.

### `POST /api/config/desc`

Sets description from form arg `value`.

### `POST /api/config/node`

Sets both ID and host from form arg `value`.

### `POST /api/config/ap`

Enables or disables AP mode. Form arg `value`: `true`/`false`.

---

### `POST /api/solenoid/on`

Activates boosh (solenoid ON). Equivalent to OSC `1.0`. The 5-second failsafe applies.

**Response:**
```json
{ "ok": true, "solenoid_active": true, "triggered_via": "http" }
```

### `POST /api/solenoid/off`

Deactivates boosh (solenoid OFF).

**Response:**
```json
{ "ok": true, "solenoid_active": false, "triggered_via": "http" }
```

### `POST /api/solenoid/trigger`

Alias for `/api/solenoid/on`.

---

## Boosh Failsafe

If boosh is activated (via any method) and OFF is not received within **5 seconds**, firmware forces it OFF automatically. Serial log: `Failsafe: BooshMain timeout -> forcing OFF.`
