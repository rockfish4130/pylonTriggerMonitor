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
3. If a post fails, firmware retries with exponential backoff capped to 30 seconds.
4. Posting uses short HTTP timeout to avoid disrupting OSC receive loop.

## Payload

Every announce/heartbeat currently includes:

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
  "ttl_sec": 30
}
```

## Config Knobs (`src/main.cpp`)

- `kPylonId`
- `kPylonDescription`
- `kPylonMdnsHost`
- `kRegistryBaseUrlPrimary`
- `kRegistryBaseUrlFallback`
- `kRegistryAnnouncePath`
- `kRegistryHeartbeatPath`
- `kRegistryTtlSec`
- `kRegistryHeartbeatIntervalMs`
- `kRegistryHttpTimeoutMs`
- `kFirmwareVersion`

## Validation

Serial output should show:
- `[REG] announce ... -> 2xx` shortly after Wi-Fi connect
- `[REG] heartbeat ... -> 2xx` roughly every 10s
- `[REG] post failed, retry in ...` when endpoint is unavailable
