# Ideas

Speculative features and directions — not committed to the roadmap.

## Choreography Protocol

Define a lightweight timing protocol so RPIBOOSH can schedule fire events at specific offsets across multiple pylons without relying on round-trip HTTP latency. Options:

- Relative offset from a shared NTP timestamp
- Sequence packets with pylon-local delay fields
- OSC bundles with timetag

## Sound Trigger Integration

Each pylon could listen for audio amplitude peaks (via an onboard mic or line-in ADC) and self-trigger boosh on beat detection, with RPIBOOSH setting sensitivity and cooldown parameters.

## MQTT Telemetry

Publish sensor readings and boosh events to an MQTT broker for real-time dashboards or integration with home-automation systems at the venue.

## Multi-Solenoid Output

Current hardware drives one solenoid via IO11. A board revision with a relay array or high-side driver could support 2–4 solenoid channels, selectable by OSC argument.

## Battery Chemistry Config

Make `kBatteryVoltFull` and `kBatteryVoltEmpty` configurable at runtime via NVS so a pylon can be reconfigured for LiFePO4 or Li-Ion without a firmware rebuild.
