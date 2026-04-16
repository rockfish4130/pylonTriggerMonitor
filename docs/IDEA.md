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


## Matt sandbox

push this new FW image OTA to 3 targets in parallel:
First query all 3 and tell me which need the OTA update:

PYLON5D90	PYLON5D90.local	192.168.4.38	Yes	0	No	2026-04-15 14:24:56
PYLON8668	PYLON8668.local	192.168.4.37	Yes	0	No	2026-04-15 14:24:56
PYLON866C	PYLON866C.local	192.168.4.39	Yes	0	No	2026-04-15 14:25:01



==========

how robust is the wifi client? does it auto retry, auto rejoin, etc?
goal is to make it quite robust to varied conditions.


=========

In general do we need to calibrate the ADC on a per-device basis and store that cal in NVS?
Just a question. Do not edit code.


========


PYLON should also display to OLED: temperature, battery V, battery charge %, battery time left on a single display page.
Should spend about 60% of time on this page.