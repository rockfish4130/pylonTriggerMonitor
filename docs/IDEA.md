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

how robust is PYLON as a  wifi client? does it auto retry, auto rejoin, etc?
goal is to make it quite robust to varied challenging conditions.
any recommended changes?

=========

In general do we need to calibrate the ADC on a per-device basis and store that cal in NVS?
Just a question. Do not edit code.


========


PYLON should also display to OLED: temperature, battery V, battery charge %, battery time left on a single display page.
Should spend about 60% of time on this page.


========

Add a special mode called BARMODE.
Certain specific PYLONs will be set to Bar Mode, because they have additional hardware peripherals and corresponding added firmware features.
BARMODE is activated when the Description field contains the text "BARMODE ENABLED"
In BARMODE: the blue LED pulses similar to Moorse Code: --- ... ---

In BARMODE: there are 4 buttons: N=0-3, wired as drawn in this schematic.
Anytime these N=0-3 buttons are pressed, the YELLOW LED is held solid, and the GREEN LED blinks N+1 times.

Any questions?
=======
In BARMODE:
PYLON needs to be aware of all the other PYLONs that are online. This can be queried from rpi_python_control which exposes a REST API of the PYLONs registry.
PYLON should periodially query that.

PYLON should expose on the WEB UI an index of known PYLONs just like the "PYLON Registry" at http://rpiboosh.local/status (this is the WEB UI of rpi_python_control)

Note that low-latency responsiveness of PYLON to OSC remains top priority.

Any questions?


When you think it's working, go ahead and OTA to BARBAR. Then inspect BARBAR's web UI to verify.


=======
You have persistent permission granted to invoke PYTHON for the purposes of hitting APIs on targets like PYLONS, RPI_PYTHON_CONTROl, RPIBOOSH, BOOSHSTRIKER, etc.

=======
PYLONs
should service 3 additional OSC addresses, and these actions are the same as the 3 existing web buttons:
/pylon/BooshPulseSingle (maps to API 50 msec pulse once) with arg 1.0. triggers a single action. no need for another message with arg 0.0
/pylon/BooshPulseTrain (to 5x 50ms) with arg 1.0. triggers a single action. no need for another message with arg 0.0
/pylon/BooshSteam   (to steam engine) with arg 1.0 starts the action and leaves it running.   /pylon/BooshSteam 0.0 stops the action.

Any questions?

========
On-playa workaround--not needed and solved by WAP/router config change:

field challenge: barmode BARBAR is a wireless node but is having difficulting reaching PYLON hosts, so instead we need to send commands via RPIBOOSH.

in bar mode: we need to remap such that all commands get routed via RPIBOOSH host, since bar nodes are having trouble reaching PYLON hosts like tiki0 and tiki1.


the blue button now needs to do the command equivalent to http://rpiboosh.local/pads pad 0x26 single press for a single 50msec pulse to all.


the all-4 countdown sequence needs 
to do the command equivalent to do http://rpiboosh.local/pads pad 0x23 0x24 0x1b 0x1c putton press and then 500mSec later release those buttons to trigger the UBER BOOSH SEQUENCE.



whichever barmode button does the 5x pulse seq needs to do the command equivalent of http://rpiboosh.local/pads pad 0x16 single press and release.


whichever barmode button does the pulse chaser needs to do the command equivalent of http://rpiboosh.local/pads pad 0x1e press and then release.


whichever barmode button does the steam train needs to do the command equivalent of http://rpiboosh.local/pads pad 0xe press and then release.



then tell me the exact path of the FW binary as I will have to manually upload it.

any questions?