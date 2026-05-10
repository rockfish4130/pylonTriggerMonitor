# Remote Battery Endurance — ESP32-C3 Super Mini

## Hardware config

- Board: ESP32-C3 Super Mini
- Power: 2× AAA lithium primary (e.g. Energizer Ultimate L92) in series, direct to 3.3V pin — LDO bypassed
- Always-on red LED: hardwired to 3.3V rail (not GPIO-controlled); cannot be disabled in software

## Cell specs

| Parameter | Value |
|---|---|
| Fresh voltage | ~3.4 V (1.7 V/cell) |
| Nominal voltage | ~3.0 V (1.5 V/cell) |
| Usable cutoff | ~2.8 V (1.4 V/cell) — brownout before cells are fully flat |
| Effective capacity | ~1050–1100 mAh (vs. 1250 mAh datasheet to 1.0 V/cell cutoff) |

## Current budget

| Consumer | Current |
|---|---|
| ESP32-C3, ESP-NOW awake (beacon + RX) | ~28–30 mA |
| ESP32-C3, deep sleep | ~0.01 mA |
| ESP32-C3, boot/wake spike | ~350 mA peak (brief) |
| Always-on LED, ~1 kΩ resistor, nominal rail | ~1.0–1.4 mA avg |

**Active total (awake):** ~30 mA  
**Idle total (deep sleep):** ~1.2 mA (LED dominates)

## Duty cycle scenarios

### Scenario A — Light use (picked up briefly, mostly sleeping)
5 min active / 55 min deep sleep, repeating

| Phase | Duration | Current | Contribution |
|---|---|---|---|
| Active | 17% (5/60 min) | 30 mA | 5.0 mA |
| Deep sleep | 83% (55/60 min) | 1.2 mA | 1.0 mA |
| **Weighted average** | | | **~6.0 mA** |

**Runtime: ~1100 / 6.0 ≈ 183 h — ~7.5 days**

---

### Scenario B — Moderate use (picked up every 30 min, used for 5 min)
5 min active / 25 min deep sleep, repeating

| Phase | Duration | Current | Contribution |
|---|---|---|---|
| Active | 17% (5/30 min) | 30 mA | 5.0 mA |
| Deep sleep | 83% (25/30 min) | 1.2 mA | 1.0 mA |
| **Weighted average** | | | **~6.0 mA** |

**Runtime: ~1100 / 6.0 ≈ 183 h — ~7.5 days**

> Note: the 5/25 and 5/55 cycles give nearly identical averages because the active fraction (17%)
> is the same in both cases. What matters is the ratio, not the absolute period length.

---

### Scenario C — Heavy use (picked up every 15 min, used for 5 min)
5 min active / 10 min deep sleep, repeating

| Phase | Duration | Current | Contribution |
|---|---|---|---|
| Active | 33% (5/15 min) | 30 mA | 10.0 mA |
| Deep sleep | 67% (10/15 min) | 1.2 mA | 0.8 mA |
| **Weighted average** | | | **~10.8 mA** |

**Runtime: ~1100 / 10.8 ≈ 102 h — ~4 days**

---

### Scenario D — Always-on, no deep sleep (firmware never sleeps)
Worst case; chip stays awake indefinitely between uses

| Phase | Duration | Current | Contribution |
|---|---|---|---|
| Always on | 100% | 31.2 mA | 31.2 mA |
| **Weighted average** | | | **~31 mA** |

**Runtime: ~1100 / 31 ≈ 35 h — ~1.5 days**

---

### Scenario summary

| Duty cycle | Avg current | Runtime |
|---|---|---|
| Light (5 on / 55 sleep) | ~6.0 mA | **~7.5 days** |
| Moderate (5 on / 25 sleep) | ~6.0 mA | **~7.5 days** |
| Heavy (5 on / 10 sleep) | ~10.8 mA | **~4 days** |
| Always-on, no sleep | ~31 mA | **~1.5 days** |

Deep sleep between uses is the dominant factor. With it, a set of cells comfortably covers a
multi-day festival. Without it, plan to swap cells mid-event.

## Gotchas

### Peak-current brownout at low battery

ESP-NOW TX bursts draw ~100–150 mA. Two AAA cells in series have ~300–600 mΩ combined internal
resistance. Near end of life (2.8 V resting), a 150 mA burst sags the rail to ~2.7 V — at or
below the ESP32-C3 brownout detector threshold (~2.44 V chip default, but configurable). The node
may start reboot-looping with cells that still have charge. Boot/wake spikes (~350 mA, brief) can
trigger the same symptom on first power-on with weak cells.

**Symptom:** device appears dead or stuck in a boot loop. Try fresh cells before diagnosing
firmware or hardware faults.

### LED cannot be turned off in software

The always-on red LED is wired directly to the 3.3V rail, not to a GPIO. Deep sleep saves the
chip's ~30 mA draw but the LED continues burning ~1.2 mA regardless of sleep state. This sets a
hard floor on idle current and is a design constraint, not a firmware bug.

### No low-battery warning

There is no ADC monitoring of the cell voltage or any low-battery indicator. The device transitions
from normal operation directly to brownout-rebooting with no user-visible warning. Users will
experience this as the remote becoming "flaky" before it stops working.

### Temperature

Lithium primaries (Li-FeS2) retain good capacity in cold vs. alkaline, but still lose ~20–30% at
or below freezing. For outdoor events in cold weather, reduce runtime estimates accordingly and
carry spare cells.

### Cell mismatch in series

The two cells age at slightly different rates. The weaker cell will be over-discharged by the
stronger one near end of life. Use matched cells from the same pack; replace both cells at the same
time rather than mixing old and new.
