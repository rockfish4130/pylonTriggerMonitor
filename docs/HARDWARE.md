# Hardware

## Board

**WEMOS / LOLIN S2 Pico** (ESP32-S2)

- MCU: Espressif ESP32-S2FNR2, 240 MHz Xtensa LX7
- Flash: 4 MB embedded
- PSRAM: 2 MB embedded
- USB: Native USB-C (USB CDC, no external UART chip)
- Built-in: SSD1306 128×32 OLED, LiPo charger, user button (GPIO0)
- Official reference: https://www.wemos.cc/en/latest/s2/s2_pico.html

## GPIO Pinout

| GPIO | Direction | Function |
|------|-----------|----------|
| 0 | INPUT_PULLUP | Dev-board BOOT/user button — boosh trigger local test |
| 3 | INPUT (ADC) | Battery voltage sense — R5/R8 voltage divider output |
| 4 | INPUT (ADC) | Thermistor temperature — NTC divider output |
| 8 | I2C SDA | OLED SDA (built-in) |
| 9 | I2C SCL | OLED SCL (built-in) |
| 11 | OUTPUT | HIGH when boosh is active (solenoid-state signal) |
| 12 | OUTPUT | White LED — 1 Hz square wave boosh indicator |
| 13 | OUTPUT (LEDC ch2) | Yellow LED — sine-wave brightness ~1.2 Hz, 0–33% |
| 14 | OUTPUT (LEDC ch1) | Blue LED — sine-wave brightness ~0.8 Hz, 0–33% |
| 15 | OUTPUT (LEDC ch0) | Green LED — sine-wave brightness ~0.4 Hz, 0–33% |
| 18 | OUTPUT | OLED reset (built-in) |
| 38 | OUTPUT | HIGH when Wi-Fi STA connected |

ADC resolution: 12-bit (0–4095), reference 3.3 V.

## Schematic Files

KiCad source in `schematic_electrical/`:

| File | Page |
|------|------|
| `ESP32_MCU.kicad_sch` | ESP32-S2 MCU connections, LED outputs, status IOs |
| `POWER.kicad_sch` | Battery sense, thermistor sense, power rails |
| `CONNECTORS.kicad_sch` | External connector pinout |

## Battery Voltage Sense Circuit

```
BATTERY_SENSE_12V
        │
       R5 (100 kΩ)
        │
        ├──── C2 (100 nF to GND, filter cap)
        │
       R8 (22 kΩ)
        │
       GND

Junction (R5/R8) ──── R4 (10 kΩ) ──── IO3 (ADC)
```

**Theoretical scale factor:** `(100k + 22k) / 22k = 5.545×`

At full-scale ADC (3.3 V on IO3): `3.3 × 5.545 = 18.3 V` measurable max.

**ADC calibration:** The ESP32-S2 ADC effective Vref is ~2.9 V, not 3.3 V, causing raw counts to read ~23% high. An empirical correction factor `kAdcCalFactor = 0.810` is applied to both battery and thermistor readings. This was derived from measured data:

| Actual (V) | Reported before cal (V) | Ratio |
|------------|------------------------|-------|
| 10.0 | 12.32 | 1.232 |
| 11.0 | 13.54 | 1.231 |
| 12.0 | 14.80 | 1.233 |
| 13.0 | 16.09 | 1.238 |
| 14.0 | 17.35 | 1.239 |

Mean overcounting factor: 1.234 → `kAdcCalFactor = 1/1.234 = 0.810`

Calibration constants in `main.cpp`:
```cpp
constexpr float kAdcCalFactor     = 0.810f;  // empirical ESP32-S2 ADC Vref correction
constexpr float kBatteryVoltFull  = 12.7f;   // 100% — SLA fully charged
constexpr float kBatteryVoltEmpty = 10.5f;   // 0%   — SLA discharged
```
Adjust `kBatteryVoltFull`/`kBatteryVoltEmpty` if using a different chemistry (LiFePO4, Li-Ion, etc.).

## Thermistor Circuit

```
+3.3V ──── THERMO_SENSOR_P ──── [NTC thermistor] ──── THERMO_SENSOR_N
                                                              │
                                                    ├── C3 (100 nF to GND)
                                                    │
                                                   R12 (10 kΩ to GND)
                                                    │
                                                   GND

THERMO_SENSOR_N junction ──── R13 (10 kΩ) ──── IO4 (ADC)
```

Thermistor is the upper element; R12 is the pull-down. ADC reads the divided voltage.

Resistance formula: `Rth = R12 × (4095 / ADC_raw − 1)`

Steinhart-Hart coefficients (empirically calibrated, shared with `boosh_box_esp32_remote_thermo`):
```cpp
c1 = 1.274219988e-03
c2 = 2.171368266e-04
c3 = 1.119659695e-07
manual_offset = +11.17 °F
```

## LED Drive Circuit

Each status LED (IO12–15) is driven directly from the ESP32 GPIO through a 1 kΩ current-limiting resistor.

At 3.3 V with 1 kΩ: ~2 mA forward current. Functional but dim. Dropping to 330 Ω gives ~6 mA for better visibility while remaining within the 40 mA GPIO current limit.

## PWM

Status LEDs use the ESP32-S2 LEDC peripheral:

| Channel | Pin | Carrier freq | Signal |
|---------|-----|-------------|--------|
| ch0 | IO15 (green) | 5 kHz | Sine envelope 0.4 Hz, 0–33% duty |
| ch1 | IO14 (blue) | 5 kHz | Sine envelope 0.8 Hz, 0–33% duty |
| ch2 | IO13 (yellow) | 5 kHz | Sine envelope 1.2 Hz, 0–33% duty |

IO12 (white) uses `millis()`-based `digitalWrite` for a simple 1 Hz square wave — LEDC at sub-1 Hz carrier was unreliable on this hardware revision.
