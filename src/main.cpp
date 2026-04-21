#include <Arduino.h>
#include <Wire.h>
#include <USB.h>
#include "esp_task_wdt.h"
#include <DNSServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <Update.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include "wifi_credentials.h"

constexpr int kI2cSda = 8;
constexpr int kI2cScl = 9;
constexpr int kOledReset = 18;
constexpr int kDevBoardButtonPin = 0;
constexpr int kLedWhitePin = 12;   // IO12 boosh trigger LED - 1Hz
constexpr int kLedGreenPin = 15;   // IO15 green
constexpr int kLedBluePin = 13;    // IO13 blue  (swapped vs. original schematic — as built)
constexpr int kLedYellowPin = 14;  // IO14 yellow (swapped vs. original schematic — as built)
constexpr int kIo38Pin = 38;       // IO38 high = WiFi connected
constexpr int kIo11Pin = 11;       // IO11 high = boosh active
constexpr int kBatteryAdcPin = 3;  // IO3 ADC - battery voltage divider (R5=100k, R8=22k)
constexpr int kThermistorAdcPin = 4; // IO4 ADC - NTC thermistor (R12=10k pull-down, R13=10k series)
// Battery voltage divider: Vbat → R5(100k) → junction → R8(22k) → GND; junction → R4(10k) → IO3
constexpr float kBatteryDividerScale = (100000.0f + 22000.0f) / 22000.0f;  // ~5.545 theoretical
// ADC readings use analogReadMilliVolts() which applies the on-chip eFuse factory calibration
// automatically. kAdcCalFactor is no longer needed.
constexpr float kThermistorVccMv = 3300.0f;  // pull-up supply voltage in mV
constexpr float kBatteryVoltEmpty = 11.2f;  // 0%  (LiFePO4 4S discharged, used for time-remaining estimate)
// Steinhart-Hart coefficients for thermistor - from empirical calibration in boosh_box_esp32_remote_thermo
constexpr float kThermistorC1 = 1.274219988e-03f;
constexpr float kThermistorC2 = 2.171368266e-04f;
constexpr float kThermistorC3 = 1.119659695e-07f;
constexpr float kThermistorR1 = 10000.0f;  // R12 pull-down
// 2-point linear calibration on raw S-H output (applied after Steinhart-Hart).
// Derived from 2-point measurement on TIKI1 with analogReadMilliVolts() ADC:
//   cold: raw 36.63°F, reference 36.00°F
//   warm: raw 74.48°F, reference 73.00°F
// scale  = (73.0 - 36.0) / (74.48 - 36.63) = 37.0 / 37.85 = 0.9775
// offset = 36.0 - 36.63 * 0.9775 = +0.20°F
constexpr float kThermistorCalScale = 0.9775f;
constexpr float kThermistorCalOffsetF = 0.20f;
constexpr int kThermistorAvgSamples = 16;
constexpr unsigned long kSensorPollIntervalMs = 5000;
constexpr unsigned long kBatteryHistoryIntervalMs = 300000; // 5 min between battery history samples
constexpr int kBatteryHistorySize = 72;             // 72 points = 6 hours of history for rate calc
// Battery plot buffers (separate from the rate-estimation ring buffer above)
constexpr int kBattPlotLongSize = 1440;             // 24 h × 60 min/h = 1440 points
constexpr int kBattPlotShortSize = 180;             // 30 min × 2 pts/min (10 s interval)
constexpr unsigned long kBattPlotLongIntervalMs  = 60000;  // 1 min
constexpr unsigned long kBattPlotShortIntervalMs = 10000;  // 10 s
// Temperature plot buffers (same size/interval as battery)
constexpr int kTempPlotLongSize  = 1440;
constexpr int kTempPlotShortSize = 180;
constexpr unsigned long kTempPlotLongIntervalMs  = 60000;
constexpr unsigned long kTempPlotShortIntervalMs = 10000;
constexpr uint8_t kOledAddress = 0x3C;
constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 32;
constexpr uint16_t kOscPort = 8000;
constexpr const char *kOscAddress        = "/pylon/BooshMain";
constexpr const char *kOscAddrPulseSingle = "/pylon/BooshPulseSingle";
constexpr const char *kOscAddrPulseTrain  = "/pylon/BooshPulseTrain";
constexpr const char *kOscAddrSteam       = "/pylon/BooshSteam";
constexpr unsigned long kBooshFailsafeTimeoutMs = 5000;
constexpr unsigned long kBooshFailsafeNoteMs = 3000;
constexpr unsigned long kPingTimeoutMs = 5000;
constexpr unsigned long kDisplayCycleMs = 3000;
constexpr const char *kPingTargetHost = "RPIBOOSH";
constexpr const char *kPylonIdDefaultPrefix = "PYLON";
constexpr const char *kPylonDescriptionDefault = "unspecified";
constexpr const char *kRegistryBaseUrlPrimary = "http://rpiboosh.local:5000";
constexpr const char *kRegistryBaseUrlFallback = "";
constexpr const char *kRegistryAnnouncePath = "/api/pylons/announce";
constexpr const char *kRegistryHeartbeatPath = "/api/pylons/heartbeat";
constexpr uint16_t kRegistryTtlSec = 30;
constexpr unsigned long kRegistryHeartbeatIntervalMs = 10000;
constexpr uint16_t kRegistryHttpTimeoutMs = 2500;
constexpr const char *kFirmwareSemver = "0.0.3";
constexpr const char *kFirmwareBuildDate = __DATE__;
constexpr const char *kFirmwareBuildTime = __TIME__;
constexpr const char *kFirmwareVersion = "0.0.3 " __DATE__ " " __TIME__;
constexpr const char *kTelemetryTemperatureDefault = "N/A";
constexpr const char *kTelemetryBatteryVoltageDefault = "N/A";
constexpr const char *kTelemetryBatteryChargePercentDefault = "N/A";
constexpr size_t kWebLogBufferMaxChars = 8192;

// ---- Bar Mode ---------------------------------------------------------------
// Activated when pylon_description contains kBarModeToken.
// Features: SOS blue LED pattern; 4 buttons on IO1/IO2/IO5/IO6 that drive a
// yellow-solid + green-blink(N+1) feedback sequence.
constexpr const char *kBarModeToken = "BARMODE ENABLED";
constexpr int kBarModeButtonPins[4] = {1, 2, 5, 6};  // IO1, IO2, IO5, IO6 (active HIGH, INPUT_PULLDOWN)
// SOS Morse timing (unit = 150 ms): dot=1u, dash=3u, elem-gap=1u, letter-gap=3u, word-gap=7u
constexpr unsigned long kSosUnitMs = 150;
// Bar button blink sequence timing
constexpr unsigned long kBarBlinkOnMs  = 250;
constexpr unsigned long kBarBlinkOffMs = 250;
constexpr unsigned long kBarModeRegistryIntervalMs = 10000;  // poll rpiboosh pylon list every 10 s
constexpr unsigned long kBarModeBtn0ChaseMs = 100;  // ms per LED step in btn0 chase pattern

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);
WiFiUDP oscUdp;
WebServer webServer(80);
bool display_inverted = false;
bool web_server_started = false;
bool boosh_failsafe_armed = false;
unsigned long boosh_failsafe_start_ms = 0;
unsigned long boosh_failsafe_note_until_ms = 0;
unsigned long boosh_failsafe_timeout_ms = kBooshFailsafeTimeoutMs;  // runtime, persisted in NVS
int pylon_index = 0;  // sequencing index reported in telemetry; persisted in NVS; default 0
unsigned long barmode_seq_max_ms = 30000;  // btn1 double-tap seq max hold duration; BARMODE NVS; default 30s
unsigned long barmode_seq_dec_ms = 50;     // delay decrement per step in btn1 seq; BARMODE NVS; default 50ms
uint8_t barmode_seq_exp_pct = 100;         // exponential factor % (1-100) applied after linear dec; 100=off
unsigned long barmode_green_timeout_ms = 300; // btn0 timed pulse open duration; BARMODE NVS; default 300ms
unsigned long barmode_all4_valve_ms    = 3000; // all-4 hold: valve open duration before auto-close; BARMODE NVS; default 3s
uint32_t barmode_btn_counts[4]   = {0,0,0,0};   // running press counts: [green, blue, orange, red]
bool     barmode_btn_disabled[4] = {false,false,false,false}; // NVS-persisted disable flags
// Ring buffer: up to 1024 button press events (ms + btn index), oldest overwritten
constexpr int kBtnEventBufSize = 1024;
uint32_t barmode_btn_event_ms[kBtnEventBufSize];
uint8_t  barmode_btn_event_btn[kBtnEventBufSize];
int      barmode_btn_event_head  = 0;   // next write slot
int      barmode_btn_event_count = 0;   // events stored (0–kBtnEventBufSize)
unsigned long wifi_connected_since_ms = 0;
uint8_t last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
bool wifi_has_ip = false;
volatile bool registry_announced = false;
volatile unsigned long registry_last_success_ms = 0;
volatile unsigned long registry_next_attempt_ms = 0;
volatile uint8_t registry_consecutive_failures = 0;
String pylon_id;
String pylon_mdns_host;
String pylon_description;
String serial_cli_line;
volatile bool telemetry_ping_last_ok = false;
volatile bool telemetry_ping_has_data = false;
volatile uint32_t telemetry_ping_sent = 0;
volatile uint32_t telemetry_ping_lost = 0;
volatile uint32_t telemetry_ping_last_ms = 0;
volatile uint32_t telemetry_ping_min_ms = 0;
volatile uint32_t telemetry_ping_max_ms = 0;
volatile uint32_t telemetry_ping_avg_ms = 0;
volatile uint32_t telemetry_ping_count = 0;
uint32_t trigger_event_count = 0;
uint32_t total_boosh_open_ms = 0;       // cumulative valve-open time across all events
unsigned long boosh_open_since_ms = 0;  // millis() when current boosh event started
unsigned long identify_until_ms = 0;    // non-zero while identify mode is active
bool barmode_active = false;            // true when description contains kBarModeToken
bool barmode_seq_active = false;        // true while button blink sequence is playing
bool barmode_btn0_held = false;         // true while button 0 is held; drives chase LEDs + OLED invert
SemaphoreHandle_t barmode_registry_mutex = nullptr;
String barmode_registry_json;           // cached GET /api/pylons response; protected by barmode_registry_mutex
unsigned long barmode_registry_last_fetch_ms = 0;
volatile bool barmode_registry_fetch_now = false;  // set to trigger immediate PingTask fetch

// ---- Sequence state ---------------------------------------------------------
enum SeqType { SEQ_NONE, SEQ_PULSE_ONCE, SEQ_PULSE_5X, SEQ_STEAM };
SeqType active_seq = SEQ_NONE;
unsigned long seq_step_start_ms = 0;  // start of current on/off phase
unsigned long seq_start_ms = 0;       // start of whole sequence
int seq_pulse_idx = 0;                // pulses fired so far
bool seq_phase_on = false;            // currently in on-phase
bool seq_abort_flag = false;          // abort requested
volatile bool current_ping_last_ok = false;
volatile unsigned long last_ping_success_ms = 0;
String target_ip_string = "";
String web_log_text;
String web_log_partial_line;
bool ap_enabled = false;
bool ap_active = false;
DNSServer dnsServer;
String user_wifi_ssid;
String user_wifi_pass;
// Sensor state
float sensor_battery_v = NAN;
float sensor_battery_pct = NAN;
float sensor_temp_f = NAN;
float sensor_battery_time_remaining_h = NAN;
unsigned long sensor_last_poll_ms = 0;
// Battery voltage history for discharge-rate estimation
float battery_v_history[kBatteryHistorySize];
unsigned long battery_v_history_ms[kBatteryHistorySize];
int battery_v_history_count = 0;
int battery_v_history_head = 0;
unsigned long battery_last_history_ms = 0;
// Battery voltage plot buffers
struct BattPlotPoint { uint32_t ms; float v; };
BattPlotPoint battery_plot_long[kBattPlotLongSize];
BattPlotPoint battery_plot_short[kBattPlotShortSize];
int battery_plot_long_head = 0,  battery_plot_long_count  = 0;
int battery_plot_short_head = 0, battery_plot_short_count = 0;
unsigned long battery_plot_long_last_ms  = 0;
unsigned long battery_plot_short_last_ms = 0;
BattPlotPoint temp_plot_long[kTempPlotLongSize];
BattPlotPoint temp_plot_short[kTempPlotShortSize];
int temp_plot_long_head = 0,  temp_plot_long_count  = 0;
int temp_plot_short_head = 0, temp_plot_short_count = 0;
unsigned long temp_plot_long_last_ms  = 0;
unsigned long temp_plot_short_last_ms = 0;

constexpr const char *kPrefsNamespace = "pylon_cfg";
constexpr const char *kPrefsKeyId = "id";
constexpr const char *kPrefsKeyHost = "host";
constexpr const char *kPrefsKeyDesc = "desc";
constexpr const char *kPrefsKeyAp = "ap_en";
constexpr const char *kPrefsKeyUserSsid = "usr_ssid";
constexpr const char *kPrefsKeyUserPass = "usr_pass";
constexpr const char *kPrefsKeyFailsafeMs = "failsafe_ms";
constexpr const char *kPrefsKeyIndex = "pylon_idx";
constexpr const char *kPrefsKeySeqMaxMs = "seq_max_ms";
constexpr const char *kPrefsKeySeqDecMs = "seq_dec_ms";
constexpr const char *kPrefsKeySeqExpPct  = "seq_exp_pct";  // 1-100; applied as factor delay*=(pct/100)
constexpr const char *kPrefsKeyBtnDisable    = "btn_dis";      // uint8 bitmask: bit0=green,1=blue,2=orange,3=red
constexpr const char *kPrefsKeyGreenTimeout  = "grn_to_ms";   // uint32 ms; btn0 timed pulse duration
constexpr const char *kPrefsKeyAll4ValveMs   = "all4_vlv_ms"; // uint32 ms; all-4 hold valve open duration
constexpr uint32_t kBooshFailsafeMinMs  = 1000;
constexpr uint32_t kBooshFailsafeMaxMs  = 60000;

void AppendWebLogLine(const String &line) {
  web_log_text += line;
  web_log_text += '\n';

  while (web_log_text.length() > kWebLogBufferMaxChars) {
    const int newline = web_log_text.indexOf('\n');
    if (newline < 0) {
      web_log_text = web_log_text.substring(web_log_text.length() - kWebLogBufferMaxChars);
      break;
    }
    web_log_text.remove(0, newline + 1);
  }
}

void AppendWebLogChar(char c) {
  if (c == '\r') {
    return;
  }
  if (c == '\n') {
    AppendWebLogLine(web_log_partial_line);
    web_log_partial_line = "";
    return;
  }
  if (web_log_partial_line.length() < 256) {
    web_log_partial_line += c;
  }
}

class ConsoleMirror : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    AppendWebLogChar(static_cast<char>(c));
    return 1;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    Serial.write(buffer, size);
    for (size_t i = 0; i < size; ++i) {
      AppendWebLogChar(static_cast<char>(buffer[i]));
    }
    return size;
  }
};

ConsoleMirror Console;

struct DisplayPageLines {
  String line1;
  String line2;
  String line3;
  String line4;
};

void RenderDisplayPage(const DisplayPageLines &page) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(page.line1);
  display.println(page.line2);
  display.println(page.line3);
  display.println(page.line4);
  display.display();
}

void SetDisplayInverted(bool inverted) {
  if (display_inverted == inverted) {
    return;
  }
  display_inverted = inverted;
  display.invertDisplay(inverted);
  digitalWrite(kIo11Pin, inverted ? HIGH : LOW);
}

void ShowStatus(const String &line1, const String &line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length() > 0) {
    display.println(line2);
  }
  display.display();
}

void LogBootStep(const char *message, const String &detail = "") {
  Console.println(message);
  ShowStatus(message, detail);
}

String TrimForDisplay(const String &input, size_t maxChars) {
  if (input.length() <= maxChars) {
    return input;
  }
  return input.substring(0, maxChars);
}

String FormatDurationHms(uint32_t totalSeconds) {
  const uint32_t hours = totalSeconds / 3600;
  const uint32_t minutes = (totalSeconds % 3600) / 60;
  const uint32_t seconds = totalSeconds % 60;
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", static_cast<unsigned long>(hours),
           static_cast<unsigned long>(minutes), static_cast<unsigned long>(seconds));
  return String(buffer);
}

String JsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input.charAt(i);
    if (c == '\\' || c == '"') { out += '\\'; out += c; continue; }
    if (c == '\n') { out += "\\n"; continue; }
    if (c == '\r') { out += "\\r"; continue; }
    if (c == '\t') { out += "\\t"; continue; }
    if ((unsigned char)c < 0x20) {
      // Other control characters: emit \u00XX
      char buf[7];
      snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
      out += buf;
      continue;
    }
    out += c;
  }
  return out;
}

const char *ResetReasonString(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external-pin";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic/crash";
    case ESP_RST_INT_WDT:   return "interrupt-watchdog";
    case ESP_RST_TASK_WDT:  return "task-watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep-wakeup";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
  }
}

String ToLowerAscii(String value) {
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value.charAt(i);
    if (c >= 'A' && c <= 'Z') {
      value.setCharAt(i, static_cast<char>(c + ('a' - 'A')));
    }
  }
  return value;
}

String BuildDefaultPylonId() {
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char id[16];
  // Use LSBs of STA Wi-Fi MAC for deterministic per-device default ID.
  snprintf(id, sizeof(id), "%s%02X%02X", kPylonIdDefaultPrefix, mac[4], mac[5]);
  return String(id);
}

String NormalizeMdnsHost(String host) {
  host.trim();
  if (host.length() == 0) {
    return host;
  }
  String lower = ToLowerAscii(host);
  if (lower.endsWith(".local")) {
    host = host.substring(0, host.length() - 6);
  }
  host.trim();
  return host;
}

bool IsValidMdnsHost(const String &host) {
  if (host.length() == 0 || host.length() > 63) {
    return false;
  }
  if (host.charAt(0) == '-' || host.charAt(host.length() - 1) == '-') {
    return false;
  }
  for (size_t i = 0; i < host.length(); ++i) {
    const char c = host.charAt(i);
    const bool is_alpha = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
    const bool is_digit = (c >= '0' && c <= '9');
    if (!(is_alpha || is_digit || c == '-')) {
      return false;
    }
  }
  return true;
}

String NormalizePylonId(String id) {
  id.trim();
  if (id.length() > 64) {
    id = id.substring(0, 64);
  }
  return id;
}

bool SavePylonConfig() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    Console.println("[CFG] failed to open Preferences namespace for write");
    return false;
  }
  prefs.putString(kPrefsKeyId, pylon_id);
  prefs.putString(kPrefsKeyHost, pylon_mdns_host);
  prefs.putString(kPrefsKeyDesc, pylon_description);
  prefs.putBool(kPrefsKeyAp, ap_enabled);
  prefs.putString(kPrefsKeyUserSsid, user_wifi_ssid);
  prefs.putString(kPrefsKeyUserPass, user_wifi_pass);
  prefs.putUInt(kPrefsKeyFailsafeMs, static_cast<uint32_t>(boosh_failsafe_timeout_ms));
  prefs.putInt(kPrefsKeyIndex, pylon_index);
  prefs.putUInt(kPrefsKeySeqMaxMs, static_cast<uint32_t>(barmode_seq_max_ms));
  prefs.putUInt(kPrefsKeySeqDecMs, static_cast<uint32_t>(barmode_seq_dec_ms));
  prefs.putUChar(kPrefsKeySeqExpPct, barmode_seq_exp_pct);
  prefs.putUInt(kPrefsKeyGreenTimeout, static_cast<uint32_t>(barmode_green_timeout_ms));
  prefs.putUInt(kPrefsKeyAll4ValveMs,  static_cast<uint32_t>(barmode_all4_valve_ms));
  {
    uint8_t mask = 0;
    for (int i = 0; i < 4; i++) if (barmode_btn_disabled[i]) mask |= (1 << i);
    prefs.putUChar(kPrefsKeyBtnDisable, mask);
  }
  prefs.end();
  return true;
}

bool ClearPylonConfigNvs() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    Console.println("[CFG] failed to open Preferences namespace for clear");
    return false;
  }
  prefs.clear();
  prefs.end();
  Console.println("[CFG] NVS namespace cleared.");
  return true;
}

void ScheduleRegistryRefreshNow() {
  registry_announced = false;
  registry_next_attempt_ms = millis();
  registry_consecutive_failures = 0;
}

void PrintPylonConfig() {
  Console.println("[CFG] current values:");
  Console.print("  id: ");
  Console.println(pylon_id);
  Console.print("  host: ");
  Console.println(pylon_mdns_host);
  Console.print("  desc: ");
  Console.println(pylon_description);
  Console.print("  ap: ");
  Console.println(ap_enabled ? "true" : "false");
  Console.print("  wifi_ssid: ");
  Console.println(user_wifi_ssid.length() > 0 ? user_wifi_ssid : "(not set)");
  Console.print("  wifi_pass: ");
  Console.println(user_wifi_pass.length() > 0 ? "***" : "(not set)");
  Console.print("  failsafe_ms: ");
  Console.println(boosh_failsafe_timeout_ms);
}

void LoadPylonConfig() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    Console.println("[CFG] failed to open Preferences namespace");
    pylon_id = BuildDefaultPylonId();
    pylon_mdns_host = pylon_id;
    pylon_description = kPylonDescriptionDefault;
    return;
  }

  String stored_id = NormalizePylonId(prefs.getString(kPrefsKeyId, ""));
  String stored_host = NormalizeMdnsHost(prefs.getString(kPrefsKeyHost, ""));
  String stored_desc = prefs.getString(kPrefsKeyDesc, "");
  ap_enabled = prefs.getBool(kPrefsKeyAp, false);
  user_wifi_ssid = prefs.getString(kPrefsKeyUserSsid, "");
  user_wifi_pass = prefs.getString(kPrefsKeyUserPass, "");
  boosh_failsafe_timeout_ms = prefs.getUInt(kPrefsKeyFailsafeMs, kBooshFailsafeTimeoutMs);
  pylon_index = prefs.getInt(kPrefsKeyIndex, 0);
  barmode_seq_max_ms = prefs.getUInt(kPrefsKeySeqMaxMs, 30000);
  barmode_seq_dec_ms = prefs.getUInt(kPrefsKeySeqDecMs, 50);
  barmode_seq_exp_pct    = (uint8_t)prefs.getUChar(kPrefsKeySeqExpPct, 100);
  barmode_green_timeout_ms = prefs.getUInt(kPrefsKeyGreenTimeout, 300);
  barmode_all4_valve_ms    = prefs.getUInt(kPrefsKeyAll4ValveMs,  3000);
  {
    const uint8_t mask = prefs.getUChar(kPrefsKeyBtnDisable, 0);
    for (int i = 0; i < 4; i++) barmode_btn_disabled[i] = (mask >> i) & 1;
  }
  stored_desc.trim();

  const bool unprogrammed = stored_id.length() == 0;
  if (unprogrammed) {
    pylon_id = BuildDefaultPylonId();
    pylon_mdns_host = pylon_id;
    pylon_description = kPylonDescriptionDefault;
    prefs.putString(kPrefsKeyId, pylon_id);
    prefs.putString(kPrefsKeyHost, pylon_mdns_host);
    prefs.putString(kPrefsKeyDesc, pylon_description);
    Console.println("[CFG] Preferences unprogrammed, wrote defaults.");
  } else {
    pylon_id = stored_id;
    if (stored_host.length() == 0) {
      stored_host = pylon_id;
    }
    if (!IsValidMdnsHost(stored_host)) {
      stored_host = pylon_id;
      Console.println("[CFG] Stored host invalid, using ID as host.");
    }
    pylon_mdns_host = stored_host;
    pylon_description = stored_desc.length() > 0 ? stored_desc : String(kPylonDescriptionDefault);
  }

  prefs.end();
  PrintPylonConfig();
}

void RestartMdnsIfConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  MDNS.end();
  if (MDNS.begin(pylon_mdns_host.c_str())) {
    Console.print("[CFG] mDNS updated: ");
    Console.print(pylon_mdns_host);
    Console.println(".local");
  } else {
    Console.println("[CFG] mDNS update failed.");
  }
}

void PrintCliHelp() {
  Console.println("[CLI] commands:");
  Console.println("  help");
  Console.println("  show");
  Console.println("  set id <value>");
  Console.println("  set host <value>");
  Console.println("  set desc <value>");
  Console.println("  set node <value>   (sets both id and host)");
  Console.println("  set ap true|false  (enable/disable WiFi AP mode)");
  Console.println("  set wifi_ssid <value>  (user WiFi SSID fallback)");
  Console.println("  set wifi_pass <value>  (user WiFi password)");
  Console.println("  set failsafe_s <value> (solenoid failsafe timeout in seconds, 1-60)");
  Console.println("  set index <value>      (barmode sequence index, 0=default)");
  Console.println("  set seq_max_s <value>  (barmode btn1 seq max hold, 1-120s, default 30)");
  Console.println("  set seq_dec_ms <value> (barmode btn1 delay decrement per step, 0-2000ms, default 50)");
  Console.println("  clear nvs          (erase saved id/host/desc)");
}

bool SetConfigFieldValue(const String &field_in, const String &value_in, bool log_output = true) {
  String field = ToLowerAscii(field_in);
  String value = value_in;
  value.trim();
  if (value.length() == 0) {
    if (log_output) {
      Console.println("[CFG] value cannot be empty");
    }
    return false;
  }

  bool changed = false;
  if (field == "id") {
    const String new_id = NormalizePylonId(value);
    if (new_id.length() == 0) {
      if (log_output) {
        Console.println("[CFG] invalid id");
      }
      return false;
    }
    pylon_id = new_id;
    changed = true;
    if (log_output) {
      Console.print("[CFG] id set: ");
      Console.println(pylon_id);
    }
  } else if (field == "host" || field == "mdns") {
    const String new_host = NormalizeMdnsHost(value);
    if (!IsValidMdnsHost(new_host)) {
      if (log_output) {
        Console.println("[CFG] invalid host; allowed: letters, digits, '-' (1..63 chars)");
      }
      return false;
    }
    pylon_mdns_host = new_host;
    changed = true;
    if (log_output) {
      Console.print("[CFG] host set: ");
      Console.println(pylon_mdns_host);
    }
  } else if (field == "desc" || field == "description") {
    pylon_description = value;
    changed = true;
    barmode_active = pylon_description.indexOf(kBarModeToken) >= 0;
    if (barmode_active) {
      for (int i = 0; i < 4; i++) pinMode(kBarModeButtonPins[i], INPUT_PULLDOWN);
      barmode_registry_fetch_now = true;
    }
    if (log_output) {
      Console.print("[CFG] desc set: ");
      Console.println(pylon_description);
      if (barmode_active) Console.println("[BarMode] ACTIVE");
    }
  } else if (field == "node") {
    const String new_id = NormalizePylonId(value);
    const String new_host = NormalizeMdnsHost(value);
    if (new_id.length() == 0 || !IsValidMdnsHost(new_host)) {
      if (log_output) {
        Console.println("[CFG] invalid node value");
      }
      return false;
    }
    pylon_id = new_id;
    pylon_mdns_host = new_host;
    changed = true;
    if (log_output) {
      Console.print("[CFG] node set (id+host): ");
      Console.println(pylon_id);
    }
  } else if (field == "ap") {
    const String v = ToLowerAscii(value);
    ap_enabled = (v == "true" || v == "1" || v == "yes" || v == "on");
    changed = true;
    if (log_output) {
      Console.print("[CFG] ap_enabled set: ");
      Console.println(ap_enabled ? "true" : "false");
    }
  } else if (field == "wifi_ssid") {
    user_wifi_ssid = value;
    changed = true;
    if (log_output) {
      Console.print("[CFG] wifi_ssid set: ");
      Console.println(user_wifi_ssid);
    }
  } else if (field == "wifi_pass") {
    user_wifi_pass = value;
    changed = true;
    if (log_output) {
      Console.println("[CFG] wifi_pass updated");
    }
  } else if (field == "failsafe_s" || field == "failsafe_ms") {
    const float secs = value.toFloat();
    const unsigned long ms = (field == "failsafe_ms")
        ? static_cast<unsigned long>(secs)
        : static_cast<unsigned long>(secs * 1000.0f);
    if (ms < kBooshFailsafeMinMs || ms > kBooshFailsafeMaxMs) {
      if (log_output) {
        Console.printf("[CFG] failsafe out of range (%lu-%lu ms)\n",
                       (unsigned long)kBooshFailsafeMinMs, (unsigned long)kBooshFailsafeMaxMs);
      }
      return false;
    }
    boosh_failsafe_timeout_ms = ms;
    changed = true;
    if (log_output) {
      Console.printf("[CFG] failsafe_ms set: %lu\n", boosh_failsafe_timeout_ms);
    }
  } else if (field == "index") {
    pylon_index = (int)value.toInt();
    changed = true;
    if (log_output) Console.printf("[CFG] pylon_index set: %d\n", pylon_index);
  } else if (field == "seq_max_s" || field == "seq_max_ms") {
    const float secs = value.toFloat();
    const unsigned long ms = (field == "seq_max_ms")
        ? static_cast<unsigned long>(secs)
        : static_cast<unsigned long>(secs * 1000.0f);
    if (ms < 1000 || ms > 120000) {
      if (log_output) Console.println("[CFG] seq_max_s out of range (1-120s)");
      return false;
    }
    barmode_seq_max_ms = ms;
    changed = true;
    if (log_output) Console.printf("[CFG] seq_max_ms set: %lu\n", barmode_seq_max_ms);
  } else if (field == "seq_dec_ms" || field == "seq_dec_s") {
    const float secs = value.toFloat();
    const unsigned long ms2 = (field == "seq_dec_ms")
        ? static_cast<unsigned long>(secs)
        : static_cast<unsigned long>(secs * 1000.0f);
    if (ms2 > 2000) {
      if (log_output) Console.println("[CFG] seq_dec_ms out of range (0-2000ms)");
      return false;
    }
    barmode_seq_dec_ms = ms2;
    changed = true;
    if (log_output) Console.printf("[CFG] seq_dec_ms set: %lu\n", barmode_seq_dec_ms);
  } else if (field == "seq_exp_pct") {
    const int pct = (int)value.toInt();
    if (pct < 1 || pct > 100) {
      if (log_output) Console.println("[CFG] seq_exp_pct out of range (1-100)");
      return false;
    }
    barmode_seq_exp_pct = (uint8_t)pct;
    changed = true;
    if (log_output) Console.printf("[CFG] seq_exp_pct set: %d\n", barmode_seq_exp_pct);
  } else if (field == "green_timeout_ms" || field == "grn_to_ms") {
    const int ms = (int)value.toInt();
    if (ms < 50 || ms > 10000) {
      if (log_output) Console.println("[CFG] green_timeout_ms out of range (50-10000ms)");
      return false;
    }
    barmode_green_timeout_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] green_timeout_ms set: %lu\n", barmode_green_timeout_ms);
  } else if (field == "all4_valve_ms") {
    const int ms = (int)value.toInt();
    if (ms < 500 || ms > 30000) {
      if (log_output) Console.println("[CFG] all4_valve_ms out of range (500-30000ms)");
      return false;
    }
    barmode_all4_valve_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] all4_valve_ms set: %lu\n", barmode_all4_valve_ms);
  } else {
    if (log_output) {
      Console.println("[CFG] unknown set field. use id|host|desc|node|ap|failsafe_s|index|seq_max_s|seq_dec_ms|seq_exp_pct");
    }
    return false;
  }

  if (!changed) {
    return true;
  }
  if (!SavePylonConfig()) {
    if (log_output) {
      Console.println("[CFG] failed to persist config");
    }
    return false;
  }
  RestartMdnsIfConnected();
  ScheduleRegistryRefreshNow();
  if (log_output) {
    PrintPylonConfig();
  }
  return true;
}

String BuildConfigApiJson() {
  String payload;
  payload.reserve(256);
  payload += "{";
  payload += "\"id\":\"" + JsonEscape(pylon_id) + "\",";
  payload += "\"host\":\"" + JsonEscape(pylon_mdns_host) + "\",";
  payload += "\"hostname\":\"" + JsonEscape(pylon_mdns_host + ".local") + "\",";
  payload += "\"description\":\"" + JsonEscape(pylon_description) + "\",";
  payload += "\"ap_enabled\":" + String(ap_enabled ? "true" : "false") + ",";
  payload += "\"ap_active\":" + String(ap_active ? "true" : "false") + ",";
  payload += "\"wifi_ssid\":\"" + JsonEscape(user_wifi_ssid) + "\",";
  payload += "\"failsafe_ms\":" + String(boosh_failsafe_timeout_ms) + ",";
  payload += "\"pylon_index\":" + String(pylon_index) + ",";
  payload += "\"seq_max_ms\":" + String(barmode_seq_max_ms) + ",";
  payload += "\"seq_dec_ms\":" + String(barmode_seq_dec_ms) + ",";
  payload += "\"seq_exp_pct\":" + String(barmode_seq_exp_pct);
  payload += "}";
  return payload;
}

void HandleCliCommand(const String &input_line) {
  String line = input_line;
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("help")) {
    PrintCliHelp();
    return;
  }
  if (line.equalsIgnoreCase("show")) {
    PrintPylonConfig();
    return;
  }
  if (line.equalsIgnoreCase("clear nvs")) {
    if (!ClearPylonConfigNvs()) {
      return;
    }
    LoadPylonConfig();
    RestartMdnsIfConnected();
    ScheduleRegistryRefreshNow();
    return;
  }
  if (!line.startsWith("set ")) {
    Console.println("[CLI] unknown command. type 'help'");
    return;
  }

  String rest = line.substring(4);
  rest.trim();
  const int sep = rest.indexOf(' ');
  if (sep <= 0) {
    Console.println("[CLI] invalid set command");
    return;
  }
  String field = ToLowerAscii(rest.substring(0, sep));
  String value = rest.substring(sep + 1);
  value.trim();
  if (value.length() == 0) {
    Console.println("[CLI] value cannot be empty");
    return;
  }
  if (!SetConfigFieldValue(field, value)) {
    return;
  }
}

void PollSerialCli() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      HandleCliCommand(serial_cli_line);
      serial_cli_line = "";
      continue;
    }
    if (serial_cli_line.length() < 192) {
      serial_cli_line += c;
    }
  }
}

unsigned long RegistryBackoffMs(uint8_t failureCount) {
  if (failureCount == 0) {
    return 0;
  }
  uint8_t shift = failureCount > 5 ? 5 : static_cast<uint8_t>(failureCount - 1);
  unsigned long backoff = 1000UL << shift;  // 1s,2s,4s,8s,16s,32s
  if (backoff > 30000UL) {
    backoff = 30000UL;
  }
  return backoff;
}

String BuildRegistryPayload() {
  const String hostname = pylon_mdns_host + ".local";
  const String ip = WiFi.localIP().toString();
  const long wifi_rssi_dbm = WiFi.RSSI();
  String payload;
  payload.reserve(680);
  payload += "{";
  payload += "\"pylon_id\":\"" + JsonEscape(pylon_id) + "\",";
  payload += "\"description\":\"" + JsonEscape(pylon_description) + "\",";
  payload += "\"hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"ip\":\"" + JsonEscape(ip) + "\",";
  payload += "\"osc_port\":" + String(kOscPort) + ",";
  payload += "\"osc_paths\":[\"" + String(kOscAddress) + "\",\""
           + String(kOscAddrPulseSingle) + "\",\""
           + String(kOscAddrPulseTrain)  + "\",\""
           + String(kOscAddrSteam)       + "\"],";
  payload += "\"pylon_index\":" + String(pylon_index) + ",";
  payload += "\"roles\":[\"boosh_main\"],";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"firmware_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"fw_semver\":\"" + JsonEscape(String(kFirmwareSemver)) + "\",";
  payload += "\"ttl_sec\":" + String(kRegistryTtlSec) + ",";
  payload += "\"telemetry\":{";
  payload += "\"ipv4\":\"" + JsonEscape(ip) + "\",";
  payload += "\"mdns_hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"firmware_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"fw_semver\":\"" + JsonEscape(String(kFirmwareSemver)) + "\",";
  payload += "\"fw_build_date\":\"" + JsonEscape(String(kFirmwareBuildDate)) + "\",";
  payload += "\"fw_build_time\":\"" + JsonEscape(String(kFirmwareBuildTime)) + "\",";
  {
    char sbuf[16];
    auto fmtOrNull = [&](float v) -> String {
      if (!isfinite(v)) return "null";
      snprintf(sbuf, sizeof(sbuf), "%.2f", v);
      return String(sbuf);
    };
    payload += "\"temperature\":" + fmtOrNull(sensor_temp_f) + ",";
    payload += "\"temperature_f\":" + fmtOrNull(sensor_temp_f) + ",";
    const float tempC = isfinite(sensor_temp_f) ? (sensor_temp_f - 32.0f) * 5.0f / 9.0f : NAN;
    payload += "\"temperature_c\":" + fmtOrNull(tempC) + ",";
    payload += "\"battery_voltage\":" + fmtOrNull(sensor_battery_v) + ",";
    payload += "\"battery_voltage_v\":" + fmtOrNull(sensor_battery_v) + ",";
    payload += "\"battery_charge\":" + fmtOrNull(sensor_battery_pct) + ",";
    payload += "\"battery_charge_pct\":" + fmtOrNull(sensor_battery_pct) + ",";
    payload += "\"battery_time_remaining_h\":" + fmtOrNull(sensor_battery_time_remaining_h) + ",";
  }
  payload += "\"wifi_rssi_dbm\":" + String(wifi_rssi_dbm) + ",";
  payload += "\"uptime_s\":" + String(static_cast<uint32_t>(millis() / 1000)) + ",";
  payload += "\"uptime\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(millis() / 1000))) + "\",";
  payload += "\"uptime_hms\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(millis() / 1000))) + "\",";
  payload += "\"trigger_event_count\":" + String(trigger_event_count) + ",";
  payload += "\"solenoid_active\":" + String(display_inverted ? "true" : "false") + ",";
  payload += "\"ping_target\":\"" + JsonEscape(String(kPingTargetHost)) + "\",";
  payload += "\"ping\":{";
  payload += "\"target\":\"" + JsonEscape(String(kPingTargetHost)) + "\",";
  payload += "\"sent\":" + String(telemetry_ping_sent) + ",";
  payload += "\"recv\":" + String(telemetry_ping_count) + ",";
  payload += "\"lost\":" + String(telemetry_ping_lost) + ",";
  payload += "\"last_ms\":" + String(telemetry_ping_last_ms) + ",";
  payload += "\"min_ms\":" + String(telemetry_ping_min_ms) + ",";
  payload += "\"max_ms\":" + String(telemetry_ping_max_ms) + ",";
  payload += "\"avg_ms\":" + String(telemetry_ping_avg_ms) + ",";
  payload += "\"count\":" + String(telemetry_ping_count) + ",";
  payload += "\"last_ok\":" + String(telemetry_ping_last_ok ? "true" : "false");
  payload += "}";
  payload += "}";
  payload += "}";
  return payload;
}

bool PostRegistryToBase(const char *baseUrl, const char *path, const String &payload, const char *kind) {
  if (baseUrl == nullptr || baseUrl[0] == '\0') {
    return false;
  }
  WiFiClient client;
  HTTPClient http;
  const String url = String(baseUrl) + String(path);
  if (!http.begin(client, url)) {
    Console.print("[REG] begin failed: ");
    Console.println(url);
    return false;
  }
  http.setConnectTimeout(kRegistryHttpTimeoutMs);
  http.setTimeout(kRegistryHttpTimeoutMs);
  http.addHeader("Content-Type", "application/json");
  const int statusCode = http.POST(payload);
  const bool ok = statusCode >= 200 && statusCode < 300;
  Console.print("[REG] ");
  Console.print(kind);
  Console.print(" ");
  Console.print(url);
  Console.print(" -> ");
  Console.println(statusCode);
  if (!ok && statusCode < 0) {
    Console.print("[REG] error: ");
    Console.println(http.errorToString(statusCode));
  }
  http.end();
  return ok;
}

bool ResolveRegistryFallbackBase(String &baseUrl) {
  IPAddress resolvedIp;
  if (target_ip_string.length() > 0 && resolvedIp.fromString(target_ip_string)) {
    baseUrl = "http://" + target_ip_string + ":5000";
    return true;
  }
  if (WiFi.hostByName("rpiboosh.local", resolvedIp) || WiFi.hostByName(kPingTargetHost, resolvedIp) ||
      WiFi.hostByName("RPIBOOSH.local", resolvedIp)) {
    target_ip_string = resolvedIp.toString();
    baseUrl = "http://" + target_ip_string + ":5000";
    return true;
  }
  return false;
}

bool PostRegistry(bool heartbeat) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  const char *path = heartbeat ? kRegistryHeartbeatPath : kRegistryAnnouncePath;
  const char *kind = heartbeat ? "heartbeat" : "announce";
  const String payload = BuildRegistryPayload();

  if (PostRegistryToBase(kRegistryBaseUrlPrimary, path, payload, kind)) {
    return true;
  }
  if (kRegistryBaseUrlFallback[0] != '\0') {
    if (PostRegistryToBase(kRegistryBaseUrlFallback, path, payload, kind)) {
      return true;
    }
  }
  String dynamicFallbackBase;
  if (ResolveRegistryFallbackBase(dynamicFallbackBase)) {
    if (dynamicFallbackBase != String(kRegistryBaseUrlPrimary) &&
        dynamicFallbackBase != String(kRegistryBaseUrlFallback) &&
        PostRegistryToBase(dynamicFallbackBase.c_str(), path, payload, kind)) {
      return true;
    }
  }
  return false;
}

void HandleRegistry(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  const bool shouldAnnounce = !registry_announced;
  const bool shouldHeartbeat =
      registry_announced && (now - registry_last_success_ms >= kRegistryHeartbeatIntervalMs);
  if (!shouldAnnounce && !shouldHeartbeat) {
    return;
  }
  if (now < registry_next_attempt_ms) {
    return;
  }
  const bool ok = PostRegistry(!shouldAnnounce);
  if (ok) {
    registry_announced = true;
    registry_last_success_ms = now;
    registry_next_attempt_ms = now + kRegistryHeartbeatIntervalMs;
    registry_consecutive_failures = 0;
    Console.println("[REG] post success");
  } else {
    registry_consecutive_failures = static_cast<uint8_t>(registry_consecutive_failures + 1);
    const unsigned long backoffMs = RegistryBackoffMs(registry_consecutive_failures);
    registry_next_attempt_ms = now + backoffMs;
    Console.print("[REG] post failed, retry in ");
    Console.print(backoffMs);
    Console.println("ms");
  }
}

const char *WifiDisconnectReasonToString(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED:
      return "UNSPEC";
    case WIFI_REASON_AUTH_EXPIRE:
      return "AUTH_EXP";
    case WIFI_REASON_AUTH_LEAVE:
      return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE:
      return "ASSOC_EXP";
    case WIFI_REASON_ASSOC_TOOMANY:
      return "ASSOC_MANY";
    case WIFI_REASON_NOT_AUTHED:
      return "NOT_AUTH";
    case WIFI_REASON_NOT_ASSOCED:
      return "NOT_ASSOC";
    case WIFI_REASON_ASSOC_LEAVE:
      return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED:
      return "ASSOC_NOAUTH";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:
      return "PWR_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
      return "CHAN_BAD";
    case WIFI_REASON_IE_INVALID:
      return "IE_BAD";
    case WIFI_REASON_MIC_FAILURE:
      return "MIC_FAIL";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      return "4WAY_TO";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
      return "GK_TO";
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
      return "4WAY_IE";
    case WIFI_REASON_GROUP_CIPHER_INVALID:
      return "G_CIPHER";
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
      return "P_CIPHER";
    case WIFI_REASON_AKMP_INVALID:
      return "AKMP_BAD";
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
      return "RSN_VER";
    case WIFI_REASON_INVALID_RSN_IE_CAP:
      return "RSN_CAP";
    case WIFI_REASON_802_1X_AUTH_FAILED:
      return "8021X";
    case WIFI_REASON_CIPHER_SUITE_REJECTED:
      return "CIPHER_REJ";
    case WIFI_REASON_BEACON_TIMEOUT:
      return "BEACON_TO";
    case WIFI_REASON_NO_AP_FOUND:
      return "NO_AP";
    case WIFI_REASON_AUTH_FAIL:
      return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:
      return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
      return "HS_TO";
    case WIFI_REASON_CONNECTION_FAIL:
      return "CONN_FAIL";
    case WIFI_REASON_AP_TSF_RESET:
      return "AP_RESET";
    case WIFI_REASON_ROAMING:
      return "ROAM";
    default:
      return "UNK";
  }
}

struct PingStats {
  uint32_t count = 0;
  uint32_t min_ms = UINT32_MAX;
  uint32_t max_ms = 0;
  uint64_t sum_ms = 0;
  uint32_t last_ms = 0;
  bool has_data() const { return count > 0; }
  uint32_t avg_ms() const { return count > 0 ? static_cast<uint32_t>(sum_ms / count) : 0; }
};

DisplayPageLines BuildPingPageLines(const String &host, const PingStats &stats, bool last_ok,
                                    unsigned long last_success_ms, unsigned long now_ms) {
  DisplayPageLines page;
  bool has_success = last_success_ms > 0;
  unsigned long since_success_ms = has_success ? (now_ms - last_success_ms) : 0;
  bool timing_out =
      (has_success && since_success_ms >= kPingTimeoutMs) || (!has_success && now_ms >= kPingTimeoutMs);
  page.line1 = host + (timing_out ? " TIMEOUT" : (last_ok ? " OK" : " FAIL"));

  if (stats.has_data()) {
    page.line2 = "last " + String(stats.last_ms) + "ms";
  } else {
    page.line2 = "last --";
  }

  if (has_success) {
    page.line3 = "since " + FormatDurationHms(static_cast<uint32_t>(since_success_ms / 1000));
  } else {
    page.line3 = "since --:--:--";
  }

  if (stats.has_data()) {
    page.line4 = "min " + String(stats.min_ms) + " max " + String(stats.max_ms) + "ms";
  } else {
    page.line4 = "";
  }
  return page;
}

void ShowPingStats(const String &host, const PingStats &stats, bool last_ok,
                   unsigned long last_success_ms, unsigned long now_ms) {
  RenderDisplayPage(BuildPingPageLines(host, stats, last_ok, last_success_ms, now_ms));
}

DisplayPageLines BuildWifiMetricsPageLines(uint8_t page, unsigned long connected_since_ms,
                                           uint8_t disconnect_reason) {
  DisplayPageLines lines;
  if (page == 0) {
    String ssid = TrimForDisplay(WiFi.SSID(), 20);
    lines.line1 = "WiFi " + ssid;
    lines.line2 = "RSSI " + String(WiFi.RSSI()) + "dBm";
    lines.line3 = "IP " + WiFi.localIP().toString();
    if (connected_since_ms > 0) {
      lines.line4 = "UP " + FormatDurationHms(static_cast<uint32_t>((millis() - connected_since_ms) / 1000));
    } else {
      lines.line4 = "UP --:--:--";
    }
  } else {
    lines.line1 = "RSN " + String(disconnect_reason) + " " + WifiDisconnectReasonToString(disconnect_reason);
    lines.line2 = "SSID " + TrimForDisplay(WiFi.SSID(), 20);
    lines.line3 = "RSSI " + String(WiFi.RSSI()) + "dBm";
    lines.line4 = "IP " + WiFi.localIP().toString();
  }
  return lines;
}

void ShowWifiMetricsPage(uint8_t page, unsigned long connected_since_ms, uint8_t last_disconnect_reason) {
  RenderDisplayPage(BuildWifiMetricsPageLines(page, connected_since_ms, last_disconnect_reason));
}

DisplayPageLines BuildNodeConfigPageLines() {
  DisplayPageLines page;
  page.line1 = "NODE CONFIG";
  page.line2 = "ID: " + TrimForDisplay(pylon_id, 14);
  page.line3 = "TRIG: " + String(trigger_event_count);
  page.line4 = "FW " + String(kFirmwareSemver);
  return page;
}

void ShowNodeConfigPage() {
  RenderDisplayPage(BuildNodeConfigPageLines());
}

DisplayPageLines BuildFirmwarePageLines() {
  DisplayPageLines page;
  page.line1 = "FIRMWARE";
  page.line2 = "VER " + String(kFirmwareSemver);
  page.line3 = String(kFirmwareBuildDate);
  page.line4 = String(kFirmwareBuildTime);
  return page;
}

void ShowFirmwarePage() {
  RenderDisplayPage(BuildFirmwarePageLines());
}

DisplayPageLines BuildSensorStatusPageLines() {
  DisplayPageLines page;
  page.line1 = TrimForDisplay(pylon_id, 21);

  if (isfinite(sensor_temp_f)) {
    float tempC = (sensor_temp_f - 32.0f) * 5.0f / 9.0f;
    char buf[22];
    snprintf(buf, sizeof(buf), "Temp %.1fF %.1fC", sensor_temp_f, tempC);
    page.line2 = buf;
  } else {
    page.line2 = "Temp --";
  }

  if (isfinite(sensor_battery_v) && isfinite(sensor_battery_pct)) {
    char buf[22];
    snprintf(buf, sizeof(buf), "Batt %.2fV  %.0f%%", sensor_battery_v, sensor_battery_pct);
    page.line3 = buf;
  } else if (isfinite(sensor_battery_v)) {
    char buf[22];
    snprintf(buf, sizeof(buf), "Batt %.2fV  --%% ", sensor_battery_v);
    page.line3 = buf;
  } else {
    page.line3 = "Batt --";
  }

  if (isfinite(sensor_battery_time_remaining_h)) {
    char buf[22];
    snprintf(buf, sizeof(buf), "Left %.1fh", sensor_battery_time_remaining_h);
    page.line4 = buf;
  } else {
    page.line4 = "Left --";
  }

  return page;
}

void ShowSensorStatusPage() {
  RenderDisplayPage(BuildSensorStatusPageLines());
}

// View 1: "42F  75%" — temp °F and battery pct, size-3 digits.
// Temp is shown as "-" if sensor is disconnected (outside -40..200°F).
// Digits at y=0 (size 3, 24px tall, ends y=24).
// Units  at y=24 (size 1, 8px tall, ends y=32) — subscripted.
// Horizontal gap is computed from remaining width so content is spread evenly.
void ShowTempPctPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  constexpr int yNum  = 0;   // size-3 digits (24px tall, ends y=24)
  constexpr int yUnit = 24;  // size-1 units  ( 8px tall, ends y=32)

  char tempBuf[5];
  const bool tempValid = isfinite(sensor_temp_f) &&
                         sensor_temp_f >= -40.0f && sensor_temp_f <= 200.0f;
  if (tempValid) {
    snprintf(tempBuf, sizeof(tempBuf), "%d", static_cast<int>(roundf(sensor_temp_f)));
  } else {
    snprintf(tempBuf, sizeof(tempBuf), "-");
  }

  char pctBuf[5];
  if (isfinite(sensor_battery_pct)) {
    snprintf(pctBuf, sizeof(pctBuf), "%d", static_cast<int>(roundf(sensor_battery_pct)));
  } else {
    snprintf(pctBuf, sizeof(pctBuf), "--");
  }

  // size-3: 18px/char. size-1: 6px/char.
  const int tempW = static_cast<int>(strlen(tempBuf)) * 18;
  const int pctW  = static_cast<int>(strlen(pctBuf))  * 18;
  const int totalContent = tempW + 6/*F*/ + pctW + 6/*%*/;
  int gap = (128 - totalContent) / 2;
  if (gap < 2) gap = 2;

  int x = 0;

  display.setTextSize(3);
  display.setCursor(x, yNum);
  display.print(tempBuf);
  x = display.getCursorX();
  display.setTextSize(1);
  display.setCursor(x, yUnit);
  display.print("F");
  x += 6 + gap;

  display.setTextSize(3);
  display.setCursor(x, yNum);
  display.print(pctBuf);
  x = display.getCursorX();
  display.setTextSize(1);
  display.setCursor(x, yUnit);
  display.print("%");

  display.display();
}

// View 2: "HH:MM  13.5V" — battery time remaining and voltage, size-2 digits.
// Time shown as "--" when unavailable or negative (nonsense estimate).
// Digits at y=8 (size 2, 16px tall, ends y=24).
// "V" unit at y=16 (size 1, 8px, ends y=24) — bottom-aligned with digits.
// Centered horizontally.
void ShowTimeVoltagePage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  constexpr int yNum  = 8;   // size-2 digits (16px tall, ends y=24)
  constexpr int yUnit = 16;  // size-1 unit   ( 8px tall, ends y=24)
  constexpr int kGap  = 6;   // px between time and voltage sections

  // Time remaining as "H:MM" (or "HH:MM", "HHH:MM")
  char timeBuf[9];
  const bool timeValid = isfinite(sensor_battery_time_remaining_h) &&
                         sensor_battery_time_remaining_h >= 0.0f;
  if (timeValid) {
    const int total_mins = static_cast<int>(sensor_battery_time_remaining_h * 60.0f + 0.5f);
    snprintf(timeBuf, sizeof(timeBuf), "%d:%02d", total_mins / 60, total_mins % 60);
  } else {
    snprintf(timeBuf, sizeof(timeBuf), "--");
  }

  char voltBuf[7];
  if (isfinite(sensor_battery_v)) {
    snprintf(voltBuf, sizeof(voltBuf), "%.1f", sensor_battery_v);
  } else {
    snprintf(voltBuf, sizeof(voltBuf), "--");
  }

  // size-2: 12px/char.  "V" at size-1: 6px.
  const int timeW = static_cast<int>(strlen(timeBuf)) * 12;
  const int voltW = static_cast<int>(strlen(voltBuf)) * 12;
  const int totalW = timeW + kGap + voltW + 6/*V*/;
  int x = (128 - totalW) / 2;
  if (x < 0) x = 0;

  display.setTextSize(2);
  display.setCursor(x, yNum);
  display.print(timeBuf);
  x += timeW + kGap;

  display.setTextSize(2);
  display.setCursor(x, yNum);
  display.print(voltBuf);
  x = display.getCursorX();
  display.setTextSize(1);
  display.setCursor(x, yUnit);
  display.print("V");

  display.display();
}

void ShowIdentifyScreen(uint8_t phase) {
  // Clear and draw centered "IDENTIFY" label
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 12);
  display.print("IDENTIFY");

  // 8×8 checkerboard XOR overlay — alternating phase gives crawling animation
  for (int x = 0; x < 128; x += 8) {
    for (int y = 0; y < 32; y += 8) {
      if (((x / 8 + y / 8) % 2) == phase) {
        display.fillRect(x, y, 8, 8, SSD1306_INVERSE);
      }
    }
  }
  display.display();
}

void SetBooshActive(bool active, const char *source) {
  if (active) {
    if (!display_inverted) {
      trigger_event_count += 1;
      boosh_open_since_ms = millis();
      Console.print("[TRIG] ");
      Console.print(source);
      Console.print(" -> event #");
      Console.println(trigger_event_count);
    }
    SetDisplayInverted(true);
    boosh_failsafe_armed = true;
    boosh_failsafe_start_ms = millis();
    return;
  }

  if (display_inverted) {
    if (boosh_open_since_ms > 0) {
      total_boosh_open_ms += (uint32_t)(millis() - boosh_open_since_ms);
      boosh_open_since_ms = 0;
    }
    Console.print("[TRIG] ");
    Console.print(source);
    Console.println(" -> OFF");
  }
  SetDisplayInverted(false);
  boosh_failsafe_armed = false;
}

void ApplyBooshState(float value, const char *source = "state") {
  const float kOnThreshold = 0.5f;
  SetBooshActive(value > kOnThreshold, source);
}

String BuildDisplayPagesJson(unsigned long now_ms) {
  PingStats stats;
  stats.count = telemetry_ping_count;
  stats.min_ms = telemetry_ping_has_data ? telemetry_ping_min_ms : UINT32_MAX;
  stats.max_ms = telemetry_ping_max_ms;
  stats.sum_ms = static_cast<uint64_t>(telemetry_ping_avg_ms) * telemetry_ping_count;
  stats.last_ms = telemetry_ping_last_ms;

  const DisplayPageLines ping = BuildPingPageLines("RPIBOOSH", stats, current_ping_last_ok, last_ping_success_ms, now_ms);
  const DisplayPageLines wifiA = BuildWifiMetricsPageLines(0, wifi_connected_since_ms, last_disconnect_reason);
  const DisplayPageLines wifiB = BuildWifiMetricsPageLines(1, wifi_connected_since_ms, last_disconnect_reason);
  const DisplayPageLines node = BuildNodeConfigPageLines();
  const DisplayPageLines firmware = BuildFirmwarePageLines();
  const DisplayPageLines status = BuildSensorStatusPageLines();

  auto encodePage = [](const DisplayPageLines &page) {
    String out = "[";
    out += "\"" + JsonEscape(page.line1) + "\",";
    out += "\"" + JsonEscape(page.line2) + "\",";
    out += "\"" + JsonEscape(page.line3) + "\",";
    out += "\"" + JsonEscape(page.line4) + "\"";
    out += "]";
    return out;
  };

  String json;
  json.reserve(600);
  json += "\"display_pages\":{";
  json += "\"status\":" + encodePage(status) + ",";
  json += "\"ping\":" + encodePage(ping) + ",";
  json += "\"wifi\":" + encodePage(wifiA) + ",";
  json += "\"wifi_detail\":" + encodePage(wifiB) + ",";
  json += "\"node\":" + encodePage(node) + ",";
  json += "\"firmware\":" + encodePage(firmware);
  json += "}";
  return json;
}

String BuildTelemetryApiJson() {
  const String hostname = pylon_mdns_host + ".local";
  const String ip = WiFi.localIP().toString();
  const long wifi_rssi_dbm = WiFi.RSSI();
  const unsigned long now = millis();
  String payload;
  payload.reserve(1200);
  payload += "{";
  payload += "\"pylon_id\":\"" + JsonEscape(pylon_id) + "\",";
  payload += "\"description\":\"" + JsonEscape(pylon_description) + "\",";
  payload += "\"hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"ip\":\"" + JsonEscape(ip) + "\",";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"firmware_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"fw_semver\":\"" + JsonEscape(String(kFirmwareSemver)) + "\",";
  payload += "\"fw_build_date\":\"" + JsonEscape(String(kFirmwareBuildDate)) + "\",";
  payload += "\"fw_build_time\":\"" + JsonEscape(String(kFirmwareBuildTime)) + "\",";
  payload += "\"uptime\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(now / 1000))) + "\",";
  payload += "\"uptime_hms\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(now / 1000))) + "\",";
  payload += "\"last_reset_reason\":\"" + JsonEscape(String(ResetReasonString(esp_reset_reason()))) + "\",";
  payload += "\"solenoid_active\":" + String(display_inverted ? "true" : "false") + ",";
  payload += "\"trigger_event_count\":" + String(trigger_event_count) + ",";
  payload += "\"total_boosh_open_s\":" + String(total_boosh_open_ms / 1000.0f, 1) + ",";
  payload += "\"ap_enabled\":" + String(ap_enabled ? "true" : "false") + ",";
  payload += "\"ap_active\":" + String(ap_active ? "true" : "false") + ",";
  payload += "\"barmode_active\":" + String(barmode_active ? "true" : "false") + ",";
  payload += "\"pylon_index\":" + String(pylon_index) + ",";
  payload += "\"seq_max_ms\":" + String(barmode_seq_max_ms) + ",";
  payload += "\"seq_dec_ms\":" + String(barmode_seq_dec_ms) + ",";
  payload += "\"seq_exp_pct\":" + String(barmode_seq_exp_pct) + ",";
  payload += "\"green_timeout_ms\":" + String(barmode_green_timeout_ms) + ",";
  payload += "\"all4_valve_ms\":" + String(barmode_all4_valve_ms) + ",";
  payload += "\"btn_press_counts\":[" + String(barmode_btn_counts[0]) + "," +
             String(barmode_btn_counts[1]) + "," + String(barmode_btn_counts[2]) + "," +
             String(barmode_btn_counts[3]) + "],";
  payload += "\"btn_disabled\":[" +
             String(barmode_btn_disabled[0]?"true":"false") + "," +
             String(barmode_btn_disabled[1]?"true":"false") + "," +
             String(barmode_btn_disabled[2]?"true":"false") + "," +
             String(barmode_btn_disabled[3]?"true":"false") + "],";
  payload += "\"wifi_ssid\":\"" + JsonEscape(user_wifi_ssid) + "\",";
  payload += "\"failsafe_ms\":" + String(boosh_failsafe_timeout_ms) + ",";
  payload += "\"target_ip\":\"" + JsonEscape(target_ip_string) + "\",";
  {
    char buf[16];
    payload += "\"battery_voltage_v\":";
    if (isfinite(sensor_battery_v)) {
      snprintf(buf, sizeof(buf), "%.2f", sensor_battery_v);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"battery_charge_pct\":";
    if (isfinite(sensor_battery_pct)) {
      snprintf(buf, sizeof(buf), "%.1f", sensor_battery_pct);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"battery_time_remaining_h\":";
    if (isfinite(sensor_battery_time_remaining_h)) {
      snprintf(buf, sizeof(buf), "%.1f", sensor_battery_time_remaining_h);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"temperature_f\":";
    if (isfinite(sensor_temp_f)) {
      snprintf(buf, sizeof(buf), "%.1f", sensor_temp_f);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
  }
  payload += "\"telemetry\":{";
  payload += "\"ipv4\":\"" + JsonEscape(ip) + "\",";
  payload += "\"mdns_hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"firmware_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"fw_semver\":\"" + JsonEscape(String(kFirmwareSemver)) + "\",";
  payload += "\"fw_build_date\":\"" + JsonEscape(String(kFirmwareBuildDate)) + "\",";
  payload += "\"fw_build_time\":\"" + JsonEscape(String(kFirmwareBuildTime)) + "\",";
  {
    char sbuf[16];
    auto fmtOrNull = [&](float v) -> String {
      if (!isfinite(v)) return "null";
      snprintf(sbuf, sizeof(sbuf), "%.2f", v);
      return String(sbuf);
    };
    payload += "\"temperature\":" + fmtOrNull(sensor_temp_f) + ",";
    payload += "\"temperature_f\":" + fmtOrNull(sensor_temp_f) + ",";
    const float tempC = isfinite(sensor_temp_f) ? (sensor_temp_f - 32.0f) * 5.0f / 9.0f : NAN;
    payload += "\"temperature_c\":" + fmtOrNull(tempC) + ",";
    payload += "\"battery_voltage\":" + fmtOrNull(sensor_battery_v) + ",";
    payload += "\"battery_voltage_v\":" + fmtOrNull(sensor_battery_v) + ",";
    payload += "\"battery_charge\":" + fmtOrNull(sensor_battery_pct) + ",";
    payload += "\"battery_charge_pct\":" + fmtOrNull(sensor_battery_pct) + ",";
    payload += "\"battery_time_remaining_h\":" + fmtOrNull(sensor_battery_time_remaining_h) + ",";
  }
  payload += "\"wifi_rssi_dbm\":" + String(wifi_rssi_dbm) + ",";
  payload += "\"uptime_s\":" + String(static_cast<uint32_t>(now / 1000)) + ",";
  payload += "\"uptime\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(now / 1000))) + "\",";
  payload += "\"uptime_hms\":\"" + JsonEscape(FormatDurationHms(static_cast<uint32_t>(now / 1000))) + "\",";
  payload += "\"ping_target\":\"" + JsonEscape(String(kPingTargetHost)) + "\",";
  payload += "\"ping\":{";
  payload += "\"target\":\"" + JsonEscape(String(kPingTargetHost)) + "\",";
  payload += "\"sent\":" + String(telemetry_ping_sent) + ",";
  payload += "\"recv\":" + String(telemetry_ping_count) + ",";
  payload += "\"lost\":" + String(telemetry_ping_lost) + ",";
  payload += "\"last_ms\":" + String(telemetry_ping_last_ms) + ",";
  payload += "\"min_ms\":" + String(telemetry_ping_min_ms) + ",";
  payload += "\"max_ms\":" + String(telemetry_ping_max_ms) + ",";
  payload += "\"avg_ms\":" + String(telemetry_ping_avg_ms) + ",";
  payload += "\"count\":" + String(telemetry_ping_count) + ",";
  payload += "\"last_ok\":" + String(telemetry_ping_last_ok ? "true" : "false") + ",";
  payload += "\"since_ok_s\":"
             + String(last_ping_success_ms > 0 ? static_cast<uint32_t>((now - last_ping_success_ms) / 1000) : 0)
             + ",";
  payload += "\"since_ok\":\""
             + JsonEscape(last_ping_success_ms > 0
                              ? FormatDurationHms(static_cast<uint32_t>((now - last_ping_success_ms) / 1000))
                              : String("--:--:--"))
             + "\"";
  payload += "}";
  payload += "},";
  payload += BuildDisplayPagesJson(now);
  payload += "}";
  return payload;
}

const char kWebUiHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Pylons Control</title>
  <style>
    :root{color-scheme:dark;background:#0a0d12;--ink:#e7edf6;--muted:#9db0c7;--panel:#121821;--panel-2:#0f141c;--accent:#4fb3ff;--accent-2:#1e7bbf;--line:#273445}
    *{box-sizing:border-box} body{margin:0;font-family:Georgia,serif;color:var(--ink);background:
      radial-gradient(circle at top,#162235 0,#0d121a 35%,#07090d 100%) fixed}
    main{max-width:1100px;margin:0 auto;padding:20px}
    h1,h2{margin:0 0 12px} .grid{display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(240px,1fr))}
    .panel{background:rgba(18,24,33,.94);border:1px solid var(--line);border-radius:16px;padding:16px;box-shadow:0 18px 40px rgba(0,0,0,.35)}
    .oled{background:linear-gradient(180deg,#0d131c 0,#080c12 100%);color:#d8e8ff;font-family:"Courier New",monospace;min-height:126px}
    .oled pre{margin:0;white-space:pre-wrap}
    .meta{display:grid;gap:8px}.row{display:flex;justify-content:space-between;gap:12px;border-top:1px solid var(--line);padding-top:8px}
    label{display:grid;gap:6px;color:var(--muted)}
    input{width:100%;background:var(--panel-2);color:var(--ink);border:1px solid var(--line);border-radius:10px;padding:10px 12px}
    input:focus{outline:2px solid rgba(79,179,255,.35);border-color:var(--accent)}
    button{border:0;border-radius:999px;background:linear-gradient(180deg,var(--accent) 0,var(--accent-2) 100%);color:#06111d;padding:12px 18px;font-size:16px;font-weight:700;cursor:pointer;transition:background .1s,box-shadow .1s,transform .1s}
    button:disabled{opacity:.5;cursor:wait}
    #trigger.held{background:linear-gradient(180deg,#f05a28 0,#c03a10 100%);color:#fff;box-shadow:0 0 28px rgba(240,90,40,.6);transform:scale(0.96)}
    #log{background:#05080d;color:#b9ffd6;min-height:260px;max-height:420px;overflow:auto;font:13px/1.35 Consolas,monospace;border-radius:12px;padding:12px;border:1px solid #193022}
    #log pre{margin:0;white-space:pre-wrap}
    .pill{display:inline-block;padding:4px 10px;border-radius:999px;background:#243244;color:#c9d7e7}
    .active{background:#f05a28;color:#fff}
  </style>
</head>
<body>
  <main>
    <div class="panel">
      <h1 id="page-title">Pylons Control</h1>
      <p><span id="solenoid" class="pill">Solenoid idle</span></p>
      <p id="fw-version"></p>
      <div style="display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:10px">
        <button id="trigger">Press and Hold Solenoid</button>
        <button id="identify-btn" style="background:linear-gradient(180deg,#b97af0 0,#7c3abf 100%);color:#fff">Blink LEDs (identify)</button>
        <span id="identify-status" style="color:var(--muted);font-size:14px"></span>
      </div>
      <div style="display:flex;gap:12px;flex-wrap:wrap;align-items:center">
        <button id="seq-pulse-once-btn">Pulse once (50 ms)</button>
        <button id="seq-pulse-5x-btn">Pulse 5&times; (50 ms)</button>
        <button id="seq-steam-btn" style="background:linear-gradient(180deg,#e8832a 0,#b85010 100%);color:#fff">&#x1F682; Steam engine (hold)</button>
        <span id="seq-status" style="color:var(--muted);font-size:14px"></span>
      </div>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Node Config</h2>
      <form id="config-form" class="meta">
        <label>ID <input id="cfg-id" name="id"></label>
        <label>Host <input id="cfg-host" name="host"></label>
        <label>Description <input id="cfg-description" name="description"></label>
        <label>Node Alias <input id="cfg-node" name="node" placeholder="sets id + host"></label>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:var(--muted);font-size:13px">WiFi Fallback</span>
          <label>SSID <input id="cfg-wifi-ssid" placeholder="leave blank to disable"></label>
          <label>Password <input id="cfg-wifi-pass" type="password" placeholder=""></label>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:var(--muted);font-size:13px">Solenoid Safety</span>
          <label>Failsafe timeout (s) <input id="cfg-failsafe-s" name="failsafe_s" type="number" min="1" max="60" step="0.1" style="width:80px"></label>
          <label>Index <input id="cfg-index" name="index" type="number" min="-99" max="99" step="1" style="width:60px"> <span style="color:var(--muted);font-size:12px">(barmode sequence order; negative=skip seq)</span></label>
          <div id="cfg-btn-disable-wrap" style="display:none;gap:10px;align-items:center">
            <span style="color:var(--muted);font-size:12px">Disable buttons:</span>
            <label style="color:#4caf50"><input type="checkbox" id="cfg-btn-dis-0" style="accent-color:#4caf50"> Green</label>
            <label style="color:#2196f3"><input type="checkbox" id="cfg-btn-dis-1" style="accent-color:#2196f3"> Blue</label>
            <label style="color:#ff9800"><input type="checkbox" id="cfg-btn-dis-2" style="accent-color:#ff9800"> Orange</label>
            <label style="color:#f44336"><input type="checkbox" id="cfg-btn-dis-3" style="accent-color:#f44336"> Red</label>
          </div>
          <div id="cfg-seq-max-wrap" style="display:none;gap:6px">
            <label>Green timeout (ms) <input id="cfg-green-timeout-ms" name="green_timeout_ms" type="number" min="50" max="10000" step="50" style="width:80px"> <span style="color:var(--muted);font-size:12px">(btn0 timed pulse open duration)</span></label>
            <label>All-4 valve open (ms) <input id="cfg-all4-valve-ms" name="all4_valve_ms" type="number" min="500" max="30000" step="500" style="width:80px"> <span style="color:var(--muted);font-size:12px">(all-4 hold: valve open duration before auto-close)</span></label>
            <label>Seq max (s) <input id="cfg-seq-max-s" name="seq_max_s" type="number" min="1" max="120" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(barmode btn1 hold timeout)</span></label>
            <label>Seq step decrement (ms) <input id="cfg-seq-dec-ms" name="seq_dec_ms" type="number" min="0" max="2000" step="10" style="width:70px"> <span style="color:var(--muted);font-size:12px">(delay reduction per pylon step)</span></label>
            <label>Seq exp factor (%) <input id="cfg-seq-exp-pct" name="seq_exp_pct" type="number" min="1" max="100" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(multiply delay each step; 100=linear only)</span></label>
          </div>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:flex;align-items:center;gap:10px">
          <input type="checkbox" id="cfg-ap" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
          <span style="color:var(--muted);font-size:14px">Enable WiFi AP &mdash; SSID: <code>PYLON_<em>id</em></code>, IP <code>10.1.2.3</code></span>
        </div>
        <div style="display:flex;gap:12px;align-items:center;flex-wrap:wrap">
          <button type="submit">Save Config</button>
          <span id="config-status"></span>
        </div>
      </form>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Firmware Update</h2>
      <div class="meta">
        <label>Firmware .bin
          <input type="file" id="ota-file" accept=".bin" style="padding:8px 12px;cursor:pointer">
        </label>
        <div>
          <div id="ota-progress-wrap" style="display:none;background:#0d131c;border-radius:8px;height:18px;overflow:hidden;border:1px solid var(--line);margin-bottom:8px">
            <div id="ota-progress-bar" style="height:100%;width:0%;background:linear-gradient(90deg,var(--accent-2),var(--accent));transition:width .15s ease"></div>
          </div>
          <div style="display:flex;gap:12px;align-items:center;flex-wrap:wrap">
            <button id="ota-btn" disabled>Upload &amp; Flash</button>
            <span id="ota-status" style="color:var(--muted);font-size:14px"></span>
          </div>
        </div>
      </div>
    </div>
    <div class="grid">
      <section class="panel oled"><h2>OLED Status</h2><pre id="oled-status"></pre></section>
      <section class="panel oled"><h2>OLED Ping</h2><pre id="oled-ping"></pre></section>
      <section class="panel oled"><h2>OLED Wi-Fi</h2><pre id="oled-wifi"></pre></section>
      <section class="panel oled"><h2>OLED Wi-Fi Detail</h2><pre id="oled-wifi-detail"></pre></section>
      <section class="panel oled"><h2>OLED Node</h2><pre id="oled-node"></pre></section>
      <section class="panel oled"><h2>OLED Firmware</h2><pre id="oled-firmware"></pre></section>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Battery Voltage — 30 min</h2>
      <img id="chart-short" src="/api/chart/battery/short" style="width:100%;border-radius:8px;display:block" alt="30-min battery chart">
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Battery Voltage — 24 h</h2>
      <img id="chart-long" src="/api/chart/battery/long" style="width:100%;border-radius:8px;display:block" alt="24-h battery chart">
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Temperature — 30 min</h2>
      <img id="chart-temp-short" src="/api/chart/temp/short" style="width:100%;border-radius:8px;display:block" alt="30-min temperature chart">
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Temperature — 24 h</h2>
      <img id="chart-temp-long" src="/api/chart/temp/long" style="width:100%;border-radius:8px;display:block" alt="24-h temperature chart">
    </div>
    <div id="registry-panel" class="panel" style="margin-top:16px;display:none">
      <h2>PYLON Registry</h2>
      <div id="registry-content" style="overflow-x:auto">
        <span style="color:var(--muted);font-size:13px">Loading...</span>
      </div>
    </div>
    <div id="btn-activity-panel" class="panel" style="margin-top:16px;display:none">
      <h2>Button Activity</h2>
      <div style="display:flex;gap:20px;margin-bottom:10px;font-size:14px">
        <span style="color:#4caf50">&#11044; Green&nbsp;<b id="cnt-0">0</b></span>
        <span style="color:#2196f3">&#11044; Blue&nbsp;<b id="cnt-1">0</b></span>
        <span style="color:#ff9800">&#11044; Orange&nbsp;<b id="cnt-2">0</b></span>
        <span style="color:#f44336">&#11044; Red&nbsp;<b id="cnt-3">0</b></span>
      </div>
      <div style="margin-bottom:6px">
        <span style="color:var(--muted);font-size:12px;margin-right:8px">Window:</span>
        <button class="win-btn" data-ms="600000">10m</button>
        <button class="win-btn" data-ms="3600000">1h</button>
        <button class="win-btn" data-ms="14400000">4h</button>
        <button class="win-btn" data-ms="43200000">12h</button>
      </div>
      <canvas id="btn-chart" height="120" style="width:100%;display:block;background:#111;border-radius:6px"></canvas>
      <div style="color:var(--muted);font-size:11px;margin-top:4px">Event timeline, newest at right. Each tick = one press. Up to 1024 events stored since boot.</div>
    </div>
    <div id="btn-ref-panel" class="panel" style="margin-top:16px;display:none">
      <h2>Button Reference</h2>
      <div style="display:grid;gap:12px;font-size:13px">
        <div>
          <div style="font-weight:600;margin-bottom:6px;color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.08em">Single Button Actions</div>
          <table style="border-collapse:collapse;width:100%">
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#4caf50;white-space:nowrap">&#11044; Green</td>
              <td style="padding:5px 0">Timed solenoid pulse — opens all pylon valves for the configured Green timeout, then closes automatically.</td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#2196f3;white-space:nowrap">&#11044; Blue tap</td>
              <td style="padding:5px 0">Single pulse — sends one 50 ms pulse to all pylons simultaneously.</td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#2196f3;white-space:nowrap">&#11044; Blue double-tap + hold</td>
              <td style="padding:5px 0">Sequential mode — fires pylons in index order, accelerating each loop until released or Seq max elapsed.</td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#ff9800;white-space:nowrap">&#11044; Orange</td>
              <td style="padding:5px 0">Pulse train — sends 5&times; 50 ms pulses to all pylons simultaneously.</td>
            </tr>
            <tr>
              <td style="padding:5px 10px 5px 0;color:#f44336;white-space:nowrap">&#11044; Red tap</td>
              <td style="padding:5px 0">Steam pulse — opens all pylon steam valves for 150 ms, then closes automatically.</td>
            </tr>
            <tr>
              <td style="padding:5px 10px 5px 0;color:#f44336;white-space:nowrap">&#11044; Red triple-tap + hold</td>
              <td style="padding:5px 0">Steam hold — tap, tap (suppressed), tap &amp; hold: opens all steam valves while held (ramping frequency); closes on release. Each tap must be within 300 ms of the previous.</td>
            </tr>
          </table>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:12px">
          <div style="font-weight:600;margin-bottom:8px;color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.08em">All-Valves Sequence (4-button unlock)</div>
          <div style="color:var(--muted);font-size:12px;margin-bottom:8px">Suppress all individual button actions. Steps must be completed in order without breaking prior holds.</div>
          <ol style="margin:0;padding:0 0 0 18px;display:grid;gap:8px">
            <li><span style="color:#2196f3;font-weight:600">Hold Blue 3 s</span> &rarr; Blue strobes 25% at 2 Hz; <span style="color:#4caf50">Green lamp ON</span>; Orange &amp; Red lamps off.</li>
            <li><span style="color:#4caf50;font-weight:600">Keep Blue + hold Green 3 s</span> &rarr; Blue &amp; Green strobe 50% at 4 Hz; <span style="color:#ff9800">Orange lamp ON</span>; Red lamp off.</li>
            <li><span style="color:#ff9800;font-weight:600">Keep Blue+Green + hold Orange 2 s</span> &rarr; Blue, Green &amp; Orange strobe 75% at 6 Hz; <span style="color:#f44336">Red lamp ON solid.</span></li>
            <li><span style="color:#f44336;font-weight:600">Keep all three + hold Red 2 s</span> &rarr; All four lamps strobe 80% at 8 Hz; <b>all pylon valves open.</b></li>
          </ol>
          <div style="margin-top:8px;color:var(--muted);font-size:12px">Valves close and all lamps extinguish when any button is released <em>or</em> after the configured <b>All-4 valve open</b> timeout. All buttons must then be released before another sequence can start.</div>
        </div>
      </div>
    </div>
    <div class="grid" style="margin-top:16px">
      <section class="panel"><h2>Telemetry</h2><div id="meta" class="meta"></div></section>
      <section class="panel"><h2>Serial Console</h2><div id="log"><pre id="log-text"></pre></div></section>
    </div>
  </main>
  <script>
    async function fetchJson(url, options) {
      const res = await fetch(url, options);
      if (!res.ok) throw new Error(await res.text());
      return res.json();
    }
    function esc(value) {
      return String(value ?? '').replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;');
    }
    function setPage(id, lines) {
      document.getElementById(id).textContent = (lines || []).join('\n');
    }
    function syncConfigField(id, nextValue) {
      const input = document.getElementById(id);
      if (!input) return;
      if (document.activeElement === input) return;
      if (input.dataset.dirty === 'true') return;
      input.value = nextValue ?? '';
    }
    let barmodeActive = false;
    function renderMeta(data) {
      barmodeActive = !!data.barmode_active;
      const rows = [
        ['Pylon ID', data.pylon_id],
        ['Description', data.description],
        ['Host', data.hostname],
        ['IP', data.ip],
        ['Firmware', data.fw_version],
        ['Temperature', data.telemetry.temperature || data.telemetry.temperature_f || 'N/A'],
        ['Battery V', data.telemetry.battery_voltage || data.telemetry.battery_voltage_v || 'N/A'],
        ['Battery %', data.telemetry.battery_charge || data.telemetry.battery_charge_pct || 'N/A'],
        ['RSSI', `${data.telemetry.wifi_rssi_dbm} dBm`],
        ['Ping', `${data.telemetry.ping.last_ok ? 'OK' : 'FAIL'} / ${data.telemetry.ping.last_ms} ms`],
        ['Target IP', data.target_ip || '--'],
        ['Triggers', String(data.trigger_event_count)],
        ['Valve open total', data.total_boosh_open_s != null ? data.total_boosh_open_s.toFixed(1) + ' s' : 'N/A'],
        ['Battery V', data.battery_voltage_v != null ? data.battery_voltage_v.toFixed(2) + ' V' : 'N/A'],
        ['Battery %', data.battery_charge_pct != null ? data.battery_charge_pct.toFixed(1) + ' %' : 'N/A'],
        ['Batt Time Left', data.battery_time_remaining_h != null ? data.battery_time_remaining_h.toFixed(1) + ' hr' : 'N/A'],
        ['Temperature', data.temperature_f != null ? data.temperature_f.toFixed(1) + ' °F' : 'N/A'],
        ['Uptime', data.telemetry.uptime_hms],
        ['Last Reset', data.last_reset_reason || '--']
      ];
      document.getElementById('meta').innerHTML = rows.map(([k,v]) => `<div class="row"><strong>${k}</strong><span>${v}</span></div>`).join('');
      document.getElementById('fw-version').textContent = `FW ${data.fw_version}`;
      updateBtnActivity(data.btn_press_counts);
      const title = `${data.pylon_id} Pylons Control`;
      document.getElementById('page-title').textContent = title;
      document.title = title;
      const pill = document.getElementById('solenoid');
      pill.textContent = data.solenoid_active ? 'Solenoid active' : 'Solenoid idle';
      pill.className = data.solenoid_active ? 'pill active' : 'pill';
      setPage('oled-status', data.display_pages.status);
      setPage('oled-ping', data.display_pages.ping);
      setPage('oled-wifi', data.display_pages.wifi);
      setPage('oled-wifi-detail', data.display_pages.wifi_detail);
      setPage('oled-node', data.display_pages.node);
      setPage('oled-firmware', data.display_pages.firmware);
      syncConfigField('cfg-id', data.pylon_id || '');
      syncConfigField('cfg-host', (data.hostname || '').replace(/\.local$/,''));
      syncConfigField('cfg-description', data.description || '');
      syncConfigField('cfg-wifi-ssid', data.wifi_ssid || '');
      const fsInput = document.getElementById('cfg-failsafe-s');
      if (fsInput && document.activeElement !== fsInput)
        fsInput.value = data.failsafe_ms != null ? (data.failsafe_ms / 1000).toFixed(1) : '5.0';
      const idxInput = document.getElementById('cfg-index');
      if (idxInput && document.activeElement !== idxInput)
        idxInput.value = data.pylon_index != null ? data.pylon_index : 0;
      const seqWrap = document.getElementById('cfg-seq-max-wrap');
      if (seqWrap) seqWrap.style.display = data.barmode_active ? 'grid' : 'none';
      const grnToInput = document.getElementById('cfg-green-timeout-ms');
      if (grnToInput && document.activeElement !== grnToInput)
        grnToInput.value = data.green_timeout_ms != null ? data.green_timeout_ms : 300;
      const all4VlvInput = document.getElementById('cfg-all4-valve-ms');
      if (all4VlvInput && document.activeElement !== all4VlvInput)
        all4VlvInput.value = data.all4_valve_ms != null ? data.all4_valve_ms : 3000;
      const seqInput = document.getElementById('cfg-seq-max-s');
      if (seqInput && document.activeElement !== seqInput)
        seqInput.value = data.seq_max_ms != null ? Math.round(data.seq_max_ms / 1000) : 30;
      const seqDecInput = document.getElementById('cfg-seq-dec-ms');
      if (seqDecInput && document.activeElement !== seqDecInput)
        seqDecInput.value = data.seq_dec_ms != null ? data.seq_dec_ms : 50;
      const seqExpInput = document.getElementById('cfg-seq-exp-pct');
      if (seqExpInput && document.activeElement !== seqExpInput)
        seqExpInput.value = data.seq_exp_pct != null ? data.seq_exp_pct : 100;
      const apBox = document.getElementById('cfg-ap');
      if (apBox && document.activeElement !== apBox) apBox.checked = !!data.ap_enabled;
      const disWrap = document.getElementById('cfg-btn-disable-wrap');
      if (disWrap) disWrap.style.display = barmodeActive ? 'flex' : 'none';
      if (barmodeActive && Array.isArray(data.btn_disabled)) {
        for (let i = 0; i < 4; i++) {
          const cb = document.getElementById('cfg-btn-dis-' + i);
          if (cb && document.activeElement !== cb) cb.checked = !!data.btn_disabled[i];
        }
      }
    }
    async function refreshTelemetry() {
      renderMeta(await fetchJson('/api/telemetry'));
    }
    async function refreshLogs() {
      const log = document.getElementById('log');
      const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 50;
      const data = await fetchJson('/api/logs');
      document.getElementById('log-text').textContent = data.log;
      if (atBottom) log.scrollTop = log.scrollHeight;
    }
    const triggerButton = document.getElementById('trigger');
    const configForm = document.getElementById('config-form');
    const configInputs = ['cfg-id', 'cfg-host', 'cfg-description', 'cfg-node', 'cfg-wifi-ssid', 'cfg-wifi-pass', 'cfg-failsafe-s', 'cfg-index', 'cfg-green-timeout-ms', 'cfg-all4-valve-ms', 'cfg-seq-max-s', 'cfg-seq-dec-ms', 'cfg-seq-exp-pct']
      .map((id) => document.getElementById(id))
      .filter(Boolean);
    let holdActive = false;

    configInputs.forEach((input) => {
      input.dataset.dirty = 'false';
      input.addEventListener('input', () => {
        input.dataset.dirty = 'true';
      });
    });

    async function setHeldState(active) {
      if (holdActive === active) return;
      holdActive = active;
      triggerButton.classList.toggle('held', active);
      await fetchJson(active ? '/api/solenoid/on' : '/api/solenoid/off',
        active ? {method:'POST'} : {method:'POST', keepalive:true});
      await refreshTelemetry();
    }

    function endHold() {
      void setHeldState(false);
    }

    triggerButton.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      triggerButton.setPointerCapture(event.pointerId);
      void setHeldState(true);
    });
    triggerButton.addEventListener('pointerup', endHold);
    triggerButton.addEventListener('pointercancel', endHold);
    triggerButton.addEventListener('lostpointercapture', endHold);
    triggerButton.addEventListener('mouseleave', endHold);
    triggerButton.addEventListener('blur', endHold);
    window.addEventListener('blur', endHold);
    document.getElementById('cfg-ap').addEventListener('change', async (e) => {
      await fetchJson('/api/config/ap', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: new URLSearchParams({value: e.target.checked ? 'true' : 'false'}).toString()
      });
    });
    window.addEventListener('pagehide', endHold);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) endHold();
    });
    configForm.addEventListener('submit', async (event) => {
      event.preventDefault();
      const status = document.getElementById('config-status');
      status.textContent = 'Saving...';
      const body = new URLSearchParams();
      const node = document.getElementById('cfg-node').value.trim();
      if (node) {
        body.set('node', node);
      } else {
        body.set('id', document.getElementById('cfg-id').value.trim());
        body.set('host', document.getElementById('cfg-host').value.trim());
      }
      body.set('description', document.getElementById('cfg-description').value.trim());
      const wifiSsid = document.getElementById('cfg-wifi-ssid').value.trim();
      if (wifiSsid) body.set('wifi_ssid', wifiSsid);
      const wifiPass = document.getElementById('cfg-wifi-pass').value;
      if (wifiPass) body.set('wifi_pass', wifiPass);
      const failsafeS = document.getElementById('cfg-failsafe-s').value.trim();
      if (failsafeS) body.set('failsafe_s', failsafeS);
      const idxVal = document.getElementById('cfg-index').value.trim();
      if (idxVal !== '') body.set('index', idxVal);
      const grnToVal = document.getElementById('cfg-green-timeout-ms').value.trim();
      if (grnToVal !== '') body.set('green_timeout_ms', grnToVal);
      const all4VlvVal = document.getElementById('cfg-all4-valve-ms').value.trim();
      if (all4VlvVal !== '') body.set('all4_valve_ms', all4VlvVal);
      const seqMaxVal = document.getElementById('cfg-seq-max-s').value.trim();
      if (seqMaxVal !== '') body.set('seq_max_s', seqMaxVal);
      const seqDecVal = document.getElementById('cfg-seq-dec-ms').value.trim();
      if (seqDecVal !== '') body.set('seq_dec_ms', seqDecVal);
      const seqExpVal = document.getElementById('cfg-seq-exp-pct').value.trim();
      if (seqExpVal !== '') body.set('seq_exp_pct', seqExpVal);
      if (barmodeActive) {
        let disStr = '';
        for (let i = 0; i < 4; i++) {
          const cb = document.getElementById('cfg-btn-dis-' + i);
          disStr += (cb && cb.checked) ? '1' : '0';
        }
        body.set('btn_disabled', disStr);
      }
      try {
        const result = await fetchJson('/api/config', {
          method:'POST',
          headers:{'Content-Type':'application/x-www-form-urlencoded'},
          body:body.toString()
        });
        status.textContent = `Saved ${result.config.hostname}`;
        configInputs.forEach((input) => {
          input.dataset.dirty = 'false';
        });
        document.getElementById('cfg-node').value = '';
        await refreshTelemetry();
        await refreshLogs();
      } catch (error) {
        status.innerHTML = `Save failed: ${esc(error.message)}`;
      }
    });
    // ---- Identify -----------------------------------------------------------
    const identifyBtn    = document.getElementById('identify-btn');
    const identifyStatus = document.getElementById('identify-status');
    identifyBtn.addEventListener('click', async () => {
      identifyBtn.disabled = true;
      identifyStatus.textContent = 'Blinking\u2026';
      try {
        await fetchJson('/api/identify', {method:'POST'});
        identifyStatus.textContent = '10 s';
        setTimeout(() => {
          identifyStatus.textContent = '';
          identifyBtn.disabled = false;
        }, 10000);
      } catch(e) {
        identifyStatus.textContent = 'failed';
        identifyBtn.disabled = false;
      }
    });

    // ---- Sequence buttons ---------------------------------------------------
    const seqStatus = document.getElementById('seq-status');

    async function runSeq(endpoint) {
      try {
        await fetchJson(endpoint, {method:'POST'});
      } catch(e) {
        seqStatus.textContent = 'failed: ' + esc(e.message);
      }
    }

    document.getElementById('seq-pulse-once-btn').addEventListener('click', async () => {
      seqStatus.textContent = '';
      await runSeq('/api/sequence/pulse_once');
    });

    document.getElementById('seq-pulse-5x-btn').addEventListener('click', async () => {
      seqStatus.textContent = '';
      await runSeq('/api/sequence/pulse_5x');
    });

    // Steam engine: hold to run, release to abort
    const steamBtn = document.getElementById('seq-steam-btn');
    let steamActive = false;
    let steamTimer = null;

    async function startSteam() {
      if (steamActive) return;
      steamActive = true;
      steamBtn.classList.add('held');
      seqStatus.textContent = 'Running\u2026';
      await runSeq('/api/sequence/steam');
      // Auto-clear status after 5.5 s (sequence is 5 s)
      steamTimer = setTimeout(() => {
        seqStatus.textContent = '';
        steamBtn.classList.remove('held');
        steamActive = false;
      }, 5500);
    }

    async function stopSteam() {
      if (!steamActive) return;
      steamActive = false;
      if (steamTimer) { clearTimeout(steamTimer); steamTimer = null; }
      steamBtn.classList.remove('held');
      seqStatus.textContent = '';
      await runSeq('/api/sequence/abort');
    }

    steamBtn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      steamBtn.setPointerCapture(e.pointerId);
      void startSteam();
    });
    steamBtn.addEventListener('pointerup', () => void stopSteam());
    steamBtn.addEventListener('pointercancel', () => void stopSteam());

    // ---- OTA upload ---------------------------------------------------------
    const otaFile   = document.getElementById('ota-file');
    const otaBtn    = document.getElementById('ota-btn');
    const otaStatus = document.getElementById('ota-status');
    const otaWrap   = document.getElementById('ota-progress-wrap');
    const otaBar    = document.getElementById('ota-progress-bar');

    otaFile.addEventListener('change', () => {
      otaBtn.disabled = !otaFile.files.length;
      otaStatus.textContent = otaFile.files.length ? otaFile.files[0].name : '';
    });

    otaBtn.addEventListener('click', () => {
      const file = otaFile.files[0];
      if (!file) return;
      if (!confirm(`Flash ${file.name} (${(file.size/1024).toFixed(1)} KB)?\nPylon will reboot after upload.`)) return;

      otaBtn.disabled = true;
      otaFile.disabled = true;
      otaWrap.style.display = 'block';
      otaBar.style.width = '0%';
      otaStatus.textContent = 'Uploading\u2026';

      const xhr = new XMLHttpRequest();
      xhr.open('POST', '/api/ota');

      xhr.upload.addEventListener('progress', (e) => {
        if (e.lengthComputable) {
          const pct = Math.round(e.loaded / e.total * 100);
          otaBar.style.width = pct + '%';
          otaStatus.textContent = `Uploading\u2026 ${pct}%`;
        }
      });

      xhr.addEventListener('load', () => {
        otaBar.style.width = '100%';
        try {
          const resp = JSON.parse(xhr.responseText);
          if (resp.ok) {
            otaBar.style.background = 'linear-gradient(90deg,#2a9d3e,#4ade70)';
            otaStatus.textContent = 'Flashed! Pylon rebooting\u2026';
          } else {
            otaBar.style.background = '#c03a10';
            otaStatus.textContent = 'Error: ' + esc(resp.error || 'unknown');
            otaBtn.disabled = false;
            otaFile.disabled = false;
          }
        } catch(_) {
          otaStatus.textContent = 'Upload complete (no JSON response)';
        }
      });

      xhr.addEventListener('error', () => {
        otaBar.style.background = '#c03a10';
        otaStatus.textContent = 'Upload failed (network error)';
        otaBtn.disabled = false;
        otaFile.disabled = false;
      });

      const form = new FormData();
      form.append('firmware', file, file.name);
      xhr.send(form);
    });

    function refreshCharts() {
      const t = Date.now();
      document.getElementById('chart-short').src      = '/api/chart/battery/short?t=' + t;
      document.getElementById('chart-long').src       = '/api/chart/battery/long?t='  + t;
      document.getElementById('chart-temp-short').src = '/api/chart/temp/short?t='    + t;
      document.getElementById('chart-temp-long').src  = '/api/chart/temp/long?t='     + t;
    }
    function renderRegistry(data) {
      const panel = document.getElementById('registry-panel');
      const el = document.getElementById('registry-content');
      if (!barmodeActive) { panel.style.display = 'none'; return; }
      panel.style.display = '';
      if (!data || data.status === 'pending') { el.innerHTML = '<span style="color:var(--muted);font-size:13px">Fetching registry...</span>'; return; }
      if (data.status === 'unavailable') { el.innerHTML = '<span style="color:var(--muted);font-size:13px">Unavailable</span>'; return; }
      const entries = (data.data && data.data.entries) || [];
      const offline = (data.data && data.data.offline_entries) || [];
      const all = [...entries, ...offline];
      if (all.length === 0) { el.innerHTML = '<span style="color:var(--muted);font-size:13px">No pylons registered</span>'; return; }
      const th = s => `<th style="text-align:left;padding:4px 10px;color:var(--muted);font-weight:500;font-size:12px;white-space:nowrap">${s}</th>`;
      const td = (s, style) => `<td style="padding:4px 10px;font-size:13px;white-space:nowrap${style?';'+style:''}">${s}</td>`;
      const badge = active => active
        ? '<span style="background:#1a3a1a;color:#4caf50;border-radius:6px;padding:1px 7px;font-size:11px">Online</span>'
        : '<span style="background:#2a1a1a;color:#e57373;border-radius:6px;padding:1px 7px;font-size:11px">Offline</span>';
      const fmt = (v, unit, dec=0) => (v == null || v === '') ? '-' : (+v).toFixed(dec) + unit;
      const rows = all.map(p => `<tr style="border-top:1px solid var(--line)">
        ${td('<b>'+esc(p.pylon_id||'?')+'</b>')}
        ${td(p.hostname ? `<a href="http://${esc(p.hostname)}" target="_blank" style="color:var(--accent)">${esc(p.hostname)}</a>` : '-')}
        ${td(esc(p.ip||'-'),'color:var(--muted)')}
        ${td(badge(p.active))}
        ${td(p.seconds_since_seen != null ? p.seconds_since_seen+'s ago' : '-','color:var(--muted)')}
        ${td(fmt(p.battery_charge_pct,'%',0))}
        ${td(fmt(p.battery_voltage_v,'V',2))}
        ${td(fmt(p.temperature_f,'\u00b0F',1))}
        ${td(fmt(p.wifi_rssi_dbm,' dBm',0))}
        ${td(p.ping_avg_ms != null ? p.ping_avg_ms+'ms' : '-')}
        ${td(esc(p.fw_version||'-'),'color:var(--muted);font-size:11px')}
      </tr>`).join('');
      el.innerHTML = `<table style="width:100%;border-collapse:collapse"><thead><tr>
        ${th('ID')}${th('Host')}${th('IP')}${th('Status')}${th('Seen')}${th('Bat%')}${th('BatV')}${th('Temp')}${th('RSSI')}${th('Ping')}${th('FW')}
      </tr></thead><tbody>${rows}</tbody></table>
      <div style="color:var(--muted);font-size:11px;margin-top:6px">${entries.length} online, ${offline.length} recently offline \u2014 ${esc((data.data&&data.data.server_time)||'')}</div>`;
    }
    async function refreshRegistry() {
      try { renderRegistry(await fetchJson('/api/registry')); }
      catch(e) { document.getElementById('registry-content').innerHTML = '<span style="color:#e57373;font-size:13px">Error: '+esc(e.message)+'</span>'; }
    }
    // --- Button activity chart ---
    const BTN_COLORS = ['#4caf50','#2196f3','#ff9800','#f44336'];
    const BTN_NAMES  = ['Green','Blue','Orange','Red'];
    let btnCounts  = [0,0,0,0];
    let btnEvents  = [];          // [{t: wall-clock ms, btn: 0-3}]
    let chartWinMs = 600000;      // default 10 min

    // Window selector buttons
    document.querySelectorAll('.win-btn').forEach(b => {
      b.addEventListener('click', () => {
        chartWinMs = parseInt(b.dataset.ms);
        document.querySelectorAll('.win-btn').forEach(x => x.style.fontWeight = '');
        b.style.fontWeight = 'bold';
        drawBtnChart();
      });
    });
    // Highlight default window button
    const defaultWinBtn = document.querySelector('.win-btn[data-ms="600000"]');
    if (defaultWinBtn) defaultWinBtn.style.fontWeight = 'bold';

    function updateBtnActivity(counts) {
      const panel = document.getElementById('btn-activity-panel');
      const refPanel = document.getElementById('btn-ref-panel');
      if (!counts || !barmodeActive) {
        if (panel) panel.style.display='none';
        if (refPanel) refPanel.style.display='none';
        return;
      }
      panel.style.display = '';
      if (refPanel) refPanel.style.display = '';
      for (let i = 0; i < 4; i++) {
        btnCounts[i] = counts[i] || 0;
        const el = document.getElementById('cnt-' + i);
        if (el) el.textContent = btnCounts[i];
      }
    }

    async function refreshBtnEvents() {
      if (!barmodeActive) return;
      try {
        const data = await fetchJson('/api/events');
        const nowWall = Date.now();
        const nowDev  = data.now_ms;
        btnEvents = (data.events || []).map(([ms, btn]) => ({
          t: nowWall - (nowDev - ms),
          btn
        }));
        drawBtnChart();
      } catch(e) {}
    }

    function drawBtnChart() {
      const canvas = document.getElementById('btn-chart');
      if (!canvas) return;
      const W = canvas.offsetWidth;
      if (!W) return;
      canvas.width = W;
      const H = canvas.height;
      const ctx = canvas.getContext('2d');
      const LP = 54, RP = 8, TP = 6, BP = 18;
      const CW = W - LP - RP, CH = H - TP - BP;
      const ROW = CH / 4;
      const now = Date.now();

      ctx.fillStyle = '#111'; ctx.fillRect(0, 0, W, H);

      // Row labels & grid
      for (let i = 0; i < 4; i++) {
        const y = TP + i * ROW;
        ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(LP, y); ctx.lineTo(W-RP, y); ctx.stroke();
        ctx.fillStyle = BTN_COLORS[i];
        ctx.font = '11px monospace'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        ctx.fillText(BTN_NAMES[i], LP-5, y + ROW/2);
      }
      ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(LP, TP+CH); ctx.lineTo(W-RP, TP+CH); ctx.stroke();

      // Event ticks
      for (const e of btnEvents) {
        const age = now - e.t;
        if (age > chartWinMs || age < 0) continue;
        const x = LP + CW * (1 - age / chartWinMs);
        const y = TP + e.btn * ROW;
        ctx.fillStyle = BTN_COLORS[e.btn];
        ctx.fillRect(Math.round(x)-2, y+2, 4, ROW-4);
      }

      // Time axis — pick sensible tick interval
      const winMin = chartWinMs / 60000;
      const tickMin = winMin <= 10 ? 2 : winMin <= 60 ? 10 : winMin <= 240 ? 30 : 60;
      const ticks = Math.floor(winMin / tickMin);
      ctx.font = '10px monospace'; ctx.textBaseline = 'top';
      for (let i = 0; i <= ticks; i++) {
        const frac = i / ticks;
        const x = LP + CW * frac;
        const minsAgo = Math.round(winMin * (1 - frac));
        const label = minsAgo === 0 ? 'now'
                    : minsAgo < 60 ? '-'+minsAgo+'m'
                    : '-'+(minsAgo/60).toFixed(minsAgo%60?1:0)+'h';
        ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, TP); ctx.lineTo(x, TP+CH); ctx.stroke();
        ctx.fillStyle = '#666'; ctx.textAlign = 'center';
        ctx.fillText(label, x, TP+CH+3);
      }
    }

    refreshTelemetry();
    refreshLogs();
    refreshCharts();
    refreshRegistry();
    refreshBtnEvents();
    setInterval(refreshTelemetry, 1000);
    setInterval(refreshLogs, 1500);
    setInterval(refreshCharts, 15000);
    setInterval(refreshRegistry, 10000);
    setInterval(refreshBtnEvents, 10000);
    setInterval(drawBtnChart, 5000);
  </script>
</body>
</html>
)HTML";

void HandleWebRoot() {
  webServer.send(200, "text/html", kWebUiHtml);
}

void HandleTelemetryApi() {
  webServer.send(200, "application/json", BuildTelemetryApiJson());
}

void HandleLogsApi() {
  String payload;
  payload.reserve(web_log_text.length() + 32);
  payload = "{\"log\":\"" + JsonEscape(web_log_text) + "\"}";
  webServer.send(200, "application/json", payload);
}

void HandleBtnEventsApi() {
  // Return ring buffer as [[ms,btn],...] in chronological order + current millis for wall-clock conversion
  String payload;
  payload.reserve(barmode_btn_event_count * 16 + 48);
  payload += "{\"now_ms\":";
  payload += String(millis());
  payload += ",\"events\":[";
  const int start = (barmode_btn_event_count < kBtnEventBufSize) ? 0 : barmode_btn_event_head;
  for (int i = 0; i < barmode_btn_event_count; i++) {
    const int idx = (start + i) % kBtnEventBufSize;
    if (i > 0) payload += ',';
    payload += '[';
    payload += String(barmode_btn_event_ms[idx]);
    payload += ',';
    payload += String(barmode_btn_event_btn[idx]);
    payload += ']';
  }
  payload += "]}";
  webServer.send(200, "application/json", payload);
}

void HandleBarModeRegistryApi() {
  if (!barmode_active) {
    webServer.send(200, "application/json", "{\"status\":\"unavailable\",\"reason\":\"barmode not active\"}");
    return;
  }
  if (!barmode_registry_mutex) {
    webServer.send(503, "application/json", "{\"status\":\"unavailable\"}");
    return;
  }
  String json;
  if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    json = barmode_registry_json;
    xSemaphoreGive(barmode_registry_mutex);
  }
  if (json.length() == 0) {
    webServer.send(503, "application/json", "{\"status\":\"pending\"}");
    return;
  }
  webServer.send(200, "application/json", json);
}

void SendApiError(int status_code, const String &message) {
  webServer.send(status_code, "application/json",
                 "{\"ok\":false,\"error\":\"" + JsonEscape(message) + "\"}");
}

void HandleConfigGetApi() {
  webServer.send(200, "application/json", BuildConfigApiJson());
}

void HandleConfigPostApi() {
  const bool has_node = webServer.hasArg("node");
  const bool has_id = webServer.hasArg("id");
  const bool has_host = webServer.hasArg("host");
  const bool has_desc = webServer.hasArg("description") || webServer.hasArg("desc");
  const bool has_wifi_ssid   = webServer.hasArg("wifi_ssid");
  const bool has_wifi_pass   = webServer.hasArg("wifi_pass");
  const bool has_failsafe_s  = webServer.hasArg("failsafe_s");
  const bool has_index       = webServer.hasArg("index");
  const bool has_seq_max_s   = webServer.hasArg("seq_max_s");
  const bool has_seq_dec_ms  = webServer.hasArg("seq_dec_ms");
  const bool has_seq_exp_pct  = webServer.hasArg("seq_exp_pct");
  const bool has_btn_disabled   = webServer.hasArg("btn_disabled");
  const bool has_green_timeout  = webServer.hasArg("green_timeout_ms");
  const bool has_all4_valve_ms  = webServer.hasArg("all4_valve_ms");

  if (!has_node && !has_id && !has_host && !has_desc &&
      !has_wifi_ssid && !has_wifi_pass && !has_failsafe_s && !has_index && !has_seq_max_s && !has_seq_dec_ms && !has_seq_exp_pct && !has_btn_disabled && !has_green_timeout && !has_all4_valve_ms) {
    SendApiError(400, "expected one of: node, id, host, description, wifi_ssid, wifi_pass, failsafe_s, index, seq_max_s, seq_dec_ms, seq_exp_pct, btn_disabled, green_timeout_ms, all4_valve_ms");
    return;
  }

  bool ok = true;
  if (has_node) {
    ok = ok && SetConfigFieldValue("node", webServer.arg("node"));
  } else {
    if (has_id) {
      ok = ok && SetConfigFieldValue("id", webServer.arg("id"));
    }
    if (has_host) {
      ok = ok && SetConfigFieldValue("host", webServer.arg("host"));
    }
  }
  if (has_desc) {
    ok = ok && SetConfigFieldValue("desc",
                                   webServer.hasArg("description") ? webServer.arg("description")
                                                                   : webServer.arg("desc"));
  }
  if (has_wifi_ssid)  ok = ok && SetConfigFieldValue("wifi_ssid",  webServer.arg("wifi_ssid"));
  if (has_wifi_pass)  ok = ok && SetConfigFieldValue("wifi_pass",  webServer.arg("wifi_pass"));
  if (has_failsafe_s) ok = ok && SetConfigFieldValue("failsafe_s", webServer.arg("failsafe_s"));
  if (has_index)      ok = ok && SetConfigFieldValue("index",      webServer.arg("index"));
  if (has_seq_max_s)  ok = ok && SetConfigFieldValue("seq_max_s",  webServer.arg("seq_max_s"));
  if (has_seq_dec_ms)  ok = ok && SetConfigFieldValue("seq_dec_ms",  webServer.arg("seq_dec_ms"));
  if (has_seq_exp_pct)   ok = ok && SetConfigFieldValue("seq_exp_pct",     webServer.arg("seq_exp_pct"));
  if (has_green_timeout) ok = ok && SetConfigFieldValue("green_timeout_ms", webServer.arg("green_timeout_ms"));
  if (has_all4_valve_ms) ok = ok && SetConfigFieldValue("all4_valve_ms",    webServer.arg("all4_valve_ms"));
  if (has_btn_disabled) {
    // Accepts "0101" bitmask string: index 0=green,1=blue,2=orange,3=red; '1'=disabled
    const String v = webServer.arg("btn_disabled");
    for (int i = 0; i < 4 && i < (int)v.length(); i++)
      barmode_btn_disabled[i] = (v[i] == '1');
    SavePylonConfig();
  }

  if (!ok) {
    SendApiError(400, "invalid config value");
    return;
  }

  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void HandleConfigIdApi() {
  if (!webServer.hasArg("value")) {
    SendApiError(400, "missing value");
    return;
  }
  if (!SetConfigFieldValue("id", webServer.arg("value"))) {
    SendApiError(400, "invalid id");
    return;
  }
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void HandleConfigHostApi() {
  if (!webServer.hasArg("value")) {
    SendApiError(400, "missing value");
    return;
  }
  if (!SetConfigFieldValue("host", webServer.arg("value"))) {
    SendApiError(400, "invalid host");
    return;
  }
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void HandleConfigDescApi() {
  if (!webServer.hasArg("value")) {
    SendApiError(400, "missing value");
    return;
  }
  if (!SetConfigFieldValue("desc", webServer.arg("value"))) {
    SendApiError(400, "invalid description");
    return;
  }
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void HandleConfigNodeApi() {
  if (!webServer.hasArg("value")) {
    SendApiError(400, "missing value");
    return;
  }
  if (!SetConfigFieldValue("node", webServer.arg("value"))) {
    SendApiError(400, "invalid node");
    return;
  }
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void SetupWebServer();  // forward declaration
void PingTask(void *);   // forward declaration
void StartSequence(SeqType type);  // forward declaration
void AbortSequence();              // forward declaration

void HandleCaptivePortalRedirect() {
  webServer.sendHeader("Location", "http://10.1.2.3/");
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(302, "text/plain", "");
}

void HandleConfigApApi() {
  if (!webServer.hasArg("value")) {
    SendApiError(400, "missing value");
    return;
  }
  if (!SetConfigFieldValue("ap", webServer.arg("value"))) {
    SendApiError(400, "invalid ap value");
    return;
  }
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"config\":" + BuildConfigApiJson() + "}");
}

void SetupApMode() {
  const String ssid = "PYLON_" + pylon_id;
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  WiFi.softAPConfig(IPAddress(10, 1, 2, 3), IPAddress(10, 1, 2, 3), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid.c_str());  // no password
  delay(100);
  dnsServer.start(53, "*", IPAddress(10, 1, 2, 3));
  ap_active = true;
  Console.print("[AP] started SSID: ");
  Console.println(ssid);
  Console.println("[AP] IP: 10.1.2.3, captive DNS active");
  SetupWebServer();
  // Captive portal detection paths for iOS/Android/Windows
  webServer.on("/generate_204", HTTP_GET, HandleCaptivePortalRedirect);
  webServer.on("/hotspot-detect.html", HTTP_GET, HandleCaptivePortalRedirect);
  webServer.on("/ncsi.txt", HTTP_GET, HandleCaptivePortalRedirect);
  webServer.on("/connecttest.txt", HTTP_GET, HandleCaptivePortalRedirect);
  webServer.onNotFound(HandleCaptivePortalRedirect);
}

void StopApMode() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
  }
  ap_active = false;
  Console.println("[AP] stopped");
}

void HandleSeqPulseOnceApi() {
  StartSequence(SEQ_PULSE_ONCE);
  webServer.send(200, "application/json", "{\"ok\":true,\"seq\":\"pulse_once\"}");
}

void HandleSeqPulse5xApi() {
  StartSequence(SEQ_PULSE_5X);
  webServer.send(200, "application/json", "{\"ok\":true,\"seq\":\"pulse_5x\"}");
}

void HandleSeqSteamApi() {
  StartSequence(SEQ_STEAM);
  webServer.send(200, "application/json", "{\"ok\":true,\"seq\":\"steam\"}");
}

void HandleSeqAbortApi() {
  AbortSequence();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void HandleIdentifyApi() {
  identify_until_ms = millis() + 10000;
  Console.println("[IDENTIFY] 10s blink started");
  webServer.send(200, "application/json", "{\"ok\":true,\"duration_s\":10}");
}

void HandleSolenoidOnApi() {
  SetBooshActive(true, "http");
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"solenoid_active\":true,\"triggered_via\":\"http\"}");
}

void HandleSolenoidOffApi() {
  AbortSequence();
  SetBooshActive(false, "http");
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"solenoid_active\":false,\"triggered_via\":\"http\"}");
}

// ---- Chart SVG (battery and temperature) ------------------------------------

// Pick a human-friendly grid step for a given data range
static float NiceStep(float range) {
  if (range <= 0.0f) return 1.0f;
  const float raw = range / 5.0f;
  const float mag = powf(10.0f, floorf(log10f(raw)));
  const float n   = raw / mag;
  if (n <= 1.0f) return 1.0f * mag;
  if (n <= 2.0f) return 2.0f * mag;
  if (n <= 5.0f) return 5.0f * mag;
  return 10.0f * mag;
}

// General time-series SVG chart.
// Pass y_min=NAN / y_max=NAN for auto-scale from data.
// val_fmt: snprintf format for the current-value label (e.g. "%.2fV" or "%.1f\xc2\xb0""F")
// stroke:  SVG color string for the polyline and value label
String BuildChartSvg(BattPlotPoint *buf, int head, int count, int capacity,
                     unsigned long span_label_ms, const char *title,
                     float y_min, float y_max,
                     const char *val_fmt, const char *stroke) {
  constexpr int W = 720, H = 200;
  constexpr int PL = 46, PR = 38, PT = 20, PB = 30;
  constexpr int PW = W - PL - PR;
  constexpr int PH = H - PT - PB;

  const int oldest_idx_pre = (head - count + capacity) % capacity;

  // Auto-scale: derive y_min / y_max from data
  const bool auto_scale = isnan(y_min) || isnan(y_max);
  if (auto_scale) {
    if (count >= 2) {
      float dmin = buf[oldest_idx_pre].v, dmax = buf[oldest_idx_pre].v;
      for (int i = 1; i < count; ++i) {
        const float v = buf[(oldest_idx_pre + i) % capacity].v;
        if (v < dmin) dmin = v;
        if (v > dmax) dmax = v;
      }
      float rng = dmax - dmin;
      if (rng < 1.0f) rng = 1.0f;
      const float pad = rng * 0.12f;
      y_min = dmin - pad;
      y_max = dmax + pad;
    } else {
      y_min = 0.0f; y_max = 100.0f;
    }
  }
  const float y_range = y_max - y_min;

  String s;
  s.reserve(7000);
  char buf8[64];

  s += "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 ";
  s += W; s += " "; s += H;
  s += "' style='background:#080c12;display:block;width:100%;max-width:720px'>";

  // Title
  s += "<text x='"; s += W / 2; s += "' y='14' text-anchor='middle' "
       "fill='#7a9bb5' font-size='11' font-family='monospace'>"; s += title; s += "</text>";

  // Y gridlines with nice step
  const float step = NiceStep(y_range);
  for (float gv = ceilf(y_min / step) * step; gv <= y_max + step * 0.01f; gv += step) {
    const int gy = PT + PH - (int)((gv - y_min) / y_range * PH);
    s += "<line x1='"; s += PL; s += "' y1='"; s += gy;
    s += "' x2='"; s += (W - PR); s += "' y2='"; s += gy;
    s += "' stroke='#1a2635' stroke-width='1'/>";
    if (step >= 1.0f) snprintf(buf8, sizeof(buf8), "%d", (int)roundf(gv));
    else              snprintf(buf8, sizeof(buf8), "%.1f", gv);
    s += "<text x='"; s += (PL - 4); s += "' y='"; s += (gy + 4);
    s += "' text-anchor='end' fill='#4a6175' font-size='10' font-family='monospace'>";
    s += buf8; s += "</text>";
  }

  // X axis baseline
  s += "<line x1='"; s += PL; s += "' y1='"; s += (PT + PH);
  s += "' x2='"; s += (W - PR); s += "' y2='"; s += (PT + PH);
  s += "' stroke='#273445' stroke-width='1'/>";

  // X time labels (5 ticks)
  for (int i = 0; i <= 4; ++i) {
    const int x = PL + (int)((float)i / 4.0f * PW);
    const unsigned long ago_ms = (unsigned long)((1.0f - (float)i / 4.0f) * span_label_ms);
    if (ago_ms == 0)                      snprintf(buf8, sizeof(buf8), "now");
    else if (span_label_ms >= 3600000UL)  snprintf(buf8, sizeof(buf8), "%ldh ago", (long)(ago_ms / 3600000UL));
    else                                  snprintf(buf8, sizeof(buf8), "%ldm ago", (long)(ago_ms / 60000UL));
    s += "<text x='"; s += x; s += "' y='"; s += (PT + PH + 14);
    s += "' text-anchor='middle' fill='#4a6175' font-size='10' font-family='monospace'>";
    s += buf8; s += "</text>";
  }

  if (count < 2) {
    s += "<text x='"; s += W / 2; s += "' y='"; s += (H / 2 + 5);
    s += "' text-anchor='middle' fill='#3a5060' font-size='13' font-family='monospace'>"
         "No data yet</text></svg>";
    return s;
  }

  const int oldest_idx = oldest_idx_pre;
  const uint32_t t_oldest = buf[oldest_idx].ms;
  const int newest_idx = (head - 1 + capacity) % capacity;
  const uint32_t t_span = buf[newest_idx].ms - t_oldest;
  if (t_span == 0) { s += "</svg>"; return s; }

  // Polyline
  s += "<polyline fill='none' stroke='"; s += stroke;
  s += "' stroke-width='1.5' stroke-linejoin='round' points='";
  for (int i = 0; i < count; ++i) {
    const int idx = (oldest_idx + i) % capacity;
    const float xf = (float)(buf[idx].ms - t_oldest) / (float)t_span;
    const float yf = (buf[idx].v - y_min) / y_range;
    snprintf(buf8, sizeof(buf8), "%d,%d ", PL + (int)(xf * PW),
             PT + PH - (int)(constrain(yf, 0.0f, 1.0f) * PH));
    s += buf8;
  }
  s += "'/>";

  // Current value label
  const float v_last  = buf[newest_idx].v;
  const float yf_last = (v_last - y_min) / y_range;
  const int py_last   = PT + PH - (int)(constrain(yf_last, 0.0f, 1.0f) * PH);
  snprintf(buf8, sizeof(buf8), val_fmt, v_last);
  s += "<text x='"; s += (W - PR + 3); s += "' y='"; s += (py_last + 4);
  s += "' fill='"; s += stroke;
  s += "' font-size='10' font-family='monospace'>"; s += buf8; s += "</text>";

  s += "</svg>";
  return s;
}

void HandleChartBatteryLongApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildChartSvg(battery_plot_long, battery_plot_long_head, battery_plot_long_count,
                  kBattPlotLongSize, 86400000UL, "Battery voltage \xe2\x80\x94 24 h (1 min/sample)",
                  10.0f, 15.0f, "%.2fV", "#4fb3ff"));
}

void HandleChartBatteryShortApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildChartSvg(battery_plot_short, battery_plot_short_head, battery_plot_short_count,
                  kBattPlotShortSize, 1800000UL, "Battery voltage \xe2\x80\x94 30 min (10 s/sample)",
                  10.0f, 15.0f, "%.2fV", "#4fb3ff"));
}

void HandleChartTempLongApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildChartSvg(temp_plot_long, temp_plot_long_head, temp_plot_long_count,
                  kTempPlotLongSize, 86400000UL, "Temperature \xe2\x80\x94 24 h (1 min/sample)",
                  NAN, NAN, "%.1f\xc2\xb0""F", "#7ce3b0"));
}

void HandleChartTempShortApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildChartSvg(temp_plot_short, temp_plot_short_head, temp_plot_short_count,
                  kTempPlotShortSize, 1800000UL, "Temperature \xe2\x80\x94 30 min (10 s/sample)",
                  NAN, NAN, "%.1f\xc2\xb0""F", "#7ce3b0"));
}

// ---- OTA update handlers ----------------------------------------------------

// Pulses green LED at 10 Hz during OTA. Called repeatedly during UPLOAD_FILE_WRITE.
void PollOtaDisplay() {
  static unsigned long last_toggle_ms = 0;
  static bool green_on = false;
  const unsigned long now = millis();
  if (now - last_toggle_ms < 50) return;
  last_toggle_ms = now;
  green_on = !green_on;
  ledcWrite(0, green_on ? 220 : 0);
}

void HandleOtaUploadBody() {
  HTTPUpload &upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Console.printf("[OTA] Start: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
    ledcWrite(0, 0);  // green — driven by PollOtaDisplay()
    ledcWrite(1, 0);  // blue off
    ledcWrite(2, 0);  // yellow off
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("OTA...");
    display.display();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Console.printf("[OTA] begin() failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    PollOtaDisplay();
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Console.printf("[OTA] write() failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    ledcWrite(0, 0);
    ShowStatus("OTA done", "Rebooting...");
    if (Update.end(true)) {
      Console.printf("[OTA] Success: %u bytes written. Rebooting.\n", upload.totalSize);
    } else {
      Console.printf("[OTA] end() failed: %s\n", Update.errorString());
    }
  }
}

void HandleOtaApi() {
  if (Update.hasError()) {
    String err = Update.errorString();
    webServer.send(500, "application/json",
                   "{\"ok\":false,\"error\":\"" + err + "\"}");
    Console.printf("[OTA] Aborted: %s\n", err.c_str());
    return;
  }
  webServer.send(200, "application/json", "{\"ok\":true,\"rebooting\":true}");
  delay(200);
  ESP.restart();
}

void SetupWebServer() {
  if (web_server_started) {
    return;
  }
  webServer.on("/", HTTP_GET, HandleWebRoot);
  webServer.on("/api/telemetry", HTTP_GET, HandleTelemetryApi);
  webServer.on("/api/logs", HTTP_GET, HandleLogsApi);
  webServer.on("/api/config", HTTP_GET, HandleConfigGetApi);
  webServer.on("/api/config", HTTP_POST, HandleConfigPostApi);
  webServer.on("/api/config/id", HTTP_POST, HandleConfigIdApi);
  webServer.on("/api/config/host", HTTP_POST, HandleConfigHostApi);
  webServer.on("/api/config/desc", HTTP_POST, HandleConfigDescApi);
  webServer.on("/api/config/node", HTTP_POST, HandleConfigNodeApi);
  webServer.on("/api/config/ap", HTTP_POST, HandleConfigApApi);
  webServer.on("/api/solenoid/on", HTTP_POST, HandleSolenoidOnApi);
  webServer.on("/api/solenoid/off", HTTP_POST, HandleSolenoidOffApi);
  webServer.on("/api/solenoid/trigger", HTTP_POST, HandleSolenoidOnApi);
  webServer.on("/api/chart/battery/long",  HTTP_GET, HandleChartBatteryLongApi);
  webServer.on("/api/chart/battery/short", HTTP_GET, HandleChartBatteryShortApi);
  webServer.on("/api/chart/temp/long",     HTTP_GET, HandleChartTempLongApi);
  webServer.on("/api/chart/temp/short",    HTTP_GET, HandleChartTempShortApi);
  webServer.on("/api/registry", HTTP_GET, HandleBarModeRegistryApi);
  webServer.on("/api/events",   HTTP_GET, HandleBtnEventsApi);
  webServer.on("/api/ota", HTTP_POST, HandleOtaApi, HandleOtaUploadBody);
  webServer.on("/api/identify", HTTP_POST, HandleIdentifyApi);
  webServer.on("/api/sequence/pulse_once", HTTP_POST, HandleSeqPulseOnceApi);
  webServer.on("/api/sequence/pulse_5x", HTTP_POST, HandleSeqPulse5xApi);
  webServer.on("/api/sequence/steam", HTTP_POST, HandleSeqSteamApi);
  webServer.on("/api/sequence/abort", HTTP_POST, HandleSeqAbortApi);
  webServer.begin();
  web_server_started = true;
  Console.println("HTTP server listening on port 80");
}

void PollDevBoardButton() {
  static bool lastPressed = false;
  const bool pressed = digitalRead(kDevBoardButtonPin) == LOW;
  if (pressed == lastPressed) {
    return;
  }

  lastPressed = pressed;
  if (pressed) {
    Console.println("Dev board button 0 pressed -> BooshMain ON");
    SetBooshActive(true, "button");
    return;
  }

  Console.println("Dev board button 0 released -> BooshMain OFF");
  SetBooshActive(false, "button");
}

void setup() {
  // Green LED on immediately as a power/boot indicator — before anything else.
  ledcSetup(0, 5000, 8);
  ledcAttachPin(kLedGreenPin, 0);
  ledcWrite(0, 255);

  USB.begin();
  Serial.begin(115200);
  esp_task_wdt_delete(NULL);  // remove loopTask from watchdog; blocking calls (ping, HTTP) are intentional
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 2000) {
    delay(10);
  }
  serial_cli_line.reserve(192);
  LoadPylonConfig();
  barmode_registry_mutex = xSemaphoreCreateMutex();
  barmode_active = pylon_description.indexOf(kBarModeToken) >= 0;
  if (barmode_active) {
    for (int i = 0; i < 4; i++) pinMode(kBarModeButtonPins[i], INPUT_PULLDOWN);
    barmode_registry_fetch_now = true;  // trigger immediate fetch on first PingTask run
    Console.println("[BarMode] ACTIVE — buttons on IO1,IO2,IO5,IO6");
    display.setRotation(2);  // 180° — display mounted upside-down in bar hardware
    // Button lamp PWM outputs: ch4=IO37(green), ch5=IO36(blue), ch6=IO34(red)
    ledcSetup(4, 5000, 8); ledcAttachPin(37, 4); ledcWrite(4, 0);
    ledcSetup(5, 5000, 8); ledcAttachPin(36, 5); ledcWrite(5, 0);
    ledcSetup(6, 5000, 8); ledcAttachPin(34, 6); ledcWrite(6, 0);
  }
  PrintCliHelp();
  pinMode(kDevBoardButtonPin, INPUT_PULLUP);
  pinMode(kBatteryAdcPin, INPUT);
  pinMode(kThermistorAdcPin, INPUT);
  analogReadResolution(12);

  pinMode(kLedWhitePin, OUTPUT);
  digitalWrite(kLedWhitePin, LOW);
  // Green already on from top of setup(). Blue/Yellow: 5kHz PWM carrier, duty updated in loop().
  ledcSetup(1, 5000, 8);
  ledcAttachPin(kLedBluePin, 1);
  ledcWrite(1, 0);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(kLedYellowPin, 2);
  ledcWrite(2, 0);

  if (barmode_active) {
    ledcSetup(3, 5000, 8); ledcAttachPin(35, 3); ledcWrite(3, 0);  // orange lamp ch3
  }

  pinMode(kIo38Pin, OUTPUT);
  digitalWrite(kIo38Pin, LOW);
  pinMode(kIo11Pin, OUTPUT);
  digitalWrite(kIo11Pin, LOW);

  Console.println("Boot: init I2C");
  Wire.begin(kI2cSda, kI2cScl);
  Console.println("Boot: init OLED");
  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledAddress)) {
    Console.println("SSD1306 init failed.");
  } else {
    display.invertDisplay(false);
    LogBootStep("Boot: OLED ready", "WEMOS S2 Pico");
  }

  Console.println("Boot: register WiFi events");
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        wifi_connected_since_ms = millis();
        wifi_has_ip = true;
        digitalWrite(kIo38Pin, HIGH);
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        last_disconnect_reason = info.wifi_sta_disconnected.reason;
        wifi_connected_since_ms = 0;
        wifi_has_ip = false;
        digitalWrite(kIo38Pin, LOW);
        break;
      default:
        break;
    }
  });

  LogBootStep("Boot: WiFi STA mode");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(100);

  LogBootStep("WiFi scan...");
  WiFi.scanNetworks(true, true);  // async — pulse blue while waiting
  while (WiFi.scanComplete() < 0) {
    const unsigned long now_s = millis();
    ledcWrite(1, ((now_s / 125) % 2 == 0) ? 200 : 0);
    delay(25);
  }
  ledcWrite(1, 0);
  int networkCount = (int)WiFi.scanComplete();
  if (networkCount < 0) networkCount = 0;
  Console.print("WiFi scan count: ");
  Console.println(networkCount);
  bool hasLowLatency = false;
  for (int i = 0; i < networkCount; ++i) {
    String ssid = WiFi.SSID(i);
    if (ssid == BOOSH_WIFI_SSID_LL) {
      hasLowLatency = true;
      break;
    }
  }

  // Try networks in priority order: LL → MW → user-defined
  struct { const char *ssid; const char *pass; } networks[3] = {
    { hasLowLatency ? BOOSH_WIFI_SSID_LL : BOOSH_WIFI_SSID_MW,
      hasLowLatency ? BOOSH_WIFI_PASS_LL : BOOSH_WIFI_PASS_MW },
    { hasLowLatency ? BOOSH_WIFI_SSID_MW : nullptr, hasLowLatency ? BOOSH_WIFI_PASS_MW : nullptr },
    { user_wifi_ssid.length() > 0 ? user_wifi_ssid.c_str() : nullptr, user_wifi_pass.c_str() },
  };
  const unsigned long kPerNetworkTimeoutMs = 15000;
  for (int ni = 0; ni < 3 && WiFi.status() != WL_CONNECTED; ++ni) {
    if (networks[ni].ssid == nullptr) continue;
    Console.print("Connecting to ");
    Console.println(networks[ni].ssid);
    ShowStatus("Connecting to", String(networks[ni].ssid));
    WiFi.begin(networks[ni].ssid, networks[ni].pass);
    unsigned long start = millis();
    unsigned long last_dot_ms = start;
    while (WiFi.status() != WL_CONNECTED && millis() - start < kPerNetworkTimeoutMs) {
      const unsigned long now_w = millis();
      ledcWrite(1, ((now_w / 125) % 2 == 0) ? 200 : 0);  // blue 4Hz
      delay(25);
      if (millis() - last_dot_ms >= 500) { last_dot_ms = millis(); Console.print("."); }
    }
    ledcWrite(1, 0);  // blue off when done searching
    if (WiFi.status() != WL_CONNECTED) {
      Console.println();
      Console.print("Failed: ");
      Console.println(networks[ni].ssid);
      WiFi.disconnect(false, false);
      delay(100);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Console.println();
    Console.print("Connected. IP: ");
    Console.println(WiFi.localIP());
    ShowStatus("WiFi connected", WiFi.localIP().toString());

    if (MDNS.begin(pylon_mdns_host.c_str())) {
      Console.print("mDNS: ");
      Console.print(pylon_mdns_host);
      Console.println(".local");
      MDNS.addService("http", "tcp", 80);
    } else {
      Console.println("mDNS init failed.");
    }

    if (oscUdp.begin(kOscPort)) {
      Console.print("OSC listening on port ");
      Console.println(kOscPort);
    } else {
      Console.println("OSC UDP begin failed.");
    }
    SetupWebServer();
  } else {
    Console.println();
    Console.println("WiFi connect failed.");
    ShowStatus("WiFi failed", "Starting AP");
    if (!ap_enabled) {
      ap_enabled = true;
      SavePylonConfig();
      Console.println("[AP] auto-enabled: no WiFi network found");
    }
  }

  if (ap_enabled) {
    SetupApMode();
  }

  // Ping and hostname resolution run in a background task so the main loop
  // never blocks on network I/O.
  xTaskCreatePinnedToCore(PingTask, "ping", 4096, nullptr, 1, nullptr, 0);
}

void HandleOscMessage(OSCMessage &msg) {
  // /pylon/BooshMain  1.0=open  0.0=close  (raw solenoid control)
  if (msg.fullMatch(kOscAddress)) {
    if (msg.size() != 1 || !msg.isFloat(0)) {
      Console.println("OSC BooshMain: ignored (unexpected args)");
      return;
    }
    const float v = msg.getFloat(0);
    Console.printf("OSC BooshMain %.3f\n", v);
    ApplyBooshState(v, "osc");
    return;
  }

  // /pylon/BooshPulseSingle  1.0 = trigger single 50 ms pulse
  if (msg.fullMatch(kOscAddrPulseSingle)) {
    if (msg.size() == 1 && msg.isFloat(0) && msg.getFloat(0) >= 0.5f) {
      Console.println("OSC BooshPulseSingle → SEQ_PULSE_ONCE");
      StartSequence(SEQ_PULSE_ONCE);
    }
    return;
  }

  // /pylon/BooshPulseTrain  1.0 = trigger 5× 50 ms pulses
  if (msg.fullMatch(kOscAddrPulseTrain)) {
    if (msg.size() == 1 && msg.isFloat(0) && msg.getFloat(0) >= 0.5f) {
      Console.println("OSC BooshPulseTrain → SEQ_PULSE_5X");
      StartSequence(SEQ_PULSE_5X);
    }
    return;
  }

  // /pylon/BooshSteam  1.0=start  0.0=stop
  if (msg.fullMatch(kOscAddrSteam)) {
    if (msg.size() == 1 && msg.isFloat(0)) {
      if (msg.getFloat(0) >= 0.5f) {
        Console.println("OSC BooshSteam 1.0 → SEQ_STEAM start");
        StartSequence(SEQ_STEAM);
      } else {
        Console.println("OSC BooshSteam 0.0 → abort");
        AbortSequence();
      }
    }
    return;
  }
}

const char *OscErrorToString(OSCErrorCode code) {
  switch (code) {
    case OSC_OK:
      return "OSC_OK";
    case BUFFER_FULL:
      return "BUFFER_FULL";
    case INVALID_OSC:
      return "INVALID_OSC";
    case ALLOCFAILED:
      return "ALLOCFAILED";
    case INDEX_OUT_OF_BOUNDS:
      return "INDEX_OUT_OF_BOUNDS";
    default:
      return "UNKNOWN";
  }
}

void PrintOscParseError(OSCMessage &msg, int packetSize, int bytesRead, const uint8_t *raw,
                        int rawCount) {
  const OSCErrorCode error = msg.getError();
  Console.print("OSC parse error: code=");
  Console.print(static_cast<int>(error));
  Console.print(" (");
  Console.print(OscErrorToString(error));
  Console.print("), packetSize=");
  Console.print(packetSize);
  Console.print(", bytesRead=");
  Console.print(bytesRead);

  const char *address = msg.getAddress();
  if (address != nullptr && address[0] != '\0') {
    Console.print(", address=");
    Console.print(address);
  }

  if (raw != nullptr && rawCount > 0) {
    Console.print(", raw[");
    Console.print(rawCount);
    Console.print("]=");
    for (int i = 0; i < rawCount; ++i) {
      if (i > 0) {
        Console.print(' ');
      }
      if (raw[i] < 0x10) {
        Console.print('0');
      }
      Console.print(raw[i], HEX);
    }
  }

  Console.println();
}

void PollOsc() {
  int packetSize = oscUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  const int originalPacketSize = packetSize;
  constexpr int kOscRawDumpMax = 128;
  uint8_t raw[kOscRawDumpMax];
  int rawCount = 0;

  OSCMessage msg;
  while (packetSize-- > 0) {
    int value = oscUdp.read();
    if (value < 0) {
      break;
    }
    uint8_t byteValue = static_cast<uint8_t>(value);
    if (rawCount < kOscRawDumpMax) {
      raw[rawCount++] = byteValue;
    }
    msg.fill(byteValue);
  }

  if (msg.hasError()) {
    PrintOscParseError(msg, originalPacketSize, rawCount, raw, rawCount);
    return;
  }

  HandleOscMessage(msg);
}

// ---- Sensor readings --------------------------------------------------------

// LiFePO4 4S OCV → SOC lookup table (open-circuit / resting voltage).
// The flat plateau (~13.1–13.4 V) is the dominant feature of this chemistry;
// a simple 2-point linear fit is badly wrong across most of the usable range.
// Points are ordered low→high voltage; interpolate linearly between them.
struct BattSocPoint { float v; float pct; };
static const BattSocPoint kLiFePO4SocTable[] = {
  { 11.20f,   0.0f },
  { 12.00f,   1.0f },
  { 12.80f,   5.0f },
  { 13.00f,  10.0f },
  { 13.08f,  20.0f },
  { 13.12f,  30.0f },
  { 13.16f,  40.0f },
  { 13.20f,  50.0f },
  { 13.24f,  60.0f },
  { 13.28f,  70.0f },
  { 13.32f,  80.0f },
  { 13.40f,  90.0f },
  { 13.60f,  99.0f },
  { 14.40f, 100.0f },
};

float BatteryVoltToSocPct(float v) {
  constexpr int n = sizeof(kLiFePO4SocTable) / sizeof(kLiFePO4SocTable[0]);
  if (v <= kLiFePO4SocTable[0].v)     return kLiFePO4SocTable[0].pct;
  if (v >= kLiFePO4SocTable[n - 1].v) return kLiFePO4SocTable[n - 1].pct;
  for (int i = 1; i < n; ++i) {
    if (v <= kLiFePO4SocTable[i].v) {
      const float t = (v - kLiFePO4SocTable[i - 1].v) /
                      (kLiFePO4SocTable[i].v - kLiFePO4SocTable[i - 1].v);
      return kLiFePO4SocTable[i - 1].pct + t * (kLiFePO4SocTable[i].pct - kLiFePO4SocTable[i - 1].pct);
    }
  }
  return 100.0f;
}

float ReadBatteryVoltage() {
  // Average 16 samples; analogReadMilliVolts() applies on-chip eFuse ADC calibration
  int32_t sum_mv = 0;
  for (int i = 0; i < 16; ++i) {
    sum_mv += analogReadMilliVolts(kBatteryAdcPin);
    delayMicroseconds(100);
  }
  const float v_adc = (sum_mv / 16.0f) / 1000.0f;
  if (v_adc <= 0.0f) return NAN;
  return v_adc * kBatteryDividerScale;
}

float ReadThermistorF() {
  // Average 16 samples; analogReadMilliVolts() applies on-chip eFuse ADC calibration
  int32_t sum_mv = 0;
  for (int i = 0; i < kThermistorAvgSamples; ++i) {
    sum_mv += analogReadMilliVolts(kThermistorAdcPin);
    delayMicroseconds(100);
  }
  const float v_mv = sum_mv / (float)kThermistorAvgSamples;
  if (v_mv <= 0.0f || v_mv >= kThermistorVccMv) return NAN;
  // Thermistor is upper element, R1 (10k) is pull-down
  // Vadc = Vcc * R1 / (Rth + R1)  →  Rth = R1 * (Vcc - Vadc) / Vadc
  const float r_th = kThermistorR1 * (kThermistorVccMv - v_mv) / v_mv;
  if (r_th <= 0.0f) return NAN;
  const float logR = logf(r_th);
  if (!isfinite(logR)) return NAN;
  float t_k = 1.0f / (kThermistorC1 + kThermistorC2 * logR + kThermistorC3 * logR * logR * logR);
  float t_f = (t_k - 273.15f) * 9.0f / 5.0f + 32.0f;
  return t_f * kThermistorCalScale + kThermistorCalOffsetF;
}

void PollSensors() {
  const unsigned long now = millis();
  if (sensor_last_poll_ms != 0 && now - sensor_last_poll_ms < kSensorPollIntervalMs) return;
  sensor_last_poll_ms = now;

  // Battery voltage
  const float v = ReadBatteryVoltage();
  if (isfinite(v)) {
    sensor_battery_v = v;
    sensor_battery_pct = constrain(BatteryVoltToSocPct(v), 0.0f, 100.0f);

    // Store history point every kBatteryHistoryIntervalMs for discharge-rate estimation
    if (battery_last_history_ms == 0 || now - battery_last_history_ms >= kBatteryHistoryIntervalMs) {
      battery_last_history_ms = now;
      battery_v_history[battery_v_history_head] = v;
      battery_v_history_ms[battery_v_history_head] = now;
      battery_v_history_head = (battery_v_history_head + 1) % kBatteryHistorySize;
      if (battery_v_history_count < kBatteryHistorySize) battery_v_history_count++;

      // Estimate time remaining via linear regression over all history points.
      // Requires 12 samples (~1 hour) for a stable slope on a slow-discharge LiFePO4 pack.
      if (battery_v_history_count >= 12) {
        const int n = battery_v_history_count;
        const int oldest_idx = (battery_v_history_head - n + kBatteryHistorySize) % kBatteryHistorySize;
        const unsigned long t0 = battery_v_history_ms[oldest_idx];  // anchor to reduce magnitude
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        for (int i = 0; i < n; i++) {
          const int idx = (oldest_idx + i) % kBatteryHistorySize;
          const double x = (battery_v_history_ms[idx] - t0) / 1000.0;  // seconds since oldest
          const double y = battery_v_history[idx];
          sum_x  += x;
          sum_y  += y;
          sum_xy += x * y;
          sum_x2 += x * x;
        }
        const double denom = n * sum_x2 - sum_x * sum_x;
        if (denom > 0.0) {
          const double slope_v_per_s = (n * sum_xy - sum_x * sum_y) / denom;
          // Threshold: 5e-7 V/s ≈ 0.0018 V/hr — catches batteries lasting up to ~1200 h
          if (slope_v_per_s < -5e-7) {
            const double remaining_v = v - kBatteryVoltEmpty;
            const double hours = remaining_v / (-slope_v_per_s * 3600.0);
            sensor_battery_time_remaining_h = static_cast<float>(
                hours < 0.0 ? -1.0 : (hours > 500.0 ? 500.0 : hours));
          } else {
            sensor_battery_time_remaining_h = NAN;  // charging or not measurably discharging
          }
        }
      }
    }

    // Plot buffers (independent intervals from the rate-estimation buffer)
    if (battery_plot_short_last_ms == 0 || now - battery_plot_short_last_ms >= kBattPlotShortIntervalMs) {
      battery_plot_short_last_ms = now;
      battery_plot_short[battery_plot_short_head] = {(uint32_t)now, v};
      battery_plot_short_head = (battery_plot_short_head + 1) % kBattPlotShortSize;
      if (battery_plot_short_count < kBattPlotShortSize) battery_plot_short_count++;
    }
    if (battery_plot_long_last_ms == 0 || now - battery_plot_long_last_ms >= kBattPlotLongIntervalMs) {
      battery_plot_long_last_ms = now;
      battery_plot_long[battery_plot_long_head] = {(uint32_t)now, v};
      battery_plot_long_head = (battery_plot_long_head + 1) % kBattPlotLongSize;
      if (battery_plot_long_count < kBattPlotLongSize) battery_plot_long_count++;
    }
  }

  // Thermistor temperature
  const float tf = ReadThermistorF();
  if (isfinite(tf)) {
    sensor_temp_f = tf;
    if (temp_plot_short_last_ms == 0 || now - temp_plot_short_last_ms >= kTempPlotShortIntervalMs) {
      temp_plot_short_last_ms = now;
      temp_plot_short[temp_plot_short_head] = {(uint32_t)now, tf};
      temp_plot_short_head = (temp_plot_short_head + 1) % kTempPlotShortSize;
      if (temp_plot_short_count < kTempPlotShortSize) temp_plot_short_count++;
    }
    if (temp_plot_long_last_ms == 0 || now - temp_plot_long_last_ms >= kTempPlotLongIntervalMs) {
      temp_plot_long_last_ms = now;
      temp_plot_long[temp_plot_long_head] = {(uint32_t)now, tf};
      temp_plot_long_head = (temp_plot_long_head + 1) % kTempPlotLongSize;
      if (temp_plot_long_count < kTempPlotLongSize) temp_plot_long_count++;
    }
  }
}

// ---- Sequence engine --------------------------------------------------------
// Drives timed solenoid pulse sequences without blocking the main loop.
// Counts one trigger event per sequence start; individual sub-pulses don't
// increment trigger_event_count so the counter stays meaningful.

void SeqSetSolenoid(bool active) {
  if (active && !display_inverted) {
    boosh_open_since_ms = millis();
  } else if (!active && display_inverted) {
    if (boosh_open_since_ms > 0) {
      total_boosh_open_ms += (uint32_t)(millis() - boosh_open_since_ms);
      boosh_open_since_ms = 0;
    }
  }
  boosh_failsafe_armed = false;
  SetDisplayInverted(active);
}

void StartSequence(SeqType type) {
  if (display_inverted) SeqSetSolenoid(false);
  active_seq = type;
  seq_step_start_ms = millis();
  seq_start_ms = millis();
  seq_pulse_idx = 0;
  seq_phase_on = false;
  seq_abort_flag = false;
  trigger_event_count += 1;
  Console.printf("[SEQ] start type=%d\n", (int)type);
}

void AbortSequence() {
  if (active_seq == SEQ_NONE) return;
  seq_abort_flag = true;
}

void PollSequence() {
  if (active_seq == SEQ_NONE) return;

  const unsigned long now = millis();
  const unsigned long elapsed = now - seq_start_ms;

  if (seq_abort_flag) {
    SeqSetSolenoid(false);
    active_seq = SEQ_NONE;
    Console.println("[SEQ] aborted");
    return;
  }

  // --- PULSE_ONCE: single 50 ms pulse ---
  if (active_seq == SEQ_PULSE_ONCE) {
    if (!seq_phase_on) {
      SeqSetSolenoid(true);
      seq_phase_on = true;
      seq_step_start_ms = now;
    } else if (now - seq_step_start_ms >= 50) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
    }
    return;
  }

  // --- PULSE_5X: 5 × (50 ms on / 50 ms off) ---
  if (active_seq == SEQ_PULSE_5X) {
    if (seq_pulse_idx >= 5) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
      return;
    }
    if (!seq_phase_on) {
      // off phase (or initial start): wait 50 ms between pulses (skip wait on first)
      if (seq_pulse_idx == 0 || now - seq_step_start_ms >= 50) {
        SeqSetSolenoid(true);
        seq_phase_on = true;
        seq_step_start_ms = now;
      }
    } else {
      if (now - seq_step_start_ms >= 50) {
        SeqSetSolenoid(false);
        seq_phase_on = false;
        seq_pulse_idx++;
        seq_step_start_ms = now;
      }
    }
    return;
  }

  // --- STEAM: exponential ramp 1→10 Hz over 4 s, then 1 s full open ---
  if (active_seq == SEQ_STEAM) {
    if (elapsed >= 5000) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
      Console.println("[SEQ] steam done");
      return;
    }
    if (elapsed >= 4000) {
      // Full-open phase
      if (!display_inverted) {
        SeqSetSolenoid(true);
        Console.println("[SEQ] steam full open");
      }
      return;
    }
    // Ramp phase: f(t) = 10^(t/4), 1 Hz → 10 Hz
    const float t_sec = elapsed / 1000.0f;
    const float freq_hz = powf(10.0f, t_sec / 4.0f);
    const uint32_t period_ms = (uint32_t)(1000.0f / freq_hz);
    const uint32_t off_ms = period_ms > 50 ? period_ms - 50 : 0;

    if (seq_phase_on) {
      if (now - seq_step_start_ms >= 50) {
        SeqSetSolenoid(false);
        seq_phase_on = false;
        seq_step_start_ms = now;
      }
    } else {
      if (seq_pulse_idx == 0 || now - seq_step_start_ms >= off_ms) {
        SeqSetSolenoid(true);
        seq_phase_on = true;
        seq_step_start_ms = now;
        seq_pulse_idx++;
      }
    }
    return;
  }
}

// ---- Bar Mode LEDs & Buttons ------------------------------------------------

// SOS Morse pattern for blue LED: ... --- ...
// Flat alternating array: [0]=ON duration, [1]=OFF duration, [2]=ON, ...
// Timing unit = kSosUnitMs (150 ms). dot=1u, dash=3u, elem-gap=1u, letter-gap=3u, word-gap=7u
static const uint16_t kSosMorseMs[] = {
  // S: dot dot dot
  150, 150,   // dot, elem-gap
  150, 150,   // dot, elem-gap
  150, 450,   // dot, letter-gap (3u)
  // O: dash dash dash
  450, 150,   // dash, elem-gap
  450, 150,   // dash, elem-gap
  450, 450,   // dash, letter-gap (3u)
  // S: dot dot dot
  150, 150,   // dot, elem-gap
  150, 150,   // dot, elem-gap
  150, 1050,  // dot, word-gap (7u)
};
constexpr int kSosMorseMsCount = (int)(sizeof(kSosMorseMs) / sizeof(kSosMorseMs[0]));

// Drives the blue LED (LEDC channel 1) with the SOS pattern.
// Called every loop iteration when barmode_active; no-op otherwise.
void PollSosBlueLed() {
  if (!barmode_active) return;
  static int step = 0;
  static unsigned long step_start_ms = 0;
  const unsigned long now = millis();
  if (step_start_ms == 0) step_start_ms = now;  // first call init

  if (now - step_start_ms >= kSosMorseMs[step]) {
    step = (step + 1) % kSosMorseMsCount;
    step_start_ms = now;
  }
  // Even step = ON, odd step = OFF
  ledcWrite(1, (step % 2 == 0) ? 200 : 0);
}

// Pylon target with its sequence index for ordered sequential firing.
struct PylonTarget {
  IPAddress ip;
  int seq_idx;  // pylon_index value from registry
};

// Low-level: build and send one OSC float packet to a single IP.
void SendOscFloatToIP(const char *addr, float value, IPAddress dest) {
  const size_t addr_len = strlen(addr);
  const size_t addr_pad = (addr_len + 4) & ~3u;
  const size_t pkt_len  = addr_pad + 8;
  uint8_t pkt[40];
  memset(pkt, 0, sizeof(pkt));
  memcpy(pkt, addr, addr_len);
  pkt[addr_pad]     = ',';
  pkt[addr_pad + 1] = 'f';
  union { float f; uint32_t u; } conv;
  conv.f = value;
  pkt[addr_pad + 4] = (uint8_t)(conv.u >> 24);
  pkt[addr_pad + 5] = (uint8_t)(conv.u >> 16);
  pkt[addr_pad + 6] = (uint8_t)(conv.u >> 8);
  pkt[addr_pad + 7] = (uint8_t)(conv.u);
  oscUdp.beginPacket(dest, kOscPort);
  oscUdp.write(pkt, pkt_len);
  oscUdp.endPacket();
}

// Extracts (IP, pylon_index) pairs from registry JSON, sorted ascending by pylon_index.
// Ties (same index) stay grouped together. Returns count (max maxCount).
int ExtractRegistryTargets(PylonTarget *dest, int maxCount) {
  String json;
  if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    json = barmode_registry_json;
    xSemaphoreGive(barmode_registry_mutex);
  }
  int count = 0, search = 0;
  while (count < maxCount) {
    int ip_pos = json.indexOf("\"ip\":\"", search);
    if (ip_pos < 0) break;
    ip_pos += 6;
    int ip_end = json.indexOf('"', ip_pos);
    if (ip_end < 0) break;
    String ip_str = json.substring(ip_pos, ip_end);
    search = ip_end + 1;
    if (ip_str.length() < 7 || ip_str.indexOf('.') < 0) continue;
    IPAddress addr;
    if (!addr.fromString(ip_str)) continue;

    // Find pylon_index within ±400 chars of this ip field (same JSON object)
    int win_start = max(0, ip_pos - 400);
    int win_end   = min((int)json.length(), ip_end + 400);
    String window = json.substring(win_start, win_end);
    int pi = window.indexOf("\"pylon_index\":");
    int seq_idx = 0;
    if (pi >= 0) seq_idx = window.substring(pi + 14).toInt();

    if (seq_idx < 0) continue;  // negative index = excluded from sequential mode

    dest[count].ip      = addr;
    dest[count].seq_idx = seq_idx;
    count++;
  }
  // Insertion sort by seq_idx (N ≤ 16, stable)
  for (int i = 1; i < count; i++) {
    PylonTarget key = dest[i];
    int j = i - 1;
    while (j >= 0 && dest[j].seq_idx > key.seq_idx) {
      dest[j + 1] = dest[j];
      j--;
    }
    dest[j + 1] = key;
  }
  return count;
}

// Extracts IPs from cached registry JSON into dest[]. Returns count (max maxCount).
int ExtractRegistryIPs(IPAddress *dest, int maxCount) {
  String json;
  if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    json = barmode_registry_json;
    xSemaphoreGive(barmode_registry_mutex);
  }
  int count = 0, idx = 0;
  while (count < maxCount) {
    int pos = json.indexOf("\"ip\":\"", idx);
    if (pos < 0) break;
    pos += 6;
    int end = json.indexOf('"', pos);
    if (end < 0) break;
    String ip = json.substring(pos, end);
    idx = end + 1;
    if (ip.length() < 7 || ip.indexOf('.') < 0) continue;
    IPAddress a;
    if (!a.fromString(ip)) continue;
    dest[count++] = a;
  }
  return count;
}

// Sends an OSC message with a single float arg to all pylons in the cached registry.
// Skips silently if registry is empty (not yet fetched). Self is included via registry.
// Called from Core 1; oscUdp send path is non-blocking.
void SendOscFloatToAllPylons(const char *addr, float value) {
  IPAddress ips[16];
  const int count = ExtractRegistryIPs(ips, 16);
  if (count == 0) {
    Console.printf("[BarMode] SendOsc %s: registry empty, skipping\n", addr);
    return;
  }
  for (int i = 0; i < count; i++) SendOscFloatToIP(addr, value, ips[i]);
  Console.printf("[BarMode] SendOsc %s %.1f → %d pylons\n", addr, value, count);
}

// LED chase pattern while button 0 is held: blue→yellow→green, 100 ms per step.
// Runs last in loop() so it always wins over other LED writers.
void PollBarModeBtn0Chase() {
  if (!barmode_btn0_held) return;
  const uint8_t phase = (uint8_t)((millis() / kBarModeBtn0ChaseMs) % 3);
  ledcWrite(1, phase == 0 ? 220 : 0);  // blue   ch1
  ledcWrite(2, phase == 1 ? 220 : 0);  // yellow ch2
  ledcWrite(0, phase == 2 ? 220 : 0);  // green  ch0
}

// Scans the 4 bar-mode buttons and drives the yellow-solid + green-blink(N+1) sequence.
// Green = LEDC channel 0, Yellow = LEDC channel 2.
// Sets barmode_seq_active so PollBlinkLeds() yields those channels.
void PollBarModeButtons() {
  if (!barmode_active) return;

  const unsigned long now = millis();

  // Per-button debounce
  static bool btn_last_raw[4] = {};
  static bool btn_stable[4]   = {};
  static unsigned long btn_change_ms[4] = {};

  for (int i = 0; i < 4; i++) {
    const bool raw = digitalRead(kBarModeButtonPins[i]) == HIGH;
    if (raw != btn_last_raw[i]) {
      btn_last_raw[i] = raw;
      btn_change_ms[i] = now;
    }
    if (now - btn_change_ms[i] >= 20) {
      btn_stable[i] = raw;
    }
  }

  // Button 0: BooshMain timed pulse; Button 1: BooshPulseSingle/seq;
  // Button 2: BooshPulseTrain; Button 3: BooshSteam hold.
  // Sequential activation: hold BLUE 3s → GREEN 3s → ORANGE 2s → RED 2s → valve open.
  {
    static bool btn_prev_stable[4] = {};

    // ---- Sequential 4-button activation state machine ----------------------
    // Phase 0: idle (normal button behavior)
    // Phase 1: BLUE held 3s → blue strobe 25% 2Hz, green ON, orange/red OFF
    // Phase 2: +GREEN held 3s → blue+green strobe 50% 4Hz, orange ON, red OFF
    // Phase 3: +ORANGE held 2s → blue+green+orange strobe 75% 6Hz, red OFF
    // Phase 4: +RED held 2s → all strobe 80% 8Hz, valve open
    // Phase 5: closing (valve closed, all lamps off, waiting for full release)
    static int           seq_phase          = 0;
    static unsigned long seq_blue_press_ms  = 0;   // rising edge of BLUE in phase 0
    static unsigned long seq_green_press_ms = 0;   // rising edge of GREEN in phase 1
    static unsigned long seq_orange_press_ms= 0;   // rising edge of ORANGE in phase 2
    static unsigned long seq_red_press_ms   = 0;   // rising edge of RED in phase 3
    static bool          seq_blue_armed     = false;
    static bool          seq_green_armed    = false;
    static bool          seq_orange_armed   = false;
    static bool          seq_red_armed      = false;
    static bool          seq_valve_open     = false;
    static unsigned long seq_valve_open_ms  = 0;

    // Rising/falling edges for sequence tracking
    const bool blue_rising   = btn_stable[1] && !btn_prev_stable[1];
    const bool blue_falling  = !btn_stable[1] && btn_prev_stable[1];
    const bool green_rising  = btn_stable[0] && !btn_prev_stable[0];
    const bool orange_rising = btn_stable[2] && !btn_prev_stable[2];
    const bool red_rising    = btn_stable[3] && !btn_prev_stable[3];

    // Arm per-button hold timers on rising edge within the correct phase
    if (blue_rising  && seq_phase == 0) { seq_blue_press_ms  = now; seq_blue_armed   = true; }
    if (blue_falling && seq_phase == 0) { seq_blue_armed = false; }
    if (green_rising  && seq_phase == 1) { seq_green_press_ms  = now; seq_green_armed  = true; }
    if (orange_rising && seq_phase == 2) { seq_orange_press_ms = now; seq_orange_armed = true; }
    if (red_rising    && seq_phase == 3) { seq_red_press_ms    = now; seq_red_armed    = true; }

    // Phase advancement (checked every tick)
    if (seq_phase == 0 && seq_blue_armed && btn_stable[1] && now - seq_blue_press_ms >= 3000) {
      seq_phase = 1;
      seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.println("[BarMode] Seq phase 1: blue held 3s");
    }
    if (seq_phase == 1 && seq_green_armed && btn_stable[0] && now - seq_green_press_ms >= 3000) {
      seq_phase = 2;
      seq_orange_armed = seq_red_armed = false;
      Console.println("[BarMode] Seq phase 2: green held 3s");
    }
    if (seq_phase == 2 && seq_orange_armed && btn_stable[2] && now - seq_orange_press_ms >= 2000) {
      seq_phase = 3;
      seq_red_armed = false;
      Console.println("[BarMode] Seq phase 3: orange held 2s");
    }
    if (seq_phase == 3 && seq_red_armed && btn_stable[3] && now - seq_red_press_ms >= 2000) {
      seq_phase = 4;
      seq_valve_open    = true;
      seq_valve_open_ms = now;
      SendOscFloatToAllPylons(kOscAddress, 1.0f);
      Console.println("[BarMode] Seq phase 4: valve open");
    }

    // Required-button-released checks: reset if an anchor is dropped
    // Note: check seq_phase before clearing seq_valve_open so we go to phase 5 if valve was open
    if (seq_phase >= 1 && !btn_stable[1]) {   // BLUE released
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) { SendOscFloatToAllPylons(kOscAddress, 0.0f); seq_valve_open = false; }
      seq_phase = next;
      seq_blue_armed = seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: blue released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase >= 2 && !btn_stable[0]) {  // GREEN released in phase 2+
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) { SendOscFloatToAllPylons(kOscAddress, 0.0f); seq_valve_open = false; }
      seq_phase = next;
      seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: green released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase >= 3 && !btn_stable[2]) {  // ORANGE released in phase 3+
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) { SendOscFloatToAllPylons(kOscAddress, 0.0f); seq_valve_open = false; }
      seq_phase = next;
      seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: orange released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase == 4 && !btn_stable[3]) {  // RED released in phase 4
      if (seq_valve_open) { SendOscFloatToAllPylons(kOscAddress, 0.0f); seq_valve_open = false; }
      seq_phase = 5;
      Console.println("[BarMode] Seq phase 5: closing (red released)");
    }

    // Phase 4: auto-close timeout
    if (seq_phase == 4 && seq_valve_open && now - seq_valve_open_ms >= barmode_all4_valve_ms) {
      SendOscFloatToAllPylons(kOscAddress, 0.0f);
      seq_valve_open = false;
      seq_phase = 5;
      Console.println("[BarMode] Seq phase 5: auto-close timeout");
    }

    // Phase 5: wait for all buttons released, then idle
    if (seq_phase == 5 && !btn_stable[0] && !btn_stable[1] && !btn_stable[2] && !btn_stable[3]) {
      seq_phase = 0;
      seq_blue_armed = seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.println("[BarMode] Seq idle");
    }

    // ---- Individual button actions (suppressed once sequence is active) ----
    if (seq_phase == 0) {

    // Button 0 — Green button: BooshMain timed pulse (duration = barmode_green_timeout_ms)
    {
      static bool          btn0_pulse_active = false;
      static unsigned long btn0_pulse_ms     = 0;

      if (!barmode_btn_disabled[0]) {
        if (btn_stable[0] && !btn_prev_stable[0]) {
          barmode_btn_counts[0]++;
          barmode_btn_event_ms[barmode_btn_event_head]  = now;
          barmode_btn_event_btn[barmode_btn_event_head] = 0;
          barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
          if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
          if (!btn0_pulse_active) {
            SendOscFloatToAllPylons(kOscAddress, 1.0f);
          }
          btn0_pulse_active = true;
          btn0_pulse_ms     = now;
          barmode_btn0_held = true;
          display.invertDisplay(true);
        }
      }
      // Physical release: clear lamp/display state; timer still handles OSC close
      if (!btn_stable[0] && btn_prev_stable[0] && barmode_btn0_held) {
        barmode_btn0_held = false;
        display.invertDisplay(false);
      }
      // Timer-based close
      if (btn0_pulse_active && now - btn0_pulse_ms >= barmode_green_timeout_ms) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        btn0_pulse_active = false;
      }
    }
    // Green lamp: disabled=30%, held=solid, idle=sine 2Hz
    {
      uint8_t v;
      if (barmode_btn_disabled[0]) {
        v = 77;  // 30%
      } else if (barmode_btn0_held) {
        v = 255;
      } else {
        const float t = millis() / 1000.0f;
        v = (uint8_t)((sinf(2.0f * M_PI * 2.0f * t) * 0.5f + 0.5f) * 255);
      }
      ledcWrite(4, v);
    }

    // Button 1 — Blue button: BooshPulseSingle
    // Normal: single fire on press. Double-tap (second press ≤300ms after release) + hold:
    // fires to each pylon sequentially 100ms apart, looping, until released or 5s max.
    {
      static unsigned long btn1_release_ms      = 0;
      static bool          btn1_seq_active       = false;
      static unsigned long btn1_seq_start_ms     = 0;
      static unsigned long btn1_seq_last_fire_ms = 0;
      static unsigned long btn1_seq_delay_ms     = 1000;  // current inter-group delay; starts 1s, -50ms/step
      static PylonTarget   btn1_seq_targets[16];
      static int           btn1_seq_count        = 0;
      static int           btn1_seq_group        = 0;  // index of current group start in sorted targets[]

      const bool rising  = btn_stable[1] && !btn_prev_stable[1];
      const bool falling = !btn_stable[1] && btn_prev_stable[1];

      if (rising && !barmode_btn_disabled[1]) {
        barmode_btn_counts[1]++;
        barmode_btn_event_ms[barmode_btn_event_head]  = now;
        barmode_btn_event_btn[barmode_btn_event_head] = 1;
        barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
        if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
        if (btn1_release_ms > 0 && now - btn1_release_ms <= 300) {
          // Double-tap: enter index-ordered sequential looping mode
          btn1_seq_count = ExtractRegistryTargets(btn1_seq_targets, 16);
          if (btn1_seq_count > 0) {
            btn1_seq_active       = true;
            btn1_seq_start_ms     = now;
            btn1_seq_delay_ms     = 1000;
            btn1_seq_last_fire_ms = now - 1000;  // fire first group immediately
            btn1_seq_group        = 0;
            Console.printf("[BarMode] Btn1 seq: %d pylons\n", btn1_seq_count);
          }
        } else {
          // Normal single fire to all pylons simultaneously
          SendOscFloatToAllPylons(kOscAddrPulseSingle, 1.0f);
        }
      }

      if (falling) {
        btn1_release_ms = now;
        if (btn1_seq_active) {
          btn1_seq_active = false;
          Console.println("[BarMode] Btn1 seq stopped");
        }
      }

      // Sequence ticker: fire one group (same pylon_index) per 100ms step
      if (btn1_seq_active) {
        if (!btn_stable[1] || now - btn1_seq_start_ms >= barmode_seq_max_ms) {
          btn1_seq_active = false;
          Console.println("[BarMode] Btn1 seq ended");
        } else if (now - btn1_seq_last_fire_ms >= btn1_seq_delay_ms) {
          btn1_seq_last_fire_ms = now;
          // Fire all targets in current group (same seq_idx)
          const int cur_val = btn1_seq_targets[btn1_seq_group].seq_idx;
          int fired = 0;
          for (int g = btn1_seq_group;
               g < btn1_seq_count && btn1_seq_targets[g].seq_idx == cur_val; g++) {
            SendOscFloatToIP(kOscAddrPulseSingle, 1.0f, btn1_seq_targets[g].ip);
            fired++;
          }
          Console.printf("[BarMode] Btn1 seq idx=%d fired=%d delay=%lums\n", cur_val, fired, btn1_seq_delay_ms);
          // Decrement delay before next step (floor at 50ms)
          // Apply linear decrement first, then exponential factor
          {
            unsigned long after_dec = (btn1_seq_delay_ms > barmode_seq_dec_ms + 50)
                                      ? btn1_seq_delay_ms - barmode_seq_dec_ms
                                      : 50;
            if (barmode_seq_exp_pct < 100) {
              after_dec = (unsigned long)(after_dec * (barmode_seq_exp_pct / 100.0f));
            }
            btn1_seq_delay_ms = (after_dec < 50) ? 50 : after_dec;
          }
          // Advance group pointer; wrap without resetting delay so acceleration accumulates
          btn1_seq_group += fired;
          if (btn1_seq_group >= btn1_seq_count) {
            btn1_seq_group = 0;
          }
        }
      }
    }

    // Blue lamp: disabled=30%, idle=200ms pulse per second
    ledcWrite(5, barmode_btn_disabled[1] ? 77 : (now % 1000 < 50) ? 255 : 51);

    // Button 2 — Orange button: BooshPulseTrain; IO35 strobes 5x pulse pattern once then returns to idle sawtooth
    {
      static bool          io35_strobe      = false;
      static unsigned long io35_strobe_start = 0;

      if (btn_stable[2] && !btn_prev_stable[2] && !barmode_btn_disabled[2]) {
        barmode_btn_counts[2]++;
        barmode_btn_event_ms[barmode_btn_event_head]  = now;
        barmode_btn_event_btn[barmode_btn_event_head] = 2;
        barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
        if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
        SendOscFloatToAllPylons(kOscAddrPulseTrain, 1.0f);
        io35_strobe       = true;
        io35_strobe_start = now;
      }

      if (io35_strobe) {
        const unsigned long elapsed = now - io35_strobe_start;
        if (elapsed < 500) {
          // Mirror 5× (50ms on / 50ms off)
          const uint8_t step = (uint8_t)(elapsed / 50);
          ledcWrite(3, (step % 2 == 0) ? 255 : 0);
        } else {
          io35_strobe = false;
        }
      }
      if (!io35_strobe) {
        // Orange lamp: disabled=30%, idle=sawtooth 4Hz
        ledcWrite(3, barmode_btn_disabled[2] ? 77 : (uint8_t)((now % 250) * 255 / 250));
      }
    }

    // Button 3 — Red button: tap=100ms steam pulse; triple-tap+hold=steam hold mode
    // State 0: idle
    // State 1: first press fired (150ms close pending); ≤300ms window for 2nd press
    // State 2: second press suppressed; ≤300ms window for 3rd press+hold
    // State 3: steam hold active (hold open until released)
    {
      static int           red_state        = 0;
      static unsigned long red_press1_ms    = 0;   // time of 1st press
      static unsigned long red_press2_ms    = 0;   // time of 2nd press
      static bool          red_close_pending= false;
      static unsigned long red_close_ms     = 0;   // when 150ms close should fire
      static unsigned long lamp_red_press_ms = 0;
      static bool          lamp_red_on      = false;
      static unsigned long lamp_red_step_ms = 0;

      const bool r_rising  = btn_stable[3] && !btn_prev_stable[3];
      const bool r_falling = !btn_stable[3] && btn_prev_stable[3];

      if (!barmode_btn_disabled[3]) {
        if (r_rising) {
          if (red_state == 0) {
            // First press: fire 100ms pulse
            barmode_btn_counts[3]++;
            barmode_btn_event_ms[barmode_btn_event_head]  = now;
            barmode_btn_event_btn[barmode_btn_event_head] = 3;
            barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
            if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
            SendOscFloatToAllPylons(kOscAddrSteam, 1.0f);
            red_close_pending = true;
            red_close_ms      = now;
            red_press1_ms     = now;
            red_state         = 1;
            Console.println("[BarMode] Red: pulse open");
          } else if (red_state == 1 && now - red_press1_ms <= 300) {
            // Second press within 300ms: suppress
            red_press2_ms = now;
            red_state     = 2;
            Console.println("[BarMode] Red: 2nd press suppressed");
          } else if (red_state == 2 && now - red_press2_ms <= 300) {
            // Third press within 300ms of second + hold: activate steam
            SendOscFloatToAllPylons(kOscAddrSteam, 1.0f);
            lamp_red_press_ms = now;
            lamp_red_on       = false;
            lamp_red_step_ms  = now - 10000;  // expire so first pulse fires immediately
            red_state         = 3;
            Console.println("[BarMode] Red: steam hold active");
          } else {
            // Out-of-window press: treat as a fresh first press
            red_close_pending = false;  // cancel any pending close (was already sent if fired)
            SendOscFloatToAllPylons(kOscAddrSteam, 1.0f);
            red_close_pending = true;
            red_close_ms      = now;
            red_press1_ms     = now;
            red_state         = 1;
          }
        }

        if (r_falling && red_state == 3) {
          SendOscFloatToAllPylons(kOscAddrSteam, 0.0f);
          red_state = 0;
          Console.println("[BarMode] Red: steam hold released");
        }

        // 150ms close timer
        if (red_close_pending && now - red_close_ms >= 150) {
          SendOscFloatToAllPylons(kOscAddrSteam, 0.0f);
          red_close_pending = false;
          Console.println("[BarMode] Red: pulse close");
        }

        // State timeout: reset to idle if window expires without next press
        if (red_state == 1 && now - red_press1_ms > 300) { red_state = 0; }
        if (red_state == 2 && now - red_press2_ms > 300) { red_state = 0; }
      }

      // Red lamp: disabled=30%, steam hold ramp, idle=Morse LAVA
      if (barmode_btn_disabled[3]) {
        ledcWrite(6, 77);
      } else if (red_state == 3) {
        // Steam ramp: freq 1→10 Hz over 4s, full on after 4s
        const unsigned long elapsed = now - lamp_red_press_ms;
        if (elapsed >= 4000) {
          ledcWrite(6, 255);
        } else {
          const float    t_sec     = elapsed / 1000.0f;
          const float    freq_hz   = powf(10.0f, t_sec / 4.0f);
          const uint32_t period_ms = (uint32_t)(1000.0f / freq_hz);
          const uint32_t off_ms    = period_ms > 50 ? period_ms - 50 : 0;
          if (lamp_red_on) {
            if (now - lamp_red_step_ms >= 50)     { lamp_red_on = false; lamp_red_step_ms = now; }
          } else {
            if (now - lamp_red_step_ms >= off_ms) { lamp_red_on = true;  lamp_red_step_ms = now; }
          }
          ledcWrite(6, lamp_red_on ? 255 : 0);
        }
      } else {
        // Idle: Morse "LAVA" (unit=150ms; L=.-.. A=.- V=...- A=.-)
        static const uint16_t kLavaMorseMs[] = {
          150, 150, 450, 150, 150, 150, 150, 450,  // L + inter-char
          150, 150, 450, 450,                        // A + inter-char
          150, 150, 150, 150, 150, 150, 450, 450,  // V + inter-char
          150, 150, 450, 1050,                       // A + inter-word
        };
        const unsigned long t = millis() % 6600UL;
        unsigned long accum = 0;
        uint8_t lamp = 0;
        for (int i = 0; i < 24; i++) {
          accum += kLavaMorseMs[i];
          if (t < accum) { lamp = (i % 2 == 0) ? 255 : 0; break; }
        }
        ledcWrite(6, lamp);
      }
    }

    } // end if (seq_phase == 0) — individual button actions

    // ---- Sequential activation lamp override (phases 1-5) ------------------
    if (seq_phase > 0) {
      uint8_t g = 0, b = 0, o = 0, r = 0;
      if (seq_phase == 1) {
        b = (now % 500 < 125) ? 64 : 0;   // blue: 2Hz 25%
        g = 255;                            // green: solid ON
      } else if (seq_phase == 2) {
        const uint8_t s = (now % 250 < 125) ? 128 : 0;  // 4Hz 50%
        b = g = s;
        o = 255;  // orange: solid ON
      } else if (seq_phase == 3) {
        const uint8_t s = (now % 167 < 125) ? 192 : 0;  // 6Hz 75%
        b = g = o = s;
        r = 255;  // red: solid ON
      } else if (seq_phase == 4) {
        const uint8_t s = (now % 125 < 100) ? 204 : 0;  // 8Hz 80%
        b = g = o = r = s;
      }
      // phase 5: all remain 0 (off) until released
      ledcWrite(3, o);  // orange
      ledcWrite(4, g);  // green
      ledcWrite(5, b);  // blue
      ledcWrite(6, r);  // red
    }

    for (int i = 0; i < 4; i++) btn_prev_stable[i] = btn_stable[i];
  }

  // Blink sequence state machine
  // seq_state: 0=idle, 1=green ON, 2=green OFF (gap between blinks)
  static uint8_t seq_state = 0;
  static int seq_blinks_left = 0;
  static unsigned long seq_step_ms = 0;

  if (seq_state == 0) {
    barmode_seq_active = false;
    // Detect first stable press; ignore while a button is still held from last trigger
    static bool btn_was_stable[4] = {};
    for (int i = 0; i < 4; i++) {
      const bool rising = btn_stable[i] && !btn_was_stable[i];
      btn_was_stable[i] = btn_stable[i];
      if (rising) {
        seq_blinks_left = i + 1;
        seq_state = 1;
        seq_step_ms = now;
        barmode_seq_active = true;
        ledcWrite(2, 200);  // yellow solid ON
        ledcWrite(0, 200);  // green ON (first blink)
        Console.printf("[BarMode] Button %d → yellow solid, green ×%d\n", i, i + 1);
        break;
      }
    }
  } else if (seq_state == 1) {  // green ON phase
    if (now - seq_step_ms >= kBarBlinkOnMs) {
      ledcWrite(0, 0);  // green OFF
      seq_blinks_left--;
      if (seq_blinks_left == 0) {
        // Sequence complete
        ledcWrite(2, 0);  // yellow OFF
        seq_state = 0;
        barmode_seq_active = false;
      } else {
        seq_state = 2;
        seq_step_ms = now;
      }
    }
  } else if (seq_state == 2) {  // gap between blinks
    if (now - seq_step_ms >= kBarBlinkOffMs) {
      ledcWrite(0, 200);  // green ON
      seq_state = 1;
      seq_step_ms = now;
    }
  }
}

// ---- Blink LEDs -------------------------------------------------------------

void PollBlinkLeds() {
  // White: simple 1Hz square wave toggle
  static unsigned long lastWhiteMs = 0;
  static bool whiteState = false;
  const unsigned long now = millis();
  if (now - lastWhiteMs >= 500) {
    lastWhiteMs = now;
    whiteState = !whiteState;
    digitalWrite(kLedWhitePin, whiteState);
  }

  // Green/Blue/Yellow: sine wave brightness, 5kHz carrier via LEDC
  const float t = now / 1000.0f;
  if (identify_until_ms > 0 && now < identify_until_ms) {
    // Identify mode: all three LEDs at 2 Hz, 120° phase offset, full brightness
    constexpr float kIdDuty = 255.0f;
    constexpr float kTwoPi = 2.0f * M_PI;
    ledcWrite(0, (uint8_t)(((1.0f + sinf(kTwoPi * 2.0f * t + 0.0f * kTwoPi / 3.0f)) / 2.0f) * kIdDuty));
    ledcWrite(1, (uint8_t)(((1.0f + sinf(kTwoPi * 2.0f * t + 1.0f * kTwoPi / 3.0f)) / 2.0f) * kIdDuty));
    ledcWrite(2, (uint8_t)(((1.0f + sinf(kTwoPi * 2.0f * t + 2.0f * kTwoPi / 3.0f)) / 2.0f) * kIdDuty));
  } else {
    if (identify_until_ms > 0) identify_until_ms = 0;  // clear expired flag
    constexpr float kMaxDuty = 0.66f * 255.0f;
    // In barmode: green/yellow are owned by PollBarModeButtons(); blue by PollSosBlueLed().
    if (!barmode_seq_active) {
      ledcWrite(0, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.4f * t)) / 2.0f) * kMaxDuty));  // green 0.4Hz
      ledcWrite(2, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 1.2f * t)) / 2.0f) * kMaxDuty));  // yellow 1.2Hz
    }
    if (!barmode_active) {
      if (!wifi_has_ip) {
        ledcWrite(1, ((now / 125) % 2 == 0) ? 200 : 0);  // blue 4Hz while WiFi searching
      } else {
        ledcWrite(1, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.8f * t)) / 2.0f) * kMaxDuty));  // blue 0.8Hz
      }
    }
  }
}

// ---- Ping task (Core 0) -----------------------------------------------------
// Runs hostname resolution and ICMP ping in a background FreeRTOS task so the
// main loop (which handles OSC) never blocks waiting for network replies.

void PingTask(void *) {
  static const char *kTargetHost = kPingTargetHost;
  static const char *kTargetHostMdns = "RPIBOOSH.local";
  IPAddress targetIp;
  bool hasIp = false;
  unsigned long lastResolveMs = 0;
  unsigned long lastPingMs = 0;
  bool pingWasDown = false;
  PingStats stats;

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      hasIp = false;
      pingWasDown = false;
      stats = PingStats{};
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    const unsigned long now = millis();

    if (!hasIp && now - lastResolveMs > 10000) {
      lastResolveMs = now;
      if (WiFi.hostByName(kTargetHost, targetIp) || WiFi.hostByName(kTargetHostMdns, targetIp)) {
        hasIp = true;
        target_ip_string = targetIp.toString();
        Console.print("[Ping] Resolved ");
        Console.print(kTargetHost);
        Console.print(" -> ");
        Console.println(target_ip_string);
      } else {
        Console.println("[Ping] Resolve failed, retrying in 10s");
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
    }

    if (!hasIp) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (now - lastPingMs >= 1000) {
      lastPingMs = now;
      telemetry_ping_sent += 1;
      const bool ok = Ping.ping(targetIp, 1);
      telemetry_ping_last_ok = ok;
      current_ping_last_ok = ok;
      if (ok) {
        const uint32_t rtt = static_cast<uint32_t>(Ping.averageTime());
        stats.last_ms = rtt;
        stats.count += 1;
        stats.sum_ms += rtt;
        if (rtt < stats.min_ms) stats.min_ms = rtt;
        if (rtt > stats.max_ms) stats.max_ms = rtt;
        last_ping_success_ms = millis();
        Console.print("[Ping] ");
        Console.print(kTargetHost);
        Console.print(" ");
        Console.print(rtt);
        Console.println("ms");
        if (pingWasDown) {
          pingWasDown = false;
          registry_announced = false;
          registry_next_attempt_ms = millis();
          registry_consecutive_failures = 0;
          Console.println("[Ping] restored: scheduling registry re-announce");
        }
      } else {
        telemetry_ping_lost += 1;
        pingWasDown = true;
        Console.println("[Ping] failed");
      }
      telemetry_ping_has_data = stats.has_data();
      telemetry_ping_last_ms = stats.last_ms;
      telemetry_ping_count = stats.count;
      telemetry_ping_avg_ms = stats.avg_ms();
      telemetry_ping_min_ms = telemetry_ping_has_data ? stats.min_ms : 0;
      telemetry_ping_max_ms = telemetry_ping_has_data ? stats.max_ms : 0;
    }

    // Registry HTTP runs here (Core 0) so it never blocks OSC on the main loop.
    HandleRegistry(millis());

    // In BARMODE: periodically fetch the full pylon list from rpiboosh for the web UI.
    if (barmode_active && barmode_registry_mutex) {
      const unsigned long now_r = millis();
      if (barmode_registry_fetch_now ||
          now_r - barmode_registry_last_fetch_ms >= kBarModeRegistryIntervalMs) {
        barmode_registry_fetch_now = false;
        barmode_registry_last_fetch_ms = now_r;
        HTTPClient http;
        http.begin(String(kRegistryBaseUrlPrimary) + "/api/pylons");
        http.setTimeout(kRegistryHttpTimeoutMs);
        const int code = http.GET();
        if (code == 200) {
          const String body = http.getString();
          if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            barmode_registry_json = body;
            xSemaphoreGive(barmode_registry_mutex);
          }
        } else {
          Console.printf("[BarMode] registry fetch failed: %d\n", code);
        }
        http.end();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // yield; ping timing is driven by lastPingMs
  }
}

// ---- Main loop --------------------------------------------------------------

void loop() {
  static bool wasConnected = false;
  static unsigned long lastDisplayMs = 0;
  static uint8_t displayPage = 0;    // 0-4; slots 0,2,4=status(60%), 1,3=other pages
  static uint8_t displayOtherIdx = 0; // cycles 0-4 through non-status pages
  static unsigned long lastIdentMs = 0;
  static uint8_t identPhase = 0;

  PollBlinkLeds();
  PollSosBlueLed();
  PollBarModeButtons();
  PollBarModeBtn0Chase();  // must run last — overwrites all 3 LEDs while btn0 held
  PollSequence();
  PollSensors();
  PollSerialCli();
  PollDevBoardButton();
  webServer.handleClient();
  if (ap_active) dnsServer.processNextRequest();

  // Apply live AP enable/disable from config changes
  if (ap_enabled && !ap_active) {
    SetupApMode();
  } else if (!ap_enabled && ap_active) {
    StopApMode();
  }

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long disconnected_since_ms = 0;
    static unsigned long last_reconnect_attempt_ms = 0;
    static unsigned long last_disconnect_display_ms = 0;
    const unsigned long now_dc = millis();

    if (wasConnected) {
      wasConnected = false;
      disconnected_since_ms = now_dc;
      last_reconnect_attempt_ms = 0;
      target_ip_string = "";
      registry_announced = false;
      registry_next_attempt_ms = 0;
      registry_consecutive_failures = 0;
      Console.println("WiFi disconnected: registry state reset.");
    }
    if (disconnected_since_ms == 0) {
      disconnected_since_ms = now_dc;  // boot with no WiFi
    }

    const unsigned long offline_ms = now_dc - disconnected_since_ms;

    // Escalating reconnect: nudge WiFi stack every 3 min, reboot after 10 min.
    if (!ap_active) {
      if (offline_ms >= 600000UL) {
        Console.println("[WiFi] Offline 10 min — rebooting.");
        ESP.restart();
      } else if (offline_ms >= 180000UL && now_dc - last_reconnect_attempt_ms >= 60000UL) {
        last_reconnect_attempt_ms = now_dc;
        Console.println("[WiFi] Offline 3+ min — forcing reconnect.");
        WiFi.reconnect();
      }
    }

    // Update display at ~4 Hz without blocking.
    if (now_dc - last_disconnect_display_ms >= 250) {
      last_disconnect_display_ms = now_dc;
      if (ap_active) {
        ShowStatus("AP mode", "PYLON_" + pylon_id);
      } else {
        ShowStatus("WiFi lost", "Reconnecting");
      }
    }
    return;
  }
  PollOsc();
  const unsigned long now = millis();
  if (!wasConnected) {
    wasConnected = true;
    registry_announced = false;
    registry_next_attempt_ms = now;
    registry_consecutive_failures = 0;
    RestartMdnsIfConnected();
    oscUdp.stop();
    oscUdp.begin(kOscPort);
    SetupWebServer();
    Console.println("WiFi connected: mDNS/OSC/web restarted, registry announce scheduled.");
  }
  // Registry HTTP is handled in PingTask (Core 0) — no blocking call here.
  if (!wifi_has_ip && wifi_connected_since_ms == 0) {
    wifi_connected_since_ms = now;
  }
  if (boosh_failsafe_armed && now - boosh_failsafe_start_ms >= boosh_failsafe_timeout_ms) {
    boosh_failsafe_armed = false;
    ApplyBooshState(0.0f);
    Console.println("Failsafe: BooshMain timeout -> forcing OFF.");
    ShowStatus("Failsafe timeout", "BooshMain OFF");
    boosh_failsafe_note_until_ms = now + kBooshFailsafeNoteMs;
  }

  // Identify mode: override normal display with animated checkerboard at 5 Hz
  if (identify_until_ms > 0 && now < identify_until_ms) {
    if (now - lastIdentMs >= 200) {
      lastIdentMs = now;
      identPhase ^= 1;
      ShowIdentifyScreen(identPhase);
    }
  } else if (boosh_failsafe_note_until_ms == 0 || now >= boosh_failsafe_note_until_ms) {
    boosh_failsafe_note_until_ms = 0;
    if (lastDisplayMs == 0) {
      lastDisplayMs = now;
    } else if (now - lastDisplayMs >= kDisplayCycleMs) {
      lastDisplayMs = now;
      if (barmode_active) {
        // Barmode: no battery/temp hardware — skip those pages, only cycle info sub-pages
        displayPage     = 3;
        displayOtherIdx = static_cast<uint8_t>((displayOtherIdx + 1) % 5);
      } else {
        displayPage = static_cast<uint8_t>((displayPage + 1) % 4);
        if (displayPage == 3) {
          displayOtherIdx = static_cast<uint8_t>((displayOtherIdx + 1) % 5);
        }
      }
    }

    // Slot 0, 1 → temp °F + battery pct  |  Slot 2 → time remaining + voltage
    // Slot 3 → info sub-pages (ping/wifi/wifi-detail/node/firmware)
    // In barmode slots 0-2 are skipped entirely (no sensor hardware).
    if (!barmode_active && (displayPage == 0 || displayPage == 1)) {
      ShowTempPctPage();
    } else if (!barmode_active && displayPage == 2) {
      ShowTimeVoltagePage();
    } else {
      // displayOtherIdx: 0=ping, 1=wifi, 2=wifi detail, 3=node, 4=firmware
      if (displayOtherIdx == 0) {
        PingStats disp_stats;
        disp_stats.count = telemetry_ping_count;
        disp_stats.min_ms = telemetry_ping_has_data ? telemetry_ping_min_ms : UINT32_MAX;
        disp_stats.max_ms = telemetry_ping_max_ms;
        disp_stats.sum_ms = static_cast<uint64_t>(telemetry_ping_avg_ms) * telemetry_ping_count;
        disp_stats.last_ms = telemetry_ping_last_ms;
        ShowPingStats("RPIBOOSH", disp_stats, current_ping_last_ok, last_ping_success_ms, now);
      } else if (displayOtherIdx == 1) {
        ShowWifiMetricsPage(0, wifi_connected_since_ms, last_disconnect_reason);
      } else if (displayOtherIdx == 2) {
        ShowWifiMetricsPage(1, wifi_connected_since_ms, last_disconnect_reason);
      } else if (displayOtherIdx == 3) {
        ShowNodeConfigPage();
      } else {
        ShowFirmwarePage();
      }
    }
  }

  delay(5);
}
