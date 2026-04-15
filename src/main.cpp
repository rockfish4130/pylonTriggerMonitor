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
constexpr int kLedGreenPin = 15;   // IO15 green - 2Hz
constexpr int kLedBluePin = 14;    // IO14 blue - 4Hz
constexpr int kLedYellowPin = 13;  // IO13 yellow - 6Hz
constexpr int kIo38Pin = 38;       // IO38 high = WiFi connected
constexpr int kIo11Pin = 11;       // IO11 high = boosh active
constexpr int kBatteryAdcPin = 3;  // IO3 ADC - battery voltage divider (R5=100k, R8=22k)
constexpr int kThermistorAdcPin = 4; // IO4 ADC - NTC thermistor (R12=10k pull-down, R13=10k series)
// Battery voltage divider: Vbat → R5(100k) → junction → R8(22k) → GND; junction → R4(10k) → IO3
constexpr float kBatteryDividerScale = (100000.0f + 22000.0f) / 22000.0f;  // ~5.545 theoretical
// ESP32-S2 ADC effective Vref is ~2.9V, not 3.3V. This causes raw counts to read ~23% high.
// kAdcCalFactor corrects both battery and thermistor readings. Derived from measured battery data:
// 10V→12.32, 11V→13.54, 12V→14.80, 13V→16.09, 14V→17.35 reported (mean ratio 1.234, factor=1/1.234)
constexpr float kAdcCalFactor = 0.810f;
constexpr float kBatteryAdcRef = 3.3f;
constexpr float kBatteryAdcFullScale = 4095.0f;
constexpr float kBatteryVoltFull = 12.7f;   // 100% (SLA fully charged)
constexpr float kBatteryVoltEmpty = 10.5f;  // 0%  (SLA discharged)
// Steinhart-Hart coefficients for thermistor - from empirical calibration in boosh_box_esp32_remote_thermo
constexpr float kThermistorC1 = 1.274219988e-03f;
constexpr float kThermistorC2 = 2.171368266e-04f;
constexpr float kThermistorC3 = 1.119659695e-07f;
constexpr float kThermistorR1 = 10000.0f;  // R12 pull-down
// 2-point linear calibration on raw S-H output (post kAdcCalFactor, no prior offset):
//   actual 48.3°F  @ S-H base 37.73°F  (backed out from PYLONS 41.46°F reading)
//   actual 116.0°F @ S-H base 112.0°F  (backed out from PYLONS 151.5°F reading)
//   slope = (116.0-48.3)/(112.0-37.73) = 0.9116
//   intercept = 48.3 - 0.9116*37.73 = 13.92°F
// Note: 156.9°F reference point was rejected — thermistor not equilibrated during that test.
constexpr float kThermistorCalScale = 0.9116f;
constexpr float kThermistorCalOffsetF = 13.92f;
constexpr int kThermistorAvgSamples = 16;
constexpr unsigned long kSensorPollIntervalMs = 5000;
constexpr unsigned long kBatteryHistoryIntervalMs = 60000; // 1 min between battery history samples
constexpr int kBatteryHistorySize = 20;             // 20 min of history for rate calc
// Battery plot buffers (separate from the rate-estimation ring buffer above)
constexpr int kBattPlotLongSize = 1440;             // 24 h × 60 min/h = 1440 points
constexpr int kBattPlotShortSize = 180;             // 30 min × 2 pts/min (10 s interval)
constexpr unsigned long kBattPlotLongIntervalMs  = 60000;  // 1 min
constexpr unsigned long kBattPlotShortIntervalMs = 10000;  // 10 s
constexpr uint8_t kOledAddress = 0x3C;
constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 32;
constexpr uint16_t kOscPort = 8000;
constexpr const char *kOscAddress = "/pylon/BooshMain";
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
constexpr const char *kFirmwareSemver = "0.0.1";
constexpr const char *kFirmwareBuildDate = __DATE__;
constexpr const char *kFirmwareBuildTime = __TIME__;
constexpr const char *kFirmwareVersion = "0.0.1 " __DATE__ " " __TIME__;
constexpr const char *kTelemetryTemperatureDefault = "N/A";
constexpr const char *kTelemetryBatteryVoltageDefault = "N/A";
constexpr const char *kTelemetryBatteryChargePercentDefault = "N/A";
constexpr size_t kWebLogBufferMaxChars = 8192;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);
WiFiUDP oscUdp;
WebServer webServer(80);
bool display_inverted = false;
bool web_server_started = false;
bool boosh_failsafe_armed = false;
unsigned long boosh_failsafe_start_ms = 0;
unsigned long boosh_failsafe_note_until_ms = 0;
unsigned long wifi_connected_since_ms = 0;
uint8_t last_disconnect_reason = WIFI_REASON_UNSPECIFIED;
bool wifi_has_ip = false;
bool registry_announced = false;
unsigned long registry_last_success_ms = 0;
unsigned long registry_next_attempt_ms = 0;
uint8_t registry_consecutive_failures = 0;
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
volatile bool ping_restored = false;  // set by ping task, cleared by main loop for registry re-announce
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

constexpr const char *kPrefsNamespace = "pylon_cfg";
constexpr const char *kPrefsKeyId = "id";
constexpr const char *kPrefsKeyHost = "host";
constexpr const char *kPrefsKeyDesc = "desc";
constexpr const char *kPrefsKeyAp = "ap_en";
constexpr const char *kPrefsKeyUserSsid = "usr_ssid";
constexpr const char *kPrefsKeyUserPass = "usr_pass";

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
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
      continue;
    }
    if (c == '\n') {
      out += "\\n";
      continue;
    }
    if (c == '\r') {
      out += "\\r";
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
    if (log_output) {
      Console.print("[CFG] desc set: ");
      Console.println(pylon_description);
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
  } else {
    if (log_output) {
      Console.println("[CFG] unknown set field. use id|host|desc|node|ap");
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
  payload += "\"wifi_ssid\":\"" + JsonEscape(user_wifi_ssid) + "\"";
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
  payload += "\"osc_paths\":[\"" + String(kOscAddress) + "\"],";
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
  json.reserve(512);
  json += "\"display_pages\":{";
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
  <title>Pylons</title>
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
      <h1>Pylons Control</h1>
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
    function renderMeta(data) {
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
      const pill = document.getElementById('solenoid');
      pill.textContent = data.solenoid_active ? 'Solenoid active' : 'Solenoid idle';
      pill.className = data.solenoid_active ? 'pill active' : 'pill';
      setPage('oled-ping', data.display_pages.ping);
      setPage('oled-wifi', data.display_pages.wifi);
      setPage('oled-wifi-detail', data.display_pages.wifi_detail);
      setPage('oled-node', data.display_pages.node);
      setPage('oled-firmware', data.display_pages.firmware);
      syncConfigField('cfg-id', data.pylon_id || '');
      syncConfigField('cfg-host', (data.hostname || '').replace(/\.local$/,''));
      syncConfigField('cfg-description', data.description || '');
      syncConfigField('cfg-wifi-ssid', data.wifi_ssid || '');
      const apBox = document.getElementById('cfg-ap');
      if (apBox && document.activeElement !== apBox) apBox.checked = !!data.ap_enabled;
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
    const configInputs = ['cfg-id', 'cfg-host', 'cfg-description', 'cfg-node', 'cfg-wifi-ssid', 'cfg-wifi-pass']
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
      document.getElementById('chart-short').src = '/api/chart/battery/short?t=' + t;
      document.getElementById('chart-long').src  = '/api/chart/battery/long?t='  + t;
    }
    refreshTelemetry();
    refreshLogs();
    refreshCharts();
    setInterval(refreshTelemetry, 1000);
    setInterval(refreshLogs, 1500);
    setInterval(refreshCharts, 15000);
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
  const bool has_wifi_ssid = webServer.hasArg("wifi_ssid");
  const bool has_wifi_pass = webServer.hasArg("wifi_pass");

  if (!has_node && !has_id && !has_host && !has_desc && !has_wifi_ssid && !has_wifi_pass) {
    SendApiError(400, "expected one of: node, id, host, description, wifi_ssid, wifi_pass");
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
  if (has_wifi_ssid) ok = ok && SetConfigFieldValue("wifi_ssid", webServer.arg("wifi_ssid"));
  if (has_wifi_pass) ok = ok && SetConfigFieldValue("wifi_pass", webServer.arg("wifi_pass"));

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

// ---- Battery chart SVG ------------------------------------------------------

String BuildBatterySvg(BattPlotPoint *buf, int head, int count, int capacity,
                       unsigned long span_label_ms, const char *title) {
  constexpr int W = 720, H = 200;
  constexpr int PL = 46, PR = 12, PT = 20, PB = 30;  // padding
  constexpr int PW = W - PL - PR;                      // plot width
  constexpr int PH = H - PT - PB;                      // plot height
  constexpr float V_MIN = 10.0f, V_MAX = 15.0f;

  String s;
  s.reserve(6000);
  char buf8[48];

  s += "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 ";
  s += W; s += " "; s += H;
  s += "' style='background:#080c12;display:block;width:100%;max-width:720px'>";

  // Title
  s += "<text x='"; s += W / 2; s += "' y='14' text-anchor='middle' "
       "fill='#7a9bb5' font-size='11' font-family='monospace'>"; s += title; s += "</text>";

  // Y gridlines and labels (10, 11, 12, 13, 14, 15 V)
  for (int vv = (int)V_MIN; vv <= (int)V_MAX; ++vv) {
    const float frac = (vv - V_MIN) / (V_MAX - V_MIN);
    const int y = PT + PH - (int)(frac * PH);
    snprintf(buf8, sizeof(buf8), "%d", y);
    s += "<line x1='"; s += PL; s += "' y1='"; s += buf8;
    s += "' x2='"; s += (W - PR); s += "' y2='"; s += buf8;
    s += "' stroke='#1a2635' stroke-width='1'/>";
    snprintf(buf8, sizeof(buf8), "%dV", vv);
    s += "<text x='"; s += (PL - 4);
    s += "' y='"; s += y + 4;
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
    // time represented: fraction of span from oldest (left) to newest (right)
    const unsigned long ago_ms = (unsigned long)((1.0f - (float)i / 4.0f) * span_label_ms);
    if (ago_ms == 0) {
      snprintf(buf8, sizeof(buf8), "now");
    } else if (span_label_ms >= 3600000UL) {
      snprintf(buf8, sizeof(buf8), "%ldh ago", (long)(ago_ms / 3600000UL));
    } else {
      snprintf(buf8, sizeof(buf8), "%ldm ago", (long)(ago_ms / 60000UL));
    }
    s += "<text x='"; s += x;
    s += "' y='"; s += (PT + PH + 14);
    s += "' text-anchor='middle' fill='#4a6175' font-size='10' font-family='monospace'>";
    s += buf8; s += "</text>";
  }

  if (count < 2) {
    s += "<text x='"; s += W / 2;
    s += "' y='"; s += (H / 2 + 5);
    s += "' text-anchor='middle' fill='#3a5060' font-size='13' font-family='monospace'>"
         "No data yet</text>";
    s += "</svg>";
    return s;
  }

  // Map ring buffer to chronological order and find actual time span
  const int oldest_idx = (head - count + capacity) % capacity;
  const uint32_t t_oldest = buf[oldest_idx].ms;
  const int newest_idx  = (head - 1 + capacity) % capacity;
  const uint32_t t_span = buf[newest_idx].ms - t_oldest;
  if (t_span == 0) {
    s += "</svg>";
    return s;
  }

  // Polyline
  s += "<polyline fill='none' stroke='#4fb3ff' stroke-width='1.5' stroke-linejoin='round' points='";
  for (int i = 0; i < count; ++i) {
    const int idx = (oldest_idx + i) % capacity;
    const float x_frac = (float)(buf[idx].ms - t_oldest) / (float)t_span;
    const float y_frac = (buf[idx].v - V_MIN) / (V_MAX - V_MIN);
    const int px = PL + (int)(x_frac * PW);
    const int py = PT + PH - (int)(constrain(y_frac, 0.0f, 1.0f) * PH);
    snprintf(buf8, sizeof(buf8), "%d,%d ", px, py);
    s += buf8;
  }
  s += "'/>";

  // Current value label at right edge
  const float v_last = buf[newest_idx].v;
  const float y_frac_last = (v_last - V_MIN) / (V_MAX - V_MIN);
  const int py_last = PT + PH - (int)(constrain(y_frac_last, 0.0f, 1.0f) * PH);
  snprintf(buf8, sizeof(buf8), "%.2fV", v_last);
  s += "<text x='"; s += (W - PR + 2);
  s += "' y='"; s += py_last + 4;
  s += "' fill='#4fb3ff' font-size='10' font-family='monospace'>"; s += buf8; s += "</text>";

  s += "</svg>";
  return s;
}

void HandleChartBatteryLongApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildBatterySvg(battery_plot_long, battery_plot_long_head, battery_plot_long_count,
                    kBattPlotLongSize, 86400000UL, "Battery voltage — 24 h (1 min/sample)"));
}

void HandleChartBatteryShortApi() {
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "image/svg+xml",
    BuildBatterySvg(battery_plot_short, battery_plot_short_head, battery_plot_short_count,
                    kBattPlotShortSize, 1800000UL, "Battery voltage — 30 min (10 s/sample)"));
}

// ---- OTA update handlers ----------------------------------------------------

void HandleOtaUploadBody() {
  HTTPUpload &upload = webServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Console.printf("[OTA] Start: %s (%u bytes)\n", upload.filename.c_str(), upload.totalSize);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Console.printf("[OTA] begin() failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Console.printf("[OTA] write() failed: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_END) {
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
  USB.begin();
  Serial.begin(115200);
  esp_task_wdt_delete(NULL);  // remove loopTask from watchdog; blocking calls (ping, HTTP) are intentional
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 2000) {
    delay(10);
  }
  serial_cli_line.reserve(192);
  LoadPylonConfig();
  PrintCliHelp();
  pinMode(kDevBoardButtonPin, INPUT_PULLUP);
  pinMode(kBatteryAdcPin, INPUT);
  pinMode(kThermistorAdcPin, INPUT);
  analogReadResolution(12);

  pinMode(kLedWhitePin, OUTPUT);
  digitalWrite(kLedWhitePin, LOW);
  // Green/Blue/Yellow: 5kHz PWM carrier, duty updated in loop() for sine wave
  ledcSetup(0, 5000, 8);
  ledcAttachPin(kLedGreenPin, 0);
  ledcWrite(0, 255);  // solid on at boot; PollBlinkLeds() transitions to sine wave
  ledcSetup(1, 5000, 8);
  ledcAttachPin(kLedBluePin, 1);
  ledcWrite(1, 0);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(kLedYellowPin, 2);
  ledcWrite(2, 0);

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
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(100);

  LogBootStep("WiFi scan...");
  int networkCount = WiFi.scanNetworks(false, true);
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
    while (WiFi.status() != WL_CONNECTED && millis() - start < kPerNetworkTimeoutMs) {
      delay(250);
      Console.print(".");
    }
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
  if (!msg.fullMatch(kOscAddress)) {
    return;
  }

  if (msg.size() != 1 || !msg.isFloat(0)) {
    Console.println("OSC " + String(kOscAddress) + " ignored (unexpected args).");
    return;
  }

  float value = msg.getFloat(0);

  Console.print("Received OSC message: ");
  Console.print(kOscAddress);
  Console.print(" with argument: ");
  Console.println(value, 3);
  ApplyBooshState(value, "osc");
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

float ReadBatteryVoltage() {
  // Average 16 samples to reduce ADC noise
  int32_t raw = 0;
  for (int i = 0; i < 16; ++i) {
    raw += analogRead(kBatteryAdcPin);
    delayMicroseconds(100);
  }
  raw >>= 4;  // divide by 16
  if (raw <= 0) return NAN;
  const float v_adc = raw * kBatteryAdcRef / kBatteryAdcFullScale;
  return v_adc * kBatteryDividerScale * kAdcCalFactor;
}

float ReadThermistorF() {
  // Average 16 samples (same pattern as reference project)
  int32_t raw = 0;
  for (int i = 0; i < kThermistorAvgSamples; ++i) {
    raw += analogRead(kThermistorAdcPin);
    delayMicroseconds(100);
  }
  raw >>= 4;
  if (raw <= 0 || raw >= 4095) return NAN;
  // Steinhart-Hart: thermistor is upper element, R1 is pull-down
  // Vadc = Vcc * R1 / (Rth + R1)  → Rth = R1*(4095/raw - 1)
  // Apply kAdcCalFactor: ADC raw is ~23% high due to ESP32-S2 Vref; dividing raw corrects the ratio.
  const float r_th = kThermistorR1 * (kBatteryAdcFullScale / ((float)raw * kAdcCalFactor) - 1.0f);
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
    const float pct = (v - kBatteryVoltEmpty) / (kBatteryVoltFull - kBatteryVoltEmpty) * 100.0f;
    sensor_battery_pct = constrain(pct, 0.0f, 100.0f);

    // Store history point every kBatteryHistoryIntervalMs for discharge-rate estimation
    if (battery_last_history_ms == 0 || now - battery_last_history_ms >= kBatteryHistoryIntervalMs) {
      battery_last_history_ms = now;
      battery_v_history[battery_v_history_head] = v;
      battery_v_history_ms[battery_v_history_head] = now;
      battery_v_history_head = (battery_v_history_head + 1) % kBatteryHistorySize;
      if (battery_v_history_count < kBatteryHistorySize) battery_v_history_count++;

      // Estimate time remaining: need at least 5 history points (~5 minutes)
      if (battery_v_history_count >= 5) {
        // Oldest point in circular buffer
        const int oldest_idx = (battery_v_history_head - battery_v_history_count + kBatteryHistorySize) % kBatteryHistorySize;
        const int newest_idx = (battery_v_history_head - 1 + kBatteryHistorySize) % kBatteryHistorySize;
        const float v_old = battery_v_history[oldest_idx];
        const float v_new = battery_v_history[newest_idx];
        const unsigned long t_span_ms = battery_v_history_ms[newest_idx] - battery_v_history_ms[oldest_idx];
        if (t_span_ms > 0) {
          const float v_per_ms = (v_new - v_old) / (float)t_span_ms;  // negative when discharging
          if (v_per_ms < -1e-7f) {  // only estimate if measurably discharging
            const float remaining_v = v - kBatteryVoltEmpty;
            sensor_battery_time_remaining_h = (remaining_v / (-v_per_ms)) / 3600000.0f;
          } else {
            sensor_battery_time_remaining_h = NAN;  // charging or stable
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
    ledcWrite(0, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.4f * t)) / 2.0f) * kMaxDuty));  // green 0.4Hz
    ledcWrite(1, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.8f * t)) / 2.0f) * kMaxDuty));  // blue 0.8Hz
    ledcWrite(2, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 1.2f * t)) / 2.0f) * kMaxDuty));  // yellow 1.2Hz
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
          ping_restored = true;  // main loop will re-announce to registry
          Console.println("[Ping] restored");
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

    vTaskDelay(pdMS_TO_TICKS(10));  // yield; ping timing is driven by lastPingMs
  }
}

// ---- Main loop --------------------------------------------------------------

void loop() {
  static bool wasConnected = false;
  static unsigned long lastDisplayMs = 0;
  static uint8_t displayPage = 0;
  static unsigned long lastIdentMs = 0;
  static uint8_t identPhase = 0;

  PollBlinkLeds();
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
    if (wasConnected) {
      wasConnected = false;
      ping_restored = false;
      target_ip_string = "";
      registry_announced = false;
      registry_next_attempt_ms = 0;
      registry_consecutive_failures = 0;
      Console.println("WiFi disconnected: registry state reset.");
    }
    if (ap_active) {
      ShowStatus("AP mode", "PYLON_" + pylon_id);
      delay(100);
    } else {
      ShowStatus("WiFi lost", "Reconnecting");
      delay(1000);
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
    SetupWebServer();
    Console.println("WiFi connected: scheduling registry announce.");
  }
  HandleRegistry(now);
  PollOsc();  // drain any packets that arrived during registry HTTP
  if (!wifi_has_ip && wifi_connected_since_ms == 0) {
    wifi_connected_since_ms = now;
  }
  if (boosh_failsafe_armed && now - boosh_failsafe_start_ms >= kBooshFailsafeTimeoutMs) {
    boosh_failsafe_armed = false;
    ApplyBooshState(0.0f);
    Console.println("Failsafe: BooshMain timeout -> forcing OFF.");
    ShowStatus("Failsafe timeout", "BooshMain OFF");
    boosh_failsafe_note_until_ms = now + kBooshFailsafeNoteMs;
  }

  // Ping runs in PingTask (background, Core 0). Check if it came back up.
  if (ping_restored) {
    ping_restored = false;
    registry_announced = false;
    registry_next_attempt_ms = now;
    registry_consecutive_failures = 0;
    Console.println("[Loop] ping restored: scheduling registry announce.");
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
      displayPage = static_cast<uint8_t>((displayPage + 1) % 5);
    }

    {
      PingStats disp_stats;
      disp_stats.count = telemetry_ping_count;
      disp_stats.min_ms = telemetry_ping_has_data ? telemetry_ping_min_ms : UINT32_MAX;
      disp_stats.max_ms = telemetry_ping_max_ms;
      disp_stats.sum_ms = static_cast<uint64_t>(telemetry_ping_avg_ms) * telemetry_ping_count;
      disp_stats.last_ms = telemetry_ping_last_ms;
      if (displayPage == 0) {
        ShowPingStats("RPIBOOSH", disp_stats, current_ping_last_ok, last_ping_success_ms, now);
      }
    }
    if (displayPage == 1) {
      ShowWifiMetricsPage(0, wifi_connected_since_ms, last_disconnect_reason);
    } else if (displayPage == 2) {
      ShowWifiMetricsPage(1, wifi_connected_since_ms, last_disconnect_reason);
    } else if (displayPage == 3) {
      ShowNodeConfigPage();
    } else if (displayPage == 4) {
      ShowFirmwarePage();
    }
  }

  delay(5);
}
