#include <Arduino.h>
#include <Wire.h>
#include <USB.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <esp_mac.h>
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
bool telemetry_ping_last_ok = false;
bool telemetry_ping_has_data = false;
uint32_t telemetry_ping_sent = 0;
uint32_t telemetry_ping_lost = 0;
uint32_t telemetry_ping_last_ms = 0;
uint32_t telemetry_ping_min_ms = 0;
uint32_t telemetry_ping_max_ms = 0;
uint32_t telemetry_ping_avg_ms = 0;
uint32_t telemetry_ping_count = 0;
uint32_t trigger_event_count = 0;
bool current_ping_last_ok = false;
unsigned long last_ping_success_ms = 0;
String target_ip_string = "";
String web_log_text;
String web_log_partial_line;

constexpr const char *kPrefsNamespace = "pylon_cfg";
constexpr const char *kPrefsKeyId = "id";
constexpr const char *kPrefsKeyHost = "host";
constexpr const char *kPrefsKeyDesc = "desc";

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
  } else {
    if (log_output) {
      Console.println("[CFG] unknown set field. use id|host|desc|node");
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
  payload += "\"description\":\"" + JsonEscape(pylon_description) + "\"";
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
  payload += "\"osc_paths\":[\"/rpiboosh/BooshMain\"],";
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
  payload += "\"temperature\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"temperature_f\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"temperature_c\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"battery_voltage\":\"" + JsonEscape(String(kTelemetryBatteryVoltageDefault)) + "\",";
  payload += "\"battery_voltage_v\":\"" + JsonEscape(String(kTelemetryBatteryVoltageDefault)) + "\",";
  payload += "\"battery_charge\":\"" + JsonEscape(String(kTelemetryBatteryChargePercentDefault)) + "\",";
  payload += "\"battery_charge_pct\":\"" + JsonEscape(String(kTelemetryBatteryChargePercentDefault)) + "\",";
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

void SetBooshActive(bool active, const char *source) {
  if (active) {
    if (!display_inverted) {
      trigger_event_count += 1;
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
  payload += "\"solenoid_active\":" + String(display_inverted ? "true" : "false") + ",";
  payload += "\"trigger_event_count\":" + String(trigger_event_count) + ",";
  payload += "\"target_ip\":\"" + JsonEscape(target_ip_string) + "\",";
  payload += "\"telemetry\":{";
  payload += "\"ipv4\":\"" + JsonEscape(ip) + "\",";
  payload += "\"mdns_hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"firmware_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"fw_semver\":\"" + JsonEscape(String(kFirmwareSemver)) + "\",";
  payload += "\"fw_build_date\":\"" + JsonEscape(String(kFirmwareBuildDate)) + "\",";
  payload += "\"fw_build_time\":\"" + JsonEscape(String(kFirmwareBuildTime)) + "\",";
  payload += "\"temperature\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"temperature_f\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"temperature_c\":\"" + JsonEscape(String(kTelemetryTemperatureDefault)) + "\",";
  payload += "\"battery_voltage\":\"" + JsonEscape(String(kTelemetryBatteryVoltageDefault)) + "\",";
  payload += "\"battery_voltage_v\":\"" + JsonEscape(String(kTelemetryBatteryVoltageDefault)) + "\",";
  payload += "\"battery_charge\":\"" + JsonEscape(String(kTelemetryBatteryChargePercentDefault)) + "\",";
  payload += "\"battery_charge_pct\":\"" + JsonEscape(String(kTelemetryBatteryChargePercentDefault)) + "\",";
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
    button{border:0;border-radius:999px;background:linear-gradient(180deg,var(--accent) 0,var(--accent-2) 100%);color:#06111d;padding:12px 18px;font-size:16px;font-weight:700;cursor:pointer}
    button:disabled{opacity:.5;cursor:wait}
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
      <button id="trigger">Press and Hold Solenoid</button>
    </div>
    <div class="panel" style="margin-top:16px">
      <h2>Node Config</h2>
      <form id="config-form" class="meta">
        <label>ID <input id="cfg-id" name="id"></label>
        <label>Host <input id="cfg-host" name="host"></label>
        <label>Description <input id="cfg-description" name="description"></label>
        <label>Node Alias <input id="cfg-node" name="node" placeholder="sets id + host"></label>
        <div style="display:flex;gap:12px;align-items:center;flex-wrap:wrap">
          <button type="submit">Save Config</button>
          <span id="config-status"></span>
        </div>
      </form>
    </div>
    <div class="grid">
      <section class="panel oled"><h2>OLED Ping</h2><pre id="oled-ping"></pre></section>
      <section class="panel oled"><h2>OLED Wi-Fi</h2><pre id="oled-wifi"></pre></section>
      <section class="panel oled"><h2>OLED Wi-Fi Detail</h2><pre id="oled-wifi-detail"></pre></section>
      <section class="panel oled"><h2>OLED Node</h2><pre id="oled-node"></pre></section>
      <section class="panel oled"><h2>OLED Firmware</h2><pre id="oled-firmware"></pre></section>
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
        ['Uptime', data.telemetry.uptime_hms]
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
    }
    async function refreshTelemetry() {
      renderMeta(await fetchJson('/api/telemetry'));
    }
    async function refreshLogs() {
      const data = await fetchJson('/api/logs');
      document.getElementById('log-text').textContent = data.log;
      const log = document.getElementById('log');
      log.scrollTop = log.scrollHeight;
    }
    const triggerButton = document.getElementById('trigger');
    const configForm = document.getElementById('config-form');
    const configInputs = ['cfg-id', 'cfg-host', 'cfg-description', 'cfg-node']
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
    refreshTelemetry();
    refreshLogs();
    setInterval(refreshTelemetry, 1000);
    setInterval(refreshLogs, 1500);
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

  if (!has_node && !has_id && !has_host && !has_desc) {
    SendApiError(400, "expected one of: node, id, host, description");
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

void HandleSolenoidOnApi() {
  SetBooshActive(true, "http");
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"solenoid_active\":true,\"triggered_via\":\"http\"}");
}

void HandleSolenoidOffApi() {
  SetBooshActive(false, "http");
  webServer.send(200, "application/json",
                 "{\"ok\":true,\"solenoid_active\":false,\"triggered_via\":\"http\"}");
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
  webServer.on("/api/solenoid/on", HTTP_POST, HandleSolenoidOnApi);
  webServer.on("/api/solenoid/off", HTTP_POST, HandleSolenoidOffApi);
  webServer.on("/api/solenoid/trigger", HTTP_POST, HandleSolenoidOnApi);
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
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 2000) {
    delay(10);
  }
  serial_cli_line.reserve(192);
  LoadPylonConfig();
  PrintCliHelp();
  pinMode(kDevBoardButtonPin, INPUT_PULLUP);

  pinMode(kLedWhitePin, OUTPUT);
  digitalWrite(kLedWhitePin, LOW);
  // Green/Blue/Yellow: 5kHz PWM carrier, duty updated in loop() for sine wave
  ledcSetup(0, 5000, 8);
  ledcAttachPin(kLedGreenPin, 0);
  ledcWrite(0, 0);
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

  const char *targetSsid = hasLowLatency ? BOOSH_WIFI_SSID_LL : BOOSH_WIFI_SSID_MW;
  const char *targetPass = hasLowLatency ? BOOSH_WIFI_PASS_LL : BOOSH_WIFI_PASS_MW;

  Console.print("Connecting to ");
  Console.println(targetSsid);
  ShowStatus("Connecting to", String(targetSsid));
  WiFi.begin(targetSsid, targetPass);

  unsigned long start = millis();
  const unsigned long timeoutMs = 20000;
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Console.print(".");
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
    ShowStatus("WiFi failed", "Check SSID");
  }
}

void HandleOscMessage(OSCMessage &msg) {
  if (!msg.fullMatch(kOscAddress)) {
    return;
  }

  if (msg.size() != 1 || !msg.isFloat(0)) {
    Console.println("OSC /rpiboosh/BooshMain ignored (unexpected args).");
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

  // Green/Blue/Yellow: sine wave brightness 0-33%, 5kHz carrier via LEDC
  const float t = now / 1000.0f;
  constexpr float kMaxDuty = 0.33f * 255.0f;
  ledcWrite(0, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.4f * t)) / 2.0f) * kMaxDuty));  // green 0.4Hz
  ledcWrite(1, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 0.8f * t)) / 2.0f) * kMaxDuty));  // blue 0.8Hz
  ledcWrite(2, (uint8_t)(((1.0f + sinf(2.0f * M_PI * 1.2f * t)) / 2.0f) * kMaxDuty));  // yellow 1.2Hz
}

void loop() {
  static const char *kTargetHost = kPingTargetHost;
  static const char *kTargetHostMdns = "RPIBOOSH.local";
  static bool wasConnected = false;
  static bool pingWasDown = false;
  static IPAddress targetIp;
  static bool hasIp = false;
  static unsigned long lastResolveMs = 0;
  static PingStats stats;
  static bool lastOk = false;
  static unsigned long lastPingMs = 0;
  static unsigned long lastPingSuccessMs = 0;
  static unsigned long lastDisplayMs = 0;
  static uint8_t displayPage = 0;

  PollBlinkLeds();
  PollSerialCli();
  PollDevBoardButton();
  webServer.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) {
      wasConnected = false;
      pingWasDown = false;
      hasIp = false;
      target_ip_string = "";
      registry_announced = false;
      registry_next_attempt_ms = 0;
      registry_consecutive_failures = 0;
      Console.println("WiFi disconnected: registry state reset.");
    }
    ShowStatus("WiFi lost", "Reconnecting");
    delay(1000);
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
  if (!hasIp && now - lastResolveMs > 10000) {
    lastResolveMs = now;
    Console.println("Resolving RPIBOOSH...");
    if (WiFi.hostByName(kTargetHost, targetIp) || WiFi.hostByName(kTargetHostMdns, targetIp)) {
      hasIp = true;
      target_ip_string = targetIp.toString();
      Console.print("Resolved to ");
      Console.println(targetIp);
    } else {
      Console.println("Resolve failed.");
      ShowStatus("Resolve failed", "RPIBOOSH");
      delay(1000);
      return;
    }
  }

  if (!hasIp) {
    delay(250);
    return;
  }

  if (now - lastPingMs >= 1000) {
    lastPingMs = now;
    telemetry_ping_sent += 1;
    lastOk = Ping.ping(targetIp, 1);
    telemetry_ping_last_ok = lastOk;
    current_ping_last_ok = lastOk;
    if (lastOk) {
      uint32_t lastMs = static_cast<uint32_t>(Ping.averageTime());
      stats.last_ms = lastMs;
      stats.count += 1;
      stats.sum_ms += lastMs;
      if (lastMs < stats.min_ms) {
        stats.min_ms = lastMs;
      }
      if (lastMs > stats.max_ms) {
        stats.max_ms = lastMs;
      }
      lastPingSuccessMs = now;
      last_ping_success_ms = now;
      Console.print("Ping ");
      Console.print(kTargetHost);
      Console.print(" ");
      Console.print(lastMs);
      Console.println("ms");
      if (pingWasDown) {
        pingWasDown = false;
        registry_announced = false;
        registry_next_attempt_ms = now;
        registry_consecutive_failures = 0;
        Console.println("Ping restored: scheduling registry announce.");
      }
    } else {
      telemetry_ping_lost += 1;
      pingWasDown = true;
      Console.println("Ping failed.");
    }
    telemetry_ping_has_data = stats.has_data();
    telemetry_ping_last_ms = stats.last_ms;
    telemetry_ping_count = stats.count;
    telemetry_ping_avg_ms = stats.avg_ms();
    telemetry_ping_min_ms = telemetry_ping_has_data ? stats.min_ms : 0;
    telemetry_ping_max_ms = telemetry_ping_has_data ? stats.max_ms : 0;
  }

  if (boosh_failsafe_note_until_ms == 0 || now >= boosh_failsafe_note_until_ms) {
    boosh_failsafe_note_until_ms = 0;
    if (lastDisplayMs == 0) {
      lastDisplayMs = now;
    } else if (now - lastDisplayMs >= kDisplayCycleMs) {
      lastDisplayMs = now;
      displayPage = static_cast<uint8_t>((displayPage + 1) % 5);
    }

    if (displayPage == 0) {
      ShowPingStats("RPIBOOSH", stats, lastOk, lastPingSuccessMs, now);
    } else if (displayPage == 1) {
      ShowWifiMetricsPage(0, wifi_connected_since_ms, last_disconnect_reason);
    } else if (displayPage == 2) {
      ShowWifiMetricsPage(1, wifi_connected_since_ms, last_disconnect_reason);
    } else if (displayPage == 3) {
      ShowNodeConfigPage();
    } else {
      ShowFirmwarePage();
    }
  }

  delay(5);
}
