#include <Arduino.h>
#include <Wire.h>
#include <USB.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESP32Ping.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#include "wifi_credentials.h"

constexpr int kI2cSda = 8;
constexpr int kI2cScl = 9;
constexpr int kOledReset = 18;
constexpr uint8_t kOledAddress = 0x3C;
constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 32;
constexpr uint16_t kOscPort = 8000;
constexpr const char *kOscAddress = "/rpiboosh/BooshMain";
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
constexpr uint16_t kRegistryHttpTimeoutMs = 800;
constexpr const char *kFirmwareVersion = "pylons " __DATE__ " " __TIME__;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledReset);
WiFiUDP oscUdp;
bool display_inverted = false;
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
uint32_t telemetry_ping_last_ms = 0;
uint32_t telemetry_ping_min_ms = 0;
uint32_t telemetry_ping_max_ms = 0;
uint32_t telemetry_ping_avg_ms = 0;
uint32_t telemetry_ping_count = 0;

constexpr const char *kPrefsNamespace = "pylon_cfg";
constexpr const char *kPrefsKeyId = "id";
constexpr const char *kPrefsKeyHost = "host";
constexpr const char *kPrefsKeyDesc = "desc";

void SetDisplayInverted(bool inverted) {
  if (display_inverted == inverted) {
    return;
  }
  display_inverted = inverted;
  display.invertDisplay(inverted);
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
  Serial.println(message);
  ShowStatus(message, detail);
}

String TrimForDisplay(const String &input, size_t maxChars) {
  if (input.length() <= maxChars) {
    return input;
  }
  return input.substring(0, maxChars);
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
    Serial.println("[CFG] failed to open Preferences namespace for write");
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
    Serial.println("[CFG] failed to open Preferences namespace for clear");
    return false;
  }
  prefs.clear();
  prefs.end();
  Serial.println("[CFG] NVS namespace cleared.");
  return true;
}

void ScheduleRegistryRefreshNow() {
  registry_announced = false;
  registry_next_attempt_ms = millis();
  registry_consecutive_failures = 0;
}

void PrintPylonConfig() {
  Serial.println("[CFG] current values:");
  Serial.print("  id: ");
  Serial.println(pylon_id);
  Serial.print("  host: ");
  Serial.println(pylon_mdns_host);
  Serial.print("  desc: ");
  Serial.println(pylon_description);
}

void LoadPylonConfig() {
  Preferences prefs;
  if (!prefs.begin(kPrefsNamespace, false)) {
    Serial.println("[CFG] failed to open Preferences namespace");
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
    Serial.println("[CFG] Preferences unprogrammed, wrote defaults.");
  } else {
    pylon_id = stored_id;
    if (stored_host.length() == 0) {
      stored_host = pylon_id;
    }
    if (!IsValidMdnsHost(stored_host)) {
      stored_host = pylon_id;
      Serial.println("[CFG] Stored host invalid, using ID as host.");
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
    Serial.print("[CFG] mDNS updated: ");
    Serial.print(pylon_mdns_host);
    Serial.println(".local");
  } else {
    Serial.println("[CFG] mDNS update failed.");
  }
}

void PrintCliHelp() {
  Serial.println("[CLI] commands:");
  Serial.println("  help");
  Serial.println("  show");
  Serial.println("  set id <value>");
  Serial.println("  set host <value>");
  Serial.println("  set desc <value>");
  Serial.println("  set node <value>   (sets both id and host)");
  Serial.println("  clear nvs          (erase saved id/host/desc)");
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
    Serial.println("[CLI] unknown command. type 'help'");
    return;
  }

  String rest = line.substring(4);
  rest.trim();
  const int sep = rest.indexOf(' ');
  if (sep <= 0) {
    Serial.println("[CLI] invalid set command");
    return;
  }
  String field = ToLowerAscii(rest.substring(0, sep));
  String value = rest.substring(sep + 1);
  value.trim();
  if (value.length() == 0) {
    Serial.println("[CLI] value cannot be empty");
    return;
  }

  bool changed = false;
  if (field == "id") {
    const String new_id = NormalizePylonId(value);
    if (new_id.length() == 0) {
      Serial.println("[CLI] invalid id");
      return;
    }
    pylon_id = new_id;
    changed = true;
    Serial.print("[CLI] id set: ");
    Serial.println(pylon_id);
  } else if (field == "host" || field == "mdns") {
    const String new_host = NormalizeMdnsHost(value);
    if (!IsValidMdnsHost(new_host)) {
      Serial.println("[CLI] invalid host; allowed: letters, digits, '-' (1..63 chars)");
      return;
    }
    pylon_mdns_host = new_host;
    changed = true;
    Serial.print("[CLI] host set: ");
    Serial.println(pylon_mdns_host);
  } else if (field == "desc" || field == "description") {
    pylon_description = value;
    changed = true;
    Serial.print("[CLI] desc set: ");
    Serial.println(pylon_description);
  } else if (field == "node") {
    const String new_id = NormalizePylonId(value);
    const String new_host = NormalizeMdnsHost(value);
    if (new_id.length() == 0 || !IsValidMdnsHost(new_host)) {
      Serial.println("[CLI] invalid node value");
      return;
    }
    pylon_id = new_id;
    pylon_mdns_host = new_host;
    changed = true;
    Serial.print("[CLI] node set (id+host): ");
    Serial.println(pylon_id);
  } else {
    Serial.println("[CLI] unknown set field. use id|host|desc|node");
    return;
  }

  if (!changed) {
    return;
  }
  if (!SavePylonConfig()) {
    Serial.println("[CFG] failed to persist config");
    return;
  }
  RestartMdnsIfConnected();
  ScheduleRegistryRefreshNow();
  PrintPylonConfig();
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
  payload.reserve(520);
  payload += "{";
  payload += "\"pylon_id\":\"" + JsonEscape(pylon_id) + "\",";
  payload += "\"description\":\"" + JsonEscape(pylon_description) + "\",";
  payload += "\"hostname\":\"" + JsonEscape(hostname) + "\",";
  payload += "\"ip\":\"" + JsonEscape(ip) + "\",";
  payload += "\"osc_port\":" + String(kOscPort) + ",";
  payload += "\"osc_paths\":[\"/rpiboosh/BooshMain\"],";
  payload += "\"roles\":[\"boosh_main\"],";
  payload += "\"fw_version\":\"" + JsonEscape(String(kFirmwareVersion)) + "\",";
  payload += "\"ttl_sec\":" + String(kRegistryTtlSec) + ",";
  payload += "\"telemetry\":{";
  payload += "\"wifi_rssi_dbm\":" + String(wifi_rssi_dbm) + ",";
  payload += "\"ping_target\":\"" + JsonEscape(String(kPingTargetHost)) + "\",";
  payload += "\"ping\":{";
  payload += "\"last_ms\":" + String(telemetry_ping_last_ms) + ",";
  payload += "\"min_ms\":" + String(telemetry_ping_min_ms) + ",";
  payload += "\"max_ms\":" + String(telemetry_ping_max_ms) + ",";
  payload += "\"avg_ms\":" + String(telemetry_ping_avg_ms) + ",";
  payload += "\"count\":" + String(telemetry_ping_count) + ",";
  payload += "\"last_ok\":" + String(telemetry_ping_last_ok ? "true" : "false");
  payload += "},";
  payload += "\"uptime_s\":" + String(static_cast<uint32_t>(millis() / 1000));
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
    Serial.print("[REG] begin failed: ");
    Serial.println(url);
    return false;
  }
  http.setConnectTimeout(kRegistryHttpTimeoutMs);
  http.setTimeout(kRegistryHttpTimeoutMs);
  http.addHeader("Content-Type", "application/json");
  const int statusCode = http.POST(payload);
  const bool ok = statusCode >= 200 && statusCode < 300;
  Serial.print("[REG] ");
  Serial.print(kind);
  Serial.print(" ");
  Serial.print(url);
  Serial.print(" -> ");
  Serial.println(statusCode);
  http.end();
  return ok;
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
    Serial.println("[REG] post success");
  } else {
    registry_consecutive_failures = static_cast<uint8_t>(registry_consecutive_failures + 1);
    const unsigned long backoffMs = RegistryBackoffMs(registry_consecutive_failures);
    registry_next_attempt_ms = now + backoffMs;
    Serial.print("[REG] post failed, retry in ");
    Serial.print(backoffMs);
    Serial.println("ms");
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

void ShowPingStats(const String &host, const PingStats &stats, bool last_ok,
                   unsigned long last_success_ms, unsigned long now_ms) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  bool has_success = last_success_ms > 0;
  unsigned long since_success_ms = has_success ? (now_ms - last_success_ms) : 0;
  bool timing_out =
      (has_success && since_success_ms >= kPingTimeoutMs) || (!has_success && now_ms >= kPingTimeoutMs);
  display.print(host);
  display.println(timing_out ? " TIMEOUT" : (last_ok ? " OK" : " FAIL"));

  if (stats.has_data()) {
    display.print("last ");
    display.print(stats.last_ms);
    display.println("ms");
  } else {
    display.println("last --");
  }

  if (has_success) {
    display.print("since ok ");
    display.print(static_cast<uint32_t>(since_success_ms / 1000));
    display.println("s");
  } else {
    display.println("since ok never");
  }

  if (stats.has_data()) {
    display.print("min ");
    display.print(stats.min_ms);
    display.print(" max ");
    display.print(stats.max_ms);
    display.println("ms");
  } else {
    display.println("");
  }

  display.display();
}

void ShowWifiMetricsPage(uint8_t page, unsigned long connected_since_ms, uint8_t last_disconnect_reason) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (page == 0) {
    String ssid = TrimForDisplay(WiFi.SSID(), 20);
    display.print("WiFi ");
    display.println(ssid);

    display.print("RSSI ");
    display.print(WiFi.RSSI());
    display.println("dBm");

    display.print("IP ");
    display.println(WiFi.localIP().toString());

    display.print("UP ");
    if (connected_since_ms > 0) {
      display.print(static_cast<uint32_t>((millis() - connected_since_ms) / 1000));
      display.println("s");
    } else {
      display.println("--");
    }
  } else {
    display.print("RSN ");
    display.print(last_disconnect_reason);
    display.print(" ");
    display.println(WifiDisconnectReasonToString(last_disconnect_reason));

    display.print("SSID ");
    display.println(TrimForDisplay(WiFi.SSID(), 20));

    display.print("RSSI ");
    display.print(WiFi.RSSI());
    display.println("dBm");

    display.print("IP ");
    display.println(WiFi.localIP().toString());
  }

  display.display();
}

void ShowNodeConfigPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("NODE CONFIG");
  display.print("ID: ");
  display.println(TrimForDisplay(pylon_id, 14));
  display.print("DESC:");
  display.setCursor(0, 24);
  display.println(TrimForDisplay(pylon_description, 21));
  display.display();
}

void ApplyBooshState(float v0, float v1, float v2) {
  const float kOnThreshold = 0.5f;
  const float kOffThreshold = 0.5f;

  if (v0 > kOnThreshold && v1 < kOffThreshold && v2 < kOffThreshold) {
    SetDisplayInverted(true);
    boosh_failsafe_armed = true;
    boosh_failsafe_start_ms = millis();
  } else if (v0 < kOffThreshold && v1 < kOffThreshold && v2 < kOffThreshold) {
    SetDisplayInverted(false);
    boosh_failsafe_armed = false;
  }
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

  Serial.println("Boot: init I2C");
  Wire.begin(kI2cSda, kI2cScl);
  Serial.println("Boot: init OLED");
  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledAddress)) {
    Serial.println("SSD1306 init failed.");
  } else {
    display.invertDisplay(false);
    LogBootStep("Boot: OLED ready", "WEMOS S2 Pico");
  }

  Serial.println("Boot: register WiFi events");
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        wifi_connected_since_ms = millis();
        wifi_has_ip = true;
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        last_disconnect_reason = info.wifi_sta_disconnected.reason;
        wifi_connected_since_ms = 0;
        wifi_has_ip = false;
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
  Serial.print("WiFi scan count: ");
  Serial.println(networkCount);
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

  Serial.print("Connecting to ");
  Serial.println(targetSsid);
  ShowStatus("Connecting to", String(targetSsid));
  WiFi.begin(targetSsid, targetPass);

  unsigned long start = millis();
  const unsigned long timeoutMs = 20000;
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
    ShowStatus("WiFi connected", WiFi.localIP().toString());

    if (MDNS.begin(pylon_mdns_host.c_str())) {
      Serial.print("mDNS: ");
      Serial.print(pylon_mdns_host);
      Serial.println(".local");
    } else {
      Serial.println("mDNS init failed.");
    }

    if (oscUdp.begin(kOscPort)) {
      Serial.print("OSC listening on port ");
      Serial.println(kOscPort);
    } else {
      Serial.println("OSC UDP begin failed.");
    }
  } else {
    Serial.println();
    Serial.println("WiFi connect failed.");
    ShowStatus("WiFi failed", "Check SSID");
  }
}

void HandleOscMessage(OSCMessage &msg) {
  if (!msg.fullMatch(kOscAddress)) {
    return;
  }

  if (msg.size() < 3 || !msg.isFloat(0) || !msg.isFloat(1) || !msg.isFloat(2)) {
    Serial.println("OSC /rpiboosh/BooshMain ignored (unexpected args).");
    return;
  }

  float v0 = msg.getFloat(0);
  float v1 = msg.getFloat(1);
  float v2 = msg.getFloat(2);

  Serial.print("Received OSC message: ");
  Serial.print(kOscAddress);
  Serial.print(" with arguments: ([");
  Serial.print(v0, 3);
  Serial.print(", ");
  Serial.print(v1, 3);
  Serial.print(", ");
  Serial.print(v2, 3);
  Serial.println("],)");
  ApplyBooshState(v0, v1, v2);
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
  Serial.print("OSC parse error: code=");
  Serial.print(static_cast<int>(error));
  Serial.print(" (");
  Serial.print(OscErrorToString(error));
  Serial.print("), packetSize=");
  Serial.print(packetSize);
  Serial.print(", bytesRead=");
  Serial.print(bytesRead);

  const char *address = msg.getAddress();
  if (address != nullptr && address[0] != '\0') {
    Serial.print(", address=");
    Serial.print(address);
  }

  if (raw != nullptr && rawCount > 0) {
    Serial.print(", raw[");
    Serial.print(rawCount);
    Serial.print("]=");
    for (int i = 0; i < rawCount; ++i) {
      if (i > 0) {
        Serial.print(' ');
      }
      if (raw[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(raw[i], HEX);
    }
  }

  Serial.println();
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

  PollSerialCli();

  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) {
      wasConnected = false;
      pingWasDown = false;
      hasIp = false;
      registry_announced = false;
      registry_next_attempt_ms = 0;
      registry_consecutive_failures = 0;
      Serial.println("WiFi disconnected: registry state reset.");
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
    Serial.println("WiFi connected: scheduling registry announce.");
  }
  HandleRegistry(now);
  if (!wifi_has_ip && wifi_connected_since_ms == 0) {
    wifi_connected_since_ms = now;
  }
  if (boosh_failsafe_armed && now - boosh_failsafe_start_ms >= kBooshFailsafeTimeoutMs) {
    boosh_failsafe_armed = false;
    ApplyBooshState(0.0f, 0.0f, 0.0f);
    Serial.println("Failsafe: BooshMain timeout -> forcing OFF.");
    ShowStatus("Failsafe timeout", "BooshMain OFF");
    boosh_failsafe_note_until_ms = now + kBooshFailsafeNoteMs;
  }
  if (!hasIp && now - lastResolveMs > 10000) {
    lastResolveMs = now;
    Serial.println("Resolving RPIBOOSH...");
    if (WiFi.hostByName(kTargetHost, targetIp) || WiFi.hostByName(kTargetHostMdns, targetIp)) {
      hasIp = true;
      Serial.print("Resolved to ");
      Serial.println(targetIp);
    } else {
      Serial.println("Resolve failed.");
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
    lastOk = Ping.ping(targetIp, 1);
    telemetry_ping_last_ok = lastOk;
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
      Serial.print("Ping ");
      Serial.print(kTargetHost);
      Serial.print(" ");
      Serial.print(lastMs);
      Serial.println("ms");
      if (pingWasDown) {
        pingWasDown = false;
        registry_announced = false;
        registry_next_attempt_ms = now;
        registry_consecutive_failures = 0;
        Serial.println("Ping restored: scheduling registry announce.");
      }
    } else {
      pingWasDown = true;
      Serial.println("Ping failed.");
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
      displayPage = static_cast<uint8_t>((displayPage + 1) % 4);
    }

    if (displayPage == 0) {
      ShowPingStats("RPIBOOSH", stats, lastOk, lastPingSuccessMs, now);
    } else if (displayPage == 1) {
      ShowWifiMetricsPage(0, wifi_connected_since_ms, last_disconnect_reason);
    } else if (displayPage == 2) {
      ShowWifiMetricsPage(1, wifi_connected_since_ms, last_disconnect_reason);
    } else {
      ShowNodeConfigPage();
    }
  }

  delay(5);
}
