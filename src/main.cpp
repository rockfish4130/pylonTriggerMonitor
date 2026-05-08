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
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>

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

// ---- Mesh (ESP-NOW) constants -----------------------------------------------
constexpr uint32_t kMeshMagic            = 0x4D455348UL; // "MESH"
constexpr uint8_t  kMeshVersion          = 3;  // bump when beacon struct changes
constexpr uint8_t  kMeshPktBeacon        = 1;
constexpr uint8_t  kMeshPktCommand       = 2;
constexpr uint8_t  kMeshPktChanChange    = 3;
constexpr uint8_t  kMeshPktPadEvent      = 4;  // remote pad → rpiboosh (all nodes)
constexpr uint8_t  kMeshPktRemoteTelem   = 5;  // remote telemetry → rpiboosh MQTT
constexpr uint32_t kMeshBeaconIntervalMs = 2000;   // broadcast beacon every 2 s
constexpr uint32_t kMeshPeerTimeoutMs    = 8000;   // drop peer after 8 s silence
constexpr uint8_t  kMeshMaxPeers         = 10;
constexpr uint8_t  kMeshQualitySlots     = 16;     // rolling 32 s window (16 × 2 s slots)
constexpr uint8_t  kMeshCmdRetries       = 3;
constexpr uint16_t kMeshCmdRetryMs       = 8;
constexpr uint8_t  kMeshDedupSlots       = 16;
constexpr uint32_t kMeshDedupWindowMs    = 500;

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
float cfg_dj_timeout_s = 10.0f;    // DJ button hold timeout; persisted in NVS; default 10s
bool  dj_btn_held      = false;     // true while DJ button is held down
unsigned long dj_btn_press_ms = 0;  // millis() when DJ button was pressed
// Configurable OSC action parameters (NVS-persisted, apply to all pylons)
unsigned long action_pulse1_dur_ms  = 50;    // pulse-once on duration
bool          action_pulse1_dis     = false; // disable pulse-once action
unsigned long action_pt_dur_ms      = 50;    // pulse-train on duration per pulse
unsigned long action_pt_off_ms      = 50;    // pulse-train off time between pulses
int           action_pt_count       = 5;     // pulse-train number of pulses
bool          action_pt_dis         = false; // disable pulse-train action
unsigned long action_steam_ramp_ms  = 4000;  // steam ramp duration (ms); 1→10 Hz
unsigned long action_steam_open_ms  = 1000;  // steam full-open duration (ms)
bool          action_steam_dis      = false; // disable steam action
int pylon_index = 0;  // sequencing index reported in telemetry; persisted in NVS; default 0
unsigned long barmode_seq_max_ms = 30000;  // btn1 double-tap seq max hold duration; BARMODE NVS; default 30s
unsigned long barmode_seq_dec_ms = 50;     // delay decrement per step in btn1 seq; BARMODE NVS; default 50ms
uint8_t barmode_seq_exp_pct = 100;         // exponential factor % (1-100) applied after linear dec; 100=off
unsigned long barmode_seq_start_ms = 200;  // initial inter-group delay when seq starts; BARMODE NVS; default 200ms
unsigned long barmode_seq_floor_ms = 50;   // minimum inter-group delay floor; BARMODE NVS; default 50ms
unsigned long barmode_green_timeout_ms = 300; // btn0 timed pulse open duration; BARMODE NVS; default 300ms
unsigned long barmode_all4_valve_ms    = 3000; // all-4 hold: valve open duration before auto-close; BARMODE NVS; default 3s
unsigned long barmode_red_seq_max_ms   = 10000; // red hold-seq max duration; BARMODE NVS; default 10s
unsigned long barmode_red_seq_valve_ms = 66;    // red hold-seq valve open time per step; BARMODE NVS; default 66ms
unsigned long barmode_red_seq_step_ms  = 200;   // red hold-seq interval between steps; BARMODE NVS; default 200ms
unsigned long barmode_all4_lockout_s   = 300;  // all-4 lockout countdown duration after sequence; BARMODE NVS; default 5 min
unsigned long barmode_all4_lockout_until_ms = 0; // millis() deadline; 0 = not locked
bool          barmode_show_wait_oled   = false; // true while blue held but lockout active → WAIT screen
unsigned long barmode_green_recovery_ms  = 0;   // recovery period after green tap; BARMODE NVS; default 0
unsigned long barmode_blue_recovery_ms   = 0;   // recovery period after blue tap; BARMODE NVS; default 0
unsigned long barmode_orange_recovery_ms = 0;   // recovery period after orange tap; BARMODE NVS; default 0
unsigned long barmode_red_recovery_ms    = 0;   // recovery period after red steam; BARMODE NVS; default 0
unsigned long barmode_recovery_wait_until_ms = 0; // WAIT screen deadline when tap blocked by recovery
unsigned long green_recovery_until  = 0;  // runtime deadline; 0 = not in recovery
unsigned long blue_recovery_until   = 0;
unsigned long orange_recovery_until = 0;
unsigned long red_recovery_until    = 0;
// Globals mirroring static locals in PollBarModeButtons for telemetry/web UI
int           barmode_seq_phase_g             = 0;   // ALL-4 sequence phase 0-5
int           barmode_red_state_g             = 0;   // red button state 0-4
bool          barmode_orange_strobe_g         = false;
unsigned long barmode_orange_strobe_start_ms_g = 0;
unsigned long barmode_red_steam_start_ms_g    = 0;
// Temperature-based recovery multipliers
float barmode_temp_thresh1_f = 50.0f;   // recovery × mult1 when avg pylon temp drops below this; BARMODE NVS
float barmode_temp_mult1     = 1.0f;    // multiplier 1; default 1.0 (no effect)
float barmode_temp_thresh2_f = 32.0f;   // recovery × mult2 when avg pylon temp drops below this; BARMODE NVS
float barmode_temp_mult2     = 1.0f;    // multiplier 2; default 1.0 (no effect)
volatile float barmode_avg_temp_f      = NAN;  // average pylon temperature from 2-min poll; NAN = no data
volatile float barmode_temp_multiplier = 1.0f; // current effective multiplier (recomputed after each poll)
bool cfg_no_thermistor = false;  // when true, temperature is always reported as null/N/A
bool cfg_no_batt_mon   = false;  // when true, battery V/SOC are always reported as null/N/A
bool cfg_route_via_rpi = false;  // BARMODE: when true, all pylon commands route via rpiboosh APIs
bool     cfg_group_pattern_en = true;  // enable group remote "find-a-friend" fire pattern (NVS)
uint32_t cfg_grp_win_ms      = 2000;  // coincidence detection window (ms)
uint32_t cfg_grp_cool_ms     = 10000; // cooldown between group triggers (ms)
uint16_t cfg_grp_qon_ms      = 66;    // quick-pulse on duration (ms)
uint16_t cfg_grp_qoff_ms     = 66;    // quick-pulse off gap (ms)
uint32_t cfg_grp_big_ms      = 600;   // big-pulse base duration; N-th burst = N × base (ms)
uint16_t cfg_grp_gap_ms      = 300;   // gap between big pulses (ms)
bool    cfg_mesh_en = true;      // enable ESP-NOW mesh
uint8_t cfg_mesh_ch = 1;         // ESP-NOW channel (1-13)
bool   cfg_use_dhcp    = true;   // false = use static IP config below
String cfg_static_ip   = "";     // static IPv4 address (e.g. "192.168.4.100")
String cfg_static_gw   = "";     // default gateway
String cfg_static_dns1 = "";     // primary DNS (defaults to gateway if empty)
String cfg_static_dns2 = "";     // secondary DNS (defaults to 8.8.8.8 if empty)
// Manually-pinned pylons (supplement the rpiboosh registry, NVS-persisted)
constexpr int kManualPylonMax = 8;
struct ManualPylon {
  char      host[64];         // hostname or dotted IP as entered
  int       index;            // pylon_index for sequential ordering
  IPAddress ip;               // resolved IP (valid when resolved==true)
  bool      resolved;         // true once ip is valid
  unsigned long last_resolve_ms; // millis() of last resolution attempt
};
ManualPylon barmode_manual_pylons[kManualPylonMax];
int         barmode_manual_pylon_count = 0;

uint32_t barmode_btn_counts[4]   = {0,0,0,0};   // running press counts: [green, blue, orange, red]
// Per-action counters (since boot): [green_pulse, blue_tap, blue_seq, orange_train, red_tap, red_steam, all4_seq]
uint32_t barmode_act_counts[7]   = {0,0,0,0,0,0,0};
bool     barmode_btn_disabled[4] = {false,false,false,false}; // NVS-persisted disable flags
bool     web_btn_pressed[4]      = {};  // set by /api/barmode/btn; merged into btn_stable
bool     barmode_btn_state[4]    = {};  // stable state snapshot for telemetry
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
volatile bool  barmode_registry_fetch_now = false;  // set to trigger immediate PingTask fetch
volatile float barmode_bpm = 0.0f;  // 0 = no BPM lock; updated by PingTask from llled.local/v0/bpm
// all-4 virtual MIDI hold flag: true while phase-4 valve is open.
// Set by main loop (PollBarModeButtons), read by PingTask which sends press/keepalive/release.
volatile bool barmode_all4_midi_held = false;

// Seen-remote table: written by PingTask (Core 0) when draining mesh_telem_queue;
// read by web handler (Core 1) under mesh_remote_mutex.
constexpr int      kMeshRemoteSlots    = 8;
constexpr uint32_t kMeshRemoteStaleMs  = 90000;   // 3 missed 30s beacons → dim
constexpr uint32_t kMeshRemoteExpireMs = 300000;  // 5 min → drop from API
struct MeshRemoteRecord {
  bool     active;
  char     remote_id[16];
  char     description[32];
  char     mac[18];
  char     forwarded_by[16];
  uint32_t uptime_s;
  uint32_t press_red;
  uint32_t press_yellow;
  uint8_t  mode;
  int8_t   rssi;
  char     hostname[40];
  char     ip[16];
  uint32_t last_seen_ms;
};
static MeshRemoteRecord  mesh_remote_table[kMeshRemoteSlots];
static SemaphoreHandle_t mesh_remote_mutex = nullptr;

// OSC proxy queue: when cfg_route_via_rpi is true, SendOscFloatToIP enqueues messages here
// instead of sending UDP directly. PingTask (Core 0) drains the queue via HTTP POST to rpiboosh.
struct OscProxyMsg {
  char     address[64];
  float    value;
  uint32_t ip_u32;   // IPAddress stored as uint32 for safe cross-core passing
  uint16_t port;
};
constexpr int kOscProxyQueueDepth = 32;
QueueHandle_t osc_proxy_queue = nullptr;

// Per-pylon ping stats (BARMODE only): PingTask pings one known pylon IP per second in rotation.
// Protected by barmode_ping_stats_mutex; read by telemetry handler and OLED renderer.
struct PylonPingStat {
  uint32_t ip_u32;    // target IP as uint32
  uint32_t last_ms;   // last RTT in ms (0 if not yet successful)
  uint32_t min_ms;    // session min RTT
  uint32_t max_ms;    // session max RTT
  uint64_t sum_ms;    // sum for average
  uint32_t count;     // successful pings
  uint32_t lost;      // timed-out pings
  bool     last_ok;   // true if last ping succeeded
  char     id[24];    // pylon_id from registry (truncated)
};
constexpr int kPylonPingStatMax = 16;
PylonPingStat     barmode_pylon_ping_stats[kPylonPingStatMax];
int               barmode_pylon_ping_count = 0;
SemaphoreHandle_t barmode_ping_stats_mutex = nullptr;

String llled_ip_string;             // cached IP for llled.local; resolved once, used as mDNS fallback

// ---- Mesh (ESP-NOW) structs and globals --------------------------------------
struct __attribute__((packed)) MeshBeaconPkt {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;       // kMeshPktBeacon
  char     node_id[16];
  uint8_t  pylon_index;
  uint8_t  role;       // 0=normal, 1=barmode
  uint32_t uptime_s;
  float    batt_v;     // battery voltage V (NaN if unavailable)
  float    batt_pct;   // battery SOC % (NaN if unavailable)
  float    temp_f;     // thermistor temp °F (NaN if unavailable)
  char     fw_ver[32]; // kFirmwareVersion truncated
};

struct __attribute__((packed)) MeshCommandPkt {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;       // kMeshPktCommand
  uint16_t seq;
  char     osc_addr[32];
  float    osc_arg;
};

struct __attribute__((packed)) MeshChanChangePkt {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;      // kMeshPktChanChange
  uint8_t  new_ch;    // target channel 1-13
  uint8_t  _pad;
  uint32_t apply_ms;  // ms from receipt to apply
};

struct __attribute__((packed)) MeshPadEventPkt {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;       // kMeshPktPadEvent
  uint16_t seq;        // sender's sequence number; used for dedup
  uint8_t  note;
  uint8_t  velocity;
  uint8_t  channel;
  char     remote_id[16];  // offset 11
  uint16_t dedup_ms;        // offset 27 — dedup window the receiver should apply
};

struct __attribute__((packed)) MeshRemoteTelemPkt {
  uint32_t magic;          // 0x4D455348
  uint8_t  version;        // 3
  uint8_t  type;           // kMeshPktRemoteTelem
  char     remote_id[16];  // null-terminated remoteID
  char     description[32];// null-terminated description
  uint8_t  mac[6];         // remote's own MAC
  uint32_t uptime_s;
  uint32_t press_red;
  uint32_t press_yellow;
  uint8_t  mode;           // 0 = wifi, 1 = mesh
  int8_t   rssi;           // WiFi RSSI; 0 in mesh mode
  // v2 tail fields — absent in older firmware; check len before reading
  char     hostname[40];   // mDNS hostname without .local, e.g. "remotec3-4fa8"
  char     ip[16];         // last-known WiFi IP as dotted-decimal, or ""
};

struct MeshPeerInfo {
  bool     active;
  uint8_t  mac[6];
  char     node_id[16];
  uint8_t  pylon_index;
  uint8_t  role;
  uint32_t last_seen_ms;
  uint32_t uptime_s;
  uint16_t qual_bits;           // rolling 16-slot bitmap; bit i set = heard beacon in slot i
  uint8_t  qual_pct;            // popcount(qual_bits) * 100 / kMeshQualitySlots
  uint32_t qual_last_update_ms; // millis() when last slot was advanced
  float    batt_v;              // battery voltage V (NaN if unavailable)
  float    batt_pct;            // battery SOC % (NaN if unavailable)
  float    temp_f;              // thermistor temp °F (NaN if unavailable)
  char     fw_ver[32];          // firmware version string
};

struct MeshDedupEntry {
  uint8_t  mac[6];
  uint16_t seq;
  uint32_t expires_ms;
};

MeshPeerInfo      mesh_peers[kMeshMaxPeers];
MeshDedupEntry    mesh_dedup[kMeshDedupSlots];
uint8_t           mesh_dedup_idx    = 0;
SemaphoreHandle_t mesh_peers_mutex  = nullptr;
volatile uint16_t mesh_cmd_seq      = 0;
volatile bool     mesh_initialized  = false;
volatile uint32_t mesh_beacons_sent = 0;
volatile uint32_t mesh_cmds_sent    = 0;
static const uint8_t kMeshBroadcastMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
volatile uint8_t       mesh_ch_pending  = 0;   // 0 = no pending chan change
volatile unsigned long mesh_ch_apply_at = 0;   // millis() when to apply
volatile int           mesh_live_peer_count = 0; // updated by MeshExpirePeers every 2s

// ---- Sequence state ---------------------------------------------------------
enum SeqType { SEQ_NONE, SEQ_PULSE_ONCE, SEQ_PULSE_5X, SEQ_STEAM, SEQ_GROUP };
SeqType active_seq = SEQ_NONE;
unsigned long seq_step_start_ms = 0;  // start of current on/off phase
unsigned long seq_start_ms = 0;       // start of whole sequence
int seq_pulse_idx = 0;                // pulses fired so far
bool seq_phase_on = false;            // currently in on-phase
bool seq_abort_flag = false;          // abort requested

// ---- Group remote "find-a-friend" pattern -----------------------------------
// N remotes pressing yellow simultaneously triggers an N-specific fire sequence.
// Suppresses the normal OSC valve-open for its duration.
struct GroupSeqStep { uint16_t ms; bool on; };
static GroupSeqStep group_seq_steps[32]; // max 4N-1 steps; N≤8 → 31
static int     group_seq_step_count = 0;
static int     group_seq_step_idx   = 0;
static int     group_seq_n_val      = 0; // N used for current/pending sequence
static bool    group_pattern_active = false; // true while SEQ_GROUP is running
volatile uint8_t group_pattern_pending_n = 0; // set by PingTask (Core0); read+cleared by loop (Core1)
volatile bool current_ping_last_ok = false;
volatile unsigned long last_ping_success_ms = 0;
String target_ip_string = "";
String web_log_text;
String web_log_partial_line;
bool ap_enabled = false;
bool ap_active = false;
bool ap_auto_enabled = false;  // true when AP was auto-started due to WiFi failure (not manual)
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
// OSC action config keys
constexpr const char *kPrefsKeyPulse1DurMs = "ps1_dur_ms";   // uint32 ms; pulse-once on duration
constexpr const char *kPrefsKeyPulse1Dis   = "ps1_dis";      // bool; disable pulse-once
constexpr const char *kPrefsKeyPtDurMs     = "pt_dur_ms";    // uint32 ms; pulse-train on duration
constexpr const char *kPrefsKeyPtOffMs     = "pt_off_ms";    // uint32 ms; pulse-train off duration
constexpr const char *kPrefsKeyPtCount     = "pt_count";     // uint8; pulse-train count
constexpr const char *kPrefsKeyPtDis       = "pt_dis";       // bool; disable pulse-train
constexpr const char *kPrefsKeyStmRampMs   = "stm_ramp_ms";  // uint32 ms; steam ramp duration
constexpr const char *kPrefsKeyStmOpenMs   = "stm_open_ms";  // uint32 ms; steam full-open duration
constexpr const char *kPrefsKeyStmDis      = "stm_dis";      // bool; disable steam
constexpr const char *kPrefsKeySeqMaxMs   = "seq_max_ms";
constexpr const char *kPrefsKeySeqDecMs   = "seq_dec_ms";
constexpr const char *kPrefsKeySeqExpPct  = "seq_exp_pct";   // 1-100; applied as factor delay*=(pct/100)
constexpr const char *kPrefsKeySeqStartMs = "seq_start_ms";  // uint32 ms; initial seq inter-group delay
constexpr const char *kPrefsKeySeqFloorMs = "seq_floor_ms";  // uint32 ms; minimum seq delay floor
constexpr const char *kPrefsKeyBtnDisable    = "btn_dis";      // uint8 bitmask: bit0=green,1=blue,2=orange,3=red
constexpr const char *kPrefsKeyGreenTimeout  = "grn_to_ms";   // uint32 ms; btn0 timed pulse duration
constexpr const char *kPrefsKeyAll4ValveMs   = "all4_vlv_ms"; // uint32 ms; all-4 hold valve open duration
constexpr const char *kPrefsKeyRedSeqMaxMs   = "red_seq_max_ms"; // uint32 ms; red hold-seq max duration
constexpr const char *kPrefsKeyRedSeqValveMs = "red_seq_vlv_ms"; // uint32 ms; red hold-seq valve open per step
constexpr const char *kPrefsKeyRedSeqStepMs  = "red_seq_stp_ms"; // uint32 ms; red hold-seq step interval
constexpr const char *kPrefsKeyAll4LockoutS  = "all4_lck_s";  // uint32 s; all-4 lockout countdown duration
constexpr const char *kPrefsKeyManualPylons  = "man_pylons";  // string; "host|index\n" lines
constexpr const char *kPrefsKeyGroupPatEn    = "grp_pat_en";  // bool; enable group remote fire pattern
constexpr const char *kPrefsKeyGrpWinMs     = "grp_win_ms";  // uint32 ms; coincidence window
constexpr const char *kPrefsKeyGrpCoolMs    = "grp_cool_ms"; // uint32 ms; cooldown between triggers
constexpr const char *kPrefsKeyGrpQOnMs     = "grp_qon_ms";  // uint16 ms; quick-pulse on
constexpr const char *kPrefsKeyGrpQOffMs    = "grp_qoff_ms"; // uint16 ms; quick-pulse off
constexpr const char *kPrefsKeyGrpBigMs     = "grp_big_ms";  // uint32 ms; big-pulse base
constexpr const char *kPrefsKeyGrpGapMs     = "grp_gap_ms";  // uint16 ms; big-pulse gap
constexpr const char *kPrefsKeyGreenRecovMs  = "grn_rec_ms";  // uint32 ms; green tap recovery period
constexpr const char *kPrefsKeyBlueRecovMs   = "blu_rec_ms";  // uint32 ms; blue tap recovery period
constexpr const char *kPrefsKeyOrangeRecovMs = "org_rec_ms";  // uint32 ms; orange tap recovery period
constexpr const char *kPrefsKeyRedRecovMs    = "red_rec_ms";  // uint32 ms; red steam recovery period
constexpr const char *kPrefsKeyTempThresh1   = "tmp_thresh1"; // float °F; temperature threshold 1
constexpr const char *kPrefsKeyTempMult1     = "tmp_mult1";   // float; multiplier 1
constexpr const char *kPrefsKeyTempThresh2   = "tmp_thresh2"; // float °F; temperature threshold 2
constexpr const char *kPrefsKeyTempMult2     = "tmp_mult2";   // float; multiplier 2
constexpr const char *kPrefsKeyNoThermistor  = "no_thermistor"; // bool; suppress temp readings
constexpr const char *kPrefsKeyNoBattMon     = "no_batt_mon";   // bool; suppress battery readings
constexpr const char *kPrefsKeyRouteViaRpi   = "route_via_rpi"; // bool; BARMODE: route all pylon cmds via rpiboosh
constexpr const char *kPrefsKeyUseDhcp       = "use_dhcp";      // bool; true=DHCP (default), false=static
constexpr const char *kPrefsKeyStaticIp      = "static_ip";     // string; static IPv4 address
constexpr const char *kPrefsKeyStaticGw      = "static_gw";     // string; static default gateway
constexpr const char *kPrefsKeyStaticDns1    = "static_dns1";   // string; primary DNS
constexpr const char *kPrefsKeyStaticDns2    = "static_dns2";   // string; secondary DNS
constexpr const char *kPrefsKeyMeshEn        = "mesh_en";        // bool; enable ESP-NOW mesh
constexpr const char *kPrefsKeyMeshCh        = "mesh_ch";        // uint8; ESP-NOW channel (1-13)
constexpr const char *kPrefsKeyDjTimeoutS    = "dj_to_s";        // float; DJ button timeout (s)
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

// Draw the mesh status indicators right-aligned in the top-right corner.
// Display is 128×32. M:N badge sits at row `by` in `text_size`; Ch:N sits
// at `by + text_size*8` in size=1. Normal callers use DrawMeshBadge(0,1):
//   y=0..7  M:N  (size=1, 8px)
//   y=8..15 Ch:N (size=1, 8px)
// This leaves rows 3-4 (y=16, y=24) fully clear of badge pixels.
// The mesh page uses DrawMeshBadge(0,2) for a larger M:N badge.
// Ch:N shows the actual HW channel; shows "C<hw>*<cfg>" when mismatched.
// No-op if mesh is disabled. Caller must call display.display() afterward.
static void DrawMeshBadge(int by, uint8_t text_size = 2) {
  if (!cfg_mesh_en) return;
  int peer_count = 0;
  if (mesh_peers_mutex && xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (int i = 0; i < kMeshMaxPeers; i++) if (mesh_peers[i].active) peer_count++;
    xSemaphoreGive(mesh_peers_mutex);
  }

  // M:N badge — right-aligned at caller-specified y, caller-specified size
  char buf[8];
  snprintf(buf, sizeof(buf), "M:%d", peer_count);
  const int charW = (text_size == 1) ? 6 : 12;
  const int w = static_cast<int>(strlen(buf)) * charW;
  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(128 - w, by);
  display.print(buf);

  // Ch:N — right-aligned just below the M:N badge (by + text_size*8), size=1.
  uint8_t hw_ch = 0;
  wifi_second_chan_t hw_sec = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&hw_ch, &hw_sec);
  char ch_buf[10];
  if (hw_ch != (uint8_t)cfg_mesh_ch) {
    snprintf(ch_buf, sizeof(ch_buf), "C%u*%u", hw_ch, (uint8_t)cfg_mesh_ch);
  } else {
    snprintf(ch_buf, sizeof(ch_buf), "Ch:%u", hw_ch);
  }
  const int ch_w = static_cast<int>(strlen(ch_buf)) * 6;
  display.setTextSize(1);
  display.setCursor(128 - ch_w, by + text_size * 8);
  display.print(ch_buf);
}

void RenderDisplayPage(const DisplayPageLines &page) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(page.line1);
  display.println(page.line2);
  display.println(page.line3);
  display.println(page.line4);
  // M:N at y=0 (size=2, 16px), Ch:N at y=16 — top-right corner.
  DrawMeshBadge(0, 2);
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
  prefs.putUInt(kPrefsKeyPulse1DurMs, static_cast<uint32_t>(action_pulse1_dur_ms));
  prefs.putBool(kPrefsKeyPulse1Dis,   action_pulse1_dis);
  prefs.putUInt(kPrefsKeyPtDurMs,     static_cast<uint32_t>(action_pt_dur_ms));
  prefs.putUInt(kPrefsKeyPtOffMs,     static_cast<uint32_t>(action_pt_off_ms));
  prefs.putUChar(kPrefsKeyPtCount,    static_cast<uint8_t>(action_pt_count));
  prefs.putBool(kPrefsKeyPtDis,       action_pt_dis);
  prefs.putUInt(kPrefsKeyStmRampMs,   static_cast<uint32_t>(action_steam_ramp_ms));
  prefs.putUInt(kPrefsKeyStmOpenMs,   static_cast<uint32_t>(action_steam_open_ms));
  prefs.putBool(kPrefsKeyStmDis,      action_steam_dis);
  prefs.putUInt(kPrefsKeySeqMaxMs,   static_cast<uint32_t>(barmode_seq_max_ms));
  prefs.putUInt(kPrefsKeySeqDecMs,   static_cast<uint32_t>(barmode_seq_dec_ms));
  prefs.putUChar(kPrefsKeySeqExpPct, barmode_seq_exp_pct);
  prefs.putUInt(kPrefsKeySeqStartMs, static_cast<uint32_t>(barmode_seq_start_ms));
  prefs.putUInt(kPrefsKeySeqFloorMs, static_cast<uint32_t>(barmode_seq_floor_ms));
  prefs.putUInt(kPrefsKeyGreenTimeout, static_cast<uint32_t>(barmode_green_timeout_ms));
  prefs.putUInt(kPrefsKeyAll4ValveMs,  static_cast<uint32_t>(barmode_all4_valve_ms));
  prefs.putUInt(kPrefsKeyRedSeqMaxMs,   static_cast<uint32_t>(barmode_red_seq_max_ms));
  prefs.putUInt(kPrefsKeyRedSeqValveMs, static_cast<uint32_t>(barmode_red_seq_valve_ms));
  prefs.putUInt(kPrefsKeyRedSeqStepMs,  static_cast<uint32_t>(barmode_red_seq_step_ms));
  prefs.putUInt(kPrefsKeyAll4LockoutS, static_cast<uint32_t>(barmode_all4_lockout_s));
  prefs.putUInt(kPrefsKeyGreenRecovMs,  static_cast<uint32_t>(barmode_green_recovery_ms));
  prefs.putUInt(kPrefsKeyBlueRecovMs,   static_cast<uint32_t>(barmode_blue_recovery_ms));
  prefs.putUInt(kPrefsKeyOrangeRecovMs, static_cast<uint32_t>(barmode_orange_recovery_ms));
  prefs.putUInt(kPrefsKeyRedRecovMs,    static_cast<uint32_t>(barmode_red_recovery_ms));
  prefs.putFloat(kPrefsKeyTempThresh1,  barmode_temp_thresh1_f);
  prefs.putFloat(kPrefsKeyTempMult1,    barmode_temp_mult1);
  prefs.putFloat(kPrefsKeyTempThresh2,  barmode_temp_thresh2_f);
  prefs.putFloat(kPrefsKeyTempMult2,    barmode_temp_mult2);
  prefs.putBool(kPrefsKeyNoThermistor,  cfg_no_thermistor);
  prefs.putBool(kPrefsKeyNoBattMon,     cfg_no_batt_mon);
  prefs.putBool(kPrefsKeyRouteViaRpi,  cfg_route_via_rpi);
  prefs.putBool(kPrefsKeyGroupPatEn,   cfg_group_pattern_en);
  prefs.putUInt(kPrefsKeyGrpWinMs,    cfg_grp_win_ms);
  prefs.putUInt(kPrefsKeyGrpCoolMs,   cfg_grp_cool_ms);
  prefs.putUShort(kPrefsKeyGrpQOnMs,  cfg_grp_qon_ms);
  prefs.putUShort(kPrefsKeyGrpQOffMs, cfg_grp_qoff_ms);
  prefs.putUInt(kPrefsKeyGrpBigMs,    cfg_grp_big_ms);
  prefs.putUShort(kPrefsKeyGrpGapMs,  cfg_grp_gap_ms);
  prefs.putBool(kPrefsKeyUseDhcp,       cfg_use_dhcp);
  prefs.putString(kPrefsKeyStaticIp,    cfg_static_ip);
  prefs.putString(kPrefsKeyStaticGw,    cfg_static_gw);
  prefs.putString(kPrefsKeyStaticDns1,  cfg_static_dns1);
  prefs.putString(kPrefsKeyStaticDns2,  cfg_static_dns2);
  prefs.putBool(kPrefsKeyMeshEn,        cfg_mesh_en);
  prefs.putUChar(kPrefsKeyMeshCh,       cfg_mesh_ch);
  prefs.putFloat(kPrefsKeyDjTimeoutS,   cfg_dj_timeout_s);
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
  Console.print("  use_dhcp: ");
  Console.println(cfg_use_dhcp ? "true" : "false");
  if (!cfg_use_dhcp) {
    Console.print("  static_ip:   "); Console.println(cfg_static_ip.length()   > 0 ? cfg_static_ip   : "(not set)");
    Console.print("  static_gw:   "); Console.println(cfg_static_gw.length()   > 0 ? cfg_static_gw   : "(not set)");
    Console.print("  static_dns1: "); Console.println(cfg_static_dns1.length() > 0 ? cfg_static_dns1 : "(default: gw)");
    Console.print("  static_dns2: "); Console.println(cfg_static_dns2.length() > 0 ? cfg_static_dns2 : "(default: 8.8.8.8)");
  }
  Console.print("  effective IP: ");
  Console.println(WiFi.localIP().toString());
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
  action_pulse1_dur_ms = prefs.getUInt(kPrefsKeyPulse1DurMs, 50);
  action_pulse1_dis    = prefs.getBool(kPrefsKeyPulse1Dis,   false);
  action_pt_dur_ms     = prefs.getUInt(kPrefsKeyPtDurMs,     50);
  action_pt_off_ms     = prefs.getUInt(kPrefsKeyPtOffMs,     50);
  action_pt_count      = (int)prefs.getUChar(kPrefsKeyPtCount, 5);
  action_pt_dis        = prefs.getBool(kPrefsKeyPtDis,       false);
  action_steam_ramp_ms = prefs.getUInt(kPrefsKeyStmRampMs,   4000);
  action_steam_open_ms = prefs.getUInt(kPrefsKeyStmOpenMs,   1000);
  action_steam_dis     = prefs.getBool(kPrefsKeyStmDis,      false);
  barmode_seq_max_ms   = prefs.getUInt(kPrefsKeySeqMaxMs,   30000);
  barmode_seq_dec_ms   = prefs.getUInt(kPrefsKeySeqDecMs,   50);
  barmode_seq_exp_pct  = (uint8_t)prefs.getUChar(kPrefsKeySeqExpPct, 100);
  barmode_seq_start_ms = prefs.getUInt(kPrefsKeySeqStartMs, 200);
  barmode_seq_floor_ms = prefs.getUInt(kPrefsKeySeqFloorMs, 50);
  barmode_green_timeout_ms = prefs.getUInt(kPrefsKeyGreenTimeout, 300);
  barmode_all4_valve_ms    = prefs.getUInt(kPrefsKeyAll4ValveMs,  3000);
  barmode_red_seq_max_ms   = prefs.getUInt(kPrefsKeyRedSeqMaxMs,   10000);
  barmode_red_seq_valve_ms = prefs.getUInt(kPrefsKeyRedSeqValveMs, 66);
  barmode_red_seq_step_ms  = prefs.getUInt(kPrefsKeyRedSeqStepMs,  200);
  barmode_all4_lockout_s   = prefs.getUInt(kPrefsKeyAll4LockoutS, 300);
  barmode_green_recovery_ms  = prefs.getUInt(kPrefsKeyGreenRecovMs,  0);
  barmode_blue_recovery_ms   = prefs.getUInt(kPrefsKeyBlueRecovMs,   0);
  barmode_orange_recovery_ms = prefs.getUInt(kPrefsKeyOrangeRecovMs, 0);
  barmode_red_recovery_ms    = prefs.getUInt(kPrefsKeyRedRecovMs,    0);
  barmode_temp_thresh1_f     = prefs.getFloat(kPrefsKeyTempThresh1,  50.0f);
  barmode_temp_mult1         = prefs.getFloat(kPrefsKeyTempMult1,    1.0f);
  barmode_temp_thresh2_f     = prefs.getFloat(kPrefsKeyTempThresh2,  32.0f);
  barmode_temp_mult2         = prefs.getFloat(kPrefsKeyTempMult2,    1.0f);
  cfg_no_thermistor          = prefs.getBool(kPrefsKeyNoThermistor,  false);
  cfg_no_batt_mon            = prefs.getBool(kPrefsKeyNoBattMon,     false);
  cfg_route_via_rpi          = prefs.getBool(kPrefsKeyRouteViaRpi,  false);
  cfg_group_pattern_en       = prefs.getBool(kPrefsKeyGroupPatEn,   true);
  cfg_grp_win_ms             = prefs.getUInt(kPrefsKeyGrpWinMs,     2000);
  cfg_grp_cool_ms            = prefs.getUInt(kPrefsKeyGrpCoolMs,    10000);
  cfg_grp_qon_ms             = prefs.getUShort(kPrefsKeyGrpQOnMs,   66);
  cfg_grp_qoff_ms            = prefs.getUShort(kPrefsKeyGrpQOffMs,  66);
  cfg_grp_big_ms             = prefs.getUInt(kPrefsKeyGrpBigMs,     600);
  cfg_grp_gap_ms             = prefs.getUShort(kPrefsKeyGrpGapMs,   300);
  cfg_use_dhcp               = prefs.getBool(kPrefsKeyUseDhcp,       true);
  cfg_static_ip              = prefs.getString(kPrefsKeyStaticIp,    "");
  cfg_static_gw              = prefs.getString(kPrefsKeyStaticGw,    "");
  cfg_static_dns1            = prefs.getString(kPrefsKeyStaticDns1,  "");
  cfg_static_dns2            = prefs.getString(kPrefsKeyStaticDns2,  "");
  cfg_mesh_en                = prefs.getBool(kPrefsKeyMeshEn,        true);
  cfg_mesh_ch                = (uint8_t)prefs.getUChar(kPrefsKeyMeshCh, 1);
  cfg_dj_timeout_s           = prefs.getFloat(kPrefsKeyDjTimeoutS, 10.0f);
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

void SaveManualPylons() {
  String blob;
  for (int i = 0; i < barmode_manual_pylon_count; i++) {
    blob += barmode_manual_pylons[i].host;
    blob += '|';
    blob += barmode_manual_pylons[i].index;
    blob += '\n';
  }
  Preferences prefs;
  prefs.begin(kPrefsNamespace, false);
  prefs.putString(kPrefsKeyManualPylons, blob);
  prefs.end();
}

void LoadManualPylons() {
  Preferences prefs;
  prefs.begin(kPrefsNamespace, true);
  String blob = prefs.getString(kPrefsKeyManualPylons, "");
  prefs.end();
  barmode_manual_pylon_count = 0;
  int start = 0;
  while (start < (int)blob.length() && barmode_manual_pylon_count < kManualPylonMax) {
    int nl = blob.indexOf('\n', start);
    if (nl < 0) nl = blob.length();
    String line = blob.substring(start, nl);
    start = nl + 1;
    int sep = line.lastIndexOf('|');
    if (sep < 1) continue;
    String host = line.substring(0, sep);
    int idx = line.substring(sep + 1).toInt();
    host.trim();
    if (host.length() == 0 || host.length() >= 64) continue;
    ManualPylon &mp = barmode_manual_pylons[barmode_manual_pylon_count++];
    strncpy(mp.host, host.c_str(), 63);
    mp.host[63] = '\0';
    mp.index = idx;
    mp.resolved = false;
    mp.last_resolve_ms = 0;
    // Attempt immediate parse for dotted-decimal IPs
    IPAddress addr;
    if (addr.fromString(host)) { mp.ip = addr; mp.resolved = true; }
  }
  Console.printf("[Manual] Loaded %d manual pylons\n", barmode_manual_pylon_count);
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
  Console.println("  reboot             (restart the device)");
  Console.println("  set use_dhcp true|false      (true=DHCP (default), false=static IP)");
  Console.println("  set static_ip <x.x.x.x>     (static IPv4 address)");
  Console.println("  set static_gw <x.x.x.x>     (default gateway)");
  Console.println("  set static_dns1 <x.x.x.x>   (primary DNS; default: gateway)");
  Console.println("  set static_dns2 <x.x.x.x>   (secondary DNS; default: 8.8.8.8)");
  Console.println("  (IP settings take effect on next reboot)");
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
  } else if (field == "seq_start_ms") {
    const int ms = (int)value.toInt();
    if (ms < 50 || ms > 10000) {
      if (log_output) Console.println("[CFG] seq_start_ms out of range (50-10000ms)");
      return false;
    }
    barmode_seq_start_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] seq_start_ms set: %lu\n", barmode_seq_start_ms);
  } else if (field == "seq_floor_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 2000) {
      if (log_output) Console.println("[CFG] seq_floor_ms out of range (10-2000ms)");
      return false;
    }
    barmode_seq_floor_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] seq_floor_ms set: %lu\n", barmode_seq_floor_ms);
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
  } else if (field == "red_seq_max_s") {
    const int s = (int)value.toInt();
    if (s < 1 || s > 120) { if (log_output) Console.println("[CFG] red_seq_max_s out of range (1-120s)"); return false; }
    barmode_red_seq_max_ms = (unsigned long)s * 1000UL;
    changed = true;
    if (log_output) Console.printf("[CFG] red_seq_max_ms set: %lu\n", barmode_red_seq_max_ms);
  } else if (field == "red_seq_valve_ms") {
    const int ms = (int)value.toInt();
    if (ms < 20 || ms > 2000) { if (log_output) Console.println("[CFG] red_seq_valve_ms out of range (20-2000ms)"); return false; }
    barmode_red_seq_valve_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] red_seq_valve_ms set: %lu\n", barmode_red_seq_valve_ms);
  } else if (field == "red_seq_step_ms") {
    const int ms = (int)value.toInt();
    if (ms < 0 || ms > 5000) { if (log_output) Console.println("[CFG] red_seq_step_ms out of range (0-5000ms)"); return false; }
    barmode_red_seq_step_ms = (unsigned long)(ms < 10 ? 10 : ms);  // floor at 10ms to prevent runaway
    changed = true;
    if (log_output) Console.printf("[CFG] red_seq_step_ms set: %lu\n", barmode_red_seq_step_ms);
  } else if (field == "all4_lockout_s") {
    const int s = (int)value.toInt();
    if (s < 0 || s > 3600) {
      if (log_output) Console.println("[CFG] all4_lockout_s out of range (0-3600s)");
      return false;
    }
    barmode_all4_lockout_s = (unsigned long)s;
    changed = true;
    if (log_output) Console.printf("[CFG] all4_lockout_s set: %lu\n", barmode_all4_lockout_s);
  } else if (field == "green_recovery_ms") {
    const long ms = value.toInt();
    if (ms < 0 || ms > 300000) { if (log_output) Console.println("[CFG] green_recovery_ms out of range (0-300000)"); return false; }
    barmode_green_recovery_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] green_recovery_ms set: %lu\n", barmode_green_recovery_ms);
  } else if (field == "blue_recovery_ms") {
    const long ms = value.toInt();
    if (ms < 0 || ms > 300000) { if (log_output) Console.println("[CFG] blue_recovery_ms out of range (0-300000)"); return false; }
    barmode_blue_recovery_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] blue_recovery_ms set: %lu\n", barmode_blue_recovery_ms);
  } else if (field == "orange_recovery_ms") {
    const long ms = value.toInt();
    if (ms < 0 || ms > 300000) { if (log_output) Console.println("[CFG] orange_recovery_ms out of range (0-300000)"); return false; }
    barmode_orange_recovery_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] orange_recovery_ms set: %lu\n", barmode_orange_recovery_ms);
  } else if (field == "red_recovery_ms") {
    const long ms = value.toInt();
    if (ms < 0 || ms > 300000) { if (log_output) Console.println("[CFG] red_recovery_ms out of range (0-300000)"); return false; }
    barmode_red_recovery_ms = (unsigned long)ms;
    changed = true;
    if (log_output) Console.printf("[CFG] red_recovery_ms set: %lu\n", barmode_red_recovery_ms);
  } else if (field == "temp_thresh1_f") {
    barmode_temp_thresh1_f = value.toFloat();
    changed = true;
    if (log_output) Console.printf("[CFG] temp_thresh1_f set: %.1f\n", barmode_temp_thresh1_f);
  } else if (field == "temp_mult1") {
    const float m = value.toFloat();
    if (m < 0.1f || m > 100.0f) { if (log_output) Console.println("[CFG] temp_mult1 out of range (0.1-100)"); return false; }
    barmode_temp_mult1 = m;
    changed = true;
    if (log_output) Console.printf("[CFG] temp_mult1 set: %.2f\n", barmode_temp_mult1);
  } else if (field == "temp_thresh2_f") {
    barmode_temp_thresh2_f = value.toFloat();
    changed = true;
    if (log_output) Console.printf("[CFG] temp_thresh2_f set: %.1f\n", barmode_temp_thresh2_f);
  } else if (field == "temp_mult2") {
    const float m = value.toFloat();
    if (m < 0.1f || m > 100.0f) { if (log_output) Console.println("[CFG] temp_mult2 out of range (0.1-100)"); return false; }
    barmode_temp_mult2 = m;
    changed = true;
    if (log_output) Console.printf("[CFG] temp_mult2 set: %.2f\n", barmode_temp_mult2);
  } else if (field == "use_dhcp") {
    const String v = ToLowerAscii(value);
    cfg_use_dhcp = (v == "true" || v == "1" || v == "yes" || v == "on");
    changed = true;
    if (log_output) Console.printf("[CFG] use_dhcp set: %s (takes effect on next reboot)\n", cfg_use_dhcp ? "true" : "false");
  } else if (field == "static_ip" || field == "static_gw" || field == "static_dns1" || field == "static_dns2") {
    IPAddress addr;
    if (!addr.fromString(value)) {
      if (log_output) Console.printf("[CFG] %s: invalid IPv4 address\n", field.c_str());
      return false;
    }
    if (field == "static_ip")   { cfg_static_ip   = value; changed = true; }
    else if (field == "static_gw")   { cfg_static_gw   = value; changed = true; }
    else if (field == "static_dns1") { cfg_static_dns1 = value; changed = true; }
    else if (field == "static_dns2") { cfg_static_dns2 = value; changed = true; }
    if (log_output) Console.printf("[CFG] %s set: %s (takes effect on next reboot)\n", field.c_str(), value.c_str());
  } else if (field == "no_thermistor") {
    cfg_no_thermistor = (value == "1" || value == "true");
    changed = true;
    if (log_output) Console.printf("[CFG] no_thermistor set: %s\n", cfg_no_thermistor ? "true" : "false");
  } else if (field == "no_batt_mon") {
    cfg_no_batt_mon = (value == "1" || value == "true");
    changed = true;
    if (log_output) Console.printf("[CFG] no_batt_mon set: %s\n", cfg_no_batt_mon ? "true" : "false");
  } else if (field == "route_via_rpi") {
    cfg_route_via_rpi = (value == "1" || value == "true");
    changed = true;
    if (log_output) Console.printf("[CFG] route_via_rpi set: %s\n", cfg_route_via_rpi ? "true" : "false");
  } else if (field == "grp_pat_en") {
    cfg_group_pattern_en = (value == "1" || value == "true");
    changed = true;
    if (log_output) Console.printf("[CFG] grp_pat_en set: %s\n", cfg_group_pattern_en ? "true" : "false");
  } else if (field == "grp_win_ms") {
    const int ms = (int)value.toInt();
    if (ms < 500 || ms > 10000) { if (log_output) Console.println("[CFG] grp_win_ms out of range (500-10000)"); return false; }
    cfg_grp_win_ms = (uint32_t)ms; changed = true;
  } else if (field == "grp_cool_ms") {
    const int ms = (int)value.toInt();
    if (ms < 1000 || ms > 120000) { if (log_output) Console.println("[CFG] grp_cool_ms out of range (1000-120000)"); return false; }
    cfg_grp_cool_ms = (uint32_t)ms; changed = true;
  } else if (field == "grp_qon_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 500) { if (log_output) Console.println("[CFG] grp_qon_ms out of range (10-500)"); return false; }
    cfg_grp_qon_ms = (uint16_t)ms; changed = true;
  } else if (field == "grp_qoff_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 500) { if (log_output) Console.println("[CFG] grp_qoff_ms out of range (10-500)"); return false; }
    cfg_grp_qoff_ms = (uint16_t)ms; changed = true;
  } else if (field == "grp_big_ms") {
    const int ms = (int)value.toInt();
    if (ms < 100 || ms > 5000) { if (log_output) Console.println("[CFG] grp_big_ms out of range (100-5000)"); return false; }
    cfg_grp_big_ms = (uint32_t)ms; changed = true;
  } else if (field == "grp_gap_ms") {
    const int ms = (int)value.toInt();
    if (ms < 0 || ms > 2000) { if (log_output) Console.println("[CFG] grp_gap_ms out of range (0-2000)"); return false; }
    cfg_grp_gap_ms = (uint16_t)ms; changed = true;
  } else if (field == "pulse1_dur_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 5000) { if (log_output) Console.println("[CFG] pulse1_dur_ms out of range (10-5000)"); return false; }
    action_pulse1_dur_ms = (unsigned long)ms;
    changed = true;
  } else if (field == "pulse1_dis") {
    action_pulse1_dis = (value == "1" || value == "true");
    changed = true;
  } else if (field == "pt_dur_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 5000) { if (log_output) Console.println("[CFG] pt_dur_ms out of range (10-5000)"); return false; }
    action_pt_dur_ms = (unsigned long)ms;
    changed = true;
  } else if (field == "pt_off_ms") {
    const int ms = (int)value.toInt();
    if (ms < 10 || ms > 5000) { if (log_output) Console.println("[CFG] pt_off_ms out of range (10-5000)"); return false; }
    action_pt_off_ms = (unsigned long)ms;
    changed = true;
  } else if (field == "pt_count") {
    const int n = (int)value.toInt();
    if (n < 1 || n > 20) { if (log_output) Console.println("[CFG] pt_count out of range (1-20)"); return false; }
    action_pt_count = n;
    changed = true;
  } else if (field == "pt_dis") {
    action_pt_dis = (value == "1" || value == "true");
    changed = true;
  } else if (field == "steam_ramp_ms") {
    const int ms = (int)value.toInt();
    if (ms < 100 || ms > 30000) { if (log_output) Console.println("[CFG] steam_ramp_ms out of range (100-30000)"); return false; }
    action_steam_ramp_ms = (unsigned long)ms;
    changed = true;
  } else if (field == "steam_open_ms") {
    const int ms = (int)value.toInt();
    if (ms < 0 || ms > 30000) { if (log_output) Console.println("[CFG] steam_open_ms out of range (0-30000)"); return false; }
    action_steam_open_ms = (unsigned long)ms;
    changed = true;
  } else if (field == "steam_dis") {
    action_steam_dis = (value == "1" || value == "true");
    changed = true;
  } else if (field == "mesh_en") {
    cfg_mesh_en = (value == "1" || value == "true");
    changed = true;
    if (log_output) Console.printf("[CFG] mesh_en set: %s (reboot to apply)\n", cfg_mesh_en ? "true" : "false");
  } else if (field == "mesh_ch") {
    const int ch = (int)value.toInt();
    if (ch < 1 || ch > 13) { if (log_output) Console.println("[CFG] mesh_ch out of range (1-13)"); return false; }
    cfg_mesh_ch = (uint8_t)ch;
    changed = true;
    if (log_output) Console.printf("[CFG] mesh_ch set: %u (reboot to apply)\n", cfg_mesh_ch);
  } else if (field == "dj_timeout_s") {
    const float s = value.toFloat();
    if (s < 1.0f || s > 120.0f) { if (log_output) Console.println("[CFG] dj_timeout_s out of range (1-120s)"); return false; }
    cfg_dj_timeout_s = s;
    changed = true;
    if (log_output) Console.printf("[CFG] dj_timeout_s set: %.1f\n", cfg_dj_timeout_s);
  } else {
    if (log_output) {
      Console.println("[CFG] unknown set field. use id|host|desc|node|ap|failsafe_s|index|seq_max_s|seq_dec_ms|seq_exp_pct|pulse1_dur_ms|pulse1_dis|pt_dur_ms|pt_off_ms|pt_count|pt_dis|steam_ramp_s|steam_open_s|steam_dis|mesh_en|mesh_ch");
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
  payload += "\"dj_timeout_s\":" + String(cfg_dj_timeout_s, 1) + ",";
  payload += "\"pylon_index\":" + String(pylon_index) + ",";
  payload += "\"seq_max_ms\":"   + String(barmode_seq_max_ms)   + ",";
  payload += "\"seq_dec_ms\":"   + String(barmode_seq_dec_ms)   + ",";
  payload += "\"seq_exp_pct\":"  + String(barmode_seq_exp_pct)  + ",";
  payload += "\"seq_start_ms\":" + String(barmode_seq_start_ms) + ",";
  payload += "\"seq_floor_ms\":" + String(barmode_seq_floor_ms);
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
  if (line.equalsIgnoreCase("reboot")) {
    Console.println("[CLI] rebooting...");
    delay(100);
    ESP.restart();
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
    const float eff_temp_f = cfg_no_thermistor ? NAN : sensor_temp_f;
    const float eff_batt_v = cfg_no_batt_mon   ? NAN : sensor_battery_v;
    const float eff_batt_pct = cfg_no_batt_mon ? NAN : sensor_battery_pct;
    const float eff_batt_h = cfg_no_batt_mon   ? NAN : sensor_battery_time_remaining_h;
    payload += "\"temperature\":" + fmtOrNull(eff_temp_f) + ",";
    payload += "\"temperature_f\":" + fmtOrNull(eff_temp_f) + ",";
    const float tempC = isfinite(eff_temp_f) ? (eff_temp_f - 32.0f) * 5.0f / 9.0f : NAN;
    payload += "\"temperature_c\":" + fmtOrNull(tempC) + ",";
    payload += "\"battery_voltage\":" + fmtOrNull(eff_batt_v) + ",";
    payload += "\"battery_voltage_v\":" + fmtOrNull(eff_batt_v) + ",";
    payload += "\"battery_charge\":" + fmtOrNull(eff_batt_pct) + ",";
    payload += "\"battery_charge_pct\":" + fmtOrNull(eff_batt_pct) + ",";
    payload += "\"battery_time_remaining_h\":" + fmtOrNull(eff_batt_h) + ",";
    payload += "\"no_thermistor\":" + String(cfg_no_thermistor ? "true" : "false") + ",";
    payload += "\"no_batt_mon\":" + String(cfg_no_batt_mon ? "true" : "false") + ",";
    payload += "\"route_via_rpi\":" + String(cfg_route_via_rpi ? "true" : "false") + ",";
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
  page.line1 = TrimForDisplay(host + (timing_out ? " TIMEOUT" : (last_ok ? " OK" : " FAIL")), 15);

  if (stats.has_data()) {
    page.line2 = TrimForDisplay("last " + String(stats.last_ms) + "ms", 15);
  } else {
    page.line2 = "last --";
  }

  if (has_success) {
    page.line3 = TrimForDisplay("since " + FormatDurationHms(static_cast<uint32_t>(since_success_ms / 1000)), 15);
  } else {
    page.line3 = "since --:--:--";  // exactly 14 chars, fits
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
    lines.line1 = "WiFi " + TrimForDisplay(WiFi.SSID(), 10);          // "WiFi " + 10 = 15 max
    lines.line2 = TrimForDisplay("RSSI " + String(WiFi.RSSI()) + "dBm", 15);
    lines.line3 = TrimForDisplay("IP " + WiFi.localIP().toString(), 15);
    if (connected_since_ms > 0) {
      lines.line4 = "UP " + FormatDurationHms(static_cast<uint32_t>((millis() - connected_since_ms) / 1000));
    } else {
      lines.line4 = "UP --:--:--";
    }
  } else {
    lines.line1 = TrimForDisplay("RSN " + String(disconnect_reason) + " " + WifiDisconnectReasonToString(disconnect_reason), 15);
    lines.line2 = "SSID " + TrimForDisplay(WiFi.SSID(), 10);          // "SSID " + 10 = 15 max
    lines.line3 = TrimForDisplay("RSSI " + String(WiFi.RSSI()) + "dBm", 15);
    lines.line4 = "IP " + WiFi.localIP().toString();
  }
  return lines;
}

void ShowWifiMetricsPage(uint8_t page, unsigned long connected_since_ms, uint8_t last_disconnect_reason) {
  RenderDisplayPage(BuildWifiMetricsPageLines(page, connected_since_ms, last_disconnect_reason));
}

DisplayPageLines BuildNodeConfigPageLines() {
  DisplayPageLines page;
  page.line1 = "NODE CONFIG";                                  // 11 chars, fits
  page.line2 = "ID: " + TrimForDisplay(pylon_id, 11);        // "ID: " + 11 = 15 max
  page.line3 = TrimForDisplay("TRIG: " + String(trigger_event_count), 15);
  page.line4 = "FW " + String(kFirmwareSemver);
  return page;
}

void ShowNodeConfigPage() {
  RenderDisplayPage(BuildNodeConfigPageLines());
}

DisplayPageLines BuildFirmwarePageLines() {
  DisplayPageLines page;
  page.line1 = "FIRMWARE";                                              // 8 chars, fits
  page.line2 = TrimForDisplay("VER " + String(kFirmwareSemver), 15);
  page.line3 = TrimForDisplay(String(kFirmwareBuildDate), 15);          // "2026-05-06" = 10, fits
  page.line4 = String(kFirmwareBuildTime);                              // line4 free
  return page;
}

void ShowFirmwarePage() {
  RenderDisplayPage(BuildFirmwarePageLines());
}

// BARMODE only: "PYLON PING" page — header + overall avg + up to 2 lines of issues.
void ShowPylonPingPage() {
  DisplayPageLines page;
  page.line1 = "PYLON PING";
  if (!barmode_ping_stats_mutex) {
    page.line2 = "no data";
    RenderDisplayPage(page);
    return;
  }
  // Snapshot stats under mutex
  uint64_t total_sum = 0;
  uint32_t total_count = 0;
  int      lost_count  = 0;
  char     issue_ids[2][24] = {{0},{0}};
  int      issue_n = 0;
  if (xSemaphoreTake(barmode_ping_stats_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    for (int i = 0; i < barmode_pylon_ping_count; i++) {
      const PylonPingStat &s = barmode_pylon_ping_stats[i];
      total_sum   += s.sum_ms;
      total_count += s.count;
      if (!s.last_ok || s.lost > 0) {
        lost_count++;
        if (issue_n < 2) strncpy(issue_ids[issue_n++], s.id, 23);
      }
    }
    xSemaphoreGive(barmode_ping_stats_mutex);
  }
  if (total_count == 0 && barmode_pylon_ping_count == 0) {
    page.line2 = "waiting...";
  } else {
    const uint32_t overall_avg = total_count ? static_cast<uint32_t>(total_sum / total_count) : 0;
    const int n = barmode_pylon_ping_count;
    page.line2 = TrimForDisplay("avg " + String(overall_avg) + "ms N=" + String(n) +
                 (lost_count ? " " + String(lost_count) + "x!" : " ok"), 15);
  }
  page.line3 = TrimForDisplay((issue_n > 0) ? (String("!") + issue_ids[0] + " timeout") : "", 15);
  page.line4 = (issue_n > 1) ? (String("!") + issue_ids[1] + " timeout") : "";  // line4 free
  RenderDisplayPage(page);
}

DisplayPageLines BuildSensorStatusPageLines() {
  DisplayPageLines page;
  page.line1 = TrimForDisplay(pylon_id, 15);

  if (isfinite(sensor_temp_f)) {
    float tempC = (sensor_temp_f - 32.0f) * 5.0f / 9.0f;
    char buf[16];
    snprintf(buf, sizeof(buf), "Temp %.0fF %.0fC", sensor_temp_f, tempC);  // "Temp 75F 24C" = 12
    page.line2 = buf;
  } else {
    page.line2 = "Temp --";
  }

  if (isfinite(sensor_battery_v) && isfinite(sensor_battery_pct)) {
    char buf[16];
    snprintf(buf, sizeof(buf), "Batt %.1fV %.0f%%", sensor_battery_v, sensor_battery_pct);  // "Batt 13.3V 87%" = 14
    page.line3 = buf;
  } else if (isfinite(sensor_battery_v)) {
    char buf[16];
    snprintf(buf, sizeof(buf), "Batt %.1fV --%%", sensor_battery_v);  // "Batt 13.3V --%" = 14
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

  // M:N at y=0 (size=2, 16px), Ch:N at y=16 — top-right
  DrawMeshBadge(0, 2);
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

  // M:N at y=0 (size=2, 16px), Ch:N at y=16 — top-right; y=0..7 is empty on this page (yNum=8)
  DrawMeshBadge(0, 2);
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
  payload += "\"free_sketch_space\":" + String(ESP.getFreeSketchSpace()) + ",";
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
  payload += "\"seq_max_ms\":"   + String(barmode_seq_max_ms)   + ",";
  payload += "\"seq_dec_ms\":"   + String(barmode_seq_dec_ms)   + ",";
  payload += "\"seq_exp_pct\":"  + String(barmode_seq_exp_pct)  + ",";
  payload += "\"seq_start_ms\":" + String(barmode_seq_start_ms) + ",";
  payload += "\"seq_floor_ms\":" + String(barmode_seq_floor_ms) + ",";
  payload += "\"green_timeout_ms\":" + String(barmode_green_timeout_ms) + ",";
  payload += "\"green_recovery_ms\":" + String(barmode_green_recovery_ms) + ",";
  payload += "\"blue_recovery_ms\":" + String(barmode_blue_recovery_ms) + ",";
  payload += "\"orange_recovery_ms\":" + String(barmode_orange_recovery_ms) + ",";
  payload += "\"red_recovery_ms\":" + String(barmode_red_recovery_ms) + ",";
  payload += "\"temp_thresh1_f\":" + String(barmode_temp_thresh1_f, 1) + ",";
  payload += "\"temp_mult1\":" + String(barmode_temp_mult1, 2) + ",";
  payload += "\"temp_thresh2_f\":" + String(barmode_temp_thresh2_f, 1) + ",";
  payload += "\"temp_mult2\":" + String(barmode_temp_mult2, 2) + ",";
  {
    const float avg = barmode_avg_temp_f;
    payload += "\"avg_pylon_temp_f\":" + (isnan(avg) ? String("null") : String(avg, 1)) + ",";
    payload += "\"temp_multiplier\":" + String(barmode_temp_multiplier, 2) + ",";
  }
  payload += "\"bpm\":" + String(barmode_bpm, 1) + ",";
  payload += "\"all4_valve_ms\":" + String(barmode_all4_valve_ms) + ",";
  payload += "\"all4_lockout_s\":" + String(barmode_all4_lockout_s) + ",";
  payload += "\"all4_lockout_remaining_s\":" + String(barmode_all4_lockout_until_ms > millis() ? (barmode_all4_lockout_until_ms - millis()) / 1000UL : 0UL) + ",";
  payload += "\"red_seq_max_ms\":" + String(barmode_red_seq_max_ms) + ",";
  payload += "\"red_seq_valve_ms\":" + String(barmode_red_seq_valve_ms) + ",";
  payload += "\"red_seq_step_ms\":" + String(barmode_red_seq_step_ms) + ",";
  payload += "\"pulse1_dur_ms\":" + String(action_pulse1_dur_ms) + ",";
  payload += "\"pulse1_dis\":" + String(action_pulse1_dis ? "true" : "false") + ",";
  payload += "\"pt_dur_ms\":" + String(action_pt_dur_ms) + ",";
  payload += "\"pt_off_ms\":" + String(action_pt_off_ms) + ",";
  payload += "\"pt_count\":" + String(action_pt_count) + ",";
  payload += "\"pt_dis\":" + String(action_pt_dis ? "true" : "false") + ",";
  payload += "\"steam_ramp_ms\":" + String(action_steam_ramp_ms) + ",";
  payload += "\"steam_open_ms\":" + String(action_steam_open_ms) + ",";
  payload += "\"steam_dis\":" + String(action_steam_dis ? "true" : "false") + ",";
  payload += "\"btn_press_counts\":[" + String(barmode_btn_counts[0]) + "," +
             String(barmode_btn_counts[1]) + "," + String(barmode_btn_counts[2]) + "," +
             String(barmode_btn_counts[3]) + "],";
  payload += "\"btn_act_counts\":[" + String(barmode_act_counts[0]) + "," +
             String(barmode_act_counts[1]) + "," + String(barmode_act_counts[2]) + "," +
             String(barmode_act_counts[3]) + "," + String(barmode_act_counts[4]) + "," +
             String(barmode_act_counts[5]) + "," + String(barmode_act_counts[6]) + "],";
  payload += "\"btn_disabled\":[" +
             String(barmode_btn_disabled[0]?"true":"false") + "," +
             String(barmode_btn_disabled[1]?"true":"false") + "," +
             String(barmode_btn_disabled[2]?"true":"false") + "," +
             String(barmode_btn_disabled[3]?"true":"false") + "],";
  payload += "\"btn_state\":[" +
             String(barmode_btn_state[0]?"true":"false") + "," +
             String(barmode_btn_state[1]?"true":"false") + "," +
             String(barmode_btn_state[2]?"true":"false") + "," +
             String(barmode_btn_state[3]?"true":"false") + "],";
  {
    const unsigned long tnow = millis();
    payload += "\"btn_recovery\":[" +
               String(tnow < green_recovery_until  ? "true" : "false") + "," +
               String(tnow < blue_recovery_until   ? "true" : "false") + "," +
               String(tnow < orange_recovery_until ? "true" : "false") + "," +
               String(tnow < red_recovery_until    ? "true" : "false") + "],";
    payload += "\"seq_phase\":"  + String(barmode_seq_phase_g) + ",";
    payload += "\"red_state\":"  + String(barmode_red_state_g) + ",";
    const unsigned long orng_ms = (barmode_orange_strobe_g &&
                                   tnow >= barmode_orange_strobe_start_ms_g &&
                                   (tnow - barmode_orange_strobe_start_ms_g) < 500UL)
                                  ? 500UL - (tnow - barmode_orange_strobe_start_ms_g) : 0UL;
    payload += "\"orange_strobe_ms\":" + String(orng_ms) + ",";
    const unsigned long steam_ms = (barmode_red_state_g == 3 &&
                                    tnow >= barmode_red_steam_start_ms_g)
                                   ? (tnow - barmode_red_steam_start_ms_g) : 0UL;
    payload += "\"red_steam_ms\":" + String(steam_ms) + ",";
  }
  payload += "\"wifi_ssid\":\"" + JsonEscape(user_wifi_ssid) + "\",";
  payload += "\"failsafe_ms\":" + String(boosh_failsafe_timeout_ms) + ",";
  payload += "\"dj_timeout_s\":" + String(cfg_dj_timeout_s, 1) + ",";
  payload += "\"target_ip\":\"" + JsonEscape(target_ip_string) + "\",";
  {
    char buf[16];
    const float eff_batt_v2   = cfg_no_batt_mon   ? NAN : sensor_battery_v;
    const float eff_batt_pct2 = cfg_no_batt_mon   ? NAN : sensor_battery_pct;
    const float eff_batt_h2   = cfg_no_batt_mon   ? NAN : sensor_battery_time_remaining_h;
    const float eff_temp_f2   = cfg_no_thermistor  ? NAN : sensor_temp_f;
    payload += "\"battery_voltage_v\":";
    if (isfinite(eff_batt_v2)) {
      snprintf(buf, sizeof(buf), "%.2f", eff_batt_v2);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"battery_charge_pct\":";
    if (isfinite(eff_batt_pct2)) {
      snprintf(buf, sizeof(buf), "%.1f", eff_batt_pct2);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"battery_time_remaining_h\":";
    if (isfinite(eff_batt_h2)) {
      snprintf(buf, sizeof(buf), "%.1f", eff_batt_h2);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"temperature_f\":";
    if (isfinite(eff_temp_f2)) {
      snprintf(buf, sizeof(buf), "%.1f", eff_temp_f2);
      payload += buf;
    } else {
      payload += "null";
    }
    payload += ",";
    payload += "\"no_thermistor\":" + String(cfg_no_thermistor ? "true" : "false") + ",";
    payload += "\"no_batt_mon\":" + String(cfg_no_batt_mon ? "true" : "false") + ",";
    payload += "\"route_via_rpi\":" + String(cfg_route_via_rpi ? "true" : "false") + ",";
    payload += "\"grp_pat_en\":" + String(cfg_group_pattern_en ? "true" : "false") + ",";
    payload += "\"grp_win_ms\":"  + String(cfg_grp_win_ms)  + ",";
    payload += "\"grp_cool_ms\":" + String(cfg_grp_cool_ms) + ",";
    payload += "\"grp_qon_ms\":"  + String(cfg_grp_qon_ms)  + ",";
    payload += "\"grp_qoff_ms\":" + String(cfg_grp_qoff_ms) + ",";
    payload += "\"grp_big_ms\":"  + String(cfg_grp_big_ms)  + ",";
    payload += "\"grp_gap_ms\":"  + String(cfg_grp_gap_ms)  + ",";
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
  // Append per-pylon ping stats when running in BARMODE
  if (barmode_active && barmode_ping_stats_mutex) {
    payload += ",\"barmode_pylon_pings\":[";
    if (xSemaphoreTake(barmode_ping_stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      for (int i = 0; i < barmode_pylon_ping_count; i++) {
        const PylonPingStat &s = barmode_pylon_ping_stats[i];
        if (i > 0) payload += ",";
        IPAddress paddr;
        paddr = s.ip_u32;
        const uint32_t avg = s.count ? static_cast<uint32_t>(s.sum_ms / s.count) : 0;
        payload += "{\"id\":\"" + JsonEscape(String(s.id)) + "\",";
        payload += "\"ip\":\"" + paddr.toString() + "\",";
        payload += "\"last_ok\":" + String(s.last_ok ? "true" : "false") + ",";
        payload += "\"last_ms\":" + String(s.last_ms) + ",";
        payload += "\"avg_ms\":" + String(avg) + ",";
        payload += "\"min_ms\":" + String(s.count ? s.min_ms : 0) + ",";
        payload += "\"max_ms\":" + String(s.max_ms) + ",";
        payload += "\"count\":" + String(s.count) + ",";
        payload += "\"lost\":" + String(s.lost) + "}";
      }
      xSemaphoreGive(barmode_ping_stats_mutex);
    }
    payload += "]";
  }
  payload += "},";
  // Mesh status — top-level key, NOT inside "telemetry"
  {
    int mesh_peer_count = 0;
    if (mesh_peers_mutex && xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      for (int i = 0; i < kMeshMaxPeers; i++) if (mesh_peers[i].active) mesh_peer_count++;
      xSemaphoreGive(mesh_peers_mutex);
    }
    payload += "\"mesh\":{";
    payload += "\"enabled\":" + String(cfg_mesh_en ? "true" : "false") + ",";
    payload += "\"active\":" + String(mesh_initialized ? "true" : "false") + ",";
    payload += "\"channel\":" + String(cfg_mesh_ch) + ",";
    payload += "\"peer_count\":" + String(mesh_peer_count) + ",";
    payload += "\"beacons_sent\":" + String(mesh_beacons_sent) + ",";
    payload += "\"cmds_sent\":" + String(mesh_cmds_sent) + ",";
    payload += "\"peers\":[";
    if (mesh_peers_mutex && xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bool first = true;
      for (int i = 0; i < kMeshMaxPeers; i++) {
        if (!mesh_peers[i].active) continue;
        if (!first) payload += ",";
        first = false;
        char mac_str[18];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mesh_peers[i].mac[0], mesh_peers[i].mac[1], mesh_peers[i].mac[2],
                 mesh_peers[i].mac[3], mesh_peers[i].mac[4], mesh_peers[i].mac[5]);
        const uint32_t age_s = (now - mesh_peers[i].last_seen_ms) / 1000;
        char bv_buf[12], bp_buf[8], tf_buf[10];
        if (isfinite(mesh_peers[i].batt_v))   snprintf(bv_buf, sizeof(bv_buf), "%.2f", mesh_peers[i].batt_v);   else strncpy(bv_buf, "null", sizeof(bv_buf));
        if (isfinite(mesh_peers[i].batt_pct)) snprintf(bp_buf, sizeof(bp_buf), "%.0f", mesh_peers[i].batt_pct); else strncpy(bp_buf, "null", sizeof(bp_buf));
        if (isfinite(mesh_peers[i].temp_f))   snprintf(tf_buf, sizeof(tf_buf), "%.1f", mesh_peers[i].temp_f);   else strncpy(tf_buf, "null", sizeof(tf_buf));
        payload += "{\"id\":\"" + JsonEscape(String(mesh_peers[i].node_id)) + "\",";
        payload += "\"mac\":\"" + String(mac_str) + "\",";
        payload += "\"index\":" + String(mesh_peers[i].pylon_index) + ",";
        payload += "\"role\":" + String(mesh_peers[i].role) + ",";
        payload += "\"qual_pct\":" + String(mesh_peers[i].qual_pct) + ",";
        payload += "\"age_s\":" + String(age_s) + ",";
        payload += "\"uptime_s\":" + String(mesh_peers[i].uptime_s) + ",";
        payload += "\"batt_v\":" + String(bv_buf) + ",";
        payload += "\"batt_pct\":" + String(bp_buf) + ",";
        payload += "\"temp_f\":" + String(tf_buf) + ",";
        payload += "\"fw_ver\":\"" + JsonEscape(String(mesh_peers[i].fw_ver)) + "\"}";
      }
      xSemaphoreGive(mesh_peers_mutex);
    }
    payload += "]},";
  }
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
    .vbtn{width:90px;height:90px;border-radius:50%;display:flex;flex-direction:column;align-items:center;justify-content:center;cursor:pointer;user-select:none;-webkit-user-select:none;font-weight:700;font-size:11px;letter-spacing:.05em;color:rgba(255,255,255,.9);border:3px solid rgba(255,255,255,0);touch-action:none}
    .vbtn.web-pressed{border-color:rgba(255,255,255,.9);box-shadow:0 0 24px 8px var(--vg)}
    .vbtn.phys-pressed{border-color:rgba(255,255,255,.55)}
    .vbtn-row{display:flex;gap:24px;justify-content:center;padding:8px 0 4px;flex-wrap:wrap}
    @media(max-width:640px){
      /* bottom-strip mode (default collapsed) */
      #vbtn-panel{position:fixed;bottom:0;left:0;right:0;margin:0 !important;border-radius:20px 20px 0 0;z-index:9000;padding:12px 12px max(12px,env(safe-area-inset-bottom))}
      .vbtn-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:4px 0}
      .vbtn{width:38vw;height:38vw;max-width:165px;max-height:165px;font-size:13px}
      .vbtn > div:first-child{font-size:min(11vw,48px)}
      .vbtn-spacer{display:block;height:max(calc(38vw + 80px),200px)}
      /* fullscreen mode */
      #vbtn-panel.vbtn-full{top:0;border-radius:0;display:flex !important;flex-direction:column;padding:max(16px,env(safe-area-inset-top)) 16px max(16px,env(safe-area-inset-bottom))}
      #vbtn-panel.vbtn-full .vbtn-row{flex:1;display:grid;grid-template-columns:1fr 1fr;grid-template-rows:1fr 1fr;gap:16px;padding:0}
      #vbtn-panel.vbtn-full .vbtn{width:100%;height:100%;max-width:none;max-height:none;font-size:16px}
      #vbtn-panel.vbtn-full .vbtn > div:first-child{font-size:14vw}
      #vbtn-panel.vbtn-full ~ .vbtn-spacer{display:none}
      /* toggle button */
      #vbtn-toggle{display:block;position:absolute;top:10px;right:14px;background:none;border:none;font-size:22px;line-height:1;padding:4px 6px;cursor:pointer;color:var(--muted);z-index:1}
    }
    @media(min-width:641px){.vbtn-spacer{display:none}#vbtn-toggle{display:none}}
    .active{background:#f05a28;color:#fff}
  </style>
</head>
<body>
  <main>
    <div class="panel">
      <h1 id="page-title">Pylons Control</h1>
      <p><span id="solenoid" class="pill">Solenoid idle</span></p>
      <p id="fw-version"></p>
      <p style="margin-top:6px"><a href="/dj" style="color:#ff4444;font-size:14px;text-decoration:none;letter-spacing:.05em">&#x1F525; DJ Access</a></p>
    </div>
    <div id="vbtn-panel" class="panel" style="margin-top:16px;display:none;position:relative">
      <button id="vbtn-toggle" onclick="toggleVbtnFull()" title="Toggle fullscreen">&#x26F6;</button>
      <h2>Virtual Buttons</h2>
      <div class="vbtn-row">
        <div class="vbtn" id="vbtn-3" data-btn="3" style="background:#f44336;--vg:#f44336"><div style="font-size:30px;line-height:1.1">&#9632;</div><div>RED</div></div>
        <div class="vbtn" id="vbtn-2" data-btn="2" style="background:#ff9800;--vg:#ff9800"><div style="font-size:30px;line-height:1.1">&#9632;</div><div>ORANGE</div></div>
        <div class="vbtn" id="vbtn-0" data-btn="0" style="background:#4caf50;--vg:#4caf50"><div style="font-size:30px;line-height:1.1">&#9632;</div><div>GREEN</div></div>
        <div class="vbtn" id="vbtn-1" data-btn="1" style="background:#2196f3;--vg:#2196f3"><div style="font-size:30px;line-height:1.1">&#9632;</div><div>BLUE</div></div>
      </div>
      <div style="color:var(--muted);font-size:11px;text-align:center;margin-top:6px">Hold to activate &mdash; mirrors physical button behavior</div>
    </div>
    <div class="panel" style="margin-top:16px">
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
    <div class="vbtn-spacer" id="vbtn-spacer" style="display:none"></div>
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
          <span style="color:var(--muted);font-size:13px">All Buttons</span>
          <label>Solenoid failsafe (s) <input id="cfg-failsafe-s" name="failsafe_s" type="number" min="1" max="60" step="0.1" style="width:80px"> <span style="color:var(--muted);font-size:12px">(auto-close if valve left open)</span></label>
          <label>DJ button timeout (s) <input id="cfg-dj-timeout-s" name="dj_timeout_s" type="number" min="1" max="120" step="1" style="width:80px"> <span style="color:var(--muted);font-size:12px">(auto-close DJ hold after this many seconds)</span></label>
          <label>Pylon index <input id="cfg-index" name="index" type="number" min="-99" max="99" step="1" style="width:60px"> <span style="color:var(--muted);font-size:12px">(sequential fire order; negative = skip)</span></label>
          <div id="cfg-btn-disable-wrap" style="display:none;gap:10px;align-items:center">
            <span style="color:var(--muted);font-size:12px">Disable buttons:</span>
            <label style="color:#f44336"><input type="checkbox" id="cfg-btn-dis-3" style="accent-color:#f44336"> Red</label>
            <label style="color:#ff9800"><input type="checkbox" id="cfg-btn-dis-2" style="accent-color:#ff9800"> Orange</label>
            <label style="color:#4caf50"><input type="checkbox" id="cfg-btn-dis-0" style="accent-color:#4caf50"> Green</label>
            <label style="color:#2196f3"><input type="checkbox" id="cfg-btn-dis-1" style="accent-color:#2196f3"> Blue</label>
          </div>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:10px">
          <span style="color:var(--muted);font-size:13px">OSC Actions</span>
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
            <span style="font-size:13px;min-width:120px">Pulse Once</span>
            <label style="font-size:12px">On (ms) <input id="cfg-pulse1-dur" name="pulse1_dur_ms" type="number" min="10" max="5000" step="1" style="width:70px"></label>
            <label style="display:flex;align-items:center;gap:4px;font-size:12px;color:var(--muted)"><input type="checkbox" id="cfg-pulse1-dis" style="accent-color:#e57373"> Disabled</label>
          </div>
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
            <span style="font-size:13px;min-width:120px">Pulse Train</span>
            <label style="font-size:12px">On (ms) <input id="cfg-pt-dur" name="pt_dur_ms" type="number" min="10" max="5000" step="1" style="width:70px"></label>
            <label style="font-size:12px">Off (ms) <input id="cfg-pt-off" name="pt_off_ms" type="number" min="10" max="5000" step="1" style="width:70px"></label>
            <label style="font-size:12px">Count <input id="cfg-pt-count" name="pt_count" type="number" min="1" max="20" step="1" style="width:55px"></label>
            <label style="display:flex;align-items:center;gap:4px;font-size:12px;color:var(--muted)"><input type="checkbox" id="cfg-pt-dis" style="accent-color:#e57373"> Disabled</label>
          </div>
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap">
            <span style="font-size:13px;min-width:120px">Steam Engine</span>
            <label style="font-size:12px">Ramp (ms) <input id="cfg-steam-ramp" name="steam_ramp_ms" type="number" min="100" max="30000" step="100" style="width:75px"></label>
            <label style="font-size:12px">Full open (ms) <input id="cfg-steam-open" name="steam_open_ms" type="number" min="0" max="30000" step="100" style="width:75px"></label>
            <label style="display:flex;align-items:center;gap:4px;font-size:12px;color:var(--muted)"><input type="checkbox" id="cfg-steam-dis" style="accent-color:#e57373"> Disabled</label>
          </div>
        </div>
        <div id="cfg-grp-red" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:#f44336;font-size:13px">&#11044; Red (Sequential Mode)</span>
          <label>Seq max hold (s) <input id="cfg-red-seq-max-s" name="red_seq_max_s" type="number" min="1" max="120" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(max hold duration; default 10s)</span></label>
          <label>Valve open (ms) <input id="cfg-red-seq-valve-ms" name="red_seq_valve_ms" type="number" min="20" max="2000" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(each pylon valve open time; default 66ms)</span></label>
          <label>Step delay (ms) <input id="cfg-red-seq-step-ms" name="red_seq_step_ms" type="number" min="10" max="5000" step="10" style="width:70px"> <span style="color:var(--muted);font-size:12px">(step interval; &lt;valve_ms = overlap wave)</span></label>
        </div>
        <div id="cfg-grp-all4" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:var(--muted);font-size:13px">All-4 Sequence</span>
          <label>Valve open duration (ms) <input id="cfg-all4-valve-ms" name="all4_valve_ms" type="number" min="500" max="30000" step="500" style="width:80px"> <span style="color:var(--muted);font-size:12px">(how long all valves stay open)</span></label>
          <label>Lockout after sequence (s) <input id="cfg-all4-lockout-s" name="all4_lockout_s" type="number" min="0" max="3600" step="30" style="width:80px"> <span style="color:var(--muted);font-size:12px">(cooldown before another sequence can start; 0=disabled)</span></label>
        </div>
        <div id="cfg-grp-green" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:#4caf50;font-size:13px">&#11044; Green</span>
          <label>Pulse open duration (ms) <input id="cfg-green-timeout-ms" name="green_timeout_ms" type="number" min="50" max="10000" step="50" style="width:80px"> <span style="color:var(--muted);font-size:12px">(how long green holds the valve open)</span></label>
        </div>
        <div id="cfg-grp-blue" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:#2196f3;font-size:13px">&#11044; Blue (Sequential Mode)</span>
          <label>Seq max hold (s) <input id="cfg-seq-max-s" name="seq_max_s" type="number" min="1" max="120" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(blue double-tap+hold max duration)</span></label>
          <label>Seq start delay (ms) <input id="cfg-seq-start-ms" name="seq_start_ms" type="number" min="50" max="10000" step="50" style="width:70px"> <span style="color:var(--muted);font-size:12px">(initial inter-pylon delay when seq starts)</span></label>
          <label>Seq step decrement (ms) <input id="cfg-seq-dec-ms" name="seq_dec_ms" type="number" min="0" max="2000" step="10" style="width:70px"> <span style="color:var(--muted);font-size:12px">(delay reduction per pylon step)</span></label>
          <label>Seq floor delay (ms) <input id="cfg-seq-floor-ms" name="seq_floor_ms" type="number" min="10" max="2000" step="10" style="width:70px"> <span style="color:var(--muted);font-size:12px">(minimum inter-pylon delay)</span></label>
          <label>Seq exp factor (%) <input id="cfg-seq-exp-pct" name="seq_exp_pct" type="number" min="1" max="100" step="1" style="width:70px"> <span style="color:var(--muted);font-size:12px">(multiply delay each step; 100=linear only)</span></label>
        </div>
        <div id="cfg-grp-routing" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <span style="color:var(--muted);font-size:13px">Network Routing</span>
          <div style="display:flex;align-items:center;gap:10px">
            <input type="checkbox" id="cfg-route-via-rpi" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
            <span style="color:var(--muted);font-size:14px">Route all wireless PYLON commands via RPIBOOSH wired controller</span>
          </div>
        </div>
        <div id="cfg-grp-group-pat" style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:10px">
          <span style="color:var(--muted);font-size:13px">Group Remote Pattern</span>
          <div style="display:flex;align-items:flex-start;gap:10px">
            <input type="checkbox" id="cfg-grp-pat-en" style="width:18px;height:18px;margin:2px 0 0;cursor:pointer;accent-color:var(--accent);flex-shrink:0">
            <span style="font-size:13px"><b>Enable group remote &ldquo;find-a-friend&rdquo; fire pattern</b><br>
              <span style="color:var(--muted)">When 2&thinsp;+ ESP-NOW remotes press their <b>yellow button</b> within the coincidence window, every pylon fires a synchronized reward pattern instead of the normal valve-open: N quick pulses, then N escalating long bursts. Suppresses the normal OSC open for the duration. A cooldown prevents re-triggering from the same press cluster.</span>
            </span>
          </div>
          <div style="display:grid;gap:6px;padding-left:4px">
            <span style="color:var(--muted);font-size:12px;font-weight:600;text-transform:uppercase;letter-spacing:.06em">Detection</span>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:13px">Coincidence window (ms)
                <input id="cfg-grp-win-ms" name="grp_win_ms" type="number" min="500" max="10000" step="100" style="width:80px;margin-left:6px">
                <span style="color:var(--muted);font-size:12px;margin-left:4px">How long after the first yellow press to wait for others. Default 2000&thinsp;ms.</span>
              </label>
            </div>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:13px">Cooldown (s)
                <input id="cfg-grp-cool-s" name="grp_cool_s" type="number" min="1" max="120" step="1" style="width:70px;margin-left:6px">
                <span style="color:var(--muted);font-size:12px;margin-left:4px">Minimum time before another group trigger can fire. Default 10&thinsp;s.</span>
              </label>
            </div>
          </div>
          <div style="display:grid;gap:6px;padding-left:4px;border-top:1px solid var(--line);padding-top:8px">
            <span style="color:var(--muted);font-size:12px;font-weight:600;text-transform:uppercase;letter-spacing:.06em">Pattern Timing</span>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:13px">Quick pulse on (ms) <input id="cfg-grp-qon-ms" name="grp_qon_ms" type="number" min="10" max="500" step="1" style="width:70px;margin-left:6px"></label>
              <label style="font-size:13px">Quick pulse off (ms) <input id="cfg-grp-qoff-ms" name="grp_qoff_ms" type="number" min="10" max="500" step="1" style="width:70px;margin-left:6px"></label>
              <span style="color:var(--muted);font-size:12px">N short &ldquo;tap tap&rdquo; pulses at the start (one per remote). Default 66&thinsp;/&thinsp;66&thinsp;ms.</span>
            </div>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:13px">Big pulse base (ms) <input id="cfg-grp-big-ms" name="grp_big_ms" type="number" min="100" max="5000" step="50" style="width:80px;margin-left:6px"></label>
              <label style="font-size:13px">Big pulse gap (ms) <input id="cfg-grp-gap-ms" name="grp_gap_ms" type="number" min="0" max="2000" step="50" style="width:70px;margin-left:6px"></label>
              <span style="color:var(--muted);font-size:12px">N escalating long bursts: 1&times;, 2&times;, 3&times;&hellip; the base. Gap between bursts. Default 600&thinsp;/&thinsp;300&thinsp;ms.</span>
            </div>
          </div>
        </div>
        <div id="cfg-grp-recovery" style="display:none;border-top:1px solid var(--line);padding-top:10px;display:grid;gap:10px">
          <span style="color:var(--muted);font-size:13px">Button Recovery</span>
          <span style="color:var(--muted);font-size:12px">After each tap action completes, the button is locked out for this duration. Lamp goes dark. Tap during lockout shows WAIT on display. 0 = disabled. Effective time shown when temp multiplier is active.</span>
          <div style="display:grid;gap:6px">
            <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap">
              <label style="color:#f44336;font-size:13px;min-width:80px">&#11044; Red</label>
              <label style="font-size:12px">Base (ms) <input id="cfg-red-recovery-ms" name="red_recovery_ms" type="number" min="0" max="300000" step="1" style="width:80px"></label>
              <span id="rec-red-eff" style="color:var(--muted);font-size:12px"></span>
            </div>
            <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap">
              <label style="color:#ff9800;font-size:13px;min-width:80px">&#11044; Orange</label>
              <label style="font-size:12px">Base (ms) <input id="cfg-orange-recovery-ms" name="orange_recovery_ms" type="number" min="0" max="300000" step="1" style="width:80px"></label>
              <span id="rec-orange-eff" style="color:var(--muted);font-size:12px"></span>
            </div>
            <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap">
              <label style="color:#4caf50;font-size:13px;min-width:80px">&#11044; Green</label>
              <label style="font-size:12px">Base (ms) <input id="cfg-green-recovery-ms" name="green_recovery_ms" type="number" min="0" max="300000" step="1" style="width:80px"></label>
              <span id="rec-green-eff" style="color:var(--muted);font-size:12px"></span>
            </div>
            <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap">
              <label style="color:#2196f3;font-size:13px;min-width:80px">&#11044; Blue</label>
              <label style="font-size:12px">Base (ms) <input id="cfg-blue-recovery-ms" name="blue_recovery_ms" type="number" min="0" max="300000" step="1" style="width:80px"></label>
              <span id="rec-blue-eff" style="color:var(--muted);font-size:12px"></span>
            </div>
          </div>
          <div style="border-top:1px solid var(--line);padding-top:8px;display:grid;gap:8px">
            <span style="color:var(--muted);font-size:13px">Temperature Multipliers</span>
            <span style="color:var(--muted);font-size:12px">When avg pylon temp drops below a threshold, all recovery periods are multiplied. If both thresholds are breached, only the colder one applies.</span>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:12px">Threshold 1 (°F) <input id="cfg-temp-thresh1" name="temp_thresh1_f" type="number" min="-60" max="120" step="0.5" style="width:70px"></label>
              <label style="font-size:12px">Multiplier <input id="cfg-temp-mult1" name="temp_mult1" type="number" min="0.1" max="100" step="0.1" style="width:65px">×</label>
            </div>
            <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:center">
              <label style="font-size:12px">Threshold 2 (°F) <input id="cfg-temp-thresh2" name="temp_thresh2_f" type="number" min="-60" max="120" step="0.5" style="width:70px"></label>
              <label style="font-size:12px">Multiplier <input id="cfg-temp-mult2" name="temp_mult2" type="number" min="0.1" max="100" step="0.1" style="width:65px">×</label>
            </div>
            <div id="rec-temp-status" style="color:var(--muted);font-size:12px"></div>
          </div>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <div style="display:flex;align-items:center;gap:10px">
            <input type="checkbox" id="cfg-no-thermistor" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
            <span style="color:var(--muted);font-size:14px">No thermistor present &mdash; temperature always N/A, hide temp plots</span>
          </div>
          <div style="display:flex;align-items:center;gap:10px">
            <input type="checkbox" id="cfg-no-batt-mon" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
            <span style="color:var(--muted);font-size:14px">No battery monitor present &mdash; voltage/SOC always N/A, hide battery plots</span>
          </div>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:flex;align-items:center;gap:10px">
          <input type="checkbox" id="cfg-ap" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
          <span style="color:var(--muted);font-size:14px">Enable WiFi AP &mdash; SSID: <code>PYLON_<em>id</em></code>, IP <code>10.1.2.3</code></span>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:10px;display:grid;gap:8px">
          <div style="font-weight:600;color:var(--accent)">MESH (ESP-NOW)</div>
          <div style="display:flex;align-items:center;gap:10px">
            <input type="checkbox" id="cfg-mesh-en" style="width:18px;height:18px;margin:0;cursor:pointer;accent-color:var(--accent)">
            <span style="color:var(--muted);font-size:14px">Enable MESH &mdash; ESP-NOW peer-to-peer, no router required. Reboot to apply.</span>
          </div>
          <label style="font-size:14px">Channel (1&ndash;13) <input id="cfg-mesh-ch" type="number" min="1" max="13" step="1" style="width:60px"> <span style="color:var(--muted);font-size:12px">all nodes must match; avoid overlapping your AP channel</span></label>
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
    <div id="mesh-panel" class="panel" style="margin-top:16px;display:none">
      <h2>MESH <span id="mesh-status-badge" style="font-size:14px;font-weight:400;color:var(--muted)"></span></h2>
      <div class="meta" style="margin-bottom:10px">
        <span>Channel: <strong id="mesh-ch-display">—</strong></span>
        <span style="margin-left:16px">Peers: <strong id="mesh-peer-count">0</strong></span>
        <span style="margin-left:16px">Beacons sent: <span id="mesh-beacons-sent">0</span></span>
        <span style="margin-left:16px">Cmds sent: <span id="mesh-cmds-sent">0</span></span>
      </div>
      <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:10px">
        <span style="font-size:13px;color:var(--muted)">Change channel:</span>
        <select id="mesh-ch-sel" style="background:#0d131c;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:4px 8px;font-size:13px">
          <option value="1">1</option><option value="2">2</option><option value="3">3</option>
          <option value="4">4</option><option value="5">5</option><option value="6">6</option>
          <option value="7">7</option><option value="8">8</option><option value="9">9</option>
          <option value="10">10</option><option value="11">11</option><option value="12">12</option>
          <option value="13">13</option>
        </select>
        <button onclick="applyMeshChanChange()" style="padding:5px 14px;font-size:13px">Apply to All (5s countdown)</button>
        <span id="mesh-ch-banner" style="display:none;color:#ff9800;font-size:13px;font-weight:600"></span>
      </div>
      <div style="overflow-x:auto">
      <table id="mesh-peer-table" style="border-collapse:collapse;font-size:13px;white-space:nowrap">
        <thead><tr style="color:var(--muted);text-align:left">
          <th style="padding:4px 8px">Node ID</th>
          <th style="padding:4px 8px">Index</th>
          <th style="padding:4px 8px">Role</th>
          <th style="padding:4px 8px">Quality</th>
          <th style="padding:4px 8px">Age</th>
          <th style="padding:4px 8px">Uptime</th>
          <th style="padding:4px 8px">Batt V</th>
          <th style="padding:4px 8px">SOC</th>
          <th style="padding:4px 8px">Temp</th>
          <th style="padding:4px 8px">FW</th>
          <th style="padding:4px 8px">MAC</th>
        </tr></thead>
        <tbody id="mesh-peer-tbody"></tbody>
      </table>
      </div>
      <div id="mesh-no-peers" style="color:var(--muted);font-size:13px;margin-top:6px;display:none">No peers discovered yet.</div>
      <div style="margin-top:16px;border-top:1px solid var(--line);padding-top:14px">
        <div style="font-weight:600;margin-bottom:8px;color:var(--accent)">Relay OSC via Mesh</div>
        <div style="font-size:13px;color:var(--muted);margin-bottom:10px">Send a command from this node to all ESP-NOW peers. Use this when a controller (e.g. rpiboosh) can reach this PYLON but not the others.</div>
        <div style="display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap">
          <label style="font-size:13px">OSC address
            <input id="relay-addr" type="text" value="/pylon/BooshPulseSingle" style="width:220px;margin-top:4px;font-family:monospace">
          </label>
          <label style="font-size:13px">Arg
            <input id="relay-arg" type="number" step="any" value="1.0" style="width:72px;margin-top:4px">
          </label>
          <button id="relay-btn" style="margin-bottom:2px">Send via Mesh</button>
          <span id="relay-status" style="font-size:13px;color:var(--muted)"></span>
        </div>
        <div style="font-size:12px;color:var(--muted);margin-top:8px">API: <code>POST /api/mesh/relay</code> &nbsp; params: <code>addr=&lt;osc_addr&gt;&amp;arg=&lt;float&gt;</code></div>
      </div>
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
    <div id="mesh-remotes-panel" class="panel" style="margin-top:16px;display:none">
      <h2>ESP-NOW Remotes</h2>
      <div id="mesh-remotes-content" style="overflow-x:auto">
        <span style="color:var(--muted);font-size:13px">Waiting for remote beacons&hellip;</span>
      </div>
    </div>
    <div id="manual-pylons-panel" class="panel" style="margin-top:16px;display:none">
      <h2>Manual Pylons</h2>
      <div style="color:var(--muted);font-size:12px;margin-bottom:10px">Pylons added here are included in all OSC fanouts alongside the rpiboosh registry. Persists across reboots.</div>
      <div id="manual-pylons-list" style="margin-bottom:12px"></div>
      <form id="manual-pylon-form" style="display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap">
        <label style="font-size:13px">Host / IP
          <input id="mp-host" type="text" placeholder="e.g. tiki1.local or 192.168.1.50" style="width:220px;margin-top:4px">
        </label>
        <label style="font-size:13px">Pylon index
          <input id="mp-index" type="number" min="0" max="99" value="0" style="width:70px;margin-top:4px">
        </label>
        <button type="submit" style="margin-bottom:2px">Add</button>
        <span id="mp-status" style="font-size:13px;color:var(--muted)"></span>
      </form>
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
              <td id="act-count-0" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#2196f3;white-space:nowrap">&#11044; Blue tap</td>
              <td style="padding:5px 0">Single pulse — sends one 50 ms pulse to all pylons simultaneously.</td>
              <td id="act-count-1" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#2196f3;white-space:nowrap">&#11044; Blue double-tap + hold</td>
              <td style="padding:5px 0">Sequential mode — fires pylons in index order, accelerating each loop until released or Seq max elapsed.</td>
              <td id="act-count-2" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#ff9800;white-space:nowrap">&#11044; Orange</td>
              <td style="padding:5px 0">Pulse train — sends 5&times; 50 ms pulses to all pylons simultaneously.</td>
              <td id="act-count-3" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
            <tr style="border-bottom:1px solid var(--line)">
              <td style="padding:5px 10px 5px 0;color:#f44336;white-space:nowrap">&#11044; Red tap</td>
              <td style="padding:5px 0">Main valve pulse — opens all pylon valves for 100 ms, then closes automatically.</td>
              <td id="act-count-4" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
            <tr>
              <td style="padding:5px 10px 5px 0;color:#f44336;white-space:nowrap">&#11044; Red triple-tap + hold</td>
              <td style="padding:5px 0">Steam hold — tap, tap (silent), tap &amp; hold: opens all steam valves while held (ramping frequency); closes on release. Each tap must be within 500 ms of the previous.</td>
              <td id="act-count-5" style="padding:5px 0 5px 12px;text-align:right;color:var(--muted);font-size:12px;white-space:nowrap"></td>
            </tr>
          </table>
        </div>
        <div style="border-top:1px solid var(--line);padding-top:12px">
          <div style="display:flex;align-items:baseline;gap:10px;margin-bottom:8px">
            <div style="font-weight:600;color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.08em">All-Valves Sequence (4-button unlock)</div>
            <div id="act-count-6" style="color:var(--muted);font-size:12px"></div>
          </div>
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
    async function applyMeshChanChange() {
      const ch = parseInt(document.getElementById('mesh-ch-sel').value);
      const banner = document.getElementById('mesh-ch-banner');
      let d;
      try {
        d = await fetchJson('/api/mesh/chanchange', {
          method: 'POST',
          headers: {'Content-Type': 'application/x-www-form-urlencoded'},
          body: 'ch=' + ch
        });
      } catch(e) { alert('Channel change failed: ' + e.message); return; }
      if (!d.ok) { alert('Error: ' + d.error); return; }
      if (!d.broadcast) {
        banner.style.display = '';
        banner.textContent = 'Channel set to ' + ch + ' (local only — mesh not active)';
        return;
      }
      let secs = Math.ceil(d.delay_ms / 1000);
      banner.style.display = '';
      banner.textContent = 'Switching all nodes to ch ' + ch + ' in ' + secs + 's\u2026';
      const iv = setInterval(() => {
        secs--;
        if (secs <= 0) {
          clearInterval(iv);
          banner.textContent = 'Channel ' + ch + ' applied \u2014 verify below';
          setTimeout(() => { banner.style.display = 'none'; }, 4000);
        } else {
          banner.textContent = 'Switching all nodes to ch ' + ch + ' in ' + secs + 's\u2026';
        }
      }, 1000);
    }
    function esc(value) {
      return String(value ?? '').replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;');
    }
    function setPage(id, lines) {
      document.getElementById(id).textContent = (lines || []).join('\n');
    }
    let formDirty = false;  // true when user has edited a config input but not yet saved
    function syncConfigField(id, nextValue) {
      const input = document.getElementById(id);
      if (!input) return;
      if (formDirty) return;  // never overwrite user edits between keystroke and save
      if (document.activeElement === input) return;
      input.value = nextValue ?? '';
    }
    let barmodeActive = false;
    let barmodePylonPings = [];  // cached from last telemetry; keyed by IP
    function renderMeta(data) {
      barmodeActive = !!data.barmode_active;
      barmodePylonPings = (data.telemetry && Array.isArray(data.telemetry.barmode_pylon_pings))
        ? data.telemetry.barmode_pylon_pings : [];
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
      updateBtnActivity(data.btn_press_counts, data.btn_act_counts);
      if (data.btn_state)    { for (let i = 0; i < 4; i++) physBtnDown[i]   = !!data.btn_state[i]; }
      if (data.btn_disabled) { for (let i = 0; i < 4; i++) vBtnDisabled[i]  = !!data.btn_disabled[i]; }
      if (data.btn_recovery) { for (let i = 0; i < 4; i++) vBtnRecovery[i]  = !!data.btn_recovery[i]; }
      vSeqPhase = data.seq_phase  || 0;
      vRedState = data.red_state  || 0;
      vOrangeStrobeMs     = data.orange_strobe_ms || 0;
      vOrangeStrobeRecvMs = Date.now();
      vRedSteamMs         = data.red_steam_ms     || 0;
      vRedSteamRecvMs     = Date.now();
      vBtnBpm = data.bpm || 0;
      const vp = document.getElementById('vbtn-panel');
      const vs = document.getElementById('vbtn-spacer');
      const wasBarmodeActive = vp && vp.style.display !== 'none';
      if (vp) vp.style.display = barmodeActive ? '' : 'none';
      if (!barmodeActive) {
        if (vs) vs.style.display = 'none';
      } else if (!wasBarmodeActive) {
        // First time barmode panel becomes visible: init fullscreen from localStorage or default
        const isMobile = window.innerWidth <= 640;
        const saved = localStorage.getItem('vbtnFull');
        setVbtnFull(saved !== null ? saved === '1' : isMobile);
      } else {
        // Already visible: just keep spacer in sync with current fullscreen state
        if (vs) vs.style.display = vbtnFullscreen ? 'none' : '';
      }
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
      if (!formDirty) {
      syncConfigField('cfg-id', data.pylon_id || '');
      syncConfigField('cfg-host', (data.hostname || '').replace(/\.local$/,''));
      syncConfigField('cfg-description', data.description || '');
      syncConfigField('cfg-wifi-ssid', data.wifi_ssid || '');
      syncConfigField('cfg-failsafe-s',       data.failsafe_ms != null ? (data.failsafe_ms / 1000).toFixed(1) : '5.0');
      syncConfigField('cfg-dj-timeout-s',     data.dj_timeout_s != null ? data.dj_timeout_s : 10);
      syncConfigField('cfg-index',             data.pylon_index != null ? data.pylon_index : 0);
      ['cfg-grp-green','cfg-grp-blue','cfg-grp-all4','cfg-grp-routing','cfg-grp-red','cfg-grp-recovery','cfg-btn-disable-wrap'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.style.display = data.barmode_active ? 'grid' : 'none';
      });
      const grpPatPanel = document.getElementById('cfg-grp-group-pat');
      if (grpPatPanel) grpPatPanel.style.display = 'grid';
      syncConfigField('cfg-green-timeout-ms',  data.green_timeout_ms  != null ? data.green_timeout_ms  : 300);
      syncConfigField('cfg-all4-valve-ms',     data.all4_valve_ms     != null ? data.all4_valve_ms     : 3000);
      syncConfigField('cfg-all4-lockout-s',    data.all4_lockout_s    != null ? data.all4_lockout_s    : 300);
      syncConfigField('cfg-red-seq-max-s',     data.red_seq_max_ms    != null ? Math.round(data.red_seq_max_ms / 1000) : 10);
      syncConfigField('cfg-red-seq-valve-ms',  data.red_seq_valve_ms  != null ? data.red_seq_valve_ms  : 66);
      syncConfigField('cfg-red-seq-step-ms',   data.red_seq_step_ms   != null ? data.red_seq_step_ms   : 200);
      syncConfigField('cfg-pulse1-dur',        data.pulse1_dur_ms     != null ? data.pulse1_dur_ms     : 50);
      const p1DisInput = document.getElementById('cfg-pulse1-dis');
      if (p1DisInput) p1DisInput.checked = !!data.pulse1_dis;
      syncConfigField('cfg-pt-dur',            data.pt_dur_ms         != null ? data.pt_dur_ms         : 50);
      syncConfigField('cfg-pt-off',            data.pt_off_ms         != null ? data.pt_off_ms         : 50);
      syncConfigField('cfg-pt-count',          data.pt_count          != null ? data.pt_count          : 5);
      const ptDisInput = document.getElementById('cfg-pt-dis');
      if (ptDisInput) ptDisInput.checked = !!data.pt_dis;
      syncConfigField('cfg-steam-ramp',        data.steam_ramp_ms     != null ? data.steam_ramp_ms     : 4000);
      syncConfigField('cfg-steam-open',        data.steam_open_ms     != null ? data.steam_open_ms     : 1000);
      const stmDisInput = document.getElementById('cfg-steam-dis');
      if (stmDisInput) stmDisInput.checked = !!data.steam_dis;
      syncConfigField('cfg-seq-max-s',         data.seq_max_ms        != null ? Math.round(data.seq_max_ms / 1000) : 30);
      syncConfigField('cfg-seq-start-ms',      data.seq_start_ms      != null ? data.seq_start_ms      : 200);
      syncConfigField('cfg-seq-dec-ms',        data.seq_dec_ms        != null ? data.seq_dec_ms        : 50);
      syncConfigField('cfg-seq-floor-ms',      data.seq_floor_ms      != null ? data.seq_floor_ms      : 50);
      syncConfigField('cfg-seq-exp-pct',       data.seq_exp_pct       != null ? data.seq_exp_pct       : 100);
      syncConfigField('cfg-green-recovery-ms', data.green_recovery_ms != null ? data.green_recovery_ms : 0);
      syncConfigField('cfg-blue-recovery-ms',  data.blue_recovery_ms  != null ? data.blue_recovery_ms  : 0);
      syncConfigField('cfg-orange-recovery-ms',data.orange_recovery_ms!= null ? data.orange_recovery_ms: 0);
      syncConfigField('cfg-red-recovery-ms',   data.red_recovery_ms   != null ? data.red_recovery_ms   : 0);
      syncConfigField('cfg-temp-thresh1', data.temp_thresh1_f != null ? data.temp_thresh1_f : 50);
      syncConfigField('cfg-temp-mult1',   data.temp_mult1     != null ? data.temp_mult1     : 1);
      syncConfigField('cfg-temp-thresh2', data.temp_thresh2_f != null ? data.temp_thresh2_f : 32);
      syncConfigField('cfg-temp-mult2',   data.temp_mult2     != null ? data.temp_mult2     : 1);
      const apBox = document.getElementById('cfg-ap');
      if (apBox && document.activeElement !== apBox) apBox.checked = !!data.ap_enabled;
      const noThermBox = document.getElementById('cfg-no-thermistor');
      if (noThermBox && document.activeElement !== noThermBox) noThermBox.checked = !!data.no_thermistor;
      const noBattBox = document.getElementById('cfg-no-batt-mon');
      if (noBattBox && document.activeElement !== noBattBox) noBattBox.checked = !!data.no_batt_mon;
      const routeViaRpiBox = document.getElementById('cfg-route-via-rpi');
      if (routeViaRpiBox && document.activeElement !== routeViaRpiBox) routeViaRpiBox.checked = !!data.route_via_rpi;
      const grpPatEnBox = document.getElementById('cfg-grp-pat-en');
      if (grpPatEnBox && document.activeElement !== grpPatEnBox) grpPatEnBox.checked = data.grp_pat_en !== false;
      syncConfigField('cfg-grp-win-ms',  data.grp_win_ms  != null ? data.grp_win_ms  : 2000);
      syncConfigField('cfg-grp-cool-s',  data.grp_cool_ms != null ? Math.round(data.grp_cool_ms / 1000) : 10);
      syncConfigField('cfg-grp-qon-ms',  data.grp_qon_ms  != null ? data.grp_qon_ms  : 66);
      syncConfigField('cfg-grp-qoff-ms', data.grp_qoff_ms != null ? data.grp_qoff_ms : 66);
      syncConfigField('cfg-grp-big-ms',  data.grp_big_ms  != null ? data.grp_big_ms  : 600);
      syncConfigField('cfg-grp-gap-ms',  data.grp_gap_ms  != null ? data.grp_gap_ms  : 300);
      const meshEnBox = document.getElementById('cfg-mesh-en');
      if (meshEnBox && document.activeElement !== meshEnBox) meshEnBox.checked = !!(data.mesh && data.mesh.enabled);
      syncConfigField('cfg-mesh-ch', data.mesh ? (data.mesh.channel || 1) : 1);
      const disWrap = document.getElementById('cfg-btn-disable-wrap');
      if (disWrap) disWrap.style.display = barmodeActive ? 'flex' : 'none';
      if (barmodeActive && Array.isArray(data.btn_disabled)) {
        for (let i = 0; i < 4; i++) {
          const cb = document.getElementById('cfg-btn-dis-' + i);
          if (cb && document.activeElement !== cb) cb.checked = !!data.btn_disabled[i];
        }
      }
      } // end if (!formDirty)
      // Display-only updates run every refresh regardless of formDirty
      {
        // Show/hide chart panels based on hardware presence flags
        const battPanels = ['chart-short','chart-long'].map(id => document.getElementById(id)?.closest('.panel')).filter(Boolean);
        const tempPanels = ['chart-temp-short','chart-temp-long'].map(id => document.getElementById(id)?.closest('.panel')).filter(Boolean);
        battPanels.forEach(p => p.style.display = data.no_batt_mon ? 'none' : '');
        tempPanels.forEach(p => p.style.display = data.no_thermistor ? 'none' : '');
      }
      {
        const mult = data.temp_multiplier != null ? data.temp_multiplier : 1;
        const effStr = (base) => (!base || mult <= 1.0) ? '' : ('\u2192 ' + Math.round(base * mult) + ' ms effective');
        const grnEff = document.getElementById('rec-green-eff');   if (grnEff) grnEff.textContent = effStr(data.green_recovery_ms);
        const bluEff = document.getElementById('rec-blue-eff');    if (bluEff) bluEff.textContent = effStr(data.blue_recovery_ms);
        const orgEff = document.getElementById('rec-orange-eff');  if (orgEff) orgEff.textContent = effStr(data.orange_recovery_ms);
        const redEff = document.getElementById('rec-red-eff');     if (redEff) redEff.textContent = effStr(data.red_recovery_ms);
        const tempStat = document.getElementById('rec-temp-status');
        if (tempStat) {
          const avgT = data.avg_pylon_temp_f;
          tempStat.textContent = avgT == null
            ? 'Avg pylon temp: \u2014 (no data yet; polls every 2 min)'
            : 'Avg pylon temp: ' + avgT.toFixed(1) + '\u00b0F \u2014 Active multiplier: ' + mult.toFixed(2) + '\u00d7';
        }
      }
      // Mesh panel (display-only, always updates)
      {
        const mesh = data.mesh;
        const meshPanel = document.getElementById('mesh-panel');
        if (meshPanel) meshPanel.style.display = (mesh && mesh.enabled) ? '' : 'none';
        if (mesh) {
          const badge = document.getElementById('mesh-status-badge');
          if (badge) badge.textContent = mesh.active ? '\u25cf ACTIVE' : '\u25cb INACTIVE';
          const chDisp = document.getElementById('mesh-ch-display');
          if (chDisp) chDisp.textContent = mesh.channel;
          if (mesh.channel) syncConfigField('mesh-ch-sel', mesh.channel);
          const pcDisp = document.getElementById('mesh-peer-count');
          if (pcDisp) pcDisp.textContent = mesh.peer_count;
          const bsDisp = document.getElementById('mesh-beacons-sent');
          if (bsDisp) bsDisp.textContent = mesh.beacons_sent;
          const csDisp = document.getElementById('mesh-cmds-sent');
          if (csDisp) csDisp.textContent = mesh.cmds_sent;
          const tbody = document.getElementById('mesh-peer-tbody');
          const noPeers = document.getElementById('mesh-no-peers');
          if (tbody) {
            tbody.innerHTML = '';
            const peers = mesh.peers || [];
            peers.forEach(p => {
              const tr = document.createElement('tr');
              tr.style.borderTop = '1px solid var(--line)';
              const qualColor = p.qual_pct >= 80 ? '#4caf50' : p.qual_pct >= 50 ? '#ff9800' : '#f44336';
              const battV   = p.batt_v   != null ? p.batt_v.toFixed(2) + 'V' : 'N/A';
              const battPct = p.batt_pct != null ? p.batt_pct.toFixed(0) + '%' : 'N/A';
              const tempStr = p.temp_f   != null ? p.temp_f.toFixed(1) + '°F' : 'N/A';
              const fwStr   = p.fw_ver   || 'N/A';
              tr.innerHTML = '<td style="padding:4px 8px"><a href="http://' + esc(p.id) + '.local/" target="_blank" style="color:var(--accent);text-decoration:none">' + esc(p.id) + '</a></td>' +
                '<td style="padding:4px 8px">' + p.index + '</td>' +
                '<td style="padding:4px 8px">' + (p.role === 1 ? 'BARMODE' : 'normal') + '</td>' +
                '<td style="padding:4px 8px;color:' + qualColor + '">' + p.qual_pct + '%</td>' +
                '<td style="padding:4px 8px">' + p.age_s + 's</td>' +
                '<td style="padding:4px 8px">' + p.uptime_s + 's</td>' +
                '<td style="padding:4px 8px">' + battV + '</td>' +
                '<td style="padding:4px 8px">' + battPct + '</td>' +
                '<td style="padding:4px 8px">' + tempStr + '</td>' +
                '<td style="padding:4px 8px;font-size:11px">' + esc(fwStr) + '</td>' +
                '<td style="padding:4px 8px;font-family:monospace;font-size:11px">' + esc(p.mac) + '</td>';
              tbody.appendChild(tr);
            });
            if (noPeers) noPeers.style.display = peers.length === 0 ? '' : 'none';
          }
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
    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  STOP — ADD NEW SYNCED ELEMENTS HERE BEFORE WRITING SYNC CODE       ║
    // ║                                                                      ║
    // ║  Every <input>, <select>, or <checkbox> whose value is written from  ║
    // ║  telemetry data MUST appear in this array. No exceptions.            ║
    // ║  Missing it means the 1s telemetry tick overwrites the user's edit.  ║
    // ║  This bug has regressed 5+ times. Adding the HTML and the sync code  ║
    // ║  is not enough — you must also add the id here.                      ║
    // ║                                                                      ║
    // ║  Rules:                                                              ║
    // ║  1. Add id to this array (inputs, selects, AND checkboxes)           ║
    // ║  2. Sync via syncConfigField(id, value) — never bare .value = x      ║
    // ║  2b. Checkboxes: sync inside if (!formDirty) with activeElement check ║
    // ╚══════════════════════════════════════════════════════════════════════╝
    const configInputs = ['cfg-id', 'cfg-host', 'cfg-description', 'cfg-node', 'cfg-wifi-ssid',
      'cfg-wifi-pass', 'cfg-failsafe-s', 'cfg-index', 'cfg-green-timeout-ms', 'cfg-all4-valve-ms',
      'cfg-all4-lockout-s', 'cfg-seq-max-s', 'cfg-seq-start-ms', 'cfg-seq-dec-ms', 'cfg-seq-floor-ms', 'cfg-seq-exp-pct',
      'cfg-red-seq-max-s', 'cfg-red-seq-valve-ms', 'cfg-red-seq-step-ms',
      'cfg-pulse1-dur', 'cfg-pt-dur', 'cfg-pt-off', 'cfg-pt-count',
      'cfg-steam-ramp', 'cfg-steam-open',
      'cfg-green-recovery-ms', 'cfg-blue-recovery-ms', 'cfg-orange-recovery-ms', 'cfg-red-recovery-ms',
      'cfg-temp-thresh1', 'cfg-temp-mult1', 'cfg-temp-thresh2', 'cfg-temp-mult2',
      'cfg-route-via-rpi', 'cfg-grp-pat-en',
      'cfg-grp-win-ms', 'cfg-grp-cool-s', 'cfg-grp-qon-ms', 'cfg-grp-qoff-ms', 'cfg-grp-big-ms', 'cfg-grp-gap-ms',
      'cfg-mesh-en', 'cfg-mesh-ch', 'cfg-dj-timeout-s',
      'mesh-ch-sel']
      .map((id) => document.getElementById(id))
      .filter(Boolean);
    let holdActive = false;

    configInputs.forEach((input) => {
      input.addEventListener('input', () => { formDirty = true; });
      input.addEventListener('change', () => { formDirty = true; });
    });
    // Also catch checkbox and number inputs in the config form
    document.getElementById('config-form').addEventListener('input', () => { formDirty = true; });

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
      const all4LckVal = document.getElementById('cfg-all4-lockout-s').value.trim();
      if (all4LckVal !== '') body.set('all4_lockout_s', all4LckVal);
      const redSeqMaxVal = document.getElementById('cfg-red-seq-max-s').value.trim();
      if (redSeqMaxVal !== '') body.set('red_seq_max_s', redSeqMaxVal);
      const redSeqVlvVal = document.getElementById('cfg-red-seq-valve-ms').value.trim();
      if (redSeqVlvVal !== '') body.set('red_seq_valve_ms', redSeqVlvVal);
      const redSeqStpVal = document.getElementById('cfg-red-seq-step-ms').value.trim();
      if (redSeqStpVal !== '') body.set('red_seq_step_ms', redSeqStpVal);
      // OSC action params
      const p1DurVal = document.getElementById('cfg-pulse1-dur').value.trim();
      if (p1DurVal !== '') body.set('pulse1_dur_ms', p1DurVal);
      body.set('pulse1_dis', document.getElementById('cfg-pulse1-dis').checked ? '1' : '0');
      const ptDurVal = document.getElementById('cfg-pt-dur').value.trim();
      if (ptDurVal !== '') body.set('pt_dur_ms', ptDurVal);
      const ptOffVal = document.getElementById('cfg-pt-off').value.trim();
      if (ptOffVal !== '') body.set('pt_off_ms', ptOffVal);
      const ptCntVal = document.getElementById('cfg-pt-count').value.trim();
      if (ptCntVal !== '') body.set('pt_count', ptCntVal);
      body.set('pt_dis', document.getElementById('cfg-pt-dis').checked ? '1' : '0');
      const stmRampVal = document.getElementById('cfg-steam-ramp').value.trim();
      if (stmRampVal !== '') body.set('steam_ramp_ms', stmRampVal);
      const stmOpenVal = document.getElementById('cfg-steam-open').value.trim();
      if (stmOpenVal !== '') body.set('steam_open_ms', stmOpenVal);
      body.set('steam_dis', document.getElementById('cfg-steam-dis').checked ? '1' : '0');
      body.set('no_thermistor', document.getElementById('cfg-no-thermistor').checked ? '1' : '0');
      body.set('no_batt_mon',   document.getElementById('cfg-no-batt-mon').checked   ? '1' : '0');
      body.set('route_via_rpi', document.getElementById('cfg-route-via-rpi').checked ? '1' : '0');
      body.set('grp_pat_en', document.getElementById('cfg-grp-pat-en').checked ? '1' : '0');
      const grpWinVal = document.getElementById('cfg-grp-win-ms').value.trim();
      if (grpWinVal !== '') body.set('grp_win_ms', grpWinVal);
      const grpCoolVal = document.getElementById('cfg-grp-cool-s').value.trim();
      if (grpCoolVal !== '') body.set('grp_cool_ms', String(Math.round(parseFloat(grpCoolVal) * 1000)));
      const grpQOnVal = document.getElementById('cfg-grp-qon-ms').value.trim();
      if (grpQOnVal !== '') body.set('grp_qon_ms', grpQOnVal);
      const grpQOffVal = document.getElementById('cfg-grp-qoff-ms').value.trim();
      if (grpQOffVal !== '') body.set('grp_qoff_ms', grpQOffVal);
      const grpBigVal = document.getElementById('cfg-grp-big-ms').value.trim();
      if (grpBigVal !== '') body.set('grp_big_ms', grpBigVal);
      const grpGapVal = document.getElementById('cfg-grp-gap-ms').value.trim();
      if (grpGapVal !== '') body.set('grp_gap_ms', grpGapVal);
      body.set('mesh_en', document.getElementById('cfg-mesh-en').checked ? '1' : '0');
      const meshChVal = document.getElementById('cfg-mesh-ch').value.trim();
      if (meshChVal !== '') body.set('mesh_ch', meshChVal);
      const seqMaxVal   = document.getElementById('cfg-seq-max-s').value.trim();
      if (seqMaxVal   !== '') body.set('seq_max_s',   seqMaxVal);
      const seqStartVal = document.getElementById('cfg-seq-start-ms').value.trim();
      if (seqStartVal !== '') body.set('seq_start_ms', seqStartVal);
      const seqDecVal   = document.getElementById('cfg-seq-dec-ms').value.trim();
      if (seqDecVal   !== '') body.set('seq_dec_ms',  seqDecVal);
      const seqFloorVal = document.getElementById('cfg-seq-floor-ms').value.trim();
      if (seqFloorVal !== '') body.set('seq_floor_ms', seqFloorVal);
      const seqExpVal   = document.getElementById('cfg-seq-exp-pct').value.trim();
      if (seqExpVal   !== '') body.set('seq_exp_pct',  seqExpVal);
      const t1Val = document.getElementById('cfg-temp-thresh1').value.trim();
      if (t1Val !== '') body.set('temp_thresh1_f', t1Val);
      const m1Val = document.getElementById('cfg-temp-mult1').value.trim();
      if (m1Val !== '') body.set('temp_mult1', m1Val);
      const t2Val = document.getElementById('cfg-temp-thresh2').value.trim();
      if (t2Val !== '') body.set('temp_thresh2_f', t2Val);
      const m2Val = document.getElementById('cfg-temp-mult2').value.trim();
      if (m2Val !== '') body.set('temp_mult2', m2Val);
      const grnRecVal = document.getElementById('cfg-green-recovery-ms').value.trim();
      if (grnRecVal !== '') body.set('green_recovery_ms', grnRecVal);
      const bluRecVal = document.getElementById('cfg-blue-recovery-ms').value.trim();
      if (bluRecVal !== '') body.set('blue_recovery_ms', bluRecVal);
      const orgRecVal = document.getElementById('cfg-orange-recovery-ms').value.trim();
      if (orgRecVal !== '') body.set('orange_recovery_ms', orgRecVal);
      const redRecVal = document.getElementById('cfg-red-recovery-ms').value.trim();
      if (redRecVal !== '') body.set('red_recovery_ms', redRecVal);
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
        formDirty = false;
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
      // Build local-ping lookup by IP for the extra column
      const localPingByIp = {};
      barmodePylonPings.forEach(s => { localPingByIp[s.ip] = s; });
      const fmtLocalPing = ip => {
        const s = localPingByIp[ip];
        if (!s) return '<span style="color:var(--muted)">-</span>';
        const color = s.last_ok ? '#4caf50' : '#e57373';
        const avg = s.avg_ms != null ? s.avg_ms + 'ms' : '-';
        const lost = s.lost > 0 ? ` <span style="color:#e57373;font-size:11px">${s.lost}x!</span>` : '';
        return `<span style="color:${color}">${avg}</span>${lost}`;
      };
      const rows = all.map(p => `<tr style="border-top:1px solid var(--line)">
        ${td('<b>'+esc(p.pylon_id||'?')+'</b>')}
        ${td(p.pylon_index != null ? p.pylon_index : '-','color:var(--muted);text-align:center')}
        ${td(p.hostname ? `<a href="http://${esc(p.hostname)}" target="_blank" style="color:var(--accent)">${esc(p.hostname)}</a>` : '-')}
        ${td(esc(p.ip||'-'),'color:var(--muted)')}
        ${td(badge(p.active))}
        ${td(p.seconds_since_seen != null ? p.seconds_since_seen+'s ago' : '-','color:var(--muted)')}
        ${td(fmt(p.battery_charge_pct,'%',0))}
        ${td(fmt(p.battery_voltage_v,'V',2))}
        ${td(fmt(p.temperature_f,'\u00b0F',1))}
        ${td(fmt(p.wifi_rssi_dbm,' dBm',0))}
        ${td(p.ping_avg_ms != null ? p.ping_avg_ms+'ms' : '-')}
        <td style="padding:4px 10px;font-size:13px;white-space:nowrap">${fmtLocalPing(p.ip||'')}</td>
        ${td(esc(p.fw_version||'-'),'color:var(--muted);font-size:11px')}
      </tr>`).join('');
      el.innerHTML = `<table style="width:100%;border-collapse:collapse"><thead><tr>
        ${th('ID')}${th('Idx')}${th('Host')}${th('IP')}${th('Status')}${th('Seen')}${th('Bat%')}${th('BatV')}${th('Temp')}${th('RSSI')}${th('RPI Ping')}${th('Local Ping')}${th('FW')}
      </tr></thead><tbody>${rows}</tbody></table>
      <div style="color:var(--muted);font-size:11px;margin-top:6px">${entries.length} online, ${offline.length} recently offline \u2014 ${esc((data.data&&data.data.server_time)||'')}</div>`;
    }
    async function refreshRegistry() {
      try { renderRegistry(await fetchJson('/api/registry')); }
      catch(e) { document.getElementById('registry-content').innerHTML = '<span style="color:#e57373;font-size:13px">Error: '+esc(e.message)+'</span>'; }
    }
    // --- Manual Pylons ---
    function renderManualPylons(data) {
      const panel = document.getElementById('manual-pylons-panel');
      if (!barmodeActive) { panel.style.display = 'none'; return; }
      panel.style.display = '';
      const list = document.getElementById('manual-pylons-list');
      if (!data || !data.entries || data.entries.length === 0) {
        list.innerHTML = '<span style="color:var(--muted);font-size:13px">None added yet.</span>';
        return;
      }
      const rows = data.entries.map(e => `<div style="display:flex;align-items:center;gap:12px;padding:5px 0;border-bottom:1px solid var(--line);font-size:13px">
        <span style="min-width:160px">${esc(e.host)}</span>
        <span style="color:var(--muted);min-width:40px">idx ${e.index}</span>
        <span style="color:${e.resolved?'#4caf50':'#e57373'};min-width:110px">${e.resolved ? e.ip : 'Resolving\u2026'}</span>
        <button onclick="removeManualPylon(${e.i})" style="padding:2px 10px;font-size:12px">Remove</button>
      </div>`).join('');
      list.innerHTML = rows;
    }
    async function refreshManualPylons() {
      try { renderManualPylons(await fetchJson('/api/pylons/manual')); }
      catch(e) {}
    }
    async function removeManualPylon(i) {
      const fd = new FormData(); fd.set('action','remove'); fd.set('i', String(i));
      try { await fetchJson('/api/pylons/manual', {method:'POST',body:fd}); refreshManualPylons(); }
      catch(e) { alert('Remove failed: ' + e.message); }
    }
    document.getElementById('manual-pylon-form').addEventListener('submit', async e => {
      e.preventDefault();
      const host = document.getElementById('mp-host').value.trim();
      const index = document.getElementById('mp-index').value.trim();
      const status = document.getElementById('mp-status');
      if (!host) { status.textContent = 'Host required'; status.style.color='#e57373'; return; }
      const fd = new FormData(); fd.set('action','add'); fd.set('host',host); fd.set('index',index||'0');
      try {
        await fetchJson('/api/pylons/manual', {method:'POST',body:fd});
        document.getElementById('mp-host').value = '';
        status.textContent = 'Added — resolving\u2026'; status.style.color='#4caf50';
        setTimeout(() => { status.textContent = ''; }, 3000);
        refreshManualPylons();
      } catch(err) { status.textContent = 'Error: '+err.message; status.style.color='#e57373'; }
    });
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

    function updateBtnActivity(counts, actCounts) {
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
      if (actCounts) {
        const labels = ['×', '×', '×', '×', '×', '×', '×'];
        for (let i = 0; i < 7; i++) {
          const el = document.getElementById('act-count-' + i);
          if (el) el.textContent = actCounts[i] != null ? labels[i] + actCounts[i] : '';
        }
      }
    }

    // Virtual Buttons — animation and press/release
    let webBtnDown  = [false,false,false,false];
    let physBtnDown = [false,false,false,false];
    let vBtnBpm = 0;

    // Lamp state synced from telemetry (1s updates); animation interpolates between
    let vBtnDisabled   = [false,false,false,false];
    let vBtnRecovery   = [false,false,false,false];
    let vSeqPhase      = 0;
    let vRedState      = 0;
    let vOrangeStrobeMs      = 0;   // remaining ms at time of last telemetry tick
    let vOrangeStrobeRecvMs  = 0;   // Date.now() when that tick arrived
    let vRedSteamMs          = 0;   // elapsed ms at last tick
    let vRedSteamRecvMs      = 0;

    const LAVA_UNITS = [1,1,3,1,1,1,1,3, 1,1,3,3, 1,1,1,1,1,1,3,3, 1,1,3,7];

    function animateVBtns() {
      const now = Date.now();
      const tSec = now / 1000;
      const bpm  = (vBtnBpm >= 40 && vBtnBpm <= 220) ? vBtnBpm : 0;
      const beat_ms = bpm ? 60000 / bpm : 1000;

      // Orange strobe: remaining time is vOrangeStrobeMs minus elapsed since last tick
      const orangeRem = Math.max(0, vOrangeStrobeMs - (now - vOrangeStrobeRecvMs));
      // Red steam: total elapsed = value at last tick + elapsed since last tick
      const steamMs   = (vRedState === 3) ? vRedSteamMs + (now - vRedSteamRecvMs) : 0;

      // ALL-4 sequence phase overrides: [g=0, b=1, o=2, r=3]
      let seqBr = null;
      if (vSeqPhase >= 1 && vSeqPhase <= 4) {
        if (vSeqPhase === 1) {
          const f = (now % 500 < 125) ? 0.25 : 0;
          seqBr = [1.0, f, 0, 0];
        } else if (vSeqPhase === 2) {
          const f = (now % 250 < 125) ? 0.5 : 0;
          seqBr = [f, f, 1.0, 0];
        } else if (vSeqPhase === 3) {
          const f = (now % 167 < 125) ? 0.75 : 0;
          seqBr = [f, f, f, 1.0];
        } else {
          const f = (now % 125 < 100) ? 0.8 : 0;
          seqBr = [f, f, f, f];
        }
      } else if (vSeqPhase === 5) {
        seqBr = [0, 0, 0, 0];
      }

      for (let i = 0; i < 4; i++) {
        const btn = document.getElementById('vbtn-' + i);
        if (!btn) continue;
        let br;

        if (seqBr) {
          br = seqBr[i];
        } else if (vBtnDisabled[i]) {
          br = 0.30;
        } else if (vBtnRecovery[i]) {
          br = 0;
        } else if (webBtnDown[i] || physBtnDown[i]) {
          br = 1.0;
        } else if (i === 0) {
          // Green: sine at BPM freq (or 2 Hz)
          const freq = bpm ? bpm / 60 : 2;
          br = Math.sin(2 * Math.PI * freq * tSec) * 0.5 + 0.5;
        } else if (i === 1) {
          // Blue: 50 ms flash per beat, base 20%
          br = (now % beat_ms < 50) ? 1.0 : 0.2;
        } else if (i === 2) {
          // Orange: 5× strobe for 500 ms post-fire, then sawtooth per beat
          if (orangeRem > 0) {
            const pos = 500 - orangeRem;
            br = (Math.floor(pos / 50) % 2 === 0) ? 1.0 : 0;
          } else {
            const sawPeriod = bpm ? beat_ms : 250;
            br = (now % sawPeriod) / sawPeriod;
          }
        } else {
          // Red: state-dependent
          if (vRedState === 3) {
            // Steam: ramp 1→10 Hz over 4 s, solid after
            if (steamMs >= 4000) {
              br = 1.0;
            } else {
              const fhz = Math.pow(10, steamMs / 4000);
              const pms = 1000 / fhz;
              br = (now % pms < 50) ? 1.0 : 0;
            }
          } else if (vRedState === 4) {
            br = 1.0;  // sequential hold: solid
          } else {
            // Idle: Morse "LAVA"
            let mu = bpm ? beat_ms / 4 : 150;
            mu = Math.max(50, Math.min(400, mu));
            const totalMs = LAVA_UNITS.reduce((a, b) => a + b, 0) * mu;
            const t = now % totalMs;
            let accum = 0; br = 0;
            for (let j = 0; j < LAVA_UNITS.length; j++) {
              accum += LAVA_UNITS[j] * mu;
              if (t < accum) { br = (j % 2 === 0) ? 1.0 : 0; break; }
            }
          }
        }

        btn.style.opacity = (0.15 + br * 0.85).toFixed(3);
        btn.classList.toggle('web-pressed', webBtnDown[i]);
        btn.classList.toggle('phys-pressed', physBtnDown[i] && !webBtnDown[i]);
      }
      requestAnimationFrame(animateVBtns);
    }
    requestAnimationFrame(animateVBtns);

    async function sendVBtn(btn, down) {
      const fd = new FormData(); fd.set('btn', btn); fd.set('down', down ? '1' : '0');
      try { await fetch('/api/barmode/btn', {method:'POST', body:fd}); } catch(e) {}
    }

    let vbtnFullscreen = false;
    function setVbtnFull(full) {
      vbtnFullscreen = full;
      const vp = document.getElementById('vbtn-panel');
      const vs = document.getElementById('vbtn-spacer');
      const tb = document.getElementById('vbtn-toggle');
      if (vp) vp.classList.toggle('vbtn-full', full);
      if (vs) vs.style.display = (barmodeActive && !full) ? '' : 'none';
      if (tb) tb.textContent = full ? '\u292B' : '\u26F6';  // ⤫ collapse / ⛶ expand
      try { localStorage.setItem('vbtnFull', full ? '1' : '0'); } catch(e) {}
    }
    function toggleVbtnFull() { setVbtnFull(!vbtnFullscreen); }

    document.querySelectorAll('.vbtn').forEach(el => {
      const i = parseInt(el.dataset.btn);
      const press   = () => { if (!webBtnDown[i]) { webBtnDown[i] = true;  sendVBtn(i, true);  } };
      const release = () => { if (webBtnDown[i])  { webBtnDown[i] = false; sendVBtn(i, false); } };
      el.addEventListener('mousedown',  press);
      el.addEventListener('touchstart', e => { e.preventDefault(); press(); }, {passive:false});
      el.addEventListener('mouseup',    release);
      el.addEventListener('mouseleave', release);
      el.addEventListener('touchend',   release);
      el.addEventListener('touchcancel',release);
    });

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

    // Mesh relay button
    document.getElementById('relay-btn').addEventListener('click', async () => {
      const addr = document.getElementById('relay-addr').value.trim();
      const arg  = document.getElementById('relay-arg').value.trim();
      const statusEl = document.getElementById('relay-status');
      if (!addr) { statusEl.textContent = 'addr required'; statusEl.style.color = '#f44336'; return; }
      statusEl.textContent = 'Sending\u2026'; statusEl.style.color = 'var(--muted)';
      try {
        const res = await fetchJson('/api/mesh/relay', {
          method: 'POST',
          headers: {'Content-Type': 'application/x-www-form-urlencoded'},
          body: 'addr=' + encodeURIComponent(addr) + '&arg=' + encodeURIComponent(arg || '0')
        });
        if (res.ok) {
          statusEl.textContent = '\u2713 sent to ' + res.peers + ' peer' + (res.peers === 1 ? '' : 's');
          statusEl.style.color = '#4caf50';
        } else {
          statusEl.textContent = 'Error: ' + (res.error || 'unknown');
          statusEl.style.color = '#f44336';
        }
      } catch(e) { statusEl.textContent = 'Request failed'; statusEl.style.color = '#f44336'; }
      setTimeout(() => { statusEl.textContent = ''; }, 4000);
    });

    refreshTelemetry();
    refreshLogs();
    refreshCharts();
    refreshRegistry();
    refreshManualPylons();
    refreshBtnEvents();
    setInterval(refreshTelemetry, 1000);
    setInterval(refreshLogs, 1500);
    setInterval(refreshCharts, 15000);
    setInterval(refreshRegistry, 10000);
    setInterval(refreshManualPylons, 5000);
    setInterval(refreshBtnEvents, 10000);
    setInterval(drawBtnChart, 5000);
    async function refreshMeshRemotes(){
      try{
        const data=await fetchJson('/api/mesh_remotes');
        const panel=document.getElementById('mesh-remotes-panel');
        const content=document.getElementById('mesh-remotes-content');
        if(!barmodeActive){panel.style.display='none';return;}
        panel.style.display='';
        if(!data.length){content.innerHTML='<span style="color:var(--muted);font-size:13px">No remotes seen yet.</span>';return;}
        let h='<table style="border-collapse:collapse;font-size:13px;width:100%"><thead><tr style="border-bottom:1px solid var(--border)">'
          +'<th style="padding:4px 8px;text-align:left">Remote ID</th>'
          +'<th style="padding:4px 8px;text-align:left">Description</th>'
          +'<th style="padding:4px 8px;text-align:left">Host / IP</th>'
          +'<th style="padding:4px 8px;text-align:left">MAC</th>'
          +'<th style="padding:4px 8px;text-align:right">Mode</th>'
          +'<th style="padding:4px 8px;text-align:right">RSSI</th>'
          +'<th style="padding:4px 8px;text-align:right">Red</th>'
          +'<th style="padding:4px 8px;text-align:right">Yellow</th>'
          +'<th style="padding:4px 8px;text-align:right">Uptime</th>'
          +'<th style="padding:4px 8px;text-align:right">Last seen</th>'
          +'<th style="padding:4px 8px;text-align:left">Via</th>'
          +'</tr></thead><tbody>';
        for(const r of data){
          const op=r.stale?'opacity:0.5;':'';
          const mode=r.mode===0?'WiFi':'Mesh';
          const upH=Math.floor(r.uptime_s/3600),upM=Math.floor((r.uptime_s%3600)/60),upS=r.uptime_s%60;
          const upStr=(upH?upH+'h ':'')+(upM?upM+'m ':'')+upS+'s';
          const hnLink=r.hostname?`<a href="http://${r.hostname}.local" target="_blank">${r.hostname}.local</a>`:'';
          const ipLink=r.ip?`<a href="http://${r.ip}" target="_blank">${r.ip}</a>`:'';
          const hostCell=hnLink&&ipLink?`${hnLink}<br><span style="font-size:11px;color:var(--muted)">${ipLink}</span>`:hnLink||ipLink||'&mdash;';
          h+=`<tr style="border-bottom:1px solid var(--border);${op}">`
            +`<td style="padding:4px 8px">${r.remote_id}</td>`
            +`<td style="padding:4px 8px">${r.description}</td>`
            +`<td style="padding:4px 8px">${hostCell}</td>`
            +`<td style="padding:4px 8px;font-size:11px">${r.mac}</td>`
            +`<td style="padding:4px 8px;text-align:right">${mode}</td>`
            +`<td style="padding:4px 8px;text-align:right">${r.rssi} dBm</td>`
            +`<td style="padding:4px 8px;text-align:right">${r.press_red}</td>`
            +`<td style="padding:4px 8px;text-align:right">${r.press_yellow}</td>`
            +`<td style="padding:4px 8px;text-align:right">${upStr}</td>`
            +`<td style="padding:4px 8px;text-align:right">${r.last_seen_s}s ago</td>`
            +`<td style="padding:4px 8px">${r.forwarded_by}</td>`
            +'</tr>';
        }
        h+='</tbody></table>';
        content.innerHTML=h;
      }catch(e){}
    }
    setInterval(refreshMeshRemotes, 10000);
    refreshMeshRemotes();
  </script>
</body>
</html>
)HTML";

void SendOscFloatToAllPylons(const char *addr, float value);  // defined later

void HandleDjBtn() {
  const bool down = webServer.arg("down") == "1";
  if (down) {
    if (barmode_active) SendOscFloatToAllPylons(kOscAddress, 1.0f);
    ApplyBooshState(1.0f, "dj");
    dj_btn_held     = true;
    dj_btn_press_ms = millis();
  } else {
    if (barmode_active) SendOscFloatToAllPylons(kOscAddress, 0.0f);
    ApplyBooshState(0.0f, "dj");
    dj_btn_held = false;
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void HandleDjPage() {
  char buf[80];
  snprintf(buf, sizeof(buf), "%.1f", cfg_dj_timeout_s);
  String html = F(
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>"
    "<title>Lava DJ Access</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{background:#111;color:#fff;font-family:system-ui,sans-serif;height:100dvh;display:flex;flex-direction:column;align-items:center;justify-content:center;overflow:hidden;user-select:none;-webkit-user-select:none}"
    "h1{font-size:clamp(18px,4vw,28px);letter-spacing:.08em;color:#ff4444;margin-bottom:8px;text-transform:uppercase}"
    "#btn{width:min(80vw,80vh);height:min(80vw,80vh);border-radius:50%;background:radial-gradient(circle at 35% 35%,#ff6666,#cc0000 60%,#660000);display:flex;flex-direction:column;align-items:center;justify-content:center;cursor:pointer;touch-action:none;box-shadow:0 0 40px #c0000060;transition:transform .08s,box-shadow .08s;border:4px solid #ff444430;font-size:clamp(16px,5vw,28px);font-weight:700;color:#fff;letter-spacing:.06em;gap:12px}"
    "#btn.held{transform:scale(.93);box-shadow:0 0 80px #ff4444cc,0 0 160px #ff444460;border-color:#ff4444}"
    "#countdown{font-size:clamp(14px,3.5vw,22px);color:#ffaaaa;margin-top:16px;height:1.4em}"
    "</style></head><body>"
    "<h1>Lava DJ Access</h1>"
    "<div id='btn'><div>&#x1F525;</div><div>HOLD TO FIRE</div></div>"
    "<div id='countdown'></div>"
    "<script>"
    "const TIMEOUT_S=");
  html += buf;
  html += F(
    ";"
    "const btn=document.getElementById('btn');"
    "const cd=document.getElementById('countdown');"
    "let held=false,timer=null,cdTimer=null,secsLeft=0;"
    "function startHold(){"
    "  if(held)return; held=true;"
    "  btn.classList.add('held');"
    "  secsLeft=TIMEOUT_S;"
    "  cd.textContent='';"
    "  send(1);"
    "  timer=setTimeout(()=>{ endHold(true); },TIMEOUT_S*1000);"
    "  cdTimer=setInterval(()=>{ secsLeft=Math.max(0,secsLeft-1); if(secsLeft<=5&&secsLeft>0)cd.textContent='Auto-close in '+secsLeft+'s'; else if(secsLeft===0)cd.textContent=''; },1000);"
    "}"
    "function endHold(timedOut){"
    "  if(!held&&!timedOut)return; held=false;"
    "  btn.classList.remove('held');"
    "  clearTimeout(timer); clearInterval(cdTimer);"
    "  cd.textContent=timedOut?'Auto-closed':''; "
    "  if(timedOut)setTimeout(()=>cd.textContent='',2000);"
    "  send(0);"
    "}"
    "async function send(down){"
    "  try{"
    "    const fd=new FormData(); fd.set('down',down);"
    "    await fetch('/api/dj/btn',{method:'POST',body:fd});"
    "  }catch(e){}"
    "}"
    "btn.addEventListener('mousedown',startHold);"
    "btn.addEventListener('touchstart',e=>{e.preventDefault();startHold();},{passive:false});"
    "document.addEventListener('mouseup',()=>endHold(false));"
    "document.addEventListener('touchend',()=>endHold(false));"
    "document.addEventListener('touchcancel',()=>endHold(false));"
    "document.addEventListener('visibilitychange',()=>{ if(document.hidden)endHold(false); });"
    "</script></body></html>");
  webServer.send(200, "text/html", html);
}

void HandleWebRoot() {
  webServer.send(200, "text/html", kWebUiHtml);
}

void HandleMeshRemotesApi() {
  const uint32_t now_ms = (uint32_t)millis();
  String out = "[";
  bool first = true;
  if (xSemaphoreTake(mesh_remote_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (int i = 0; i < kMeshRemoteSlots; i++) {
      const MeshRemoteRecord &r = mesh_remote_table[i];
      if (!r.active) continue;
      const uint32_t age_ms = now_ms - r.last_seen_ms;
      if (age_ms > kMeshRemoteExpireMs) continue;
      if (!first) out += ',';
      first = false;
      out += "{\"remote_id\":\"";   out += r.remote_id;
      out += "\",\"description\":\""; out += r.description;
      out += "\",\"mac\":\"";       out += r.mac;
      out += "\",\"forwarded_by\":\""; out += r.forwarded_by;
      out += "\",\"uptime_s\":";    out += r.uptime_s;
      out += ",\"press_red\":";     out += r.press_red;
      out += ",\"press_yellow\":";  out += r.press_yellow;
      out += ",\"mode\":";          out += r.mode;
      out += ",\"rssi\":";          out += r.rssi;
      out += ",\"last_seen_s\":";   out += (age_ms / 1000);
      out += ",\"stale\":";         out += (age_ms > kMeshRemoteStaleMs ? "true" : "false");
      out += ",\"hostname\":\"";    out += r.hostname;
      out += "\",\"ip\":\"";        out += r.ip;
      out += "\"}";
    }
    xSemaphoreGive(mesh_remote_mutex);
  }
  out += ']';
  webServer.send(200, "application/json", out);
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

void HandleManualPylonsGetApi() {
  String payload = "{\"entries\":[";
  for (int i = 0; i < barmode_manual_pylon_count; i++) {
    if (i > 0) payload += ',';
    payload += "{\"i\":" + String(i) + ",";
    payload += "\"host\":\"" + JsonEscape(String(barmode_manual_pylons[i].host)) + "\",";
    payload += "\"index\":" + String(barmode_manual_pylons[i].index) + ",";
    payload += "\"ip\":\"" + (barmode_manual_pylons[i].resolved ? barmode_manual_pylons[i].ip.toString() : String("")) + "\",";
    payload += "\"resolved\":" + String(barmode_manual_pylons[i].resolved ? "true" : "false") + "}";
  }
  payload += "],\"max\":" + String(kManualPylonMax) + "}";
  webServer.send(200, "application/json", payload);
}

void HandleManualPylonsPostApi() {
  const String action = webServer.arg("action");
  if (action == "add") {
    String host = webServer.arg("host");
    host.trim();
    if (host.length() == 0 || host.length() >= 64) {
      SendApiError(400, "host required, max 63 chars"); return;
    }
    if (barmode_manual_pylon_count >= kManualPylonMax) {
      SendApiError(400, "max manual pylons reached (" + String(kManualPylonMax) + ")"); return;
    }
    // Reject duplicate hosts
    for (int i = 0; i < barmode_manual_pylon_count; i++) {
      if (String(barmode_manual_pylons[i].host).equalsIgnoreCase(host)) {
        SendApiError(400, "host already in list"); return;
      }
    }
    const int idx = (int)webServer.arg("index").toInt();
    ManualPylon &mp = barmode_manual_pylons[barmode_manual_pylon_count++];
    strncpy(mp.host, host.c_str(), 63); mp.host[63] = '\0';
    mp.index = idx;
    mp.resolved = false;
    mp.last_resolve_ms = 0;
    // Try immediate parse for dotted IPs
    IPAddress addr;
    if (addr.fromString(host)) { mp.ip = addr; mp.resolved = true; }
    SaveManualPylons();
    Console.printf("[Manual] Added: %s idx=%d\n", mp.host, mp.index);
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else if (action == "remove") {
    const int slot = (int)webServer.arg("i").toInt();
    if (slot < 0 || slot >= barmode_manual_pylon_count) {
      SendApiError(400, "invalid slot"); return;
    }
    Console.printf("[Manual] Removed: %s\n", barmode_manual_pylons[slot].host);
    for (int i = slot; i < barmode_manual_pylon_count - 1; i++)
      barmode_manual_pylons[i] = barmode_manual_pylons[i + 1];
    barmode_manual_pylon_count--;
    SaveManualPylons();
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else {
    SendApiError(400, "action must be add or remove");
  }
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
  const bool has_seq_max_s    = webServer.hasArg("seq_max_s");
  const bool has_seq_start_ms = webServer.hasArg("seq_start_ms");
  const bool has_seq_dec_ms   = webServer.hasArg("seq_dec_ms");
  const bool has_seq_floor_ms = webServer.hasArg("seq_floor_ms");
  const bool has_seq_exp_pct  = webServer.hasArg("seq_exp_pct");
  const bool has_btn_disabled    = webServer.hasArg("btn_disabled");
  const bool has_green_timeout   = webServer.hasArg("green_timeout_ms");
  const bool has_all4_valve_ms   = webServer.hasArg("all4_valve_ms");
  const bool has_all4_lockout_s  = webServer.hasArg("all4_lockout_s");
  const bool has_red_seq_max_s   = webServer.hasArg("red_seq_max_s");
  const bool has_red_seq_valve_ms = webServer.hasArg("red_seq_valve_ms");
  const bool has_red_seq_step_ms  = webServer.hasArg("red_seq_step_ms");
  const bool has_pulse1_dur_ms   = webServer.hasArg("pulse1_dur_ms");
  const bool has_pulse1_dis      = webServer.hasArg("pulse1_dis");
  const bool has_pt_dur_ms       = webServer.hasArg("pt_dur_ms");
  const bool has_pt_off_ms       = webServer.hasArg("pt_off_ms");
  const bool has_pt_count        = webServer.hasArg("pt_count");
  const bool has_pt_dis          = webServer.hasArg("pt_dis");
  const bool has_steam_ramp_ms   = webServer.hasArg("steam_ramp_ms");
  const bool has_steam_open_ms   = webServer.hasArg("steam_open_ms");
  const bool has_steam_dis       = webServer.hasArg("steam_dis");
  const bool has_green_rec_ms    = webServer.hasArg("green_recovery_ms");
  const bool has_blue_rec_ms     = webServer.hasArg("blue_recovery_ms");
  const bool has_orange_rec_ms   = webServer.hasArg("orange_recovery_ms");
  const bool has_red_rec_ms      = webServer.hasArg("red_recovery_ms");
  const bool has_temp_thresh1    = webServer.hasArg("temp_thresh1_f");
  const bool has_temp_mult1      = webServer.hasArg("temp_mult1");
  const bool has_temp_thresh2    = webServer.hasArg("temp_thresh2_f");
  const bool has_temp_mult2      = webServer.hasArg("temp_mult2");
  const bool has_no_thermistor   = webServer.hasArg("no_thermistor");
  const bool has_no_batt_mon     = webServer.hasArg("no_batt_mon");
  const bool has_route_via_rpi   = webServer.hasArg("route_via_rpi");
  const bool has_grp_pat_en      = webServer.hasArg("grp_pat_en");
  const bool has_grp_win_ms      = webServer.hasArg("grp_win_ms");
  const bool has_grp_cool_ms     = webServer.hasArg("grp_cool_ms");
  const bool has_grp_qon_ms      = webServer.hasArg("grp_qon_ms");
  const bool has_grp_qoff_ms     = webServer.hasArg("grp_qoff_ms");
  const bool has_grp_big_ms      = webServer.hasArg("grp_big_ms");
  const bool has_grp_gap_ms      = webServer.hasArg("grp_gap_ms");
  const bool has_mesh_en         = webServer.hasArg("mesh_en");
  const bool has_mesh_ch         = webServer.hasArg("mesh_ch");
  const bool has_dj_timeout_s    = webServer.hasArg("dj_timeout_s");

  if (!has_node && !has_id && !has_host && !has_desc &&
      !has_wifi_ssid && !has_wifi_pass && !has_failsafe_s && !has_index && !has_seq_max_s && !has_seq_start_ms && !has_seq_dec_ms && !has_seq_floor_ms && !has_seq_exp_pct && !has_btn_disabled && !has_green_timeout && !has_all4_valve_ms && !has_all4_lockout_s &&
      !has_red_seq_max_s && !has_red_seq_valve_ms && !has_red_seq_step_ms &&
      !has_pulse1_dur_ms && !has_pulse1_dis && !has_pt_dur_ms && !has_pt_off_ms && !has_pt_count && !has_pt_dis &&
      !has_steam_ramp_ms && !has_steam_open_ms && !has_steam_dis &&
      !has_green_rec_ms && !has_blue_rec_ms && !has_orange_rec_ms && !has_red_rec_ms &&
      !has_temp_thresh1 && !has_temp_mult1 && !has_temp_thresh2 && !has_temp_mult2 &&
      !has_no_thermistor && !has_no_batt_mon && !has_route_via_rpi &&
      !has_grp_pat_en && !has_grp_win_ms && !has_grp_cool_ms && !has_grp_qon_ms &&
      !has_grp_qoff_ms && !has_grp_big_ms && !has_grp_gap_ms &&
      !has_mesh_en && !has_mesh_ch && !has_dj_timeout_s) {
    SendApiError(400, "no recognized config field");
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
  if (has_seq_max_s)    ok = ok && SetConfigFieldValue("seq_max_s",    webServer.arg("seq_max_s"));
  if (has_seq_start_ms) ok = ok && SetConfigFieldValue("seq_start_ms", webServer.arg("seq_start_ms"));
  if (has_seq_dec_ms)   ok = ok && SetConfigFieldValue("seq_dec_ms",   webServer.arg("seq_dec_ms"));
  if (has_seq_floor_ms) ok = ok && SetConfigFieldValue("seq_floor_ms", webServer.arg("seq_floor_ms"));
  if (has_seq_exp_pct)  ok = ok && SetConfigFieldValue("seq_exp_pct",  webServer.arg("seq_exp_pct"));
  if (has_green_timeout) ok = ok && SetConfigFieldValue("green_timeout_ms", webServer.arg("green_timeout_ms"));
  if (has_all4_valve_ms)  ok = ok && SetConfigFieldValue("all4_valve_ms",   webServer.arg("all4_valve_ms"));
  if (has_all4_lockout_s) ok = ok && SetConfigFieldValue("all4_lockout_s",  webServer.arg("all4_lockout_s"));
  if (has_red_seq_max_s)   ok = ok && SetConfigFieldValue("red_seq_max_s",   webServer.arg("red_seq_max_s"));
  if (has_red_seq_valve_ms) ok = ok && SetConfigFieldValue("red_seq_valve_ms", webServer.arg("red_seq_valve_ms"));
  if (has_red_seq_step_ms)  ok = ok && SetConfigFieldValue("red_seq_step_ms",  webServer.arg("red_seq_step_ms"));
  if (has_pulse1_dur_ms) ok = ok && SetConfigFieldValue("pulse1_dur_ms", webServer.arg("pulse1_dur_ms"));
  if (has_pulse1_dis)    ok = ok && SetConfigFieldValue("pulse1_dis",    webServer.arg("pulse1_dis"));
  if (has_pt_dur_ms)     ok = ok && SetConfigFieldValue("pt_dur_ms",     webServer.arg("pt_dur_ms"));
  if (has_pt_off_ms)     ok = ok && SetConfigFieldValue("pt_off_ms",     webServer.arg("pt_off_ms"));
  if (has_pt_count)      ok = ok && SetConfigFieldValue("pt_count",      webServer.arg("pt_count"));
  if (has_pt_dis)        ok = ok && SetConfigFieldValue("pt_dis",        webServer.arg("pt_dis"));
  if (has_steam_ramp_ms) ok = ok && SetConfigFieldValue("steam_ramp_ms", webServer.arg("steam_ramp_ms"));
  if (has_steam_open_ms) ok = ok && SetConfigFieldValue("steam_open_ms", webServer.arg("steam_open_ms"));
  if (has_steam_dis)     ok = ok && SetConfigFieldValue("steam_dis",     webServer.arg("steam_dis"));
  if (has_green_rec_ms)  ok = ok && SetConfigFieldValue("green_recovery_ms",  webServer.arg("green_recovery_ms"));
  if (has_blue_rec_ms)   ok = ok && SetConfigFieldValue("blue_recovery_ms",   webServer.arg("blue_recovery_ms"));
  if (has_orange_rec_ms) ok = ok && SetConfigFieldValue("orange_recovery_ms", webServer.arg("orange_recovery_ms"));
  if (has_red_rec_ms)    ok = ok && SetConfigFieldValue("red_recovery_ms",    webServer.arg("red_recovery_ms"));
  if (has_temp_thresh1)  ok = ok && SetConfigFieldValue("temp_thresh1_f",     webServer.arg("temp_thresh1_f"));
  if (has_temp_mult1)    ok = ok && SetConfigFieldValue("temp_mult1",         webServer.arg("temp_mult1"));
  if (has_temp_thresh2)  ok = ok && SetConfigFieldValue("temp_thresh2_f",     webServer.arg("temp_thresh2_f"));
  if (has_temp_mult2)    ok = ok && SetConfigFieldValue("temp_mult2",         webServer.arg("temp_mult2"));
  if (has_no_thermistor) ok = ok && SetConfigFieldValue("no_thermistor",      webServer.arg("no_thermistor"));
  if (has_no_batt_mon)   ok = ok && SetConfigFieldValue("no_batt_mon",        webServer.arg("no_batt_mon"));
  if (has_route_via_rpi) ok = ok && SetConfigFieldValue("route_via_rpi", webServer.arg("route_via_rpi"));
  if (has_grp_pat_en)    ok = ok && SetConfigFieldValue("grp_pat_en",    webServer.arg("grp_pat_en"));
  if (has_grp_win_ms)    ok = ok && SetConfigFieldValue("grp_win_ms",    webServer.arg("grp_win_ms"));
  if (has_grp_cool_ms)   ok = ok && SetConfigFieldValue("grp_cool_ms",   webServer.arg("grp_cool_ms"));
  if (has_grp_qon_ms)    ok = ok && SetConfigFieldValue("grp_qon_ms",    webServer.arg("grp_qon_ms"));
  if (has_grp_qoff_ms)   ok = ok && SetConfigFieldValue("grp_qoff_ms",   webServer.arg("grp_qoff_ms"));
  if (has_grp_big_ms)    ok = ok && SetConfigFieldValue("grp_big_ms",    webServer.arg("grp_big_ms"));
  if (has_grp_gap_ms)    ok = ok && SetConfigFieldValue("grp_gap_ms",    webServer.arg("grp_gap_ms"));
  if (has_mesh_en)       ok = ok && SetConfigFieldValue("mesh_en",        webServer.arg("mesh_en"));
  if (has_mesh_ch)       ok = ok && SetConfigFieldValue("mesh_ch",            webServer.arg("mesh_ch"));
  if (has_dj_timeout_s)  ok = ok && SetConfigFieldValue("dj_timeout_s",       webServer.arg("dj_timeout_s"));
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
void PingTask(void *);      // forward declaration
void TempPollTask(void *);  // forward declaration
void StartSequence(SeqType type);  // forward declaration
void AbortSequence();              // forward declaration
void MeshBroadcastOsc(const char *osc_addr, float osc_arg);          // forward declaration
void MeshBroadcastChanChange(uint8_t new_ch, uint32_t delay_ms);    // forward declaration
void MeshUnicastOsc(const uint8_t *mac, const char *osc_addr, float osc_arg);  // forward declaration
void MeshInit();  // forward declaration

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
  const String ssid = "FIRE_PYLON_" + pylon_id;
  const bool sta_connected = WiFi.status() == WL_CONNECTED;
  // AP channel strategy:
  // - AP+STA mode: radio is shared; AP channel must match the STA channel.
  //   Read the live STA channel before the mode switch.
  // - AP-only mode: use mesh channel so ESP-NOW stays on the same channel.
  //   Default regulatory domain allows ch 1-11 only; cap at 11 to prevent
  //   softAP from failing silently and falling back to the ESP default SSID.
  uint8_t ap_ch;
  if (sta_connected) {
    ap_ch = (uint8_t)WiFi.channel();
    if (ap_ch == 0) ap_ch = 1;
  } else {
    const uint8_t mesh_ch = cfg_mesh_en ? (uint8_t)cfg_mesh_ch : 1;
    ap_ch = (mesh_ch >= 1 && mesh_ch <= 11) ? mesh_ch : 1;
  }
  if (sta_connected) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  WiFi.softAPConfig(IPAddress(10, 1, 2, 3), IPAddress(10, 1, 2, 3), IPAddress(255, 255, 255, 0));
  bool ap_ok = WiFi.softAP(ssid.c_str(), nullptr, ap_ch);
  if (!ap_ok) {
    Console.printf("[AP] softAP ch=%u failed, retrying ch=1\n", ap_ch);
    ap_ok = WiFi.softAP(ssid.c_str(), nullptr, 1);
  }
  delay(100);
  // WiFi mode change tears down and restarts the WiFi driver, which invalidates
  // the esp_now_init() done at boot. Re-init ESP-NOW so mesh keeps working.
  // Guard: only re-init if mesh was already initialised (runtime mode change).
  // At boot, setup() calls MeshInit() explicitly after SetupApMode() returns.
  if (cfg_mesh_en && mesh_initialized) MeshInit();
  dnsServer.start(53, "*", IPAddress(10, 1, 2, 3));
  ap_active = true;
  Console.printf("[AP] started SSID: %s ch=%u ok=%d\n", ssid.c_str(), ap_ch, (int)ap_ok);
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
  // WiFi mode change restarts the driver; re-init ESP-NOW so mesh keeps working.
  // Guard: only re-init if mesh was already initialised (runtime mode change).
  if (cfg_mesh_en && mesh_initialized) MeshInit();
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

void HandleBarmodeBtn() {
  if (!barmode_active) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"not in barmode\"}");
    return;
  }
  const int btn  = webServer.arg("btn").toInt();
  const bool down = webServer.arg("down") == "1";
  if (btn < 0 || btn > 3) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"invalid btn 0-3\"}");
    return;
  }
  web_btn_pressed[btn] = down;
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// POST /api/mesh/relay  params: addr=<osc_addr>&arg=<float>
// Lets any non-ESP-NOW node (e.g. rpiboosh) relay an OSC command onto the mesh.
// Also applies the command locally on this node (BARBAR fires too).
void HandleMeshRelayApi() {
  if (!cfg_mesh_en || !mesh_initialized) {
    webServer.send(503, "application/json", "{\"ok\":false,\"error\":\"mesh not active\"}");
    return;
  }
  const String addr  = webServer.arg("addr");
  const String arg_s = webServer.arg("arg");
  if (addr.length() == 0) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing addr\"}");
    return;
  }
  const float arg_f = arg_s.length() ? arg_s.toFloat() : 0.0f;
  int peer_count = 0;
  if (mesh_peers_mutex && xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < kMeshMaxPeers; i++) if (mesh_peers[i].active) peer_count++;
    xSemaphoreGive(mesh_peers_mutex);
  }
  MeshBroadcastOsc(addr.c_str(), arg_f);
  // Apply locally so this node (e.g. BARBAR) also fires
  if (addr == kOscAddress) {
    if (group_pattern_active && arg_f >= 0.5f) Console.println("[GroupPat] suppressed relay open");
    else ApplyBooshState(arg_f, "mesh-relay");
  } else if (addr == kOscAddrPulseSingle && arg_f >= 0.5f) {
    if (!action_pulse1_dis) StartSequence(SEQ_PULSE_ONCE);
  } else if (addr == kOscAddrPulseTrain && arg_f >= 0.5f) {
    if (!action_pt_dis) StartSequence(SEQ_PULSE_5X);
  } else if (addr == kOscAddrSteam) {
    if (arg_f >= 0.5f) { if (!action_steam_dis) StartSequence(SEQ_STEAM); }
    else AbortSequence();
  }
  Console.printf("[Mesh/relay] %s %.3f -> %d peers + local\n", addr.c_str(), arg_f, peer_count);
  String resp = "{\"ok\":true,\"addr\":\"";
  resp += addr;
  resp += "\",\"arg\":";
  resp += String(arg_f, 3);
  resp += ",\"peers\":";
  resp += String(peer_count);
  resp += "}";
  webServer.send(200, "application/json", resp);
}

// POST /api/mesh/chanchange  param: ch=<1-13>
// Broadcasts a channel-change packet to all mesh peers with a 5 s countdown,
// then applies the change locally after the same delay. Fallback: if mesh is
// not active, saves cfg_mesh_ch locally only (single-node use).
void HandleMeshChanChangeApi() {
  const int ch = webServer.arg("ch").toInt();
  if (ch < 1 || ch > 13) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"ch must be 1-13\"}");
    return;
  }
  const uint32_t delay_ms = 5000;
  if (cfg_mesh_en && mesh_initialized) {
    MeshBroadcastChanChange((uint8_t)ch, delay_ms);
    webServer.send(200, "application/json",
      "{\"ok\":true,\"ch\":" + String(ch) + ",\"delay_ms\":" + String(delay_ms) + ",\"broadcast\":true}");
  } else {
    // Mesh not active — apply locally only, no countdown
    SetConfigFieldValue("mesh_ch", String(ch));
    SavePylonConfig();
    webServer.send(200, "application/json",
      "{\"ok\":true,\"ch\":" + String(ch) + ",\"delay_ms\":0,\"broadcast\":false}");
  }
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
    // Use actual file size when available; UPDATE_SIZE_UNKNOWN can trigger a false
    // UPDATE_ERROR_SPACE on some nodes because it sets size = full partition size
    // and then checks ESP.getFreeSketchSpace() < that value.
    const size_t ota_size = (upload.totalSize > 0) ? upload.totalSize : UPDATE_SIZE_UNKNOWN;
    if (!Update.begin(ota_size)) {
      Console.printf("[OTA] begin() failed: %s (size=%u)\n", Update.errorString(), ota_size);
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
  webServer.on("/api/pylons/manual", HTTP_GET,  HandleManualPylonsGetApi);
  webServer.on("/api/pylons/manual", HTTP_POST, HandleManualPylonsPostApi);
  webServer.on("/api/events",   HTTP_GET, HandleBtnEventsApi);
  webServer.on("/api/ota", HTTP_POST, HandleOtaApi, HandleOtaUploadBody);
  webServer.on("/api/identify", HTTP_POST, HandleIdentifyApi);
  webServer.on("/api/sequence/pulse_once", HTTP_POST, HandleSeqPulseOnceApi);
  webServer.on("/api/sequence/pulse_5x", HTTP_POST, HandleSeqPulse5xApi);
  webServer.on("/api/sequence/steam", HTTP_POST, HandleSeqSteamApi);
  webServer.on("/api/sequence/abort", HTTP_POST, HandleSeqAbortApi);
  webServer.on("/api/mesh/relay",     HTTP_POST, HandleMeshRelayApi);
  webServer.on("/api/mesh/chanchange", HTTP_POST, HandleMeshChanChangeApi);
  webServer.on("/api/barmode/btn",    HTTP_POST, HandleBarmodeBtn);
  webServer.on("/api/mesh_remotes",   HTTP_GET,  HandleMeshRemotesApi);
  webServer.on("/dj",                 HTTP_GET,  HandleDjPage);
  webServer.on("/api/dj/btn",         HTTP_POST, HandleDjBtn);
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
  LoadManualPylons();
  barmode_registry_mutex = xSemaphoreCreateMutex();
  barmode_ping_stats_mutex = xSemaphoreCreateMutex();
  osc_proxy_queue = xQueueCreate(kOscProxyQueueDepth, sizeof(OscProxyMsg));
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
        Console.printf("[WiFi] GOT_IP — auto_ap=%d active=%d\n", ap_auto_enabled, ap_active);
        // If AP was auto-started because WiFi was unavailable, tear it down now
        // that STA has an IP. Manually-enabled AP (ap_auto_enabled==false) is left alone.
        if (ap_auto_enabled && ap_active) {
          ap_auto_enabled = false;
          ap_enabled = false;
          // StopApMode() touches WiFi mode — call from main loop via flag instead.
          // We just clear the flags here; the main loop !ap_enabled && ap_active
          // branch will call StopApMode() on the next iteration.
        }
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        last_disconnect_reason = info.wifi_sta_disconnected.reason;
        wifi_connected_since_ms = 0;
        wifi_has_ip = false;
        digitalWrite(kIo38Pin, LOW);
        Console.printf("[WiFi] DISCONNECTED reason=%u ap_active=%d\n",
                       info.wifi_sta_disconnected.reason, ap_active);
        // Pin radio to mesh channel when STA-only mode loses WiFi.
        // In AP or APSTA mode the AP interface owns the channel; changing the
        // STA channel is a no-op. When AP is active all nodes stay on whatever
        // channel they were last on (LavaLounge's channel), so they all remain
        // on the same channel and ESP-NOW continues to work — no action needed.
        if (cfg_mesh_en && !ap_active) {
          esp_wifi_set_channel((uint8_t)cfg_mesh_ch, WIFI_SECOND_CHAN_NONE);
        }
        break;
      default:
        break;
    }
  });

  LogBootStep("Boot: WiFi STA mode");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // Disable auto-reconnect when mesh is active: background reconnect scans
  // change the WiFi channel asynchronously, disrupting ESP-NOW. The manual
  // reconnect watchdog in loop() handles recovery instead.
  WiFi.setAutoReconnect(!cfg_mesh_en);
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

  // Apply static IP if configured (must be done before WiFi.begin)
  if (!cfg_use_dhcp && cfg_static_ip.length() > 0 && cfg_static_gw.length() > 0) {
    IPAddress ip, gw, subnet(255, 255, 255, 0), dns1, dns2;
    if (ip.fromString(cfg_static_ip) && gw.fromString(cfg_static_gw)) {
      if (cfg_static_dns1.length() > 0) dns1.fromString(cfg_static_dns1); else dns1 = gw;
      if (cfg_static_dns2.length() > 0) dns2.fromString(cfg_static_dns2); else dns2.fromString("8.8.8.8");
      WiFi.config(ip, gw, subnet, dns1, dns2);
      Console.printf("[WiFi] Static IP: %s  GW: %s  DNS: %s / %s\n",
                     ip.toString().c_str(), gw.toString().c_str(),
                     dns1.toString().c_str(), dns2.toString().c_str());
    } else {
      Console.println("[WiFi] Static IP config invalid — falling back to DHCP");
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
    ap_auto_enabled = true;  // remember this was automatic, not user-requested
  }

  if (ap_enabled) {
    SetupApMode();
  }

  // Initialise ESP-NOW mesh (no-op if cfg_mesh_en is false)
  MeshInit();

  // Ping and hostname resolution run in a background task so the main loop
  // never blocks on network I/O.
  xTaskCreatePinnedToCore(PingTask,     "ping",     4096, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(TempPollTask, "tmppoll",  4096, nullptr, 0, nullptr, 0);  // lowest priority
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
    if (group_pattern_active && v >= 0.5f) { Console.println("[GroupPat] suppressed OSC open"); return; }
    ApplyBooshState(v, "osc");
    // Mesh relay: barmode node forwards received UDP commands to ESP-NOW peers
    if (barmode_active) MeshBroadcastOsc(kOscAddress, v);
    return;
  }

  // /pylon/BooshPulseSingle  1.0 = trigger single pulse
  if (msg.fullMatch(kOscAddrPulseSingle)) {
    if (action_pulse1_dis) { Console.println("OSC BooshPulseSingle: disabled"); return; }
    if (msg.size() == 1 && msg.isFloat(0) && msg.getFloat(0) >= 0.5f) {
      Console.println("OSC BooshPulseSingle → SEQ_PULSE_ONCE");
      StartSequence(SEQ_PULSE_ONCE);
      if (barmode_active) MeshBroadcastOsc(kOscAddrPulseSingle, 1.0f);
    }
    return;
  }

  // /pylon/BooshPulseTrain  1.0 = trigger pulse train
  if (msg.fullMatch(kOscAddrPulseTrain)) {
    if (action_pt_dis) { Console.println("OSC BooshPulseTrain: disabled"); return; }
    if (msg.size() == 1 && msg.isFloat(0) && msg.getFloat(0) >= 0.5f) {
      Console.println("OSC BooshPulseTrain → SEQ_PULSE_5X");
      StartSequence(SEQ_PULSE_5X);
      if (barmode_active) MeshBroadcastOsc(kOscAddrPulseTrain, 1.0f);
    }
    return;
  }

  // /pylon/BooshSteam  1.0=start  0.0=stop
  if (msg.fullMatch(kOscAddrSteam)) {
    if (action_steam_dis && msg.size() == 1 && msg.isFloat(0) && msg.getFloat(0) >= 0.5f) {
      Console.println("OSC BooshSteam: disabled"); return;
    }
    if (msg.size() == 1 && msg.isFloat(0)) {
      const float sv = msg.getFloat(0);
      if (sv >= 0.5f) {
        Console.println("OSC BooshSteam 1.0 → SEQ_STEAM start");
        StartSequence(SEQ_STEAM);
      } else {
        Console.println("OSC BooshSteam 0.0 → abort");
        AbortSequence();
      }
      if (barmode_active) MeshBroadcastOsc(kOscAddrSteam, sv);
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
    boosh_failsafe_armed    = true;          // arm failsafe so a stalled loop still closes the valve
    boosh_failsafe_start_ms = millis();
  } else if (!active) {
    if (display_inverted && boosh_open_since_ms > 0) {
      total_boosh_open_ms += (uint32_t)(millis() - boosh_open_since_ms);
      boosh_open_since_ms = 0;
    }
    boosh_failsafe_armed = false;
  }
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
  if (type == SEQ_GROUP) {
    // Build flat step table: N quick pulses (66ms on/off each) then N big pulses (600*i ms, 300ms gap)
    const int n = max(2, min(group_seq_n_val, 8));
    group_seq_step_count = 0;
    group_seq_step_idx   = 0;
    group_pattern_active = true;
    for (int i = 0; i < n; i++) {
      group_seq_steps[group_seq_step_count++] = {cfg_grp_qon_ms,  true};
      group_seq_steps[group_seq_step_count++] = {cfg_grp_qoff_ms, false};
    }
    for (int i = 0; i < n; i++) {
      group_seq_steps[group_seq_step_count++] = {(uint16_t)(cfg_grp_big_ms * (i + 1)), true};
      if (i < n - 1) group_seq_steps[group_seq_step_count++] = {cfg_grp_gap_ms, false};
    }
    Console.printf("[SEQ] group start N=%d steps=%d\n", n, group_seq_step_count);
  }
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
    group_pattern_active = false;
    Console.println("[SEQ] aborted");
    return;
  }

  // --- PULSE_ONCE: single configurable-duration pulse ---
  if (active_seq == SEQ_PULSE_ONCE) {
    if (!seq_phase_on) {
      SeqSetSolenoid(true);
      seq_phase_on = true;
      seq_step_start_ms = now;
    } else if (now - seq_step_start_ms >= action_pulse1_dur_ms) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
    }
    return;
  }

  // --- PULSE_5X: N × (on_ms / off_ms), all configurable ---
  if (active_seq == SEQ_PULSE_5X) {
    if (seq_pulse_idx >= action_pt_count) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
      return;
    }
    if (!seq_phase_on) {
      // off phase (or initial start): skip wait on first pulse
      if (seq_pulse_idx == 0 || now - seq_step_start_ms >= action_pt_off_ms) {
        SeqSetSolenoid(true);
        seq_phase_on = true;
        seq_step_start_ms = now;
      }
    } else {
      if (now - seq_step_start_ms >= action_pt_dur_ms) {
        SeqSetSolenoid(false);
        seq_phase_on = false;
        seq_pulse_idx++;
        seq_step_start_ms = now;
      }
    }
    return;
  }

  // --- STEAM: exponential ramp 1→10 Hz over configurable ramp_s, then open for open_s ---
  if (active_seq == SEQ_STEAM) {
    const unsigned long ramp_ms = action_steam_ramp_ms;
    const unsigned long open_ms = action_steam_open_ms;
    if (elapsed >= ramp_ms + open_ms) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
      Console.println("[SEQ] steam done");
      return;
    }
    if (elapsed >= ramp_ms) {
      // Full-open phase
      if (!display_inverted) {
        SeqSetSolenoid(true);
        Console.println("[SEQ] steam full open");
      }
      return;
    }
    // Ramp phase: f(t) = 10^(t/ramp_s), 1 Hz → 10 Hz
    const float t_sec = elapsed / 1000.0f;
    const float freq_hz = powf(10.0f, t_sec / (action_steam_ramp_ms / 1000.0f));
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

  // --- GROUP: table-driven find-a-friend pattern ---
  if (active_seq == SEQ_GROUP) {
    if (group_seq_step_idx >= group_seq_step_count) {
      SeqSetSolenoid(false);
      active_seq = SEQ_NONE;
      group_pattern_active = false;
      Console.println("[SEQ] group done");
      return;
    }
    const GroupSeqStep &step = group_seq_steps[group_seq_step_idx];
    if (!seq_phase_on) {
      SeqSetSolenoid(step.on);
      seq_phase_on = true;
      seq_step_start_ms = now;
    } else if (now - seq_step_start_ms >= step.ms) {
      group_seq_step_idx++;
      seq_phase_on = false;
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

// Fast SOS pattern for yellow LED: ... --- ...
// Unit = 60 ms. dot=1u, dash=3u, elem-gap=1u, letter-gap=3u, word-gap=7u
static const uint16_t kSosFastMorseMs[] = {
  // S: dot dot dot
  60, 60,    // dot, elem-gap
  60, 60,    // dot, elem-gap
  60, 180,   // dot, letter-gap (3u)
  // O: dash dash dash
  180, 60,   // dash, elem-gap
  180, 60,   // dash, elem-gap
  180, 180,  // dash, letter-gap (3u)
  // S: dot dot dot
  60, 60,    // dot, elem-gap
  60, 60,    // dot, elem-gap
  60, 420,   // dot, word-gap (7u)
};
constexpr int kSosFastMorseMsCount = (int)(sizeof(kSosFastMorseMs) / sizeof(kSosFastMorseMs[0]));

// Drives the yellow LED (LEDC channel 2) with a fast SOS alarm pattern.
// Active when: mesh enabled, WiFi offline, zero live mesh peers.
// Barmode owns yellow for lamp output — suppressed there.
void PollSosYellowLed() {
  if (barmode_active) return;
  if (!cfg_mesh_en) return;
  if (wifi_has_ip) return;
  if (mesh_live_peer_count > 0) return;

  static int step = 0;
  static unsigned long step_start_ms = 0;
  const unsigned long now = millis();
  if (step_start_ms == 0) step_start_ms = now;

  if (now - step_start_ms >= kSosFastMorseMs[step]) {
    step = (step + 1) % kSosFastMorseMsCount;
    step_start_ms = now;
  }
  // Even step = ON (full brightness), odd step = OFF
  ledcWrite(2, (step % 2 == 0) ? 255 : 0);
}

// Pylon target with its sequence index for ordered sequential firing.
struct PylonTarget {
  IPAddress ip;
  int seq_idx;      // pylon_index value from registry / mesh peer
  bool via_mesh;    // true = reach this pylon via ESP-NOW unicast (no IP)
  uint8_t mesh_mac[6];
  bool is_self;     // true = fire local solenoid (barmode self-slot in sequential)
};

// Pending-close slot for red sequential overlap (step_ms < valve_ms → simultaneous opens)
struct RedSeqClose {
  bool          active;
  PylonTarget   target;
  unsigned long close_at;
};

// Low-level: build and send one OSC float packet to a single IP.
// When cfg_route_via_rpi is true, enqueues to osc_proxy_queue instead of sending UDP;
// PingTask (Core 0) drains the queue and forwards via POST /api/osc/send on rpiboosh.
void SendOscFloatToIP(const char *addr, float value, IPAddress dest) {
  if (cfg_route_via_rpi && osc_proxy_queue) {
    OscProxyMsg msg;
    strncpy(msg.address, addr, sizeof(msg.address) - 1);
    msg.address[sizeof(msg.address) - 1] = '\0';
    msg.value  = value;
    msg.ip_u32 = (uint32_t)dest;
    msg.port   = kOscPort;
    if (xQueueSend(osc_proxy_queue, &msg, 0) != pdTRUE) {
      Console.println("[OSCProxy] queue full, message dropped");
    }
    return;
  }
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
// Returns base_ms × current temp multiplier (rounded). Returns 0 if base_ms is 0.
unsigned long ApplyTempMult(unsigned long base_ms) {
  if (base_ms == 0) return 0;
  const float mult = barmode_temp_multiplier;
  if (mult <= 1.0f) return base_ms;
  return (unsigned long)(base_ms * mult + 0.5f);
}

int ExtractRegistryTargets(PylonTarget *dest, int maxCount) {
  if (!barmode_registry_mutex) return 0;  // not a barmode node
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

    dest[count].ip       = addr;
    dest[count].seq_idx  = seq_idx;
    dest[count].via_mesh = false;
    dest[count].is_self  = false;
    memset(dest[count].mesh_mac, 0, 6);
    count++;
  }
  // Append resolved manual pylons (skip if IP already in list)
  for (int i = 0; i < barmode_manual_pylon_count && count < maxCount; i++) {
    if (!barmode_manual_pylons[i].resolved) continue;
    const IPAddress &mp_ip = barmode_manual_pylons[i].ip;
    bool dup = false;
    for (int j = 0; j < count; j++) { if (dest[j].ip == mp_ip) { dup = true; break; } }
    if (!dup) {
      dest[count].ip       = mp_ip;
      dest[count].seq_idx  = barmode_manual_pylons[i].index;
      dest[count].via_mesh = false;
      dest[count].is_self  = false;
      memset(dest[count].mesh_mac, 0, 6);
      count++;
    }
  }
  // Append active mesh peers not already covered by pylon_index in the IP list.
  // Prefer the IP path when a peer is reachable both ways (rpiboosh online); fall back
  // to ESP-NOW unicast when rpiboosh is offline and the peer is mesh-only.
  if (cfg_mesh_en && mesh_peers_mutex &&
      xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < kMeshMaxPeers && count < maxCount; i++) {
      if (!mesh_peers[i].active) continue;
      const int pidx = (int)mesh_peers[i].pylon_index;
      bool dup = false;
      for (int j = 0; j < count; j++) {
        if (!dest[j].via_mesh && dest[j].seq_idx == pidx) { dup = true; break; }
      }
      if (!dup) {
        dest[count].ip       = IPAddress(0, 0, 0, 0);
        dest[count].seq_idx  = pidx;
        dest[count].via_mesh = true;
        dest[count].is_self  = false;
        memcpy(dest[count].mesh_mac, mesh_peers[i].mac, 6);
        count++;
      }
    }
    xSemaphoreGive(mesh_peers_mutex);
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
// Also appends resolved manual pylons (deduplicated by IP).
int ExtractRegistryIPs(IPAddress *dest, int maxCount) {
  if (!barmode_registry_mutex) return 0;  // not a barmode node
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
  // Append resolved manual pylons (skip if already in list)
  for (int i = 0; i < barmode_manual_pylon_count && count < maxCount; i++) {
    if (!barmode_manual_pylons[i].resolved) continue;
    const IPAddress &mp_ip = barmode_manual_pylons[i].ip;
    bool dup = false;
    for (int j = 0; j < count; j++) { if (dest[j] == mp_ip) { dup = true; break; } }
    if (!dup) dest[count++] = mp_ip;
  }
  return count;
}

// Sends an OSC message with a single float arg to all pylons in the cached registry.
// Skips silently if registry is empty (not yet fetched). Self is included via registry.
// Called from Core 1; oscUdp send path is non-blocking.
// Dispatch one OSC command to a single PylonTarget — IP or ESP-NOW unicast.
void SendOscToPylonTarget(const char *addr, float value, const PylonTarget &t) {
  if (t.is_self) {
    ApplyBooshState(value > 0.5f ? 1 : 0, "barmode-self");
  } else if (t.via_mesh) {
    MeshUnicastOsc(t.mesh_mac, addr, value);
  } else {
    SendOscFloatToIP(addr, value, t.ip);
  }
}

void SendOscFloatToAllPylons(const char *addr, float value) {
  PylonTarget targets[16];
  const int count = ExtractRegistryTargets(targets, 16);
  if (count == 0) {
    Console.printf("[BarMode] SendOsc %s: no targets (registry empty, no mesh peers)\n", addr);
    return;
  }
  for (int i = 0; i < count; i++) SendOscToPylonTarget(addr, value, targets[i]);
  Console.printf("[BarMode] SendOsc %s %.1f -> %d targets\n", addr, value, count);
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
    const bool raw = (digitalRead(kBarModeButtonPins[i]) == HIGH) || web_btn_pressed[i];
    if (raw != btn_last_raw[i]) {
      btn_last_raw[i] = raw;
      btn_change_ms[i] = now;
    }
    if (now - btn_change_ms[i] >= 20) {
      btn_stable[i] = raw;
    }
    barmode_btn_state[i] = btn_stable[i];  // expose for telemetry
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
    if (blue_falling && seq_phase == 0) { seq_blue_armed = false; barmode_show_wait_oled = false; }
    if (green_rising  && seq_phase == 1) { seq_green_press_ms  = now; seq_green_armed  = true; }
    if (orange_rising && seq_phase == 2) { seq_orange_press_ms = now; seq_orange_armed = true; }
    if (red_rising    && seq_phase == 3) { seq_red_press_ms    = now; seq_red_armed    = true; }

    // Phase advancement (checked every tick)
    if (seq_phase == 0 && seq_blue_armed && btn_stable[1] && now - seq_blue_press_ms >= 3000) {
      if (barmode_all4_lockout_s > 0 && now < barmode_all4_lockout_until_ms) {
        // locked out — show WAIT screen while blue is held
        barmode_show_wait_oled = true;
      } else {
        barmode_show_wait_oled = false;
        seq_phase = 1;
        seq_green_armed = seq_orange_armed = seq_red_armed = false;
        Console.println("[BarMode] Seq phase 1: blue held 3s");
      }
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
      ApplyBooshState(1.0f, "barmode");
      barmode_all4_midi_held = true;
      barmode_act_counts[6]++;
      Console.println("[BarMode] Seq phase 4: valve open");
    }

    // Required-button-released checks: reset if an anchor is dropped
    // Note: check seq_phase before clearing seq_valve_open so we go to phase 5 if valve was open
    if (seq_phase >= 1 && !btn_stable[1]) {   // BLUE released
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        ApplyBooshState(0.0f, "barmode");
        seq_valve_open = false;
        barmode_all4_midi_held = false;
        if (barmode_all4_lockout_s > 0) barmode_all4_lockout_until_ms = now + barmode_all4_lockout_s * 1000UL;
        Console.printf("[BarMode] Lockout started: %lu s\n", barmode_all4_lockout_s);
      }
      seq_phase = next;
      seq_blue_armed = seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: blue released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase >= 2 && !btn_stable[0]) {  // GREEN released in phase 2+
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        ApplyBooshState(0.0f, "barmode");
        seq_valve_open = false;
        barmode_all4_midi_held = false;
        if (barmode_all4_lockout_s > 0) barmode_all4_lockout_until_ms = now + barmode_all4_lockout_s * 1000UL;
        Console.printf("[BarMode] Lockout started: %lu s\n", barmode_all4_lockout_s);
      }
      seq_phase = next;
      seq_green_armed = seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: green released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase >= 3 && !btn_stable[2]) {  // ORANGE released in phase 3+
      const int next = (seq_phase == 4) ? 5 : 0;
      if (seq_valve_open) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        ApplyBooshState(0.0f, "barmode");
        seq_valve_open = false;
        barmode_all4_midi_held = false;
        if (barmode_all4_lockout_s > 0) barmode_all4_lockout_until_ms = now + barmode_all4_lockout_s * 1000UL;
        Console.printf("[BarMode] Lockout started: %lu s\n", barmode_all4_lockout_s);
      }
      seq_phase = next;
      seq_orange_armed = seq_red_armed = false;
      Console.printf("[BarMode] Seq %s: orange released\n", next == 0 ? "reset" : "closing");
    } else if (seq_phase == 4 && !btn_stable[3]) {  // RED released in phase 4
      if (seq_valve_open) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        ApplyBooshState(0.0f, "barmode");
        seq_valve_open = false;
        barmode_all4_midi_held = false;
        if (barmode_all4_lockout_s > 0) barmode_all4_lockout_until_ms = now + barmode_all4_lockout_s * 1000UL;
        Console.printf("[BarMode] Lockout started: %lu s\n", barmode_all4_lockout_s);
      }
      seq_phase = 5;
      Console.println("[BarMode] Seq phase 5: closing (red released)");
    }

    // Phase 4: auto-close timeout
    if (seq_phase == 4 && seq_valve_open && now - seq_valve_open_ms >= barmode_all4_valve_ms) {
      SendOscFloatToAllPylons(kOscAddress, 0.0f);
      ApplyBooshState(0.0f, "barmode");
      seq_valve_open = false;
      barmode_all4_midi_held = false;
      if (barmode_all4_lockout_s > 0) barmode_all4_lockout_until_ms = now + barmode_all4_lockout_s * 1000UL;
      Console.printf("[BarMode] Lockout started: %lu s\n", barmode_all4_lockout_s);
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
          if (now < green_recovery_until) {
            // In recovery: show WAIT, block action
            barmode_recovery_wait_until_ms = green_recovery_until;
          } else {
            barmode_btn_counts[0]++;
            barmode_btn_event_ms[barmode_btn_event_head]  = now;
            barmode_btn_event_btn[barmode_btn_event_head] = 0;
            barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
            if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
            if (!btn0_pulse_active) {
              SendOscFloatToAllPylons(kOscAddress, 1.0f);
              ApplyBooshState(1.0f, "barmode");
              barmode_act_counts[0]++;
            }
            btn0_pulse_active = true;
            btn0_pulse_ms     = now;
            barmode_btn0_held = true;
            display.invertDisplay(true);
          }
        }
      }
      // Physical release: clear lamp/display state; timer still handles OSC close
      if (!btn_stable[0] && btn_prev_stable[0] && barmode_btn0_held) {
        barmode_btn0_held = false;
        display.invertDisplay(false);
        if (barmode_green_recovery_ms > 0) green_recovery_until = now + ApplyTempMult(barmode_green_recovery_ms);
      }
      // Timer-based close
      if (btn0_pulse_active && now - btn0_pulse_ms >= barmode_green_timeout_ms) {
        SendOscFloatToAllPylons(kOscAddress, 0.0f);
        ApplyBooshState(0.0f, "barmode");
        btn0_pulse_active = false;
      }
    }
    // Green lamp: disabled=30%, recovery=off, held=solid, idle=sine 2Hz
    {
      uint8_t v;
      if (barmode_btn_disabled[0]) {
        v = 77;  // 30%
      } else if (now < green_recovery_until) {
        v = 0;  // recovery: lamp off
      } else if (barmode_btn0_held) {
        v = 255;
      } else {
        const float t    = millis() / 1000.0f;
        const float bpm  = barmode_bpm;
        const float freq = (bpm >= 40.0f) ? (bpm / 60.0f) : 2.0f;  // 1 sine per beat, or 2Hz default
        v = (uint8_t)((sinf(2.0f * M_PI * freq * t) * 0.5f + 0.5f) * 255);
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
      static bool          btn1_single_fired      = false;  // true if single-tap fired this press

      const bool rising  = btn_stable[1] && !btn_prev_stable[1];
      const bool falling = !btn_stable[1] && btn_prev_stable[1];

      if (rising && !barmode_btn_disabled[1]) {
        if (now < blue_recovery_until) {
          // In recovery: show WAIT, block action
          barmode_recovery_wait_until_ms = blue_recovery_until;
        } else {
          barmode_btn_counts[1]++;
          barmode_btn_event_ms[barmode_btn_event_head]  = now;
          barmode_btn_event_btn[barmode_btn_event_head] = 1;
          barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
          if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
          if (btn1_release_ms > 0 && now - btn1_release_ms <= 300) {
            // Double-tap: enter index-ordered sequential looping mode
            seq_blue_armed = false;  // prevent all-4 from hijacking this hold
            btn1_single_fired = false;
            btn1_seq_count = ExtractRegistryTargets(btn1_seq_targets, 16);
            if (btn1_seq_count > 0) {
              btn1_seq_active       = true;
              btn1_seq_start_ms     = now;
              btn1_seq_delay_ms     = barmode_seq_start_ms;
              btn1_seq_last_fire_ms = now - barmode_seq_start_ms;  // fire first group immediately
              btn1_seq_group        = 0;
              barmode_act_counts[2]++;
              Console.printf("[BarMode] Btn1 seq: %d pylons\n", btn1_seq_count);
            }
          } else {
            // Normal single fire to all pylons simultaneously
            SendOscFloatToAllPylons(kOscAddrPulseSingle, 1.0f);
            if (!action_pulse1_dis) StartSequence(SEQ_PULSE_ONCE);
            barmode_act_counts[1]++;
            btn1_single_fired = true;
          }
        }
      }

      if (falling) {
        btn1_release_ms = now;
        if (btn1_seq_active) {
          btn1_seq_active = false;
          Console.println("[BarMode] Btn1 seq stopped");
        } else if (btn1_single_fired && barmode_blue_recovery_ms > 0) {
          blue_recovery_until = now + ApplyTempMult(barmode_blue_recovery_ms);
        }
        btn1_single_fired = false;
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
            const unsigned long floor = barmode_seq_floor_ms;
            unsigned long after_dec = (btn1_seq_delay_ms > barmode_seq_dec_ms + floor)
                                      ? btn1_seq_delay_ms - barmode_seq_dec_ms
                                      : floor;
            if (barmode_seq_exp_pct < 100) {
              after_dec = (unsigned long)(after_dec * (barmode_seq_exp_pct / 100.0f));
            }
            btn1_seq_delay_ms = (after_dec < floor) ? floor : after_dec;
          }
          // Advance group pointer; wrap without resetting delay so acceleration accumulates
          btn1_seq_group += fired;
          if (btn1_seq_group >= btn1_seq_count) {
            btn1_seq_group = 0;
          }
        }
      }
    }

    // Blue lamp: disabled=30%, recovery=off, idle=200ms pulse per second
    {
      const float bpm_b = barmode_bpm;
      const unsigned long blue_period = (bpm_b >= 40.0f)
          ? (unsigned long)(60000.0f / bpm_b) : 1000UL;  // 1 pulse per beat, or 1Hz default
      ledcWrite(5, barmode_btn_disabled[1] ? 77 : (now < blue_recovery_until) ? 0 : (now % blue_period < 50) ? 255 : 51);
    }

    // Button 2 — Orange button: BooshPulseTrain; IO35 strobes 5x pulse pattern once then returns to idle sawtooth
    {
      static bool          io35_strobe           = false;
      static unsigned long io35_strobe_start      = 0;
      static bool          btn2_pending_release   = false;  // true if action fired; start recovery on release

      const bool o_rising  = btn_stable[2] && !btn_prev_stable[2];
      const bool o_falling = !btn_stable[2] && btn_prev_stable[2];

      if (o_rising && !barmode_btn_disabled[2]) {
        if (now < orange_recovery_until) {
          // In recovery: show WAIT, block action
          barmode_recovery_wait_until_ms = orange_recovery_until;
        } else {
          barmode_btn_counts[2]++;
          barmode_btn_event_ms[barmode_btn_event_head]  = now;
          barmode_btn_event_btn[barmode_btn_event_head] = 2;
          barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
          if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
          SendOscFloatToAllPylons(kOscAddrPulseTrain, 1.0f);
          if (!action_pt_dis) StartSequence(SEQ_PULSE_5X);
          barmode_act_counts[3]++;
          io35_strobe        = true;
          io35_strobe_start  = now;
          btn2_pending_release = true;
          barmode_orange_strobe_g = true;
          barmode_orange_strobe_start_ms_g = now;
        }
      }

      if (o_falling && btn2_pending_release) {
        if (barmode_orange_recovery_ms > 0) orange_recovery_until = now + ApplyTempMult(barmode_orange_recovery_ms);
        btn2_pending_release = false;
      }

      if (io35_strobe) {
        const unsigned long elapsed = now - io35_strobe_start;
        if (elapsed < 500) {
          // Mirror 5× (50ms on / 50ms off)
          const uint8_t step = (uint8_t)(elapsed / 50);
          ledcWrite(3, (step % 2 == 0) ? 255 : 0);
        } else {
          io35_strobe = false;
          barmode_orange_strobe_g = false;
        }
      }
      if (!io35_strobe) {
        // Orange lamp: disabled=30%, recovery=off, idle=sawtooth 4Hz
        {
          const float bpm_o = barmode_bpm;
          const unsigned long saw_period = (bpm_o >= 40.0f)
              ? (unsigned long)(60000.0f / bpm_o) : 250UL;  // 1 ramp per beat, or 4Hz default
          uint8_t v;
          if (barmode_btn_disabled[2]) v = 77;
          else if (now < orange_recovery_until) v = 0;
          else v = (uint8_t)((now % saw_period) * 255 / saw_period);
          ledcWrite(3, v);
        }
      }
    }

    // Button 3 — Red button: hold=sequential pylon fire (ping-pong index order, open+close per pylon)
    // Triple-quick-tap+hold=BooshSteam hold (unchanged).
    // State 0: idle
    // State 1: first quick-tap released (<400ms held); 500ms window for 2nd tap (triple-tap steam)
    // State 2: second tap done; 500ms window for 3rd press+hold (steam)
    // State 3: steam hold active
    // State 4: sequential hold active — fires pylons in ping-pong index order
    {
      static int           red_state          = 0;
      static unsigned long red_seq_press_ms   = 0;   // millis() when state-4 press started
      static unsigned long red_press1_ms      = 0;   // millis() of 1st quick-tap release
      static unsigned long red_press2_ms      = 0;   // millis() of 2nd tap
      static unsigned long lamp_red_press_ms  = 0;
      static bool          lamp_red_on        = false;
      static unsigned long lamp_red_step_ms   = 0;
      static unsigned long red_seq_lamp_until = 0;   // lamp on while valve open

      // Sequential state
      static PylonTarget   red_seq_targets[16];
      static int           red_seq_count      = 0;
      static int           red_seq_pos        = 0;
      static bool          red_seq_ascending  = true;
      static unsigned long red_seq_start_ms   = 0;
      static unsigned long red_seq_last_ms    = 0;   // millis() of last step fire
      // Pending closes array — supports overlap (step_ms < valve_ms → multiple simultaneous open)
      static RedSeqClose   red_seq_closes[16]   = {};
      // "Complete first pass" guarantee
      static int           red_seq_fires         = 0;    // fires completed in current hold
      static bool          red_seq_released_early = false; // button released before pass done

      const bool r_rising  = btn_stable[3] && !btn_prev_stable[3];
      const bool r_falling = !btn_stable[3] && btn_prev_stable[3];

      if (!barmode_btn_disabled[3]) {
        // Always drain expired closes regardless of red_state.
        // This lets valve timers complete cleanly even after state transitions away from 4
        // (e.g. quick-tap → state 1 mid-pass; closes still time out correctly).
        for (auto &c : red_seq_closes) {
          if (c.active && now >= c.close_at) {
            SendOscToPylonTarget(kOscAddress, 0.0f, c.target);
            c.active = false;
          }
        }

        // --- Rising edge ---
        if (r_rising) {
          if (red_state == 0 || (red_state == 1 && now - red_press1_ms > 500) || (red_state == 2 && now - red_press2_ms > 500)) {
            // State 0 or out-of-window: start sequential
            barmode_btn_counts[3]++;
            barmode_btn_event_ms[barmode_btn_event_head]  = now;
            barmode_btn_event_btn[barmode_btn_event_head] = 3;
            barmode_btn_event_head = (barmode_btn_event_head + 1) % kBtnEventBufSize;
            if (barmode_btn_event_count < kBtnEventBufSize) barmode_btn_event_count++;
            red_seq_count      = ExtractRegistryTargets(red_seq_targets, 15);
            // Add self (BARBAR) as one slot in the ping-pong sequence
            if (red_seq_count < 16) {
              PylonTarget self_t = {};
              self_t.is_self = true;
              self_t.seq_idx = -1;
              red_seq_targets[red_seq_count++] = self_t;
            }
            red_seq_pos            = 0;
            red_seq_ascending      = true;
            red_seq_start_ms       = now;
            red_seq_last_ms        = now - barmode_red_seq_step_ms;  // fire first step immediately
            memset(red_seq_closes, 0, sizeof(red_seq_closes));
            red_seq_press_ms       = now;
            red_seq_fires          = 0;
            red_seq_released_early = false;
            barmode_act_counts[4]++;
            red_state          = 4;
            Console.printf("[BarMode] Red: seq start, %d pylons (incl. self)\n", red_seq_count);
          } else if (red_state == 1 && now - red_press1_ms <= 500) {
            // Second tap in triple-tap sequence
            red_press2_ms = now;
            red_state     = 2;
            Console.println("[BarMode] Red: 2nd tap");
          } else if (red_state == 2 && now - red_press2_ms <= 500) {
            // Third press + hold → steam
            SendOscFloatToAllPylons(kOscAddrSteam, 1.0f);
            if (!action_steam_dis) StartSequence(SEQ_STEAM);
            barmode_act_counts[5]++;
            lamp_red_press_ms = now;
            lamp_red_on       = false;
            lamp_red_step_ms  = now - 10000;
            red_state         = 3;
            barmode_red_steam_start_ms_g = now;
            Console.println("[BarMode] Red: steam hold active");
          }
        }

        // --- Falling edge ---
        if (r_falling) {
          if (red_state == 4) {
            const unsigned long held = now - red_seq_press_ms;
            const bool first_pass_done = (red_seq_fires >= red_seq_count);
            if (held < 400 && now < red_recovery_until) {
              // Recovery active: flush all closes immediately and stop
              for (auto &c : red_seq_closes) { if (c.active) { SendOscToPylonTarget(kOscAddress, 0.0f, c.target); c.active = false; } }
              barmode_recovery_wait_until_ms = red_recovery_until;
              red_state = 0;
              Console.printf("[BarMode] Red: quick-tap blocked by recovery (%lums held)\n", held);
            } else if (held < 400) {
              // Quick tap: go to state 1 immediately so the steam gesture window opens NOW.
              // Any open valves close on their timers via the always-running drain loop above.
              // Do NOT set released_early — no more steps fire (state != 4 after this).
              red_state = 1;
              red_press1_ms = now;
              Console.printf("[BarMode] Red: quick-tap (%lums), steam window open\n", held);
            } else if (!first_pass_done) {
              // Long hold released before first pass done: stay in state 4 to complete pass
              red_seq_released_early = true;
              Console.printf("[BarMode] Red: long-hold released early (%d/%d fired), completing pass\n", red_seq_fires, red_seq_count);
            } else {
              // Long hold, pass done: flush and stop
              for (auto &c : red_seq_closes) { if (c.active) { SendOscToPylonTarget(kOscAddress, 0.0f, c.target); c.active = false; } }
              red_state = 0;
              Console.printf("[BarMode] Red: seq end (%lums)\n", held);
            }
          } else if (red_state == 3) {
            SendOscFloatToAllPylons(kOscAddrSteam, 0.0f);
            if (!action_steam_dis) AbortSequence();
            red_state = 0;
            if (barmode_red_recovery_ms > 0) red_recovery_until = now + ApplyTempMult(barmode_red_recovery_ms);
            Console.println("[BarMode] Red: steam released");
          }
        }

        // --- State 4: sequential tick ---
        if (red_state == 4) {
          if (now - red_seq_start_ms >= barmode_red_seq_max_ms) {
            // Max time elapsed: stop
            for (auto &c : red_seq_closes) { if (c.active) { SendOscToPylonTarget(kOscAddress, 0.0f, c.target); c.active = false; } }
            red_state = 0;
            Console.println("[BarMode] Red: seq max time");
          } else {
            // If early-released (long hold) and first pass complete: drain then stop
            bool any_close_active = false;
            for (auto &c : red_seq_closes) { if (c.active) { any_close_active = true; break; } }
            if (red_seq_released_early && red_seq_fires >= red_seq_count) {
              if (!any_close_active) {
                red_state = 0;
                Console.println("[BarMode] Red: pass complete after early release, stopped");
              }
            } else {
              // Step tick
              if (red_seq_count > 0 && now - red_seq_last_ms >= barmode_red_seq_step_ms) {
                red_seq_last_ms = now;
                // Claim a free close slot (with overlap, old slots may still be active)
                int slot = -1;
                for (int i = 0; i < 16; i++) { if (!red_seq_closes[i].active) { slot = i; break; } }
                if (slot < 0) slot = 0;  // all full (shouldn't happen with ≤16 pylons) — reuse slot 0
                // Fire open to current position
                SendOscToPylonTarget(kOscAddress, 1.0f, red_seq_targets[red_seq_pos]);
                red_seq_closes[slot].target   = red_seq_targets[red_seq_pos];
                red_seq_closes[slot].close_at = now + barmode_red_seq_valve_ms;
                red_seq_closes[slot].active   = true;
                red_seq_lamp_until   = now + barmode_red_seq_valve_ms;
                red_seq_fires++;
                Console.printf("[BarMode] Red seq: open idx=%d fires=%d slot=%d\n", red_seq_targets[red_seq_pos].seq_idx, red_seq_fires, slot);
                // Advance ping-pong
                if (red_seq_count == 1) {
                  red_seq_pos = 0;
                } else if (red_seq_ascending) {
                  red_seq_pos++;
                  if (red_seq_pos >= red_seq_count) {
                    red_seq_ascending = false;
                    red_seq_pos = red_seq_count - 2;
                  }
                } else {
                  red_seq_pos--;
                  if (red_seq_pos < 0) {
                    red_seq_ascending = true;
                    red_seq_pos = 1;
                  }
                }
              }
            }
          }
        }

        // --- Triple-tap window timeouts ---
        if (red_state == 1 && now - red_press1_ms > 500) { red_state = 0; }
        if (red_state == 2 && now - red_press2_ms > 500) { red_state = 0; }
      }

      // Red lamp: disabled=30%, seq=pulse with valve, steam=ramp, recovery=off, idle=Morse LAVA
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
      } else if (red_state == 4 || now < red_seq_lamp_until) {
        // Sequential: on while valve is open
        ledcWrite(6, now < red_seq_lamp_until ? 255 : 0);
      } else if (now < red_recovery_until) {
        // Recovery: lamp off
        ledcWrite(6, 0);
      } else {
        // Idle: Morse "LAVA" — unit scales to beat/4 when BPM locked (16th note), else 150ms
        static const uint8_t kLavaMorseUnits[] = {
          1,1,3,1,1,1,1,3,  // L + inter-char
          1,1,3,3,            // A + inter-char
          1,1,1,1,1,1,3,3,  // V + inter-char
          1,1,3,7,            // A + inter-word
        };
        const float bpm_r = barmode_bpm;
        const unsigned long beat_r = (bpm_r >= 40.0f) ? (unsigned long)(60000.0f / bpm_r) : 600UL;
        const unsigned long mu = beat_r / 4;
        const unsigned long morse_unit = (mu < 50) ? 50 : (mu > 400) ? 400 : mu;
        unsigned long total_ms = 0;
        for (int i = 0; i < 24; i++) total_ms += kLavaMorseUnits[i] * morse_unit;
        const unsigned long t = millis() % total_ms;
        unsigned long accum = 0;
        uint8_t lamp = 0;
        for (int i = 0; i < 24; i++) {
          accum += kLavaMorseUnits[i] * morse_unit;
          if (t < accum) { lamp = (i % 2 == 0) ? 255 : 0; break; }
        }
        ledcWrite(6, lamp);
      }
      barmode_red_state_g = red_state;
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

    barmode_seq_phase_g = seq_phase;
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

// ---- Mesh (ESP-NOW) functions -----------------------------------------------

// Called from ESP-NOW recv callback (Core 0 WiFi task) — must be ISR-safe.
// Upserts a beacon into the peer table; updates quality bitmap on the 2 s slot.
static void MeshUpsertPeer(const uint8_t *mac, const MeshBeaconPkt &pkt) {
  if (xSemaphoreTakeFromISR(mesh_peers_mutex, nullptr) != pdTRUE) return;
  const uint32_t now = (uint32_t)millis();
  int slot = -1;
  for (int i = 0; i < kMeshMaxPeers; i++) {
    if (mesh_peers[i].active && memcmp(mesh_peers[i].mac, mac, 6) == 0) {
      slot = i; break;
    }
  }
  if (slot < 0) {
    for (int i = 0; i < kMeshMaxPeers; i++) {
      if (!mesh_peers[i].active) { slot = i; break; }
    }
  }
  if (slot < 0) { xSemaphoreGiveFromISR(mesh_peers_mutex, nullptr); return; }  // table full
  const bool is_new = !mesh_peers[slot].active;
  MeshPeerInfo &p = mesh_peers[slot];
  if (is_new) {
    p.batt_v   = NAN;
    p.batt_pct = NAN;
    p.temp_f   = NAN;
    p.fw_ver[0] = '\0';
  }
  p.active    = true;
  memcpy(p.mac, mac, 6);
  // Register peer's unicast MAC with ESP-NOW on first discovery so MeshUnicastOsc can target it.
  if (is_new && !esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer_info;
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, mac, 6);
    peer_info.channel = 0;
    peer_info.ifidx   = ap_active ? WIFI_IF_AP : WIFI_IF_STA;
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);
  }
  strncpy(p.node_id, pkt.node_id, sizeof(p.node_id) - 1);
  p.node_id[sizeof(p.node_id) - 1] = '\0';
  p.pylon_index = pkt.pylon_index;
  p.role        = pkt.role;
  p.uptime_s    = pkt.uptime_s;
  p.batt_v      = pkt.batt_v;
  p.batt_pct    = pkt.batt_pct;
  p.temp_f      = pkt.temp_f;
  strncpy(p.fw_ver, pkt.fw_ver, sizeof(p.fw_ver) - 1);
  p.fw_ver[sizeof(p.fw_ver) - 1] = '\0';
  // Advance quality slot if ≥ 2 s have passed since last update
  if (now - p.qual_last_update_ms >= kMeshBeaconIntervalMs) {
    p.qual_bits = (uint16_t)((p.qual_bits << 1) & 0xFFFF);  // shift in a 0
    p.qual_last_update_ms = now;
  }
  p.qual_bits |= 1u;  // mark this slot as heard
  {
    uint16_t b = p.qual_bits; uint8_t cnt = 0;
    while (b) { cnt += (b & 1); b >>= 1; }
    p.qual_pct = (uint8_t)(cnt * 100 / kMeshQualitySlots);
  }
  p.last_seen_ms = now;
  xSemaphoreGiveFromISR(mesh_peers_mutex, nullptr);
}

// Returns true if this {src_mac, seq} pair was seen within the dedup window.
// Records the entry if not a duplicate. NOT thread-safe; call only from Core 1 OSC context.
static bool MeshIsDuplicate(const uint8_t *mac, uint16_t seq) {
  const uint32_t now = (uint32_t)millis();
  for (int i = 0; i < kMeshDedupSlots; i++) {
    if (mesh_dedup[i].expires_ms > now &&
        memcmp(mesh_dedup[i].mac, mac, 6) == 0 &&
        mesh_dedup[i].seq == seq) return true;
  }
  // Record new entry
  memcpy(mesh_dedup[mesh_dedup_idx].mac, mac, 6);
  mesh_dedup[mesh_dedup_idx].seq        = seq;
  mesh_dedup[mesh_dedup_idx].expires_ms = now + kMeshDedupWindowMs;
  mesh_dedup_idx = (mesh_dedup_idx + 1) % kMeshDedupSlots;
  return false;
}

// Queue for ESP-NOW → OSC command delivery to main loop (Core 1).
// Using a small FreeRTOS queue so the recv callback (WiFi task, Core 0) never
// calls StartSequence() or ApplyBooshState() directly on Core 1 data.
struct MeshOscEvent {
  char    osc_addr[32];
  float   osc_arg;
  uint8_t src_mac[6];
  uint16_t seq;
};
constexpr int kMeshOscQueueDepth = 8;
static QueueHandle_t mesh_osc_queue = nullptr;

// Pad event bridge queue: MeshOnRecv → PingTask → rpiboosh /send_virtual_midi.
struct MeshPadEvent {
  uint8_t  note;
  uint8_t  velocity;
  uint8_t  channel;
  uint16_t seq;
  char     remote_id[16];
};
constexpr int kMeshPadQueueDepth = 8;
static QueueHandle_t mesh_pad_queue = nullptr;

// Remote telemetry queue: MeshOnRecv → PingTask → rpiboosh MQTT retained publish.
// Payload is pre-formatted in the callback so PingTask just calls publish().
struct MeshTelemQueueItem {
  MeshRemoteTelemPkt pkt;     // raw packet (74 bytes)
  uint8_t            src_mac[6]; // ESP-NOW sender MAC
};
constexpr int kMeshTelemQueueDepth = 4;
static QueueHandle_t mesh_telem_queue = nullptr;

// Per-remote dedup table for type-4 pad events. Keyed by remote_id; 8 slots covers
// all realistic remotes. Oldest entry evicted when full. Accessed only from MeshOnRecv
// (WiFi task, Core 0) so no mutex needed.
struct PadDedupEntry {
  char     remote_id[16];
  uint16_t last_seq;
  uint32_t last_ms;
  bool     active;
};
constexpr int kPadDedupSlots = 8;
static PadDedupEntry pad_dedup[kPadDedupSlots];

// ESP-NOW receive callback — runs in WiFi task context (Core 0).
static void MeshOnRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (!cfg_mesh_en) return;
  // Need at least magic(4)+version(1)+type(1) = 6 bytes before reading pkt_type
  if (len < 6) return;
  if (data[0] != (uint8_t)(kMeshMagic & 0xFF)) return;  // quick magic sanity
  const uint8_t pkt_type = data[5];  // offset 4=version, 5=type in both structs
  if (pkt_type == kMeshPktBeacon && len >= (int)sizeof(MeshBeaconPkt)) {
    MeshBeaconPkt pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != kMeshMagic || pkt.version != kMeshVersion) return;
    MeshUpsertPeer(mac, pkt);
  } else if (pkt_type == kMeshPktCommand && len >= (int)sizeof(MeshCommandPkt)) {
    MeshCommandPkt pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != kMeshMagic || pkt.version != kMeshVersion) return;
    if (mesh_osc_queue) {
      MeshOscEvent ev;
      strncpy(ev.osc_addr, pkt.osc_addr, sizeof(ev.osc_addr) - 1);
      ev.osc_addr[sizeof(ev.osc_addr) - 1] = '\0';
      ev.osc_arg = pkt.osc_arg;
      memcpy(ev.src_mac, mac, 6);
      ev.seq = pkt.seq;
      xQueueSendFromISR(mesh_osc_queue, &ev, nullptr);
    }
  } else if (pkt_type == kMeshPktChanChange && len >= (int)sizeof(MeshChanChangePkt)) {
    MeshChanChangePkt pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.magic != kMeshMagic) return;
    if (pkt.new_ch < 1 || pkt.new_ch > 13) return;
    mesh_ch_pending  = pkt.new_ch;
    mesh_ch_apply_at = millis() + pkt.apply_ms;
    Console.printf("[Mesh] chan change pending: ch=%u in %ums\n", pkt.new_ch, (unsigned)pkt.apply_ms);
  } else if (pkt_type == kMeshPktPadEvent && len >= (int)sizeof(MeshPadEventPkt)) {
    if (mesh_pad_queue) {
      const MeshPadEventPkt *p = reinterpret_cast<const MeshPadEventPkt *>(data);
      if (p->magic != kMeshMagic) return;
      // Dedup: each remote broadcasts to all nodes; only forward the first arrival.
      const uint32_t now_ms = (uint32_t)millis();
      int match = -1, oldest_i = 0;
      for (int i = 0; i < kPadDedupSlots; i++) {
        if (pad_dedup[i].active && strncmp(pad_dedup[i].remote_id, p->remote_id, 16) == 0) {
          match = i; break;
        }
        if (!pad_dedup[i].active || now_ms - pad_dedup[i].last_ms > now_ms - pad_dedup[oldest_i].last_ms)
          oldest_i = i;
      }
      const int idx = (match >= 0) ? match : oldest_i;
      if (match < 0) {
        strlcpy(pad_dedup[idx].remote_id, p->remote_id, sizeof(pad_dedup[idx].remote_id));
        pad_dedup[idx].active = true;
      } else if (pad_dedup[idx].last_seq == p->seq && now_ms - pad_dedup[idx].last_ms < p->dedup_ms) {
        return;  // duplicate from another node forwarding the same broadcast
      }
      pad_dedup[idx].last_seq = p->seq;
      pad_dedup[idx].last_ms  = now_ms;
      MeshPadEvent ev;
      ev.note     = p->note;
      ev.velocity = p->velocity;
      ev.channel  = p->channel;
      ev.seq      = p->seq;
      strlcpy(ev.remote_id, p->remote_id, sizeof(ev.remote_id));
      xQueueSendFromISR(mesh_pad_queue, &ev, nullptr);
    }
  } else if (pkt_type == kMeshPktRemoteTelem &&
             len >= (int)offsetof(MeshRemoteTelemPkt, hostname)) {
    if (mesh_telem_queue) {
      MeshTelemQueueItem item;
      memset(&item.pkt, 0, sizeof(item.pkt));
      memcpy(&item.pkt, data, min((size_t)len, sizeof(item.pkt)));
      item.pkt.remote_id[15]   = '\0';
      item.pkt.description[31] = '\0';
      item.pkt.hostname[39]    = '\0';
      item.pkt.ip[15]          = '\0';
      memcpy(item.src_mac, mac, 6);
      xQueueSendFromISR(mesh_telem_queue, &item, nullptr);
    }
  }
}

// Broadcasts a beacon packet announcing this node. Called from PingTask (Core 0).
static void MeshSendBeacon() {
  if (!mesh_initialized) return;
  MeshBeaconPkt pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic   = kMeshMagic;
  pkt.version = kMeshVersion;
  pkt.type    = kMeshPktBeacon;
  strncpy(pkt.node_id, pylon_id.c_str(), sizeof(pkt.node_id) - 1);
  pkt.pylon_index = (uint8_t)pylon_index;
  pkt.role        = barmode_active ? 1 : 0;
  pkt.uptime_s    = (uint32_t)(millis() / 1000);
  pkt.batt_v      = cfg_no_batt_mon   ? NAN : sensor_battery_v;
  pkt.batt_pct    = cfg_no_batt_mon   ? NAN : sensor_battery_pct;
  pkt.temp_f      = cfg_no_thermistor ? NAN : sensor_temp_f;
  strncpy(pkt.fw_ver, kFirmwareVersion, sizeof(pkt.fw_ver) - 1);
  pkt.fw_ver[sizeof(pkt.fw_ver) - 1] = '\0';
  esp_now_send(kMeshBroadcastMac, (uint8_t *)&pkt, sizeof(pkt));
  mesh_beacons_sent++;
}

// Builds a MeshCommandPkt into pkt. Shared by broadcast and unicast.
static void MeshFillCmdPkt(MeshCommandPkt &pkt, const char *osc_addr, float osc_arg) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic   = kMeshMagic;
  pkt.version = kMeshVersion;
  pkt.type    = kMeshPktCommand;
  pkt.seq     = mesh_cmd_seq++;
  strncpy(pkt.osc_addr, osc_addr, sizeof(pkt.osc_addr) - 1);
  pkt.osc_arg = osc_arg;
}

// Broadcasts an OSC command to all mesh peers (with retry). Safe to call from
// Core 1 (button handlers, web handlers) — esp_now_send is thread-safe.
void MeshBroadcastOsc(const char *osc_addr, float osc_arg) {
  if (!mesh_initialized || !cfg_mesh_en) return;
  MeshCommandPkt pkt;
  MeshFillCmdPkt(pkt, osc_addr, osc_arg);
  for (uint8_t i = 0; i < kMeshCmdRetries; i++) {
    esp_now_send(kMeshBroadcastMac, (uint8_t *)&pkt, sizeof(pkt));
    if (i < kMeshCmdRetries - 1) vTaskDelay(pdMS_TO_TICKS(kMeshCmdRetryMs));
  }
  mesh_cmds_sent++;
  Console.printf("[Mesh] broadcast %s %.1f seq=%u\n", osc_addr, osc_arg, pkt.seq);
}

// Broadcasts a channel-change packet with a countdown delay, then arms the
// local pending change so this node switches at the same time as peers.
// Safe to call from Core 1 (web handler). delay_ms must be long enough for
// the broadcast to reach all peers before anyone switches (~5000ms is safe).
void MeshBroadcastChanChange(uint8_t new_ch, uint32_t delay_ms) {
  if (!mesh_initialized || !cfg_mesh_en) return;
  MeshChanChangePkt pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic    = kMeshMagic;
  pkt.version  = kMeshVersion;
  pkt.type     = kMeshPktChanChange;
  pkt.new_ch   = new_ch;
  pkt.apply_ms = delay_ms;
  for (uint8_t i = 0; i < kMeshCmdRetries; i++) {
    esp_now_send(kMeshBroadcastMac, (uint8_t *)&pkt, sizeof(pkt));
    if (i < kMeshCmdRetries - 1) vTaskDelay(pdMS_TO_TICKS(kMeshCmdRetryMs));
  }
  mesh_ch_pending  = new_ch;
  mesh_ch_apply_at = millis() + delay_ms;
  Console.printf("[Mesh] chan change broadcast: ch=%u in %ums\n", new_ch, (unsigned)delay_ms);
}

// Sends an OSC command to a single mesh peer by MAC (unicast with retry).
// The peer must have been registered via esp_now_add_peer (done in MeshUpsertPeer).
void MeshUnicastOsc(const uint8_t *mac, const char *osc_addr, float osc_arg) {
  if (!mesh_initialized || !cfg_mesh_en) return;
  MeshCommandPkt pkt;
  MeshFillCmdPkt(pkt, osc_addr, osc_arg);
  for (uint8_t i = 0; i < kMeshCmdRetries; i++) {
    esp_now_send(mac, (uint8_t *)&pkt, sizeof(pkt));
    if (i < kMeshCmdRetries - 1) vTaskDelay(pdMS_TO_TICKS(kMeshCmdRetryMs));
  }
  mesh_cmds_sent++;
  Console.printf("[Mesh] unicast %02X:%02X:%02X %s %.1f seq=%u\n",
                 mac[0], mac[1], mac[2], osc_addr, osc_arg, pkt.seq);
}

// Removes peers not heard within kMeshPeerTimeoutMs. Called from PingTask.
static void MeshExpirePeers() {
  if (!mesh_peers_mutex) return;
  const uint32_t now = (uint32_t)millis();
  if (xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  int live = 0;
  for (int i = 0; i < kMeshMaxPeers; i++) {
    if (mesh_peers[i].active && now - mesh_peers[i].last_seen_ms > kMeshPeerTimeoutMs) {
      Console.printf("[Mesh] peer expired: %s\n", mesh_peers[i].node_id);
      mesh_peers[i].active = false;
    }
    if (mesh_peers[i].active) live++;
  }
  mesh_live_peer_count = live;
  xSemaphoreGive(mesh_peers_mutex);
}

// Advances quality slot for peers that haven't had a slot update this interval.
// Called from PingTask every 2 s alongside beacon send.
static void MeshUpdateQuality() {
  if (!mesh_peers_mutex) return;
  const uint32_t now = (uint32_t)millis();
  if (xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  for (int i = 0; i < kMeshMaxPeers; i++) {
    if (!mesh_peers[i].active) continue;
    if (now - mesh_peers[i].qual_last_update_ms >= kMeshBeaconIntervalMs) {
      // Slot elapsed without a heard beacon → shift in a 0
      mesh_peers[i].qual_bits = (uint16_t)((mesh_peers[i].qual_bits << 1) & 0xFFFF);
      mesh_peers[i].qual_last_update_ms = now;
      uint16_t b = mesh_peers[i].qual_bits; uint8_t cnt = 0;
      while (b) { cnt += (b & 1); b >>= 1; }
      mesh_peers[i].qual_pct = (uint8_t)(cnt * 100 / kMeshQualitySlots);
    }
  }
  xSemaphoreGive(mesh_peers_mutex);
}

// Drains mesh_osc_queue and executes commands. Called from loop() (Core 1).
// Dedup check is here (Core 1 only) to avoid mutex contention.
void PollMeshOscQueue() {
  if (!mesh_osc_queue) return;
  MeshOscEvent ev;
  while (xQueueReceive(mesh_osc_queue, &ev, 0) == pdTRUE) {
    if (MeshIsDuplicate(ev.src_mac, ev.seq)) {
      Console.printf("[Mesh] dedup drop %s seq=%u\n", ev.osc_addr, ev.seq);
      continue;
    }
    Console.printf("[Mesh] rx cmd %s %.1f seq=%u\n", ev.osc_addr, ev.osc_arg, ev.seq);
    // Dispatch to the same handlers OSC UDP would use
    const String addr(ev.osc_addr);
    const float val = ev.osc_arg;
    if (addr == kOscAddress) {
      if (group_pattern_active && val >= 0.5f) { Console.println("[GroupPat] suppressed mesh open"); continue; }
      ApplyBooshState(val, "mesh");
    } else if (addr == kOscAddrPulseSingle) {
      if (!action_pulse1_dis && val >= 0.5f) StartSequence(SEQ_PULSE_ONCE);
    } else if (addr == kOscAddrPulseTrain) {
      if (!action_pt_dis && val >= 0.5f) StartSequence(SEQ_PULSE_5X);
    } else if (addr == kOscAddrSteam) {
      if (!action_steam_dis) {
        if (val >= 0.5f) StartSequence(SEQ_STEAM);
        else             AbortSequence();
      }
    }
  }
}

// Initialises (or re-initialises) ESP-NOW. Safe to call multiple times.
// FreeRTOS objects are created once and reused; ESP-NOW is deinitialized first
// when called after a prior successful init (e.g. after a WiFi mode change).
void MeshInit() {
  if (!cfg_mesh_en) return;
  // Create FreeRTOS objects exactly once; reuse across re-inits.
  if (!mesh_peers_mutex) mesh_peers_mutex = xSemaphoreCreateMutex();
  if (!mesh_osc_queue)   mesh_osc_queue   = xQueueCreate(kMeshOscQueueDepth, sizeof(MeshOscEvent));
  if (!mesh_pad_queue)   mesh_pad_queue   = xQueueCreate(kMeshPadQueueDepth, sizeof(MeshPadEvent));
  if (!mesh_telem_queue)  mesh_telem_queue  = xQueueCreate(kMeshTelemQueueDepth, sizeof(MeshTelemQueueItem));
  if (!mesh_remote_mutex) mesh_remote_mutex = xSemaphoreCreateMutex();
  memset(mesh_peers, 0, sizeof(mesh_peers));
  memset(mesh_dedup, 0, sizeof(mesh_dedup));
  // If ESP-NOW was previously initialised, tear it down cleanly before re-init.
  if (mesh_initialized) {
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    mesh_initialized = false;
    delay(10);
  }
  // NOTE: do NOT call esp_wifi_set_channel() here. In STA mode the channel is
  // locked by the AP association; the call silently fails and corrupts the
  // peer.channel expectation. Use channel=0 on the broadcast peer so ESP-NOW
  // follows the current WiFi channel automatically.
  if (esp_now_init() != ESP_OK) {
    Console.println("[Mesh] esp_now_init FAILED");
    return;
  }
  esp_now_register_recv_cb(MeshOnRecv);
  // Register broadcast peer — channel=0 means "use current WiFi channel"
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, kMeshBroadcastMac, 6);
  peer.channel = 0;        // 0 = follow current interface channel
  peer.ifidx   = ap_active ? WIFI_IF_AP : WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  mesh_initialized = true;
  Console.printf("[Mesh] init OK wifi_ch=%u\n", (unsigned)WiFi.channel());
}

// OLED page: shows all mesh peers, 2 per page, cycling on each call.
// Display is 128×32. Layout per page:
//   y=0..15  size=2  M:N badge top-right (DrawMeshBadge)
//   y=0..7   size=1  "MESH" or "MESH 2/3" top-left (page indicator when >1 page)
//   y=16..23 size=1  peer slot A: id+quality left, Ch:N badge right
//   y=24..31 size=1  peer slot B: id+quality left (no badge, full width)
// Each call advances to the next page; wraps when peer count changes.
void ShowMeshPage() {
  // Collect ALL active peers (up to kMeshMaxPeers)
  struct { char id[17]; uint8_t qual_pct; } peers[kMeshMaxPeers];
  int filled = 0;

  if (mesh_peers_mutex && xSemaphoreTake(mesh_peers_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (int i = 0; i < kMeshMaxPeers; i++) {
      if (!mesh_peers[i].active) continue;
      strncpy(peers[filled].id, mesh_peers[i].node_id, 16);
      peers[filled].id[16] = '\0';
      peers[filled].qual_pct = mesh_peers[i].qual_pct;
      filled++;
    }
    xSemaphoreGive(mesh_peers_mutex);
  }

  // Internal page cycling: 2 peers per page, advance at most once per 1500ms.
  // Re-entry after >2s gap resets to page 0 (display switched away and back).
  static uint8_t s_page = 0;
  static unsigned long s_last_advance_ms = 0;
  static unsigned long s_last_call_ms = 0;

  const unsigned long call_ms = millis();
  if (s_last_call_ms == 0 || call_ms - s_last_call_ms > 2000UL) {
    s_page = 0;
    s_last_advance_ms = call_ms;
  }
  s_last_call_ms = call_ms;

  const int num_pages = (filled <= 2) ? 1 : (filled + 1) / 2;
  if (s_page >= (uint8_t)num_pages) s_page = 0;
  if (call_ms - s_last_advance_ms >= 1500UL) {
    s_page = (uint8_t)((s_page + 1) % num_pages);
    s_last_advance_ms = call_ms;
  }
  const int cur_page = s_page;

  // Render
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (!cfg_mesh_en) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("MESH disabled");
    display.display();
    return;
  }
  if (!mesh_initialized) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("MESH init fail");
    display.display();
    return;
  }

  // Header: "MESH" or "MESH 2/3" + M:N+Ch:N badges
  // "MESH 2/3" = 8 chars = 48px, well left of M:N badge at x=92
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (num_pages > 1) {
    char hdr[12];
    snprintf(hdr, sizeof(hdr), "MESH %d/%d", cur_page + 1, num_pages);
    display.print(hdr);
  } else {
    display.print("MESH");
  }
  DrawMeshBadge(0, 2);  // M:N at y=0 (size=2), Ch:N at y=16

  // Two peer rows for this page
  const int page_start = cur_page * 2;
  for (int i = 0; i < 2; i++) {
    const int pi = page_start + i;
    if (pi >= filled) break;
    display.setCursor(0, 16 + i * 8);
    char pbuf[14];  // 13 chars max — stays left of Ch:N badge at x=92
    snprintf(pbuf, sizeof(pbuf), "%-8s %3u%%", peers[pi].id, peers[pi].qual_pct);
    display.print(pbuf);
  }

  display.display();
}

// ---- Ping task (Core 0) -----------------------------------------------------
// Runs hostname resolution and ICMP ping in a background FreeRTOS task so the
// main loop (which handles OSC) never blocks waiting for network replies.

// Polls pylon temperatures every 2 minutes and updates barmode_avg_temp_f / barmode_temp_multiplier.
// Runs at priority 0 (lowest) on Core 0. Yields between each pylon fetch so it never starves
// PingTask, OSC, or web serving. Timeout per request is short (500ms) to keep latency spikes small.
void TempPollTask(void *) {
  // Initial delay: wait for WiFi and registry to settle before first poll
  vTaskDelay(pdMS_TO_TICKS(30000));

  for (;;) {
    if (!barmode_active || WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    PylonTarget targets[16];
    const int tcount = ExtractRegistryTargets(targets, 16);

    if (tcount > 0) {
      float sum = 0.0f;
      int valid = 0;
      for (int i = 0; i < tcount; i++) {
        HTTPClient http;
        http.begin("http://" + targets[i].ip.toString() + "/api/telemetry");
        http.setTimeout(500);  // short timeout: never hold Core 0 long
        if (http.GET() == 200) {
          const String body = http.getString();
          const int idx = body.indexOf("\"temperature_f\":");
          if (idx >= 0) {
            const String valStr = body.substring(idx + 16);
            if (!valStr.startsWith("null")) {
              const float t = valStr.toFloat();
              if (t > -30.0f) { sum += t; valid++; }  // ignore <= -30F (disconnected thermistor/ADC noise)
            }
          }
        }
        http.end();
        vTaskDelay(pdMS_TO_TICKS(200));  // yield between requests; lets higher-priority tasks run
      }

      if (valid > 0) {
        const float avg = sum / valid;
        barmode_avg_temp_f = avg;
        // Recompute multiplier: if below both thresholds, only the colder one applies
        const bool b1 = avg < barmode_temp_thresh1_f;
        const bool b2 = avg < barmode_temp_thresh2_f;
        float mult = 1.0f;
        if (b1 && b2) {
          mult = (barmode_temp_thresh1_f <= barmode_temp_thresh2_f) ? barmode_temp_mult1 : barmode_temp_mult2;
        } else if (b1) {
          mult = barmode_temp_mult1;
        } else if (b2) {
          mult = barmode_temp_mult2;
        }
        barmode_temp_multiplier = mult;
        Console.printf("[TempPoll] avg=%.1f°F mult=%.2fx (%d/%d pylons)\n", avg, mult, valid, tcount);
      } else {
        Console.println("[TempPoll] no valid readings");
      }
    }

    // Sleep 2 minutes before next poll cycle
    vTaskDelay(pdMS_TO_TICKS(120000));
  }
}

void PingTask(void *) {
  static const char *kTargetHost = kPingTargetHost;
  static const char *kTargetHostMdns = "RPIBOOSH.local";
  IPAddress targetIp;
  bool hasIp = false;
  unsigned long lastResolveMs = 0;
  unsigned long lastPingMs = 0;
  unsigned long lastMeshMs = 0;
  bool pingWasDown = false;
  PingStats stats;

  for (;;) {
    const unsigned long now = millis();

    // ---- Mesh beacon + maintenance (every 2 s, no WiFi STA required) --------
    if (cfg_mesh_en && now - lastMeshMs >= kMeshBeaconIntervalMs) {
      lastMeshMs = now;
      MeshSendBeacon();
      MeshUpdateQuality();
      MeshExpirePeers();
    }

    // ---- Pending channel change apply ----------------------------------------
    if (mesh_ch_pending && (long)(now - mesh_ch_apply_at) >= 0) {
      const uint8_t new_ch = mesh_ch_pending;
      mesh_ch_pending = 0;
      cfg_mesh_ch = new_ch;
      SavePylonConfig();  // use full config save — same path as web UI, reliable on AP-enabled nodes
      if (ap_active) {
        // AP channel must match mesh channel — restart AP on the new channel.
        // SetupApMode reads cfg_mesh_ch which is already updated above.
        StopApMode();
        SetupApMode();
        Console.printf("[Mesh] channel applied: ch=%u (AP restarted on new channel)\n", cfg_mesh_ch);
      } else if (!WiFi.isConnected()) {
        esp_wifi_set_channel(cfg_mesh_ch, WIFI_SECOND_CHAN_NONE);
        Console.printf("[Mesh] channel applied: ch=%u (STA disconnected, channel pinned)\n", cfg_mesh_ch);
      } else {
        Console.printf("[Mesh] channel applied: ch=%u (STA connected, will apply on reconnect)\n", cfg_mesh_ch);
      }
    }

    // ---- Mesh channel mismatch detection ----------------------------------------
    // If WiFi is connected but we see 0 live mesh peers for >20 s, the STA radio
    // is locked to the AP's channel which likely differs from cfg_mesh_ch.
    // Force-disconnect WiFi so the DISCONNECTED event re-pins the radio to
    // cfg_mesh_ch and the rest of the mesh can be heard again.
    // This timer resets whenever peers are visible or WiFi is not connected.
    {
      static unsigned long wifi_no_peer_since_ms = 0;
      if (cfg_mesh_en && wifi_has_ip && mesh_live_peer_count == 0) {
        if (wifi_no_peer_since_ms == 0) wifi_no_peer_since_ms = now;
        else if (now - wifi_no_peer_since_ms >= 20000UL) {
          Console.printf("[Mesh] WiFi connected but 0 peers for 20s — channel mismatch suspected "
                         "(WiFi ch vs cfg_mesh_ch=%u). Disconnecting WiFi to restore mesh channel.\n",
                         cfg_mesh_ch);
          wifi_no_peer_since_ms = 0;
          WiFi.disconnect(false, false);
        }
      } else {
        wifi_no_peer_since_ms = 0;
      }
    }

    if (WiFi.status() != WL_CONNECTED) {
      hasIp = false;
      pingWasDown = false;
      stats = PingStats{};
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Pad event bridge — drain first, before ping/registry, to minimise latency.
    // Uses cached IP (target_ip_string) to avoid mDNS lookup on every POST.
    // Coincidence table for group pattern detection (Core 0 only — no mutex).
    struct GroupCoincEntry { char remote_id[16]; uint32_t press_ms; };
    static GroupCoincEntry group_coinc[8] = {};
    static uint32_t group_coinc_last_trigger_ms = 0;
    static uint32_t group_coinc_first_press_ms  = 0;

    if (mesh_pad_queue) {
      MeshPadEvent pev;
      while (xQueueReceive(mesh_pad_queue, &pev, 0) == pdTRUE) {
        const String url = (target_ip_string.length() > 0)
            ? "http://" + target_ip_string + ":5000/send_virtual_midi"
            : String(kRegistryBaseUrlPrimary) + "/send_virtual_midi";
        const String body = "{\"note\":" + String(pev.note) +
                            ",\"velocity\":" + String(pev.velocity) +
                            ",\"channel\":" + String(pev.channel) +
                            ",\"remoteID\":\"" + String(pev.remote_id) + "\"" +
                            ",\"seq\":" + String(pev.seq) + "}";
        HTTPClient http;
        http.begin(url);
        http.setTimeout(200);
        http.addHeader("Content-Type", "application/json");
        const int code = http.POST(body);
        http.end();
        Console.printf("[PadBridge] note=%u vel=%u ch=%u -> %d (from %.15s)\n",
                       pev.note, pev.velocity, pev.channel, code, pev.remote_id);

        // Group coincidence detection: yellow press (note=7, vel=0x7D) from distinct remotes
        if (pev.note == 7 && pev.velocity == 0x7D && cfg_group_pattern_en &&
            group_pattern_pending_n == 0) {
          const uint32_t nc = (uint32_t)millis();
          const uint32_t cooldown_elapsed = nc - group_coinc_last_trigger_ms;
          if (group_coinc_last_trigger_ms == 0 || cooldown_elapsed >= cfg_grp_cool_ms) {
            // Evict stale entries (older than 2s window) and upsert this remote
            int slot = -1, empty_slot = -1;
            for (int i = 0; i < 8; i++) {
              if (group_coinc[i].remote_id[0] != '\0') {
                if (nc - group_coinc[i].press_ms > cfg_grp_win_ms) {
                  group_coinc[i].remote_id[0] = '\0'; // evict stale
                } else if (strncmp(group_coinc[i].remote_id, pev.remote_id, 16) == 0) {
                  slot = i; // update existing
                }
              }
              if (group_coinc[i].remote_id[0] == '\0' && empty_slot < 0) empty_slot = i;
            }
            const int target = (slot >= 0) ? slot : empty_slot;
            if (target >= 0) {
              strlcpy(group_coinc[target].remote_id, pev.remote_id, 16);
              group_coinc[target].press_ms = nc;
            }
            // Track time of first entry; fire deferred to post-drain window check below
            if (group_coinc_first_press_ms == 0) group_coinc_first_press_ms = nc;
          }
        }
      }
    }

    // Group pattern window-expiry: after draining all queued presses, fire if window elapsed.
    if (group_coinc_first_press_ms != 0 && group_pattern_pending_n == 0) {
      const uint32_t nc2 = (uint32_t)millis();
      const bool in_cooldown = group_coinc_last_trigger_ms != 0 &&
                               (nc2 - group_coinc_last_trigger_ms) < cfg_grp_cool_ms;
      if (nc2 - group_coinc_first_press_ms >= cfg_grp_win_ms || in_cooldown) {
        if (!in_cooldown) {
          int cnt = 0;
          for (int i = 0; i < 8; i++)
            if (group_coinc[i].remote_id[0] != '\0') cnt++;
          if (cnt >= 2) {
            group_pattern_pending_n = (uint8_t)(cnt > 8 ? 8 : cnt);
            group_coinc_last_trigger_ms = nc2;
            Console.printf("[GroupPat] window closed N=%d\n", cnt);
          }
        }
        memset(group_coinc, 0, sizeof(group_coinc));
        group_coinc_first_press_ms = 0;
      }
    }

    // Remote telemetry bridge: drain mesh_telem_queue, publish MQTT + update remote table.
    if (mesh_telem_queue) {
      static WiFiClient   mqttWifiClient;
      static PubSubClient mqttClient(mqttWifiClient);
      static bool         mqttInit = false;
      if (!mqttInit) { mqttClient.setBufferSize(512); mqttInit = true; }
      if (!mqttClient.connected()) {
        const String broker = (target_ip_string.length() > 0) ? target_ip_string : "rpiboosh.local";
        mqttClient.setServer(broker.c_str(), 1883);
        mqttClient.connect(("PYL_" + pylon_id.substring(0, 16)).c_str());
      }
      mqttClient.loop();
      MeshTelemQueueItem item;
      while (xQueueReceive(mesh_telem_queue, &item, 0) == pdTRUE) {
        char remMac[18];
        snprintf(remMac, sizeof(remMac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 item.pkt.mac[0], item.pkt.mac[1], item.pkt.mac[2],
                 item.pkt.mac[3], item.pkt.mac[4], item.pkt.mac[5]);
        // MQTT publish (retained)
        if (mqttClient.connected()) {
          char topic[48], payload[384];
          snprintf(topic, sizeof(topic), "boosh/remote/%s/telemetry", item.pkt.remote_id);
          snprintf(payload, sizeof(payload),
                   "{\"remoteID\":\"%s\",\"description\":\"%s\",\"mac\":\"%s\","
                   "\"uptime_s\":%lu,\"press_red\":%lu,\"press_yellow\":%lu,"
                   "\"mode\":\"%s\",\"rssi\":%d,\"forwarded_by\":\"%s\","
                   "\"hostname\":\"%s\",\"ip\":\"%s\"}",
                   item.pkt.remote_id, item.pkt.description, remMac,
                   (unsigned long)item.pkt.uptime_s, (unsigned long)item.pkt.press_red,
                   (unsigned long)item.pkt.press_yellow,
                   item.pkt.mode ? "mesh" : "wifi", (int)item.pkt.rssi, pylon_id.c_str(),
                   item.pkt.hostname, item.pkt.ip);
          const bool ok = mqttClient.publish(topic, payload, true);
          Console.printf("[Telem] MQTT %s -> %s\n", topic, ok ? "ok" : "fail");
        }
        // Update in-memory remote table (for /api/mesh_remotes web UI)
        if (mesh_remote_mutex && xSemaphoreTake(mesh_remote_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          const uint32_t now_t = millis();
          int match = -1, oldest = 0;
          for (int i = 0; i < kMeshRemoteSlots; i++) {
            if (mesh_remote_table[i].active &&
                strncmp(mesh_remote_table[i].remote_id, item.pkt.remote_id, 16) == 0) {
              match = i; break;
            }
            if (!mesh_remote_table[i].active ||
                now_t - mesh_remote_table[i].last_seen_ms > now_t - mesh_remote_table[oldest].last_seen_ms)
              oldest = i;
          }
          MeshRemoteRecord &r = mesh_remote_table[(match >= 0) ? match : oldest];
          r.active = true;
          strlcpy(r.remote_id,    item.pkt.remote_id,   sizeof(r.remote_id));
          strlcpy(r.description,  item.pkt.description, sizeof(r.description));
          strlcpy(r.mac,          remMac,               sizeof(r.mac));
          strlcpy(r.forwarded_by, pylon_id.c_str(),     sizeof(r.forwarded_by));
          r.uptime_s     = item.pkt.uptime_s;
          r.press_red    = item.pkt.press_red;
          r.press_yellow = item.pkt.press_yellow;
          r.mode         = item.pkt.mode;
          r.rssi         = item.pkt.rssi;
          strlcpy(r.hostname, item.pkt.hostname, sizeof(r.hostname));
          strlcpy(r.ip,       item.pkt.ip,       sizeof(r.ip));
          r.last_seen_ms = now_t;
          xSemaphoreGive(mesh_remote_mutex);
        }
      }
    }

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
        // Try primary URL (rpiboosh.local); fall back to resolved IP if mDNS fails
        auto doRegistryGet = [](const String &baseUrl) -> String {
          HTTPClient http;
          http.begin(baseUrl + "/api/pylons");
          http.setTimeout(kRegistryHttpTimeoutMs);
          const int code = http.GET();
          String body;
          if (code == 200) body = http.getString();
          else Console.printf("[BarMode] registry fetch %s: %d\n", baseUrl.c_str(), code);
          http.end();
          return body;
        };
        String body = doRegistryGet(String(kRegistryBaseUrlPrimary));
        if (body.length() == 0 && target_ip_string.length() > 0) {
          body = doRegistryGet("http://" + target_ip_string + ":5000");
        }
        if (body.length() > 0) {
          if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            barmode_registry_json = body;
            xSemaphoreGive(barmode_registry_mutex);
          }
        }
      }
    }

    // In BARMODE: poll llled.local BPM every 3s for lamp sync
    if (barmode_active) {
      // Resolve unresolved manual pylons (retry every 15s per entry)
      const unsigned long now_r = millis();
      for (int i = 0; i < barmode_manual_pylon_count; i++) {
        ManualPylon &mp = barmode_manual_pylons[i];
        if (mp.resolved) continue;
        if (now_r - mp.last_resolve_ms < 15000 && mp.last_resolve_ms != 0) continue;
        mp.last_resolve_ms = now_r;
        IPAddress addr;
        if (addr.fromString(mp.host)) {
          mp.ip = addr; mp.resolved = true;
          Console.printf("[Manual] Parsed IP %s\n", mp.host);
        } else if (WiFi.hostByName(mp.host, addr)) {
          mp.ip = addr; mp.resolved = true;
          Console.printf("[Manual] Resolved %s → %s\n", mp.host, addr.toString().c_str());
        }
      }

      static unsigned long lastBpmFetchMs = 0;
      const unsigned long now_b = millis();
      if (now_b - lastBpmFetchMs >= 3000) {
        lastBpmFetchMs = now_b;
        // Resolve llled.local IP once and cache for fallback
        if (llled_ip_string.length() == 0) {
          IPAddress llledIp;
          if (WiFi.hostByName("llled.local", llledIp)) {
            llled_ip_string = llledIp.toString();
          }
        }
        auto doBpmGet = [](const String &url) -> int {
          // Returns HTTP code; body parsing happens outside
          HTTPClient http;
          http.begin(url);
          http.setTimeout(2000);
          const int code = http.GET();
          http.end();
          return code;
        };
        // Try mDNS first, then IP fallback
        auto doBpmFetch = [](const String &url) -> String {
          HTTPClient http;
          http.begin(url);
          http.setTimeout(2000);
          const int code = http.GET();
          String body;
          if (code == 200) body = http.getString();
          http.end();
          return body;
        };
        String body = doBpmFetch("http://llled.local/v0/bpm");
        if (body.length() == 0 && llled_ip_string.length() > 0) {
          body = doBpmFetch("http://" + llled_ip_string + "/v0/bpm");
        }
        if (body.length() > 0) {
          const bool enabled = body.indexOf("\"enabled\":true") >= 0;
          float bpm = 0.0f;
          const int bp = body.indexOf("\"bpm\":");
          if (bp >= 0) bpm = body.substring(bp + 6).toFloat();
          barmode_bpm = (enabled && bpm >= 40.0f && bpm <= 220.0f) ? bpm : 0.0f;
          Console.printf("[BPM] %.1f (enabled=%d)\n", barmode_bpm, (int)enabled);
        } else {
          barmode_bpm = 0.0f;  // offline or error: revert to default patterns
          Console.println("[BPM] fetch failed, reverting to default patterns");
        }
      }
    }

    // BARMODE: per-pylon ping — one pylon per second, rotating through the fleet.
    // Runs on Core 0 only; 250ms timeout per ping so it never starves other Core-0 work.
    if (barmode_active && barmode_ping_stats_mutex) {
      static unsigned long lastPylonPingMs = 0;
      static int           pylonPingIdx    = 0;
      const unsigned long  now_p           = millis();
      if (now_p - lastPylonPingMs >= 1000) {
        lastPylonPingMs = now_p;
        // Snapshot IPs + IDs from registry JSON
        struct LPTarget { uint32_t ip_u32; char id[24]; };
        LPTarget targets[kPylonPingStatMax];
        int tgtCount = 0;
        {
          String json;
          if (xSemaphoreTake(barmode_registry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            json = barmode_registry_json;
            xSemaphoreGive(barmode_registry_mutex);
          }
          int search = 0;
          while (tgtCount < kPylonPingStatMax) {
            int ipPos = json.indexOf("\"ip\":\"", search);
            if (ipPos < 0) break;
            ipPos += 6;
            int ipEnd = json.indexOf('"', ipPos);
            if (ipEnd < 0) break;
            const String ipStr = json.substring(ipPos, ipEnd);
            search = ipEnd + 1;
            if (ipStr.length() < 7 || ipStr.indexOf('.') < 0) continue;
            IPAddress addr;
            if (!addr.fromString(ipStr)) continue;
            const uint32_t ip32 = (uint32_t)addr;
            bool dup = false;
            for (int j = 0; j < tgtCount; j++) { if (targets[j].ip_u32 == ip32) { dup = true; break; } }
            if (dup) continue;
            targets[tgtCount].ip_u32 = ip32;
            // Extract pylon_id from nearby JSON window
            const int ws = max(0, ipPos - 400);
            const int we = min((int)json.length(), ipEnd + 400);
            const String win = json.substring(ws, we);
            const int pi = win.indexOf("\"pylon_id\":\"");
            if (pi >= 0) {
              const int ps = pi + 12;
              const int pe = win.indexOf('"', ps);
              const String pid = (pe > 0) ? win.substring(ps, pe) : ipStr;
              pid.toCharArray(targets[tgtCount].id, sizeof(targets[tgtCount].id));
            } else {
              strncpy(targets[tgtCount].id, ipStr.c_str(), sizeof(targets[tgtCount].id) - 1);
            }
            targets[tgtCount].id[sizeof(targets[tgtCount].id) - 1] = '\0';
            tgtCount++;
          }
        }
        // Append resolved manual pylons (skip duplicates by IP)
        for (int i = 0; i < barmode_manual_pylon_count && tgtCount < kPylonPingStatMax; i++) {
          if (!barmode_manual_pylons[i].resolved) continue;
          const uint32_t mip = (uint32_t)barmode_manual_pylons[i].ip;
          bool dup = false;
          for (int j = 0; j < tgtCount; j++) { if (targets[j].ip_u32 == mip) { dup = true; break; } }
          if (!dup) {
            targets[tgtCount].ip_u32 = mip;
            strncpy(targets[tgtCount].id, barmode_manual_pylons[i].host, sizeof(targets[tgtCount].id) - 1);
            targets[tgtCount].id[sizeof(targets[tgtCount].id) - 1] = '\0';
            tgtCount++;
          }
        }
        if (tgtCount > 0) {
          pylonPingIdx = pylonPingIdx % tgtCount;
          IPAddress tgtAddr;
          tgtAddr = targets[pylonPingIdx].ip_u32;
          const char *tgtId = targets[pylonPingIdx].id;
          const bool ok  = Ping.ping(tgtAddr, 1);
          const uint32_t rtt = ok ? static_cast<uint32_t>(Ping.averageTime()) : 0;
          Console.printf("[PylonPing] %s %s %ums\n", tgtId, ok ? "ok" : "lost", rtt);
          if (xSemaphoreTake(barmode_ping_stats_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            int slot = -1;
            for (int i = 0; i < barmode_pylon_ping_count; i++) {
              if (barmode_pylon_ping_stats[i].ip_u32 == targets[pylonPingIdx].ip_u32) { slot = i; break; }
            }
            if (slot < 0 && barmode_pylon_ping_count < kPylonPingStatMax) {
              slot = barmode_pylon_ping_count++;
              barmode_pylon_ping_stats[slot] = {};
              barmode_pylon_ping_stats[slot].ip_u32 = targets[pylonPingIdx].ip_u32;
              barmode_pylon_ping_stats[slot].min_ms = UINT32_MAX;
            }
            if (slot >= 0) {
              strncpy(barmode_pylon_ping_stats[slot].id, tgtId,
                      sizeof(barmode_pylon_ping_stats[slot].id) - 1);
              barmode_pylon_ping_stats[slot].id[sizeof(barmode_pylon_ping_stats[slot].id) - 1] = '\0';
              barmode_pylon_ping_stats[slot].last_ok = ok;
              if (ok) {
                barmode_pylon_ping_stats[slot].last_ms  = rtt;
                barmode_pylon_ping_stats[slot].count++;
                barmode_pylon_ping_stats[slot].sum_ms  += rtt;
                if (rtt < barmode_pylon_ping_stats[slot].min_ms) barmode_pylon_ping_stats[slot].min_ms = rtt;
                if (rtt > barmode_pylon_ping_stats[slot].max_ms) barmode_pylon_ping_stats[slot].max_ms = rtt;
              } else {
                barmode_pylon_ping_stats[slot].lost++;
              }
            }
            xSemaphoreGive(barmode_ping_stats_mutex);
          }
          pylonPingIdx = (pylonPingIdx + 1) % tgtCount;
        }
      }
    }

    // OSC proxy: drain queue and forward messages via rpiboosh POST /api/osc/send.
    // Active when cfg_route_via_rpi is true; queue is populated by SendOscFloatToIP on Core 1.
    if (osc_proxy_queue) {
      OscProxyMsg msg;
      while (xQueueReceive(osc_proxy_queue, &msg, 0) == pdTRUE) {
        IPAddress dest;
        dest = msg.ip_u32;
        const String host = dest.toString();
        const String body = "{\"address\":\"" + String(msg.address) +
                            "\",\"args\":[" + String(msg.value, 4) +
                            "],\"host\":\"" + host +
                            "\",\"port\":" + String(msg.port) + "}";
        HTTPClient http;
        http.begin(String(kRegistryBaseUrlPrimary) + "/api/osc/send");
        http.setTimeout(500);
        http.addHeader("Content-Type", "application/json");
        const int code = http.POST(body);
        http.end();
        Console.printf("[OSCProxy] %s %.3f -> %s : %d\n", msg.address, msg.value, host.c_str(), code);
      }
    }

    // all-4 rpiboosh solenoid API: open main + ring while phase-4 is active.
    // POST /api/solenoid/main/open and /api/solenoid/ring/open — no body, no headers.
    // Each /open auto-closes after 5s; re-calling /open resets the timer, so we
    // repeat every 3s as keepalive. On release we POST /close immediately.
    // Runs here (Core 0) so blocking HTTP never touches the main OSC loop.
    if (barmode_active) {
      static bool          sol_held_prev = false;
      static unsigned long sol_ka_ms     = 0;
      const  bool          sol_held_now  = barmode_all4_midi_held;

      // Try primary URL; fall back to resolved IP on port 5000 if primary fails.
      auto postSolenoid = [](const char *path) {
        const String primary = String(kRegistryBaseUrlPrimary) + path;
        HTTPClient http;
        http.begin(primary);
        http.setTimeout(500);
        const int code = http.POST("");
        http.end();
        if (code == 200) return;
        // Fallback to resolved IP
        if (target_ip_string.length() > 0) {
          const String fallback = "http://" + target_ip_string + ":5000" + path;
          http.begin(fallback);
          http.setTimeout(500);
          http.POST("");
          http.end();
        }
      };

      if (sol_held_now && !sol_held_prev) {
        // Rising edge: open both solenoid groups
        Console.println("[BarMode] all4 rpiboosh solenoid open (main+ring)");
        postSolenoid("/api/solenoid/main/open");
        postSolenoid("/api/solenoid/ring/open");
        sol_ka_ms = millis();
      } else if (!sol_held_now && sol_held_prev) {
        // Falling edge: close both solenoid groups immediately
        Console.println("[BarMode] all4 rpiboosh solenoid close (main+ring)");
        postSolenoid("/api/solenoid/main/close");
        postSolenoid("/api/solenoid/ring/close");
      } else if (sol_held_now && millis() - sol_ka_ms >= 3000) {
        // Keepalive: re-open to reset the 5s server-side auto-close timer
        sol_ka_ms = millis();
        postSolenoid("/api/solenoid/main/open");
        postSolenoid("/api/solenoid/ring/open");
      }

      sol_held_prev = sol_held_now;
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

  PollMeshOscQueue();
  PollBlinkLeds();
  PollSosBlueLed();
  PollSosYellowLed();  // overrides yellow with fast SOS when mesh_en + no WiFi + 0 peers
  PollBarModeButtons();
  PollBarModeBtn0Chase();  // must run last — overwrites all 3 LEDs while btn0 held
  if (group_pattern_pending_n > 0 && active_seq == SEQ_NONE) {
    group_seq_n_val = group_pattern_pending_n;
    group_pattern_pending_n = 0;
    StartSequence(SEQ_GROUP);
  }
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

  const unsigned long now = millis();
  const bool wifi_up = (WiFi.status() == WL_CONNECTED);

  if (!wifi_up) {
    static unsigned long disconnected_since_ms = 0;
    static unsigned long last_reconnect_attempt_ms = 0;

    if (wasConnected) {
      wasConnected = false;
      disconnected_since_ms = now;
      last_reconnect_attempt_ms = 0;
      target_ip_string = "";
      registry_announced = false;
      registry_next_attempt_ms = 0;
      registry_consecutive_failures = 0;
      Console.println("WiFi disconnected: registry state reset.");
      // If AP is active and was started on the STA channel (e.g. LavaLounge ch 6),
      // restart it on cfg_mesh_ch now so ESP-NOW peers can find us.
      if (ap_active && cfg_mesh_en) {
        uint8_t hw_ch = 0;
        esp_wifi_get_channel(&hw_ch, nullptr);
        if (hw_ch != (uint8_t)cfg_mesh_ch) {
          Console.printf("[AP] STA lost — restarting AP on mesh ch=%u (was ch=%u)\n",
                         cfg_mesh_ch, hw_ch);
          StopApMode();
          SetupApMode();
        }
      }
    }
    if (disconnected_since_ms == 0) {
      disconnected_since_ms = now;  // boot with no WiFi
    }

    const unsigned long offline_ms = now - disconnected_since_ms;

    // Reconnect watchdog: every 30s without live mesh peers; every 10 min with them.
    // With live peers the radio is pinned to cfg_mesh_ch for ESP-NOW. WiFi.reconnect()
    // temporarily disrupts that channel (~5-10s) but the DISCONNECTED handler re-pins
    // it if the attempt fails, so the disruption is bounded and infrequent.
    // The 10-min reboot is suppressed in AP mode (AP mode is intentional).
    // With setAutoReconnect(false), failed attempts fire DISCONNECTED again.
    const unsigned long reconnect_interval_ms =
        (cfg_mesh_en && mesh_live_peer_count > 0) ? 600000UL : 30000UL;
    if (now - last_reconnect_attempt_ms >= reconnect_interval_ms) {
      last_reconnect_attempt_ms = now;
      Console.printf("[WiFi] Offline %.0fs — reconnect attempt (peers=%d ap=%d).\n",
                     offline_ms / 1000.0f, (int)mesh_live_peer_count, ap_active);
      WiFi.reconnect();
    }
    if (!ap_active && offline_ms >= 600000UL) {
      Console.println("[WiFi] Offline 10 min — rebooting.");
      ESP.restart();
    }
    // No return — fall through so OLED page cycling continues even without WiFi.
    // The WiFi metrics pages and mesh page in the cycle convey connectivity state.
  } else {
    PollOsc();
    if (!wasConnected) {
      wasConnected = true;
      registry_announced = false;
      registry_next_attempt_ms = now;
      registry_consecutive_failures = 0;
      RestartMdnsIfConnected();
      oscUdp.stop();
      oscUdp.begin(kOscPort);
      SetupWebServer();
      Console.printf("WiFi connected: mDNS/OSC/web restarted (ap_active=%d ap_auto=%d)\n",
                     ap_active, ap_auto_enabled);
    }
    // Registry HTTP is handled in PingTask (Core 0) — no blocking call here.
    if (!wifi_has_ip && wifi_connected_since_ms == 0) {
      wifi_connected_since_ms = now;
    }
  }
  if (boosh_failsafe_armed && millis() - boosh_failsafe_start_ms >= boosh_failsafe_timeout_ms) {
    boosh_failsafe_armed = false;
    ApplyBooshState(0.0f);
    Console.println("Failsafe: BooshMain timeout -> forcing OFF.");
    ShowStatus("Failsafe timeout", "BooshMain OFF");
    boosh_failsafe_note_until_ms = now + kBooshFailsafeNoteMs;
  }

  // DJ button timeout: auto-release if held beyond cfg_dj_timeout_s
  if (dj_btn_held && now - dj_btn_press_ms >= (unsigned long)(cfg_dj_timeout_s * 1000.0f)) {
    dj_btn_held = false;
    if (barmode_active) SendOscFloatToAllPylons(kOscAddress, 0.0f);
    ApplyBooshState(0.0f, "dj-timeout");
    Console.println("[DJ] button timeout -> forcing OFF");
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

    // Barmode WAIT screen: override display while blue held during lockout, or on blocked tap (recovery)
    static unsigned long lastWaitDisplayMs = 0;
    const bool show_recovery_wait = (barmode_recovery_wait_until_ms > 0 && now < barmode_recovery_wait_until_ms);
    if (barmode_show_wait_oled) {
      if (now - lastWaitDisplayMs >= 500) {
        lastWaitDisplayMs = now;
        unsigned long rem_ms = (barmode_all4_lockout_until_ms > now) ? (barmode_all4_lockout_until_ms - now) : 0;
        unsigned int rem_s = (unsigned int)(rem_ms / 1000);
        const unsigned int mins = rem_s / 60;
        const unsigned int secs = rem_s % 60;
        char timebuf[6];
        snprintf(timebuf, sizeof(timebuf), "%02u:%02u", mins, secs);
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("WAIT");
        display.setTextSize(3);
        // "MM:SS" at size 3 = 5 chars × 18px = 90px wide; center at x = (128-90)/2 = 19
        display.setCursor(19, 8);
        display.print(timebuf);
        display.display();
      }
    } else if (show_recovery_wait) {
      if (now - lastWaitDisplayMs >= 500) {
        lastWaitDisplayMs = now;
        unsigned long rem_ms = (barmode_recovery_wait_until_ms > now) ? (barmode_recovery_wait_until_ms - now) : 0;
        unsigned int rem_s = (unsigned int)((rem_ms + 999) / 1000);  // ceiling seconds
        char secbuf[8];
        if (rem_s >= 60) {
          snprintf(secbuf, sizeof(secbuf), "%u:%02u", rem_s / 60, rem_s % 60);
        } else {
          snprintf(secbuf, sizeof(secbuf), "%us", rem_s);
        }
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print("WAIT");
        display.setTextSize(3);
        display.setCursor(0, 8);
        display.print(secbuf);
        display.display();
      }
    } else if (lastDisplayMs == 0) {
      lastDisplayMs = now;
    } else if (now - lastDisplayMs >= (
        // Mesh container page dwells long enough for all sub-pages (1500ms each).
        // All other pages use the normal kDisplayCycleMs.
        (displayPage == 3 && displayOtherIdx == 6)
          ? (unsigned long)(max(1, ((int)mesh_live_peer_count + 1) / 2)) * 1500UL
          : (unsigned long)kDisplayCycleMs)) {
      lastDisplayMs = now;
      if (barmode_active) {
        // Barmode: no battery/temp hardware — skip those pages, only cycle info sub-pages
        displayPage     = 3;
        displayOtherIdx = static_cast<uint8_t>((displayOtherIdx + 1) % 7);
      } else {
        displayPage = static_cast<uint8_t>((displayPage + 1) % 4);
        if (displayPage == 3) {
          displayOtherIdx = static_cast<uint8_t>((displayOtherIdx + 1) % 7);
        }
      }
    }

    // Slot 0, 1 → temp °F + battery pct  |  Slot 2 → time remaining + voltage
    // Slot 3 → info sub-pages (ping/wifi/wifi-detail/node/firmware)
    // In barmode slots 0-2 are skipped entirely (no sensor hardware).
    if (barmode_show_wait_oled || show_recovery_wait) {
      // WAIT screen rendered above; skip normal pages
    } else if (!barmode_active && (displayPage == 0 || displayPage == 1)) {
      ShowTempPctPage();
    } else if (!barmode_active && displayPage == 2) {
      ShowTimeVoltagePage();
    } else {
      // displayOtherIdx: 0=ping, 1=wifi, 2=wifi detail, 3=node, 4=firmware, 5=pylon-ping(barmode), 6=mesh
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
      } else if (displayOtherIdx == 5 && barmode_active) {
        ShowPylonPingPage();
      } else if (displayOtherIdx == 6) {
        ShowMeshPage();
      } else {
        ShowFirmwarePage();
      }
    }
  }

  delay(5);
}
