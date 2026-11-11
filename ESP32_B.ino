/*
  ============================================================================
  SMART WATER TANK - ESP32_B (INDOOR MAIN CONTROLLER)
  ============================================================================
  Board target : ESP32 DevKit / ESP32-WROOM-32 (or equivalent)
  Arduino core : esp32:esp32 3.3.11 (tested build target; 3.x ESP-NOW API)
  Libraries    : U8g2 (SSD1306 full framebuffer)
  Web server   : built-in WebServer (HTTP polling, JSON; no CDN/cloud)

  Responsibilities:
    * receive water level, touch states and START/STOP commands from ESP32_A
    * send sensor/status information back to ESP32_A
    * 750 L tank percentage/volume model, full-tank buzzer
    * FILLING/DRAINING touch inputs -> indicator LEDs and dashboard capsules
    * HC-SR04 person detection -> Relay 1 automatic mirror light
    * phone Mirror Light control, 3 s button manual override, Relay 2, RF
    * 0.96" I2C OLED, Wi-Fi AP "Smart Water Tank", local dashboard
    * four physical buttons: one buzzer-stop/manual-override, three RESERVED

  The dashboard HTML is embedded below as DASHBOARD_HTML and is identical to
  index.html.  If you edit index.html, re-embed it (see README.md, section
  "Editing the dashboard").

  USER-SPECIFIC VALUES TO CHANGE (marked ">>> CHANGE"):
  1. ESP32_A_MAC = ESP32_A station MAC (printed by ESP32_A at boot).
  2. WIFI_CHANNEL = same channel as ESP32_A.
  3. AP_WIFI_PASSWORD = your chosen password (min 8 chars).
  4. Verify every GPIO against YOUR board before wiring.
  5. Set RELAY_ACTIVE_HIGH to match your relay module.
  6. Verify the RF remote PCB before connecting its pads (README).
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <FS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
   CONFIGURATION - ESP32_B
   ========================================================================== */

#define DEBUG_SERIAL 1                 // 1 = diagnostic serial output, 0 = quiet

// ESP32_A station MAC address (printed by ESP32_A at boot).
static const uint8_t ESP32_A_MAC[6] = {0x00, 0x70, 0x07, 0xA1, 0xDC, 0x80};

// Wi-Fi AP + ESP-NOW channel. MUST exactly match ESP32_A.ino. Default 6.
#define WIFI_CHANNEL 6
static const char AP_SSID[] = "Smart Water Tank";          // exact SSID required
static const char AP_WIFI_PASSWORD[] = "dks12345";
static const uint8_t AP_MAX_CLIENTS = 4;

// ArduinoOTA (wireless firmware upload while connected to ESP32_B AP).
// One-time USB flash of this OTA-enabled firmware is required first.
static const char OTA_HOSTNAME[] = "ESP32-WATER-B";
static const char OTA_PASSWORD[] = "swtb-ota-2026";  // >>> CHANGE before deployment

// GPIO allocation (ESP32 DevKit V1 style). Verify YOUR board pinout!
#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22
#define OLED_I2C_ADDRESS   0x3C
#define OLED_WIDTH         128
#define OLED_HEIGHT        64

#define PIN_HCSR_TRIG      25
#define PIN_HCSR_ECHO      26

#define PIN_RELAY1         32   // Mirror Light
#define PIN_RELAY2         33   // Bulb
#define RELAY_ACTIVE_HIGH  false // Most 5 V opto relay boards are ACTIVE LOW

#define PIN_BUZZER         13   // drives NPN base resistor, not buzzer directly
#define BUZZER_ACTIVE_HIGH true
#define BUZZER_IS_ACTIVE   true  // TMB12A05 is an active buzzer; see README

#define PIN_BTN_BUZZER_STOP 4    // other side of button to GND; INPUT_PULLUP
#define BUTTON_ACTIVE_LOW   true

#define PIN_IND_BLUE       16   // FILLING indicator (NPN/MOSFET driver)
#define PIN_IND_RED        17   // DRAINING indicator
#define IND_ACTIVE_HIGH    true

#define PIN_RF_START       18   // to isolated remote-button switching circuit
#define PIN_RF_STOP        19
#define RF_ACTIVE_HIGH     true

// RESERVED, intentionally unassigned physical buttons (do NOT init in code):
//   GPIO 2, GPIO 14, GPIO 23.  Assign only after reviewing boot-strap rules.

// Timing / filtering.
static const uint32_t LINK_TIMEOUT_MS        = 8000;
static const uint32_t OLED_REFRESH_MS        = 250;
static const uint32_t HC_SAMPLE_PERIOD_MS    = 250;
static const uint32_t HC_ECHO_TIMEOUT_US     = 30000;
static const uint32_t HC_MIN_ECHO_US         = 100;
static const uint32_t HC_MAX_ECHO_US         = 25000;
static const float    HC_DETECT_ON_CM        = 100.0f;
static const float    HC_DETECT_OFF_CM       = 105.0f;  // hysteresis
static const uint32_t HC_ON_CONFIRM_MS        = 400;
static const uint32_t HC_OFF_CONFIRM_MS       = 800;
static const uint8_t  HC_FILTER_SAMPLES      = 5;
static const float    SOUND_CM_PER_US        = 0.0343f;

static const uint32_t STATUS_TX_PERIOD_MS    = 1000;
static const uint32_t MOTOR_MAX_PRESS_MS     = 4000;
static const uint32_t BUTTON_DEBOUNCE_MS     = 50;
static const uint32_t BUTTON_LONG_PRESS_MS   = 3000;

static const float    FULL_LEVEL_PERCENT     = 97.0f; // buzzer alert threshold
static const uint8_t  FULL_CONFIRM_READINGS  = 2;
static const float    REARM_PERCENT          = 93.0f;
static const uint32_t BUZZER_ALERT_MAX_MS    = 40000;
static const uint32_t BUZZER_TEST_MS         = 5000;
static const uint32_t BUZZER_BEEP_PERIOD_MS  = 333;  // 3 beeps per second
static const uint32_t BUZZER_BEEP_ON_MS      = 120;

#define B_SETTINGS_MAGIC 0x42535432UL   // "BST2"
#define B_SETTINGS_VERSION 2u

// History storage limits. Samples are CSV lines in LittleFS; old lines are
// compacted automatically so the log stays bounded.
#define HISTORY_FILE "/history.csv"
#define EVENTS_FILE  "/events.csv"
#define HISTORY_MIN_INTERVAL_SEC 30u
#define HISTORY_MAX_INTERVAL_SEC 3600u
#define HISTORY_MAX_POINTS_RETURNED 1400
#define EVENTS_MAX_LINES 1200
#define HISTORY_MAX_BYTES 700000UL

// OLED field bits.
#define OLED_SHOW_PERCENT   0x01u
#define OLED_SHOW_LITRES    0x02u
#define OLED_SHOW_LINK      0x04u
#define OLED_SHOW_BOTTOM_L  0x08u
#define OLED_SHOW_STATUS    0x10u
#define OLED_SHOW_DISTANCE  0x20u

// Protocol constants (must match ESP32_A.ino).
#define PROTOCOL_VERSION 2u
#define PKT_MAGIC 0x57544131UL   // "WTA1"

#define FLAG_NO_ECHO  0x0001u
#define FLAG_RANGE    0x0002u
#define FLAG_JUMP     0x0004u
#define FLAG_STUCK    0x0008u
#define FLAG_TIMEOUT  0x0010u

enum PktType : uint8_t {
  PKT_DATA     = 1,
  PKT_STATUS   = 2,
  PKT_COMMAND  = 3,
  PKT_SETTINGS = 4
};

#define CMD_MOTOR_START 1u
#define CMD_MOTOR_STOP  2u
#define CMD_REBOOT_A    3u

#define STATUS_RELAY1       0x0001u
#define STATUS_RELAY2       0x0002u
#define STATUS_BUZZER       0x0004u
#define STATUS_MOTOR_START  0x0008u
#define STATUS_MOTOR_STOP   0x0010u
#define STATUS_PERSON       0x0020u
#define STATUS_WATER_ONLINE 0x0040u

struct __attribute__((packed)) WaterPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t flags;
  uint32_t seq;
  uint32_t uptimeMs;
  float    distanceCm;
  float    percent;
  uint16_t litres;
  uint8_t  valid;
  uint8_t  filling;
  uint8_t  draining;
  uint8_t  validEchoes;
  uint8_t  timeoutEchoes;
  uint8_t  resetReason;
  uint8_t  reserved;
  uint32_t settingsRevision;
  uint16_t crc;
};

// ESP32_A protocol v1 packet (legacy A firmware without runtime settings).
// B still accepts it so a B-only update keeps the existing tank sensor online.
struct __attribute__((packed)) WaterPacketV1 {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t flags;
  uint32_t seq;
  uint32_t uptimeMs;
  float    distanceCm;
  float    percent;
  uint16_t litres;
  uint8_t  valid;
  uint8_t  filling;
  uint8_t  draining;
  uint8_t  reserved;
  uint16_t crc;
};

struct __attribute__((packed)) StatusPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t flags;
  uint32_t seq;
  uint32_t uptimeMs;
  uint32_t ackSeq;
  float    hcDistanceCm;
  float    waterPercent;
  float    waterDistanceCm;
  uint8_t  hcValid;
  uint8_t  reserved[3];
  uint16_t crc;
};

struct __attribute__((packed)) CommandPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;
  uint8_t  command;
  uint8_t  pressed;
  uint32_t seq;
  uint32_t uptimeMs;
  uint8_t  reserved[4];
  uint16_t crc;
};

struct __attribute__((packed)) SettingsPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t reserved0;
  uint32_t revision;
  float    fullDistanceCm;
  float    emptyDistanceCm;
  float    emptyDeadbandCm;
  float    tankCapacityL;
  float    sensorMinCm;
  float    sensorMaxCm;
  float    smoothAlpha;
  float    jumpLimitCm;
  uint32_t stuckTimeoutMs;
  uint16_t measurePeriodMs;
  uint16_t sampleSpacingMs;
  uint8_t  jumpConfirm;
  uint8_t  samplesPerCycle;
  uint8_t  reserved[2];
  uint16_t crc;
};

struct __attribute__((packed)) AppSettings {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  uint32_t revision;
  // Water sensor configuration forwarded to ESP32_A.
  uint32_t aRevision;
  float    fullDistanceCm;
  float    emptyDistanceCm;
  float    emptyDeadbandCm;
  float    tankCapacityL;
  float    sensorMinCm;
  float    sensorMaxCm;
  float    smoothAlpha;
  float    jumpLimitCm;
  uint32_t stuckTimeoutMs;
  uint16_t measurePeriodMs;
  uint16_t sampleSpacingMs;
  uint8_t  jumpConfirm;
  uint8_t  samplesPerCycle;
  uint8_t  reserved1[2];
  // Mirror sensor.
  uint8_t  mirrorEnabled;
  float    mirrorOnCm;
  float    mirrorOffCm;
  uint16_t mirrorOnConfirmMs;
  uint16_t mirrorOffConfirmMs;
  uint16_t mirrorInvalidHoldoffMs;
  uint16_t mirrorMaxOnMinutes;
  // Buzzer.
  uint8_t  buzzerEnabled;
  uint8_t  buzzerIntensity;
  uint8_t  buzzerConfirmReadings;
  uint8_t  reserved2;
  float    buzzerFullPercent;
  float    buzzerRearmPercent;
  uint32_t buzzerDurationMs;
  uint16_t buzzerBeepPeriodMs;
  uint16_t buzzerBeepOnMs;
  uint16_t reserved3;
  // OLED.
  uint8_t  oledShow;
  uint8_t  oledPercentSize;
  uint8_t  oledLitresSize;
  uint8_t  oledBottomSize;
  uint8_t  oledContrast;
  uint8_t  oledInvert;
  uint8_t  reserved4[2];
  // History / admin.
  uint16_t historyIntervalSec;
  uint8_t  historyRetentionDays;
  uint8_t  historyEnabled;
  char     reservedAdminPass[8];  // legacy NVS slot, passcode disabled
  uint16_t reservedAdminSession;
  uint16_t reserved5;
};

static_assert(sizeof(WaterPacket) == 39, "WaterPacket size must match ESP32_A");
static_assert(sizeof(WaterPacketV1) == 32, "WaterPacketV1 size must match legacy ESP32_A");
static_assert(sizeof(SettingsPacket) == 58, "SettingsPacket size must match ESP32_A");

/* ==========================================================================
   DEBUG
   ========================================================================== */

#if DEBUG_SERIAL
  #define DBG(...) Serial.printf(__VA_ARGS__)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(...) do {} while (0)
  #define DBGLN(x) do {} while (0)
#endif

/* ==========================================================================
   EMBEDDED DASHBOARD (identical copy of index.html)
   ========================================================================== */

static const char DASHBOARD_HTML[] PROGMEM = R"WTDASH(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<meta name="theme-color" content="#eef8f2">
<title>Smart Water Tank</title>
<style>
:root{--ink:#10243d;--muted:#66778e;--line:#dce6ef;--green:#0e9a63;--red:#df3c4f;--blue:#128fd1}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;width:100%;height:100%;overflow:hidden;overscroll-behavior:none}
body{font-family:Inter,system-ui,-apple-system,"Segoe UI",Arial,sans-serif;color:var(--ink);touch-action:manipulation;background:radial-gradient(circle at 12% -8%,#d9f8e8 0,transparent 34%),radial-gradient(circle at 92% 4%,#dff1ff 0,transparent 30%),linear-gradient(155deg,#fbfefd 0%,#f0f7fb 55%,#e9f1f8 100%)}
button{font:inherit}
.app{width:min(100%,700px);height:100dvh;margin:auto;padding:clamp(8px,1.8vh,15px) 14px max(20px,env(safe-area-inset-bottom));display:flex;flex-direction:column;gap:clamp(7px,1.4vh,12px)}
header{min-height:clamp(54px,8.6vh,70px);flex:none;display:flex;align-items:center;justify-content:space-between;gap:8px}
.brand{display:flex;align-items:center;gap:12px;min-width:0}
.logo{width:clamp(54px,8.4vh,68px);height:clamp(54px,8.4vh,68px);flex:none;border-radius:18px;background:linear-gradient(145deg,#ffffff,#d8f4e3);border:1px solid #c2e7d1;display:grid;place-items:center;box-shadow:inset 0 2px 3px #fff,inset 0 -7px 12px #0b855218,0 10px 22px #0d7c4b22,0 2px 0 #fff}
.logo img{width:94%;height:94%;object-fit:contain;border-radius:14px;filter:drop-shadow(0 3px 3px #0a5d3b22)}
.brand-title{font-size:clamp(23px,4vw,33px);font-weight:950;letter-spacing:-.95px;white-space:nowrap;text-shadow:0 2px 0 #fff,0 8px 18px #102b451f}
.credit{font-size:clamp(9px,1.55vh,11px);color:var(--muted);margin-top:2px;font-weight:750;letter-spacing:.15px}
.connection{display:flex;align-items:center;gap:7px;padding:9px 12px;border-radius:24px;background:linear-gradient(145deg,#f4fff9,#ddf8e9);border:1px solid #aee3c4;color:#087c4a;font-size:11px;font-weight:950;letter-spacing:.7px;flex:none;box-shadow:inset 0 1px 0 #fff,0 5px 12px #0d9b5e1f}
.connection.offline{background:linear-gradient(145deg,#fff7f8,#ffe4e7);border-color:#f0b9c1;color:#c52c40;box-shadow:inset 0 1px 0 #fff,0 5px 12px #df3c4f1f}
.dot{width:9px;height:9px;border-radius:50%;background:#16a66a;box-shadow:0 0 0 3px #16a66a20,0 0 12px #16a66a88}
.offline .dot{background:#df3c4f;box-shadow:0 0 0 3px #df3c4f20,0 0 12px #df3c4f88}
.card{background:linear-gradient(160deg,#ffffff 0%,#f9fcff 100%);border:1px solid var(--line);border-radius:22px;box-shadow:0 18px 36px #142b4712,0 3px 0 #fff inset,0 -5px 14px #102b4510 inset}
.tank{padding:clamp(10px,1.8vh,16px) 17px;flex:none;background:linear-gradient(150deg,#ffffff 0%,#f4fbf8 52%,#edf6fc 100%)}
.card-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:8px}
.eyebrow{font-size:11px;font-weight:950;letter-spacing:1.25px;color:#687b91;text-transform:uppercase}
.live{font-size:10px;color:var(--green);font-weight:950;letter-spacing:.75px;text-shadow:0 1px 0 #fff}
.tank.stale .live{opacity:.35}
.water-row{display:flex;align-items:flex-end;justify-content:space-between;gap:8px}
.percent{font-size:clamp(58px,10.5vh,92px);font-weight:950;letter-spacing:-4.5px;line-height:.82;color:#0d2440;font-variant-numeric:tabular-nums;text-shadow:0 2px 0 #fff,0 9px 18px #102b4522}
.percent small{font-size:.36em;color:#65778c;letter-spacing:-.5px;margin-left:3px}
.volume{text-align:right;padding-bottom:4px}
.volume strong{font-size:clamp(38px,6.6vh,58px);font-weight:950;display:block;letter-spacing:-3px;line-height:.84;font-variant-numeric:tabular-nums;color:#0d2440;text-shadow:0 2px 0 #fff,0 7px 15px #102b451c}
.volume span{font-size:clamp(12px,1.9vh,15px);color:var(--muted);font-weight:900;letter-spacing:.8px}
.bar-wrap{height:13px;border-radius:14px;background:linear-gradient(180deg,#dfe8f0,#eef4f8);border:1px solid #d5e0ea;overflow:hidden;margin:12px 0 6px;box-shadow:inset 0 3px 6px #102b451c,0 1px 0 #fff}
.bar{height:100%;width:0%;border-radius:14px;background:linear-gradient(90deg,#0c86c9,#22b8b0 55%,#53d68f);box-shadow:0 0 14px #18a9a8aa,inset 0 2px 0 #ffffff66,inset 0 -3px 5px #075b6d38;transition:width .4s}
.scale{display:flex;justify-content:space-between;color:#788a9d;font-size:9px;font-weight:850;letter-spacing:.2px}
.tank-bottom{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-top:8px;min-height:30px}
.touch-capsules{display:flex;align-items:center;gap:6px;min-width:0}
.capsule{font-size:clamp(9px,1.55vh,12px);font-weight:950;letter-spacing:.75px;border-radius:999px;padding:7px 12px;border:1px solid #dbe3eb;background:linear-gradient(180deg,#f8fafc,#e9eef3);color:#65768a;box-shadow:inset 0 1px 0 #fff,0 3px 0 #b9c5d0,0 6px 10px #102b4518;white-space:nowrap;transition:.18s}
.capsule.hidden{display:none}
.capsule.filling.active{color:#fff;border-color:#2eb7f0;background:linear-gradient(145deg,#59d4ff,#087fce);box-shadow:inset 0 2px 0 #ffffff70,0 4px 0 #075f9e,0 0 15px #1ba8ffcc,0 0 32px #1ba8ff66;animation:bluePulse 1.7s ease-in-out infinite}
.capsule.overflowing.active{color:#fff;border-color:#f27182;background:linear-gradient(145deg,#ff8794,#d9243d);box-shadow:inset 0 2px 0 #ffffff70,0 4px 0 #a81329,0 0 15px #ff405fcc,0 0 32px #ff405f66;animation:redPulse 1.35s ease-in-out infinite}
.capsule.stable{color:#087c4a;border-color:#b9e6cd;background:linear-gradient(180deg,#f4fff9,#dff7e9);box-shadow:inset 0 1px 0 #fff,0 3px 0 #9fd4b7,0 0 12px #20aa6b33}
.capsule.stable.offline{color:#bd2d40;border-color:#efbdc4;background:linear-gradient(180deg,#fff8f9,#ffe5e8);box-shadow:inset 0 1px 0 #fff,0 3px 0 #dda7af}
@keyframes bluePulse{50%{filter:brightness(1.14);box-shadow:inset 0 2px 0 #ffffff80,0 4px 0 #075f9e,0 0 23px #36c2ff,0 0 48px #1ba8ff88}}
@keyframes redPulse{50%{filter:brightness(1.12);box-shadow:inset 0 2px 0 #ffffff80,0 4px 0 #a81329,0 0 23px #ff5772,0 0 48px #ff405f88}}
.distance{font-size:9px;color:#6f8195;font-weight:750;text-align:right;line-height:1.2}
.tank.stale .percent{opacity:1;text-decoration:underline;text-decoration-thickness:1px;text-underline-offset:4px;text-decoration-color:#8c9bad}
.tank.stale .volume strong,.tank.stale .bar{opacity:.45}
.controls{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,1.45fr) minmax(0,1fr);gap:11px}
.motor-card{padding:11px;display:flex;flex-direction:column;align-items:center;min-height:0}
.motor-title{text-align:center;margin-bottom:3px}
.motor-title .control-state{margin-top:4px;font-size:clamp(8px,1.3vh,10px);letter-spacing:.45px}
.motor-buttons{display:flex;flex:1;min-height:0;width:100%;flex-direction:column;align-items:center;justify-content:center;gap:clamp(8px,1.4vh,14px)}
.motor-unit{display:flex;flex-direction:column;align-items:center;gap:5px;min-height:0}
.motor-btn{width:clamp(98px,18.5vh,162px);height:clamp(98px,18.5vh,162px);max-width:100%;aspect-ratio:1;border:none;border-radius:50%;display:grid;place-items:center;cursor:pointer;touch-action:none;box-shadow:none;transition:transform .12s,filter .12s,opacity .12s}
.motor-btn svg{width:43%;height:43%;filter:none}
.motor-btn.start{background:#11a86a;color:#fff}
.motor-btn.stop{background:#e04658;color:#fff}
.motor-btn:active,.motor-btn.held{transform:scale(.96);filter:brightness(.96);box-shadow:none}
.motor-caption{font-size:clamp(14px,2.35vh,19px);font-weight:900;letter-spacing:1.2px;color:#1e314a;text-shadow:none}
.side-card{padding:9px;display:flex;flex-direction:column;justify-content:center;gap:clamp(7px,1.3vh,12px);min-height:0}
.control-tile{flex:1;min-height:0;border:1px solid #e0e8f0;background:linear-gradient(160deg,#ffffff,#f4f8fc);border-radius:17px;padding:9px 6px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:clamp(5px,1vh,9px);text-align:center;box-shadow:inset 0 1px 0 #fff,0 5px 12px #142b470d,0 2px 0 #d8e2eb}
.control-tile.light-on{background:linear-gradient(160deg,#fffdf1,#fff2c5);border-color:#efd16f;box-shadow:inset 0 1px 0 #fff,0 0 13px #f4d36a99,0 0 29px #f4d36a55,0 5px 12px #142b4712}
.control-tile.bulb-on{background:linear-gradient(160deg,#f4fcff,#dff4ff);border-color:#9ed4f4;box-shadow:inset 0 1px 0 #fff,0 0 13px #75c6f599,0 0 29px #75c6f555,0 5px 12px #142b4712}
.control-name{font-size:clamp(13px,2.1vh,17px);font-weight:950;line-height:1.08;letter-spacing:-.2px}
.control-state{font-size:clamp(9px,1.4vh,11px);color:#78889b;font-weight:850;line-height:1.1}
.timer{font-size:clamp(8px,1.25vh,10px);color:#5d7088;font-weight:850;min-height:10px}
.toggle{width:clamp(72px,11.5vh,92px);height:clamp(41px,6.5vh,52px);flex:none;border:1px solid #b9c7d5;background:linear-gradient(180deg,#dfe6ed,#ccd6e0);border-radius:32px;padding:4px;cursor:pointer;transition:.18s;box-shadow:inset 0 3px 6px #102b4526,0 3px 0 #aab7c4,0 6px 10px #102b4518}
.toggle:after{content:"";display:block;width:clamp(31px,4.9vh,41px);height:clamp(31px,4.9vh,41px);border-radius:50%;background:linear-gradient(145deg,#fff,#e9eef3);box-shadow:0 3px 6px #0003,inset 0 1px 1px #fff;transition:.18s}
.toggle.on{background:linear-gradient(145deg,#33d493,#0b8a59);border-color:#0b8a59;box-shadow:inset 0 2px 4px #064d3226,0 3px 0 #07633f,0 0 14px #12a96b88}
.toggle.on:after{transform:translateX(clamp(28px,4.4vh,39px))}
.buzzer-actions{display:flex;flex-direction:column;align-items:center;gap:clamp(4px,0.7vh,7px);width:100%}
.buzzer-btn{width:min(100%,198px);border:1px solid #bd1b32;background:linear-gradient(145deg,#ff7584,#db2340 58%,#b51631);color:#fff;border-radius:15px;padding:clamp(13px,2.1vh,18px) 10px;font-size:clamp(12px,2vh,15px);font-weight:950;letter-spacing:.45px;white-space:nowrap;box-shadow:inset 0 2px 0 #ffffff66,0 6px 0 #901127,0 10px 15px #d91f3d33;cursor:pointer;text-shadow:0 1px 1px #780d1f88}
.buzzer-btn:active{transform:translateY(5px);box-shadow:inset 0 2px 3px #0004,0 1px 0 #901127}
.buzzer-btn.test{border-color:#087a4c;background:linear-gradient(145deg,#59e6a5,#16b978 58%,#087a4c);font-size:clamp(10px,1.6vh,12px);padding:clamp(8px,1.3vh,11px) 10px;box-shadow:inset 0 2px 0 #ffffff66,0 5px 0 #05633e,0 9px 13px #0d9b5e26;text-shadow:0 1px 1px #04523388}
.buzzer-btn.test:active{transform:translateY(4px);box-shadow:inset 0 2px 3px #0004,0 1px 0 #05633e}
footer{flex:none;text-align:center;font-size:clamp(9px,1.55vh,11px);line-height:1.2;color:#4e6178;font-weight:950;letter-spacing:.5px;white-space:nowrap;padding-bottom:2px;margin-bottom:clamp(8px,2vh,20px);text-shadow:0 1px 0 #fff}

.head-actions{display:flex;flex-direction:column;align-items:flex-end;gap:6px;flex:none}
.admin-open{border:1px solid #9ecbb3;background:linear-gradient(180deg,#ffffff,#e7f8ee);color:#087c4a;border-radius:11px;padding:6px 11px;font-size:10px;font-weight:950;letter-spacing:.8px;box-shadow:inset 0 1px 0 #fff,0 3px 0 #b9d8c7,0 5px 9px #102b4514;cursor:pointer}
.admin-open:active{transform:translateY(2px);box-shadow:inset 0 1px 3px #102b4520,0 1px 0 #b9d8c7}
.admin-overlay{position:fixed;inset:0;z-index:50;background:linear-gradient(155deg,#f7fcff 0%,#eef7f2 50%,#e9f1f8 100%);overflow-y:auto;overscroll-behavior:contain;padding:14px 14px max(24px,env(safe-area-inset-bottom));display:block}
.admin-overlay[hidden],.admin-notice[hidden]{display:none!important}
.admin-shell{width:min(100%,980px);margin:0 auto;display:flex;flex-direction:column;gap:12px}
.admin-head{display:flex;align-items:flex-start;justify-content:space-between;gap:10px;flex-wrap:wrap}
.admin-eyebrow{font-size:10px;font-weight:950;letter-spacing:1.4px;color:#0e9a63}
.admin-head h1{margin:2px 0 3px;font-size:clamp(22px,4vw,32px);letter-spacing:-.8px}
.admin-sub{font-size:11px;color:#66778e;font-weight:750}
.admin-head-actions{display:flex;gap:6px;flex-wrap:wrap}
.admin-head-actions button,.btn{border:1px solid #cbd9e5;background:linear-gradient(180deg,#fff,#edf4fa);color:#1c334d;border-radius:11px;padding:8px 12px;font-size:11px;font-weight:900;cursor:pointer;box-shadow:0 3px 8px #142b4712,inset 0 1px 0 #fff}
.admin-head-actions button:active,.btn:active{transform:translateY(1px)}
.btn.primary{border-color:#0b8a59;background:linear-gradient(145deg,#33d493,#0b8a59);color:#fff;box-shadow:inset 0 1px 0 #ffffff55,0 4px 0 #07633f,0 6px 12px #0b8a5933}
.btn.danger{border-color:#d83a4f;background:linear-gradient(145deg,#ff7a88,#d9243d);color:#fff;box-shadow:inset 0 1px 0 #ffffff55,0 4px 0 #a81329,0 6px 12px #df3c4f33}
.admin-notice{border-radius:12px;padding:10px 12px;font-size:12px;font-weight:800;background:#e9fbf1;border:1px solid #abe3c4;color:#087c4a}
.admin-notice.error{background:#fff0f2;border-color:#efb9c1;color:#bd2d40}
.admin-live{display:grid;grid-template-columns:repeat(auto-fit,minmax(128px,1fr));gap:8px}
.admin-stat{background:linear-gradient(160deg,#fff,#f4f9fd);border:1px solid #dce6ef;border-radius:13px;padding:9px 10px;box-shadow:inset 0 1px 0 #fff,0 4px 9px #142b470c}
.admin-stat span{display:block;font-size:9px;font-weight:950;letter-spacing:.8px;color:#74869a;text-transform:uppercase}
.admin-stat strong{display:block;font-size:14px;margin-top:2px;letter-spacing:-.2px;word-break:break-word}
.admin-card{background:#fff;border:1px solid #dce6ef;border-radius:18px;padding:13px;box-shadow:0 12px 26px #142b470e,inset 0 1px 0 #fff}
.admin-card h2{margin:0;font-size:15px;letter-spacing:-.2px}
.admin-card-head{display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap;margin-bottom:9px}
.admin-card p.note{margin:6px 0 0;color:#6b7d91;font-size:10px;line-height:1.45;font-weight:700}
.admin-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(142px,1fr));gap:9px}
.field{display:flex;flex-direction:column;gap:4px;min-width:0}
.field label{font-size:9px;font-weight:950;letter-spacing:.65px;color:#6d8094;text-transform:uppercase}
.field input,.field select{width:100%;border:1px solid #cbd9e5;background:#f8fbfe;border-radius:9px;padding:8px 9px;font:inherit;font-size:12px;font-weight:750;color:#10243d;outline:none}
.field input:focus,.field select:focus{border-color:#39b47d;box-shadow:0 0 0 3px #39b47d22;background:#fff}
.checkline{display:flex;align-items:center;gap:8px;border:1px solid #dce6ef;background:#f8fbfe;border-radius:10px;padding:8px 10px;font-size:11px;font-weight:850;color:#253b55;min-height:38px}
.checkline input{width:17px;height:17px;accent-color:#0e9a63;flex:none}
.seg{display:flex;gap:4px;flex-wrap:wrap}
.seg button{border:1px solid #cbd9e5;background:#f8fbfe;border-radius:8px;padding:6px 9px;font-size:10px;font-weight:900;color:#53677d;cursor:pointer}
.seg button.on{background:#0e9a63;border-color:#0e9a63;color:#fff;box-shadow:0 2px 0 #07633f}
.graph-wrap{position:relative;background:linear-gradient(180deg,#f9fdff,#f2f8fc);border:1px solid #dce6ef;border-radius:13px;height:clamp(190px,30vh,300px);overflow:hidden}
#histChart{width:100%;height:100%;display:block}
.chart-empty{position:absolute;inset:0;display:grid;place-items:center;font-size:12px;font-weight:850;color:#8293a6;pointer-events:none}
.chart-empty[hidden]{display:none}
.legend{display:flex;gap:12px;flex-wrap:wrap;margin-top:7px;font-size:10px;font-weight:800;color:#63768b}
.legend i{display:inline-block;width:16px;height:3px;border-radius:2px;vertical-align:middle;margin-right:4px}
.event-list{display:flex;flex-direction:column;gap:6px;max-height:280px;overflow:auto;padding-right:2px}
.event-item{border:1px solid #e3ebf2;background:#f9fcff;border-radius:10px;padding:8px 10px;display:flex;justify-content:space-between;gap:10px;align-items:flex-start}
.event-item b{font-size:11px;display:block}
.event-item small{font-size:9px;color:#788a9d;font-weight:750}
.event-item .age{font-size:9px;color:#8293a6;white-space:nowrap;font-weight:800}
.event-item.warn{background:#fff9ed;border-color:#f0d9a8}
.event-item.bad{background:#fff2f4;border-color:#efc5cb}
.settings-section{border-top:1px solid #e8eef4;padding-top:11px;margin-top:11px}
.settings-section:first-of-type{border-top:0;padding-top:0;margin-top:0}
.settings-section h3{margin:0 0 8px;font-size:12px;letter-spacing:.4px;text-transform:uppercase;color:#2a405a}
.oled-layout{display:grid;grid-template-columns:minmax(0,1.1fr) minmax(190px,.9fr);gap:10px;align-items:start}
.oled-frame{background:#08110d;border:7px solid #24352d;border-radius:16px;padding:5px;box-shadow:inset 0 0 18px #000,0 10px 20px #102b4522}
.oled-frame canvas{width:100%;aspect-ratio:2/1;display:block;border-radius:5px;background:#050907}
.savebar{position:sticky;bottom:8px;z-index:3;margin-top:13px;display:flex;align-items:center;gap:10px;justify-content:space-between;background:#ffffffed;backdrop-filter:blur(8px);border:1px solid #dce6ef;border-radius:14px;padding:9px;box-shadow:0 10px 24px #142b4720}
.savebar span{font-size:10px;color:#6b7d91;font-weight:800}
.info-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:7px;font-size:10px}
.info-row{display:flex;justify-content:space-between;gap:8px;border-bottom:1px dashed #e7eef5;padding:4px 0}
.info-row span{color:#73859a;font-weight:750}
.info-row b{text-align:right;word-break:break-word}
@media(max-width:520px){.oled-layout{grid-template-columns:1fr}.admin-overlay{padding:10px 9px max(18px,env(safe-area-inset-bottom))}.admin-live{grid-template-columns:repeat(2,minmax(0,1fr))}.admin-head-actions{width:100%;justify-content:flex-end}.admin-head-actions button{flex:1}}
@media(max-height:620px){.app{gap:5px;padding-top:5px}.logo{width:48px;height:48px}.brand-title{font-size:20px}.tank{padding:7px 12px}.bar-wrap{margin:6px 0 4px}.tank-bottom{margin-top:5px}.percent{font-size:clamp(48px,10vh,72px)}.volume strong{font-size:clamp(34px,6.2vh,46px)}.motor-btn{width:clamp(76px,16vh,112px);height:clamp(76px,16vh,112px)}.motor-buttons{gap:5px}.motor-caption{font-size:12px}.side-card{gap:5px;padding:6px}.control-tile{padding:5px}.toggle{width:60px;height:35px}.toggle:after{width:27px;height:27px}.toggle.on:after{transform:translateX(23px)}.buzzer-btn{padding:9px 6px}}
@media(orientation:landscape) and (max-height:520px){.app{width:min(100%,980px);padding:4px 12px 5px;gap:4px}header{min-height:36px}.logo{width:42px;height:42px}.brand-title{font-size:18px}.credit{font-size:8px}.tank{padding:5px 12px}.percent{font-size:42px}.volume strong{font-size:42px;letter-spacing:-2px}.volume span{font-size:10px}.controls{grid-template-columns:.95fr 1.05fr}.motor-card{padding:4px}.motor-buttons{flex-direction:row;justify-content:space-evenly;gap:5px}.motor-btn{width:min(22vh,112px);height:min(22vh,112px)}.motor-caption{font-size:10px}.side-card{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px;padding:5px}.control-tile{padding:4px;gap:4px}.control-name{font-size:11px}.toggle{width:50px;height:29px}.toggle:after{width:21px;height:21px}.toggle.on:after{transform:translateX(19px)}.buzzer-btn{font-size:8px;padding:7px 3px}footer{margin-bottom:5px}}
@media(max-width:430px){.app{padding-left:8px;padding-right:8px;gap:7px}.brand{gap:8px}.brand-title{font-size:clamp(18px,5.4vw,23px)}.connection{padding:7px 9px;font-size:10px}.tank{padding:9px 11px}.percent{font-size:clamp(50px,12vw,72px)}.volume strong{font-size:clamp(34px,8vw,48px);letter-spacing:-2px}.volume span{font-size:clamp(10px,2.8vw,12px)}.controls{grid-template-columns:minmax(0,1.4fr) minmax(0,1fr);gap:7px}.motor-card{padding:7px 5px}.side-card{padding:6px;gap:6px}.control-tile{padding:6px 3px;gap:5px}.control-name{font-size:clamp(11px,3.1vw,14px)}.control-state{font-size:9px}.timer{font-size:8px}.motor-btn{width:clamp(80px,21vw,118px);height:clamp(80px,21vw,118px)}.motor-caption{font-size:clamp(11px,3vw,14px)}.toggle{width:clamp(54px,15vw,70px);height:35px}.toggle:after{width:27px;height:27px}.toggle.on:after{transform:translateX(clamp(18px,6vw,30px))}.buzzer-btn{font-size:clamp(9px,2.6vw,11px);padding:10px 3px}.capsule{padding:6px 9px;font-size:9px}.distance{font-size:8px}footer{font-size:clamp(7px,2vw,9px);letter-spacing:.2px;margin-bottom:8px}}
</style>
</head>
<body>
<main class="app">
<header>
 <div class="brand">
  <div class="logo" aria-hidden="true">
   <img src="data:image/png;base64,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" width="200" height="200" alt="">
  </div>
  <div><div class="brand-title">Smart Water Tank</div><div class="credit">Designed by Aman Sharma</div></div>
 </div>
 <div class="head-actions">
  <button class="admin-open" id="adminOpen" type="button">ADMIN</button>
  <div class="connection offline" id="connection"><i class="dot"></i><span id="connectionText">OFFLINE</span></div>
 </div>
</header>
<section class="card tank" id="tankCard">
 <div class="card-head"><span class="eyebrow">Tank overview</span><span class="live">● LIVE DATA</span></div>
 <div class="water-row"><div class="percent" id="percent">--<small>%</small></div><div class="volume"><strong id="litres">-- L</strong><span id="capacityLabel">OF 750 L</span></div></div>
 <div class="bar-wrap"><div class="bar" id="bar"></div></div>
 <div class="scale"><span>0 L · EMPTY</span><span id="scaleFull">750 L · FULL</span></div>
 <div class="tank-bottom">
  <div class="touch-capsules" id="touchCapsules">
   <span class="capsule filling hidden" id="fillCapsule">FILLING</span>
   <span class="capsule overflowing hidden" id="overflowCapsule">OVERFLOWING</span>
   <span class="capsule stable" id="stableCapsule">STABLE</span>
  </div>
  <span class="distance" id="distance">Sensor distance: —</span>
 </div>
</section>
<section class="controls">
 <section class="card motor-card">
  <div class="motor-title"><span class="eyebrow">Motor control · RF remote</span><div class="control-state" id="motorStatus">REMOTE IDLE</div></div>
  <div class="motor-buttons">
   <div class="motor-unit">
    <button class="motor-btn start" id="motorStart" aria-label="Start motor — press and hold" aria-pressed="false"><svg viewBox="0 0 48 48" fill="none" stroke="currentColor" stroke-width="4.5" stroke-linecap="round"><path d="M24 5v20"/><path d="M12.5 12.5a18 18 0 1 0 23 0"/></svg></button>
    <div class="motor-caption">START</div>
   </div>
   <div class="motor-unit">
    <button class="motor-btn stop" id="motorStop" aria-label="Stop motor — press and hold" aria-pressed="false"><svg viewBox="0 0 48 48" fill="none" stroke="currentColor" stroke-width="4.5" stroke-linecap="round"><path d="M24 5v20"/><path d="M12.5 12.5a18 18 0 1 0 23 0"/></svg></button>
    <div class="motor-caption">STOP</div>
   </div>
  </div>
 </section>
 <section class="card side-card">
  <div class="control-tile" id="mirrorTile">
   <div class="control-name">Mirror Light</div>
   <button class="toggle" id="relay1" aria-label="Toggle Mirror Light" aria-pressed="false"></button>
   <div class="control-state" id="r1state">OFF</div>
   <div class="timer" id="mirrorInfo">—</div>
  </div>
  <div class="control-tile" id="bulbTile">
   <div class="control-name">Bulb</div>
   <button class="toggle" id="relay2" aria-label="Toggle bulb" aria-pressed="false"></button>
   <div class="control-state" id="r2state">OFF</div>
  </div>
  <div class="control-tile">
   <div class="control-name">Buzzer</div>
   <div class="buzzer-actions">
    <button class="buzzer-btn test" id="buzzerTest">TEST BUZZER (5s)</button>
    <button class="buzzer-btn" id="buzzerStop">STOP BUZZER</button>
   </div>
   <div class="control-state" id="buzzerState">READY</div>
  </div>
 </section>
</section>
<footer>SMART WATER TANK · LOCAL ESP32 DASHBOARD · DESIGNED BY AMAN SHARMA</footer>

<div class="admin-overlay" id="adminOverlay" hidden>
 <div class="admin-shell">
  <header class="admin-head">
   <div><div class="admin-eyebrow">SMART WATER TANK · CONTROL CENTRE</div><h1>Technical Admin</h1><div class="admin-sub" id="adminSub">Settings are saved to ESP32 NVS</div></div>
   <div class="admin-head-actions">
    <button type="button" id="adminRefresh">Refresh</button>
    <button type="button" class="btn primary" id="adminClose">Back to dashboard</button>
   </div>
  </header>
  <div class="admin-notice" id="adminNotice" hidden></div>
  <div class="admin-live">
   <div class="admin-stat"><span>ESP32_A water</span><strong id="adAState">—</strong></div>
   <div class="admin-stat"><span>ESP-NOW link</span><strong id="adLink">—</strong></div>
   <div class="admin-stat"><span>A uptime</span><strong id="adAUptime">—</strong></div>
   <div class="admin-stat"><span>A reset reason</span><strong id="adAReset">—</strong></div>
   <div class="admin-stat"><span>Echoes OK / timeout</span><strong id="adEchoes">—</strong></div>
   <div class="admin-stat"><span>A flags</span><strong id="adFlags">—</strong></div>
   <div class="admin-stat"><span>A settings sync</span><strong id="adSync">—</strong></div>
   <div class="admin-stat"><span>Live level</span><strong id="adLevel">—</strong></div>
  </div>

  <section class="admin-card">
   <div class="admin-card-head"><div><h2>Water history</h2><p class="note" id="historyMeta">Loading…</p></div><div class="seg" id="rangeSeg"><button type="button" data-range="0" class="on">ALL</button><button type="button" data-range="86400">24H</button><button type="button" data-range="43200">12H</button><button type="button" data-range="21600">6H</button></div></div>
   <div class="graph-wrap"><canvas id="histChart"></canvas><div class="chart-empty" id="chartEmpty">No history samples yet</div></div>
   <div class="legend"><span><i style="background:#128fd1"></i>Filling / rising</span><span><i style="background:#df3c4f"></i>Draining / falling</span><span><i style="background:#9aabba"></i>Invalid sample</span></div>
   <p class="note">The ESP32 has no real-time clock, so the graph is an approximate timeline based on the saved sampling interval. Old samples are trimmed automatically after the retention period.</p>
  </section>

  <section class="admin-card">
   <div class="admin-card-head"><h2>Noise &amp; event log</h2><span class="admin-sub" id="eventCount">—</span></div>
   <div class="event-list" id="eventList"><div class="admin-sub">Loading…</div></div>
  </section>

  <form id="adminForm" autocomplete="off">
   <section class="admin-card">
    <div class="settings-section">
     <h3>Water sensor, calibration &amp; filtering → ESP32_A</h3>
     <div class="admin-grid">
      <div class="field"><label for="fullDistanceCm">Full distance (cm)</label><input id="fullDistanceCm" name="fullDistanceCm" type="number" step="0.1" min="1" max="120"></div>
      <div class="field"><label for="emptyDistanceCm">Empty distance (cm)</label><input id="emptyDistanceCm" name="emptyDistanceCm" type="number" step="0.1" min="1" max="300"></div>
      <div class="field"><label for="emptyDeadbandCm">Empty deadband (cm)</label><input id="emptyDeadbandCm" name="emptyDeadbandCm" type="number" step="0.1" min="0" max="20"></div>
      <div class="field"><label for="tankCapacityL">Tank capacity (L)</label><input id="tankCapacityL" name="tankCapacityL" type="number" step="1" min="1" max="65000"></div>
      <div class="field"><label for="sensorMinCm">Sensor min (cm)</label><input id="sensorMinCm" name="sensorMinCm" type="number" step="0.1" min="1" max="100"></div>
      <div class="field"><label for="sensorMaxCm">Sensor max (cm)</label><input id="sensorMaxCm" name="sensorMaxCm" type="number" step="0.1" min="2" max="400"></div>
      <div class="field"><label for="smoothAlpha">Smoothing alpha</label><input id="smoothAlpha" name="smoothAlpha" type="number" step="0.01" min="0.05" max="0.95"></div>
      <div class="field"><label for="jumpLimitCm">Jump limit (cm)</label><input id="jumpLimitCm" name="jumpLimitCm" type="number" step="0.1" min="5" max="100"></div>
      <div class="field"><label for="jumpConfirm">Jump confirmations</label><input id="jumpConfirm" name="jumpConfirm" type="number" step="1" min="1" max="10"></div>
      <div class="field"><label for="stuckTimeoutMs">Stuck timeout (ms)</label><input id="stuckTimeoutMs" name="stuckTimeoutMs" type="number" step="1000" min="10000" max="86400000"></div>
      <div class="field"><label for="measurePeriodMs">Measure period (ms)</label><input id="measurePeriodMs" name="measurePeriodMs" type="number" step="50" min="200" max="10000"></div>
      <div class="field"><label for="sampleSpacingMs">Sample spacing (ms)</label><input id="sampleSpacingMs" name="sampleSpacingMs" type="number" step="10" min="20" max="500"></div>
      <div class="field"><label for="samplesPerCycle">Samples / cycle</label><input id="samplesPerCycle" name="samplesPerCycle" type="number" step="1" min="3" max="9"></div>
     </div>
     <p class="note">Changes are sent over ESP-NOW and saved by ESP32_A. A reboot of ESP32_A is not normally required.</p>
    </div>

    <div class="settings-section">
     <h3>Mirror person detection</h3>
     <div class="admin-grid">
      <label class="checkline"><input id="mirrorEnabled" name="mirrorEnabled" type="checkbox"> Auto mirror enabled</label>
      <div class="field"><label for="mirrorOnCm">Detect ON below (cm)</label><input id="mirrorOnCm" name="mirrorOnCm" type="number" step="0.1" min="5" max="300"></div>
      <div class="field"><label for="mirrorOffCm">Detect OFF above (cm)</label><input id="mirrorOffCm" name="mirrorOffCm" type="number" step="0.1" min="6" max="400"></div>
      <div class="field"><label for="mirrorOnConfirmMs">ON confirm (ms)</label><input id="mirrorOnConfirmMs" name="mirrorOnConfirmMs" type="number" step="50" min="50" max="10000"></div>
      <div class="field"><label for="mirrorMaxOnMinutes">Max auto-on (min, 0=off)</label><input id="mirrorMaxOnMinutes" name="mirrorMaxOnMinutes" type="number" step="1" min="0" max="1440"></div>
     </div>
     <p class="note">The mirror turns off immediately when the person moves beyond the OFF distance or the sensor loses them. Manual Mirror Light controls stay available on the dashboard.</p>
    </div>

    <div class="settings-section">
     <h3>Buzzer</h3>
     <div class="admin-grid">
      <label class="checkline"><input id="buzzerEnabled" name="buzzerEnabled" type="checkbox"> Full-tank buzzer enabled</label>
      <div class="field"><label for="buzzerFullPercent">Start threshold (%)</label><input id="buzzerFullPercent" name="buzzerFullPercent" type="number" step="0.5" min="50" max="100"></div>
      <div class="field"><label for="buzzerRearmPercent">Re-arm below (%)</label><input id="buzzerRearmPercent" name="buzzerRearmPercent" type="number" step="0.5" min="0" max="99"></div>
      <div class="field"><label for="buzzerConfirmReadings">Confirm readings</label><input id="buzzerConfirmReadings" name="buzzerConfirmReadings" type="number" step="1" min="1" max="20"></div>
      <div class="field"><label for="buzzerDurationMs">Max duration (ms)</label><input id="buzzerDurationMs" name="buzzerDurationMs" type="number" step="1000" min="1000" max="300000"></div>
      <div class="field"><label for="buzzerBeepsPerSec">Beeps per second</label><input id="buzzerBeepsPerSec" name="buzzerBeepsPerSec" type="number" step="0.1" min="0.2" max="10"></div>
      <div class="field"><label for="buzzerBeepOnMs">Beep ON (ms)</label><input id="buzzerBeepOnMs" name="buzzerBeepOnMs" type="number" step="10" min="10" max="5000"></div>
      <div class="field"><label for="buzzerIntensity">Intensity 0–100%</label><input id="buzzerIntensity" name="buzzerIntensity" type="number" step="1" min="0" max="100"></div>
     </div>
     <p class="note">Beeps per second ≈ 1000 ÷ beep period. Intensity uses PWM and is approximate on active buzzers; some modules only click instead of changing volume.</p>
    </div>

    <div class="settings-section">
     <h3>OLED display</h3>
     <div class="oled-layout">
      <div class="admin-grid">
       <label class="checkline"><input type="checkbox" data-oled-bit="1"> Show percent</label>
       <label class="checkline"><input type="checkbox" data-oled-bit="4"> Show link state</label>
       <label class="checkline"><input type="checkbox" data-oled-bit="8"> Show bottom litres</label>
       <label class="checkline"><input type="checkbox" data-oled-bit="16"> Show live/stale status</label>
       <label class="checkline"><input type="checkbox" data-oled-bit="32"> Show distance</label>
       <div class="field"><label for="oledPercentSize">Percent scale</label><select id="oledPercentSize" name="oledPercentSize"><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option><option selected>6</option></select></div>
       <div class="field"><label for="oledBottomSize">Bottom font size</label><select id="oledBottomSize" name="oledBottomSize"><option>1</option><option>2</option><option>3</option><option>4</option></select></div>
       <div class="field"><label for="oledContrast">Contrast 1–255</label><input id="oledContrast" name="oledContrast" type="number" step="1" min="1" max="255"></div>
       <label class="checkline"><input id="oledInvert" name="oledInvert" type="checkbox"> Invert display</label>
      </div>
      <div>
       <div class="oled-frame"><canvas id="oledPreview" width="256" height="128"></canvas></div>
       <p class="note">Approximate 128×64 preview. Large digits use 1 px smooth curves, not scaled block pixels.</p>
      </div>
     </div>
    </div>

    <div class="settings-section">
     <h3>History storage</h3>
     <div class="admin-grid">
      <label class="checkline"><input id="historyEnabled" name="historyEnabled" type="checkbox"> Save history</label>
      <div class="field"><label for="historyIntervalSec">Sample every (seconds)</label><input id="historyIntervalSec" name="historyIntervalSec" type="number" step="10" min="30" max="3600"></div>
      <div class="field"><label for="historyRetentionDays">Keep for (days)</label><input id="historyRetentionDays" name="historyRetentionDays" type="number" step="1" min="1" max="30"></div>
     </div>
     <p class="note">Retention is automatic: the oldest CSV lines are removed when the configured window is exceeded. Storage is LittleFS on ESP32_B.</p>
    </div>

    <div class="savebar"><button type="submit" class="btn primary" id="saveSettings">Save all settings</button><span id="saveHint">No unsaved changes</span></div>
   </section>
  </form>

  <section class="admin-card">
   <div class="admin-card-head"><h2>Maintenance actions</h2></div>
   <div class="seg">
    <button type="button" class="btn" id="rebootA">Reboot ESP32_A</button>
    <button type="button" class="btn danger" id="rebootB">Reboot ESP32_B</button>
    <button type="button" class="btn danger" id="clearHistory">Clear history</button>
    <button type="button" class="btn danger" id="clearEvents">Clear events</button>
   </div>
   <p class="note">Rebooting ESP32_A temporarily drops water data. Rebooting ESP32_B also drops this dashboard for a few seconds.</p>
  </section>

  <section class="admin-card">
   <div class="admin-card-head"><h2>ESP32_A &amp; ESP32_B technical information</h2><span class="admin-sub" id="infoStamp">—</span></div>
   <div class="info-grid" id="infoGrid"></div>
  </section>
 </div>
</div>

<script>
(function(){
'use strict';
const $=id=>document.getElementById(id);
const tankCard=$('tankCard'), connection=$('connection'), connectionText=$('connectionText');
const percentEl=$('percent'), litresEl=$('litres'), barEl=$('bar'), distanceEl=$('distance');
const fillCapsule=$('fillCapsule'), overflowCapsule=$('overflowCapsule'), stableCapsule=$('stableCapsule');
const motorStatusEl=$('motorStatus');
const relay1Btn=$('relay1'), relay2Btn=$('relay2'), mirrorTile=$('mirrorTile'), bulbTile=$('bulbTile');
const r1state=$('r1state'), r2state=$('r2state'), mirrorInfo=$('mirrorInfo'), buzzerStateEl=$('buzzerState');
const startBtn=$('motorStart'), stopBtn=$('motorStop');

const POLL_MS=500, POLL_TIMEOUT_MS=2500;
let current={online:false,linkOnline:false,sensorValid:false,percent:null,litres:null,distance:null,filling:false,draining:false,hcsr04Valid:false,hcsr04Distance:null,personDetected:false,relay1:false,mirrorOverride:false,autoBlocked:false,relay2:false,buzzerActive:false,buzzerTest:false,buzzerTestRemainingMs:0,buzzerArmed:true,buzzerState:"READY",motorStart:false,motorStop:false,motorStartRemainingMs:0,motorStopRemainingMs:0,capacityL:750}, reachable=false, failures=0, inFlight=false, pollTimer=null;
let mirrorBusy=false, bulbBusy=false, buzzerBusy=false;

const motor={
 start:{btn:startBtn, held:false, pointerId:null},
 stop:{btn:stopBtn, held:false, pointerId:null}
};

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function num(v){return (typeof v==='number'&&isFinite(v))?v:null;}
function setOnline(online,linkOk){
 connection.classList.toggle('offline',!online);
 if(!online){
  connectionText.textContent='OFFLINE';
 }else{
  connectionText.textContent=(linkOk===false)?'B ONLINE · A LINK DOWN':'ONLINE';
 }
}
function renderCapsules(filling,overflowing,linkOk){
 filling=linkOk&&filling===true;
 overflowing=linkOk&&overflowing===true;
 fillCapsule.classList.toggle('hidden',!filling);
 fillCapsule.classList.toggle('active',filling);
 overflowCapsule.classList.toggle('hidden',!overflowing);
 overflowCapsule.classList.toggle('active',overflowing);
 stableCapsule.classList.toggle('hidden',filling||overflowing);
 stableCapsule.classList.toggle('offline',!linkOk);
 stableCapsule.textContent=linkOk?'STABLE':'NO SIGNAL';
}
function flashError(el,msg){
 if(!el) return;
 el.textContent=msg||'COMMAND FAILED';
 clearTimeout(el._errTimer);
 el._errTimer=setTimeout(()=>{ if(current) render(current, reachable); },1400);
}
function renderUnreachable(){
 reachable=false;
 if(current && typeof current==='object') render(current,false);
 else {
  tankCard.classList.remove('stale');
  percentEl.innerHTML='--<small>%</small>';
  litresEl.textContent='-- L';
  barEl.style.width='0%';
  distanceEl.textContent='ESP32_B unreachable · reconnecting…';
  renderCapsules(false,false,false);
  r1state.textContent='OFF'; mirrorInfo.textContent='ESP32_B unreachable';
  r2state.textContent='OFF';
  buzzerStateEl.textContent='READY';
  motorStatusEl.textContent='ESP32_B unreachable';
 }
 setOnline(false,false);
 connectionText.textContent='ESP32_B unreachable · reconnecting…';
}
function render(s,bReachable){
 if(!s||typeof s!=='object') return;
 current=s;
 const bOk = (bReachable===undefined)?reachable:(bReachable===true);
 const capacity=num(s.capacityL);
 if(capacity!==null&&capacity>0){
  $('capacityLabel').textContent='OF '+Math.round(capacity)+' L';
  $('scaleFull').textContent=Math.round(capacity)+' L · FULL';
 }
 const linkOk=(bOk && s.linkOnline===true);
 const waterOnline=(bOk && s.online===true);
 setOnline(bOk, linkOk);
 const pct=num(s.percent);
 const hasPct=(pct!==null && linkOk);
 const shownPct=hasPct?clamp(pct,0,100):null;
 const stale=!waterOnline;

 tankCard.classList.toggle('stale',stale && linkOk);
 if(hasPct){
  percentEl.innerHTML=Math.round(shownPct)+'<small>%</small>';
  const litres=num(s.litres);
  litresEl.textContent=Math.round(litres!==null?litres:(capacity&&capacity>0?shownPct*capacity/100:shownPct*7.5))+' L';
  barEl.style.width=shownPct.toFixed(1)+'%';
 }else{
  percentEl.innerHTML='--<small>%</small>';
  litresEl.textContent='-- L';
  barEl.style.width='0%';
 }
 const dist=num(s.distance);
 if(!bOk){
  distanceEl.textContent='ESP32_B unreachable · reconnecting…';
 }else if(linkOk && dist!==null){
  distanceEl.textContent=(stale?'Last distance: ':'Sensor distance: ')+dist.toFixed(1)+' cm'+(stale?' · STALE':'');
 }else if(!linkOk){
  distanceEl.textContent='ESP-NOW link to ESP32_A unavailable · water data paused';
 }else{
  distanceEl.textContent='Sensor distance: —';
 }
 renderCapsules(s.filling,s.draining,linkOk);

 // Local controls (relay1/2, buzzer, RF outputs) are on ESP32_B: always show live B state when B reachable.
 const mirrorOn=(s.relay1===true);
 relay1Btn.classList.toggle('on',mirrorOn);
 relay1Btn.setAttribute('aria-pressed',mirrorOn?'true':'false');
 mirrorTile.classList.toggle('light-on',mirrorOn);
 {
  const person=(s.personDetected===true);
  const override=(s.mirrorOverride===true);
  const blocked=(s.autoBlocked===true);
  const hc=num(s.hcsr04Distance);
  r1state.textContent=mirrorOn?'ON':'OFF';
  let info=override?'MANUAL OVERRIDE · ':blocked?'AUTO BLOCKED · ':(person?'AUTO · PERSON · ':'AUTO · CLEAR · ');
  info+=hc!==null?(hc/100).toFixed(1)+' m':'—';
  if(!bOk) info='ESP32_B unreachable';
  else if(!linkOk) info+=' · ESP-NOW A offline';
  mirrorInfo.textContent=info;
 }

 const bulbOn=(s.relay2===true);
 relay2Btn.classList.toggle('on',bulbOn);
 relay2Btn.setAttribute('aria-pressed',bulbOn?'true':'false');
 bulbTile.classList.toggle('bulb-on',bulbOn);
 r2state.textContent=bulbOn?'ON':'OFF';

 {
  const b=String(s.buzzerState||'READY').toUpperCase();
  const testing=(s.buzzerTest===true)||b==='TESTING';
  if(testing){
   const rem=Number(s.buzzerTestRemainingMs);
   buzzerStateEl.textContent='TESTING'+(rem>0?' · '+Math.ceil(rem/1000)+'s':' · AUTO STOP');
  }else{
   buzzerStateEl.textContent=(s.buzzerActive===true)?'ALERTING':b;
  }
 }

 const startPressed=(s.motorStart===true), stopPressed=(s.motorStop===true);
 startBtn.setAttribute('aria-pressed',startPressed?'true':'false');
 stopBtn.setAttribute('aria-pressed',stopPressed?'true':'false');
 if(!bOk){
  motorStatusEl.textContent='ESP32_B unreachable';
 }else if(startPressed){
  const rs=Math.ceil((num(s.motorStartRemainingMs)||0)/1000);
  motorStatusEl.textContent='START PRESSED'+(rs>0?' · '+rs+'s':'')+(linkOk?'':' · ESP-NOW A offline');
 }else if(stopPressed){
  const rs=Math.ceil((num(s.motorStopRemainingMs)||0)/1000);
  motorStatusEl.textContent='STOP PRESSED'+(rs>0?' · '+rs+'s':'')+(linkOk?'':' · ESP-NOW A offline');
 }else{
  motorStatusEl.textContent=linkOk?'REMOTE IDLE':'ESP-NOW A offline · remote motor unavailable';
 }
}

async function sendCommand(path,body,errorEl){
 const ctrl=new AbortController();
 const to=setTimeout(()=>ctrl.abort(),3500);
 try{
  const r=await fetch(path,{
   method:'POST',cache:'no-store',
   headers:{'Content-Type':'application/json'},
   body:JSON.stringify(body||{}),
   signal:ctrl.signal
  });
  if(!r.ok) throw new Error('HTTP '+r.status);
  const s=await r.json();
  if(s&&s.error) throw new Error(s.error);
  reachable=true; failures=0;
  render(s,true);
  return true;
 }catch(e){
  if(errorEl) flashError(errorEl,'FAILED · RETRY');
  return false;
 }finally{
  clearTimeout(to);
 }
}

function schedulePoll(){
 clearTimeout(pollTimer);
 if(document.hidden) return;
 const delay=reachable?POLL_MS:Math.min(800+failures*400,4000);
 pollTimer=setTimeout(pollState,delay);
}
async function pollState(){
 if(inFlight) return;
 inFlight=true;
 const ctrl=new AbortController();
 const to=setTimeout(()=>ctrl.abort(),POLL_TIMEOUT_MS);
 try{
  const r=await fetch('/api/state',{cache:'no-store',signal:ctrl.signal});
  if(!r.ok) throw new Error('HTTP '+r.status);
  const s=await r.json();
  reachable=true; failures=0;
  render(s,true);
 }catch(e){
  failures++;
  renderUnreachable();
 }finally{
  clearTimeout(to);
  inFlight=false;
  schedulePoll();
 }
}

function pressMotor(name){
 const m=motor[name];
 if(!m||m.held) return;
 m.held=true;
 m.btn.classList.add('held');
 sendCommand('/api/motor',{button:name,pressed:true},motorStatusEl);
}
function releaseMotor(name){
 const m=motor[name];
 if(!m||!m.held) return;
 m.held=false;
 m.pointerId=null;
 m.btn.classList.remove('held');
 sendCommand('/api/motor',{button:name,pressed:false},motorStatusEl);
}
function beaconRelease(name){
 const m=motor[name];
 if(!m||!m.held) return;
 m.held=false;
 m.pointerId=null;
 m.btn.classList.remove('held');
 const body=JSON.stringify({button:name,pressed:false});
 try{
  if(navigator.sendBeacon&&typeof Blob!=='undefined'){
   navigator.sendBeacon('/api/motor',new Blob([body],{type:'application/json'}));
  }else{
   fetch('/api/motor',{method:'POST',headers:{'Content-Type':'application/json'},body:body,keepalive:true});
  }
 }catch(e){}
}
function releaseAllMotors(beacon){
 if(beacon){beaconRelease('start');beaconRelease('stop');}
 else{releaseMotor('start');releaseMotor('stop');}
}
function bindMomentary(name){
 const m=motor[name];
 m.btn.addEventListener('pointerdown',e=>{
  if(e.pointerType==='mouse'&&e.button!==0) return;
  e.preventDefault();
  if(m.held) return;
  m.pointerId=e.pointerId;
  try{ if(m.btn.setPointerCapture) m.btn.setPointerCapture(e.pointerId); }catch(err){}
  pressMotor(name);
 });
 const end=e=>{
  if(e&&m.pointerId!==null&&e.pointerId!==m.pointerId) return;
  try{
   if(e&&m.btn.hasPointerCapture&&m.btn.hasPointerCapture(e.pointerId)){
    m.btn.releasePointerCapture(e.pointerId);
   }
  }catch(err){}
  releaseMotor(name);
 };
 ['pointerup','pointercancel','lostpointercapture','pointerleave'].forEach(ev=>m.btn.addEventListener(ev,end));
 m.btn.addEventListener('keydown',e=>{
  if((e.key===' '||e.key==='Enter')&&!e.repeat){e.preventDefault();pressMotor(name);}
 });
 m.btn.addEventListener('keyup',e=>{
  if(e.key===' '||e.key==='Enter'){e.preventDefault();releaseMotor(name);}
 });
 m.btn.addEventListener('blur',()=>releaseMotor(name));
 m.btn.addEventListener('contextmenu',e=>e.preventDefault());
}
bindMomentary('start');
bindMomentary('stop');

window.addEventListener('blur',()=>releaseAllMotors(true));
document.addEventListener('visibilitychange',()=>{
 if(document.hidden){
  releaseAllMotors(true);
  clearTimeout(pollTimer);
 }else{
  clearTimeout(pollTimer);
  if(!inFlight) pollState(); else schedulePoll();
 }
});
window.addEventListener('pagehide',()=>releaseAllMotors(true));

relay1Btn.addEventListener('click',async()=>{
 if(mirrorBusy) return;
 mirrorBusy=true;
 try{
  const on=!(current&&current.relay1===true);
  await sendCommand('/api/mirror-light',{on:on},mirrorInfo);
 }finally{mirrorBusy=false;}
});
relay2Btn.addEventListener('click',async()=>{
 if(bulbBusy) return;
 bulbBusy=true;
 try{
  const on=!(current&&current.relay2===true);
  await sendCommand('/api/bulb',{on:on},r2state);
 }finally{bulbBusy=false;}
});
$('buzzerTest').addEventListener('click',async()=>{
 if(buzzerBusy) return;
 buzzerBusy=true;
 try{ await sendCommand('/api/buzzer/test',{},buzzerStateEl); }
 finally{buzzerBusy=false;}
});
$('buzzerStop').addEventListener('click',async()=>{
 if(buzzerBusy) return;
 buzzerBusy=true;
 try{ await sendCommand('/api/buzzer/stop',{},buzzerStateEl); }
 finally{buzzerBusy=false;}
});

render(current,false);
pollState();
})();
</script>
<script>
(function(){
'use strict';
const $=id=>document.getElementById(id);
const overlay=$('adminOverlay');
const adminForm=$('adminForm'), saveHint=$('saveHint');
let adminData=null, liveState=null, history=null, events=[];
let liveTimer=null, rangeSec=0, formDirty=false, saving=false;

function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function fmtDur(ms){
 if(!isFinite(ms)||ms<0) return '—';
 const s=Math.floor(ms/1000), d=Math.floor(s/86400), h=Math.floor((s%86400)/3600), m=Math.floor((s%3600)/60), r=s%60;
 if(d) return d+'d '+h+'h';
 if(h) return h+'h '+m+'m';
 if(m) return m+'m '+r+'s';
 return r+'s';
}
function fmtAgo(sec){
 if(!isFinite(sec)||sec<0) return 'earlier boot';
 if(sec<60) return sec+'s ago';
 if(sec<3600) return Math.floor(sec/60)+'m ago';
 if(sec<86400) return Math.floor(sec/3600)+'h '+Math.floor((sec%3600)/60)+'m ago';
 return Math.floor(sec/86400)+'d '+Math.floor((sec%86400)/3600)+'h ago';
}
function resetName(n){
 const names=['UNKNOWN','POWER-ON','EXTERNAL','SOFTWARE','PANIC','INT WDT','TASK WDT','WDT','DEEP SLEEP','BROWNOUT','SDIO','USB','JTAG','EFUSE','PWR GLITCH','CPU LOCKUP','VBAT'];
 return names[n]||('CODE '+n);
}
function flagsText(f){
 f=Number(f)||0; const a=[];
 if(f&1)a.push('NO ECHO'); if(f&2)a.push('RANGE'); if(f&4)a.push('JUMP'); if(f&8)a.push('STUCK'); if(f&16)a.push('TIMEOUT');
 return a.length?a.join(' · '):'OK';
}
function notice(msg,error){
 const el=$('adminNotice'); el.textContent=msg; el.classList.toggle('error',!!error); el.hidden=false;
 clearTimeout(notice._t); notice._t=setTimeout(()=>{el.hidden=true;}, error?7000:4200);
}
function openOverlay(){overlay.hidden=false; overlay.scrollTop=0;}
function closeAdmin(){overlay.hidden=true; stopLive();}
function stopLive(){ if(liveTimer){clearInterval(liveTimer);liveTimer=null;} }
function startLive(){
 stopLive();
 pollLive();
 liveTimer=setInterval(pollLive,2500);
}
async function adminApi(path,opts){
 opts=opts||{};
 const headers=Object.assign({},opts.headers||{});
 if(opts.body) headers['Content-Type']='application/json';
 const r=await fetch(path,Object.assign({},opts,{headers:headers,cache:'no-store'}));
 if(!r.ok){
  let msg='Request failed';
  try{const j=await r.json(); if(j&&j.error)msg=j.error;}catch(e){}
  throw new Error(msg);
 }
 return r.json();
}
async function refreshAll(){
 try{
  await Promise.all([loadSettings(),loadHistory(),loadEvents()]);
 }catch(e){ notice(e.message,true); }
}
async function loadSettings(){
 const d=await adminApi('/api/admin/settings');
 adminData=d;
 applySettings(d.settings||{});
 renderInfo(d.info||{},d.state||{});
 if(d.state) updateLive(d.state);
}
function applySettings(s){
 const nums=['fullDistanceCm','emptyDistanceCm','emptyDeadbandCm','tankCapacityL','sensorMinCm','sensorMaxCm','smoothAlpha','jumpLimitCm','jumpConfirm','stuckTimeoutMs','measurePeriodMs','sampleSpacingMs','samplesPerCycle','mirrorOnCm','mirrorOffCm','mirrorOnConfirmMs','mirrorMaxOnMinutes','buzzerFullPercent','buzzerRearmPercent','buzzerConfirmReadings','buzzerDurationMs','buzzerBeepOnMs','buzzerIntensity','oledPercentSize','oledBottomSize','oledContrast','historyIntervalSec','historyRetentionDays'];
 nums.forEach(k=>{const el=$(k); if(el&&s[k]!==undefined)el.value=s[k];});
 const bps=$('buzzerBeepsPerSec'); if(bps&&s.buzzerBeepPeriodMs)bps.value=(1000/Number(s.buzzerBeepPeriodMs)).toFixed(2);
 ['mirrorEnabled','buzzerEnabled','historyEnabled'].forEach(k=>{const el=$(k); if(el)el.checked=s[k]===true;});
 const inv=$('oledInvert'); if(inv)inv.checked=s.oledInvert===true;
 const show=Number(s.oledShow)||0;
 document.querySelectorAll('[data-oled-bit]').forEach(el=>{el.checked=(show&Number(el.dataset.oledBit))!==0;});
 formDirty=false; saveHint.textContent='No unsaved changes';
 renderOled();
}
function numVal(id){ const v=Number($(id).value); if(!isFinite(v)) throw new Error('Invalid value in '+id); return v; }
function collectSettings(){
 const out={};
 ['fullDistanceCm','emptyDistanceCm','emptyDeadbandCm','tankCapacityL','sensorMinCm','sensorMaxCm','smoothAlpha','jumpLimitCm','jumpConfirm','stuckTimeoutMs','measurePeriodMs','sampleSpacingMs','samplesPerCycle','mirrorOnCm','mirrorOffCm','mirrorOnConfirmMs','mirrorMaxOnMinutes','buzzerFullPercent','buzzerRearmPercent','buzzerConfirmReadings','buzzerDurationMs','buzzerBeepOnMs','buzzerIntensity','oledPercentSize','oledBottomSize','oledContrast','historyIntervalSec','historyRetentionDays'].forEach(k=>{out[k]=numVal(k);});
 ['mirrorEnabled','buzzerEnabled','historyEnabled','oledInvert'].forEach(k=>{out[k]=$(k).checked;});
 let show=0; document.querySelectorAll('[data-oled-bit]').forEach(el=>{if(el.checked)show|=Number(el.dataset.oledBit);});
 out.oledShow=show;
 const bps=numVal('buzzerBeepsPerSec');
 if(bps<=0) throw new Error('Beeps per second must be greater than 0');
 out.buzzerBeepPeriodMs=Math.max(100,Math.min(5000,Math.round(1000/bps)));
 return out;
}
async function saveSettings(e){
 e.preventDefault();
 if(saving) return;
 saving=true; const btn=$('saveSettings'); btn.disabled=true; saveHint.textContent='Saving…';
 try{
  const d=await adminApi('/api/admin/settings',{method:'POST',body:JSON.stringify(collectSettings())});
  adminData=d; applySettings(d.settings||{}); renderInfo(d.info||{},d.state||{});
  if(d.state) updateLive(d.state);
  notice('Settings saved. ESP32_A settings are syncing automatically.');
  loadHistory().catch(()=>{});
 }catch(err){ notice(err.message,true); }
 finally{ saving=false; btn.disabled=false; if(!formDirty)saveHint.textContent='Saved'; }
}
async function loadHistory(){
 history=await adminApi('/api/history');
 const pts=(history&&history.points)||[];
 $('historyMeta').textContent=pts.length?(pts.length+' shown · '+history.count+' stored · every '+history.intervalSec+'s · keep '+history.retentionDays+' days'):'No samples stored yet';
 $('chartEmpty').hidden=pts.length>0;
 drawChart();
}
function drawChart(){
 const c=$('histChart'); if(!c||!history) return;
 const wrap=c.parentElement, dpr=Math.min(window.devicePixelRatio||1,2);
 const w=Math.max(10,wrap.clientWidth), h=Math.max(10,wrap.clientHeight);
 if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);}
 const ctx=c.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
 const all=(history.points||[]).filter(p=>isFinite(p.p));
 let pts=all;
 if(rangeSec>0) pts=all.filter(p=>Number(p.ago)<=rangeSec);
 $('chartEmpty').hidden=pts.length>0;
 if(!pts.length) return;
 const padL=31,padR=10,padT=12,padB=22, pw=w-padL-padR, ph=h-padT-padB;
 const maxAgo=Math.max(1,rangeSec||pts[0].ago||1);
 const xFor=p=>padL+(1-Math.min(1,Number(p.ago)||0)/maxAgo)*pw;
 const yFor=p=>padT+(1-Math.max(0,Math.min(100,Number(p.p)))/100)*ph;
 ctx.strokeStyle='#dce6ef'; ctx.lineWidth=1; ctx.font='9px system-ui,sans-serif'; ctx.fillStyle='#8293a6'; ctx.textAlign='right'; ctx.textBaseline='middle';
 [0,25,50,75,100].forEach(v=>{const y=padT+(1-v/100)*ph;ctx.beginPath();ctx.moveTo(padL,y);ctx.lineTo(w-padR,y);ctx.stroke();ctx.fillText(v+'%',padL-5,y);});
 ctx.textAlign='left'; ctx.textBaseline='top';
 const oldAgo=rangeSec||pts[0].ago;
 ctx.fillText(oldAgo>86400?(oldAgo/86400).toFixed(1)+'d ago':oldAgo>3600?(oldAgo/3600).toFixed(1)+'h ago':'oldest',padL,h-16);
 ctx.textAlign='right'; ctx.fillText('now',w-padR,h-16);
 // area under valid data
 ctx.beginPath(); let started=false;
 pts.forEach(p=>{if(Number(p.p)<0)return; const x=xFor(p),y=yFor(p); if(!started){ctx.moveTo(x,padT+ph);ctx.lineTo(x,y);started=true;}else ctx.lineTo(x,y);});
 if(started){ctx.lineTo(xFor(pts[pts.length-1]),padT+ph);ctx.closePath();ctx.fillStyle='rgba(18,143,209,.08)';ctx.fill();}
 // colored segments
 for(let i=1;i<pts.length;i++){
  const a=pts[i-1],b=pts[i];
  if(Number(a.p)<0||Number(b.p)<0) continue;
  const rise=Number(b.p)>=Number(a.p);
  const equal=Math.abs(Number(b.p)-Number(a.p))<0.01;
  ctx.strokeStyle=equal?'#9aabba':(rise?'#128fd1':'#df3c4f');
  ctx.lineWidth=2; ctx.lineJoin='round'; ctx.lineCap='round';
  ctx.beginPath();ctx.moveTo(xFor(a),yFor(a));ctx.lineTo(xFor(b),yFor(b));ctx.stroke();
 }
 if(pts.length===1){ctx.fillStyle='#128fd1';ctx.beginPath();ctx.arc(xFor(pts[0]),yFor(pts[0]),3,0,Math.PI*2);ctx.fill();}
}
function eventInfo(code){
 return {
  1:['No echo','Ultrasonic returned fewer than 3 valid echoes','warn'],
  2:['Echo timeout','A burst timed out waiting for the echo','warn'],
  3:['Jump rejected','A sudden distance jump was rejected as noise','warn'],
  4:['Stuck sensor','Raw distance did not change for the stuck timeout','bad'],
  5:['Out of range','Distance was outside the configured sensor range','bad'],
  6:['ESP-NOW lost','Link from ESP32_A stopped','bad'],
  7:['ESP-NOW restored','Link from ESP32_A is back','ok'],
  8:['Mirror limit','Automatic mirror light hit its maximum on-time','warn'],
  9:['Mirror dropout','Person state cleared after HC-SR04 dropout','warn'],
  10:['Reboot','A reboot was requested from admin','warn'],
  11:['Settings','New ESP32_A settings revision saved','ok'],
  12:['Admin access','Legacy admin access event','ok'],
  13:['Data cleared','History and/or events were cleared','warn']
 }[Number(code)]||['Event '+code,'Technical event','warn'];
}
function renderEvents(){
 const box=$('eventList');
 if(!events.length){box.innerHTML='<div class="admin-sub">No events logged yet.</div>';return;}
 const nowSec=liveState&&isFinite(liveState.uptimeMs)?Math.floor(liveState.uptimeMs/1000):0;
 box.innerHTML=events.map(ev=>{
  const info=eventInfo(ev.code), age=(nowSec&&ev.sec<=nowSec)?fmtAgo(nowSec-ev.sec):'earlier boot';
  const detail=Number(ev.v)?('value '+Number(ev.v).toFixed(1)):'';
  return '<div class="event-item '+(info[2]==='bad'?'bad':info[2]==='warn'?'warn':'')+'"><div><b>'+esc(info[0])+'</b><small>'+esc(info[1])+(detail?' · '+esc(detail):'')+'</small></div><span class="age">'+esc(age)+'</span></div>';
 }).join('');
}
async function loadEvents(){
 const d=await adminApi('/api/events');
 events=(d.events||[]).slice().reverse();
 $('eventCount').textContent=events.length+' recent';
 renderEvents();
}
async function pollLive(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});
  if(!r.ok) throw new Error('state unavailable');
  updateLive(await r.json());
 }catch(e){}
}
function updateLive(s){
 if(!s)return; liveState=s;
 $('adAState').textContent=(s.online===true)?'ONLINE':(s.linkOnline===true?'STALE':'OFFLINE');
 $('adLink').textContent=(s.linkOnline===true)?'ONLINE':'OFFLINE';
 $('adAUptime').textContent=fmtDur(s.aUptimeMs);
 $('adAReset').textContent=resetName(Number(s.aResetReason));
 $('adEchoes').textContent=(s.aValidEchoes!=null?s.aValidEchoes:'—')+' / '+(s.aTimeoutEchoes!=null?s.aTimeoutEchoes:'—');
 $('adFlags').textContent=flagsText(s.aFlags);
 const aProto=Number(s.aProtocol)||0;
 $('adSync').textContent=(aProto===1)?'LEGACY A · v1':((s.settingsSynced===true)?'SYNCED':('PENDING · A has '+ (s.aSettingsRevision!=null?s.aSettingsRevision:'?')));
 const pct=Number(s.percent), litres=Number(s.litres);
 $('adLevel').textContent=(isFinite(pct)?pct.toFixed(0):'—')+'% · '+(isFinite(litres)?litres.toFixed(0):'—')+' L';
 if(events.length)renderEvents();
 renderOled();
}
function renderInfo(info,state){
 if(state) liveState=Object.assign({},liveState||{},state);
 const bUptime=Number(info.bUptimeMs);
 const fsTotal=Number(info.fsTotal), fsUsed=Number(info.fsUsed);
 const pctFs=fsTotal>0?Math.round(fsUsed/fsTotal*100):0;
 const rows=[
  ['ESP32_B AP MAC',info.bMac],['ESP32_B STA MAC',info.bStaMac],['ESP32_A MAC',info.aMac],
  ['Access point',info.ssid+' · '+info.ip],['Channel',info.channel],
  ['B uptime',isFinite(bUptime)?fmtDur(bUptime):'—'],['B reset reason',resetName(Number(info.bResetReason))],
  ['Free heap',isFinite(Number(info.freeHeap))?Math.round(Number(info.freeHeap)/1024)+' KB':'—'],
  ['Flash size',isFinite(Number(info.flashSize))?Math.round(Number(info.flashSize)/1024)+' KB':'—'],
  ['LittleFS used',isFinite(fsUsed)?Math.round(fsUsed/1024)+' / '+Math.round(fsTotal/1024)+' KB ('+pctFs+'%)':'—'],
  ['History lines',info.historyLines],['Events stored',info.eventSeq],
  ['A settings rev',info.aSettingsRevision],['A settings synced',info.settingsSynced?'YES':'PENDING'],
  ['HC-SR04 trigger / echo',info.pins?info.pins.hcTrig+' / '+info.pins.hcEcho:'—'],
  ['Relay 1 / Relay 2',info.pins?info.pins.relay1+' / '+info.pins.relay2:'—'],
  ['Buzzer / button',info.pins?info.pins.buzzer+' / '+info.pins.button:'—'],
  ['OLED SDA / SCL',info.pins?info.pins.oledSda+' / '+info.pins.oledScl:'—']
 ];
 $('infoGrid').innerHTML=rows.map(r=>'<div class="info-row"><span>'+esc(r[0])+'</span><b>'+esc(r[1]==null?'—':r[1])+'</b></div>').join('');
 $('infoStamp').textContent='Updated '+new Date().toLocaleTimeString();
}
function renderOled(){
 const c=$('oledPreview'); if(!c)return;
 const ctx=c.getContext('2d');
 const invert=$('oledInvert').checked;
 ctx.setTransform(1,0,0,1,0,0);
 ctx.fillStyle=invert?'#f2f6f3':'#050907'; ctx.fillRect(0,0,c.width,c.height);
 ctx.setTransform(2,0,0,2,0,0);
 const ink=invert?'#08110d':'#eafff3';
 let bits=0; document.querySelectorAll('[data-oled-bit]').forEach(el=>{if(el.checked)bits|=Number(el.dataset.oledBit);});
 const text=(str,x,y,size,align)=>{ctx.fillStyle=ink;ctx.font='900 '+(size*8)+'px ui-monospace,Menlo,monospace';ctx.textAlign=align||'left';ctx.textBaseline='top';ctx.fillText(String(str),x,y);};
 const pct=liveState&&isFinite(liveState.percent)?Math.round(liveState.percent):null;
 const litres=liveState&&isFinite(liveState.litres)?Math.round(liveState.litres):null;
 const dist=liveState&&isFinite(liveState.distance)?Number(liveState.distance):null;
 const link=!!(liveState&&liveState.linkOnline);
 const pctText=pct==null?'--':String(pct);
 const litreText=litres==null?'--L':litres+'L';
 if(bits&1){
  const scale=Number($('oledPercentSize').value)||6;
  const numberPx=scale<=2?24:(scale<=4?32:42);
  const signPx=scale<=2?18:(scale<=4?22:30);
  const baseline=Math.round((46-numberPx)/2+numberPx);
  const family='Arial,Helvetica,sans-serif';
  ctx.fillStyle=ink; ctx.textAlign='left'; ctx.textBaseline='alphabetic';
  ctx.font='500 '+numberPx+'px '+family;
  const numberWidth=ctx.measureText(pctText).width;
  ctx.font='500 '+signPx+'px '+family;
  const signWidth=ctx.measureText('%').width;
  const x=Math.max(0,(128-numberWidth-signWidth)/2);
  ctx.font='500 '+numberPx+'px '+family;
  ctx.fillText(pctText,x,baseline);
  ctx.font='500 '+signPx+'px '+family;
  ctx.fillText('%',x+numberWidth,baseline);
 }
 if((bits&32)&&dist!=null)text(dist.toFixed(0)+'cm',0,47,1,'left');
 const bottom=Number($('oledBottomSize').value)||1;
 if(bits&4)text(link?'CONNECTED':'NO LINK',0,56,bottom,'left');
 if(bits&8)text(litreText,128,56,bottom,'right');
 if((bits&16)&&!(bits&4))text(link?'LIVE':'STALE',0,56,bottom,'left');
}
function bind(){
 $('adminOpen').addEventListener('click',async()=>{
  openOverlay(); startLive(); await refreshAll();
 });
 $('adminClose').addEventListener('click',()=>{closeAdmin();});
 $('adminRefresh').addEventListener('click',()=>refreshAll());
 adminForm.addEventListener('input',()=>{formDirty=true;saveHint.textContent='Unsaved changes';renderOled();});
 adminForm.addEventListener('change',()=>{formDirty=true;saveHint.textContent='Unsaved changes';renderOled();});
 adminForm.addEventListener('submit',saveSettings);
 document.querySelectorAll('[data-oled-bit]').forEach(el=>el.addEventListener('change',renderOled));
 ['oledPercentSize','oledBottomSize','oledInvert'].forEach(id=>$(id).addEventListener('change',renderOled));
 $('rangeSeg').addEventListener('click',e=>{
  const b=e.target.closest('button[data-range]'); if(!b)return;
  rangeSec=Number(b.dataset.range)||0;
  $('rangeSeg').querySelectorAll('button').forEach(x=>x.classList.toggle('on',x===b));
  drawChart();
 });
 $('rebootA').addEventListener('click',async()=>{
  if(!confirm('Reboot ESP32_A now?'))return;
  try{await adminApi('/api/admin/reboot-a',{method:'POST'});notice('Reboot command sent to ESP32_A.');}catch(e){notice(e.message,true);}
 });
 $('rebootB').addEventListener('click',async()=>{
  if(!confirm('Reboot ESP32_B now? The dashboard will disconnect briefly.'))return;
  try{await adminApi('/api/admin/reboot-b',{method:'POST'});}catch(e){notice(e.message,true);}
 });
 $('clearHistory').addEventListener('click',async()=>{
  if(!confirm('Delete all stored water history?'))return;
  try{await adminApi('/api/admin/clear-data',{method:'POST',body:JSON.stringify({history:true})});notice('History cleared.');await loadHistory();}catch(e){notice(e.message,true);}
 });
 $('clearEvents').addEventListener('click',async()=>{
  if(!confirm('Delete all stored events?'))return;
  try{await adminApi('/api/admin/clear-data',{method:'POST',body:JSON.stringify({events:true})});notice('Events cleared.');await loadEvents();}catch(e){notice(e.message,true);}
 });
 let rt=null;
 window.addEventListener('resize',()=>{clearTimeout(rt);rt=setTimeout(drawChart,150);});
 if(window.ResizeObserver){new ResizeObserver(()=>{if(!overlay.hidden)drawChart();}).observe($('histChart').parentElement);}
}
bind();
})();
</script>
</body>
</html>)WTDASH";

/* ==========================================================================
   GLOBAL STATE
   ========================================================================== */

WebServer server(80);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, PIN_I2C_SCL, PIN_I2C_SDA);
bool oledReady = false;

// --- Persistent configuration / admin ---
Preferences prefsB;
AppSettings settings;
bool     settingsDirty = false;
uint32_t lastSettingsPushMs = 0;
uint32_t aDesiredRevision = 0;
uint32_t aSeenRevision = 0;
bool     haveASettingsRevision = false;
volatile bool rebootARequested = false;

// --- History / event storage ---
uint32_t historySeq = 0;
uint32_t historyLineCount = 0;
uint32_t historyLastSampleMs = 0;
uint32_t eventSeq = 0;
uint32_t lastEventLogMs[16] = {0};

// --- ESP32_A diagnostics ---
uint8_t  lastAValidEchoes = 0;
uint8_t  lastATimeoutEchoes = 0;
uint8_t  lastAResetReason = 0;
uint32_t lastAQueueDrops = 0;

// --- ESP32_A water data ---
bool     haveValidWater = false;
bool     lastPacketValid = false;
float    lastValidPercent = 0.0f;
float    lastValidDistanceCm = 0.0f;
uint16_t lastValidLitres = 0;
bool     fillingActive = false;
bool     drainingActive = false;
uint32_t lastValidRxMs = 0;
uint32_t lastAnyRxMs = 0;
uint32_t lastSeq = 0;
uint32_t lastAUptimeMs = 0;
bool     haveSeq = false;
uint16_t lastFlags = 0;
uint8_t  aProtocolVersion = PROTOCOL_VERSION;

// --- Buzzer ---
bool     buzzerAlerting = false;
bool     buzzerTesting = false;
bool     buzzerArmed = true;
uint32_t buzzerStartMs = 0;
uint32_t buzzerTestStartMs = 0;
uint8_t  fullConfirmCount = 0;

// --- HC-SR04 / Relay 1 ---
volatile uint32_t hcRiseUs = 0;
volatile uint32_t hcFallUs = 0;
volatile bool     hcEchoActive = false;
volatile bool     hcEchoComplete = false;
bool     hcWaiting = false;
uint32_t hcWaitDeadlineUs = 0;
uint32_t hcNextSampleMs = 0;

float    hcRing[HC_FILTER_SAMPLES];
bool     hcRingValid[HC_FILTER_SAMPLES];
uint8_t  hcRingHead = 0;
float    hcDistanceCm = 0.0f;
bool     hcValid = false;
bool     haveHcReading = false;
uint32_t lastValidHcMs = 0;
uint32_t hcNearSinceMs = 0;
uint32_t hcFarSinceMs = 0;
uint32_t autoRelay1OnSinceMs = 0;
bool     autoRelay1On = false;
bool     personDetected = false;
bool     autoBlocked = false;

bool     mirrorOverride = false;
bool     relay1On = false;
bool     relay2On = false;

// --- Two-way ESP-NOW status to ESP32_A ---
uint32_t statusSeq = 0;
uint32_t lastAckSeq = 0;
uint32_t lastStatusTxMs = 0;

// --- Motor RF momentary outputs ---
bool     motorStartPressed = false;
bool     motorStopPressed = false;
uint32_t motorStartMs = 0;
uint32_t motorStopMs = 0;

// --- ESP-NOW receive queue ---
struct RxItem {
  uint8_t data[64];
  uint8_t len;
  uint8_t src[6];
};
QueueHandle_t rxQueue = nullptr;

// --- Physical button ---
int      buttonLastReading = HIGH;
bool     buttonStableState = HIGH;
uint32_t buttonLastChangeMs = 0;
bool     buttonPressActive = false;
bool     buttonLongHandled = false;
uint32_t buttonPressStartMs = 0;

// --- OLED/status ---
uint32_t lastOledMs = 0;
bool     prevOnline = false;
bool     prevLinkOnline = false;
bool     prevBlue = false;
bool     prevRed = false;

/* ==========================================================================
   CRC16-CCITT (identical to ESP32_A)
   ========================================================================== */

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else              crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void sendApiError(int code, const char *message);
bool computeLinkOnline();
bool computeOnline();

/* ==========================================================================
   PERSISTENT SETTINGS
   ========================================================================== */

void defaultSettings(AppSettings &c) {
  memset(&c, 0, sizeof(c));
  c.magic = B_SETTINGS_MAGIC;
  c.version = B_SETTINGS_VERSION;
  c.revision = 1;
  c.aRevision = 1;
  c.fullDistanceCm = 10.0f;
  c.emptyDistanceCm = 76.0f;
  c.emptyDeadbandCm = 2.0f;
  c.tankCapacityL = 750.0f;
  c.sensorMinCm = 2.0f;
  c.sensorMaxCm = 150.0f;
  c.smoothAlpha = 0.35f;
  c.jumpLimitCm = 30.0f;
  c.stuckTimeoutMs = 600000UL;
  c.measurePeriodMs = 1000;
  c.sampleSpacingMs = 60;
  c.jumpConfirm = 3;
  c.samplesPerCycle = 5;

  c.mirrorEnabled = 1;
  c.mirrorOnCm = 100.0f;
  c.mirrorOffCm = 105.0f;
  c.mirrorOnConfirmMs = 400;
  c.mirrorOffConfirmMs = 0;
  c.mirrorInvalidHoldoffMs = 0;
  c.mirrorMaxOnMinutes = 0;

  c.buzzerEnabled = 1;
  c.buzzerIntensity = 100;
  c.buzzerConfirmReadings = FULL_CONFIRM_READINGS;
  c.buzzerFullPercent = FULL_LEVEL_PERCENT;
  c.buzzerRearmPercent = REARM_PERCENT;
  c.buzzerDurationMs = BUZZER_ALERT_MAX_MS;
  c.buzzerBeepPeriodMs = BUZZER_BEEP_PERIOD_MS;
  c.buzzerBeepOnMs = BUZZER_BEEP_ON_MS;

  c.oledShow = OLED_SHOW_PERCENT | OLED_SHOW_LINK | OLED_SHOW_BOTTOM_L;
  c.oledPercentSize = 6;
  c.oledLitresSize = 2;
  c.oledBottomSize = 1;
  c.oledContrast = 255;
  c.oledInvert = 0;

  c.historyIntervalSec = 300;
  c.historyRetentionDays = 4;
  c.historyEnabled = 1;
}

void clampSettings(AppSettings &c) {
  if (!isfinite(c.fullDistanceCm)) c.fullDistanceCm = 10.0f;
  if (!isfinite(c.emptyDistanceCm)) c.emptyDistanceCm = 76.0f;
  if (!isfinite(c.emptyDeadbandCm)) c.emptyDeadbandCm = 2.0f;
  if (!isfinite(c.tankCapacityL)) c.tankCapacityL = 750.0f;
  if (!isfinite(c.sensorMinCm)) c.sensorMinCm = 2.0f;
  if (!isfinite(c.sensorMaxCm)) c.sensorMaxCm = 150.0f;
  if (!isfinite(c.smoothAlpha)) c.smoothAlpha = 0.35f;
  if (!isfinite(c.jumpLimitCm)) c.jumpLimitCm = 30.0f;
  if (!isfinite(c.mirrorOnCm)) c.mirrorOnCm = 100.0f;
  if (!isfinite(c.mirrorOffCm)) c.mirrorOffCm = 105.0f;
  if (!isfinite(c.buzzerFullPercent)) c.buzzerFullPercent = 97.0f;
  if (!isfinite(c.buzzerRearmPercent)) c.buzzerRearmPercent = 93.0f;

  c.fullDistanceCm = constrain(c.fullDistanceCm, 1.0f, 120.0f);
  c.sensorMinCm = constrain(c.sensorMinCm, 1.0f, 100.0f);
  c.sensorMaxCm = constrain(c.sensorMaxCm, c.sensorMinCm + 5.0f, 400.0f);
  c.emptyDistanceCm = constrain(c.emptyDistanceCm, c.fullDistanceCm + 3.0f, 300.0f);
  c.emptyDeadbandCm = constrain(c.emptyDeadbandCm, 0.0f, 20.0f);
  c.tankCapacityL = constrain(c.tankCapacityL, 1.0f, 65000.0f);
  c.smoothAlpha = constrain(c.smoothAlpha, 0.05f, 0.95f);
  c.jumpLimitCm = constrain(c.jumpLimitCm, 5.0f, 100.0f);
  c.jumpConfirm = constrain(c.jumpConfirm, 1, 10);
  c.samplesPerCycle = constrain(c.samplesPerCycle, 3, 9);
  c.measurePeriodMs = constrain(c.measurePeriodMs, 200, 10000);
  c.sampleSpacingMs = constrain(c.sampleSpacingMs, 20, 500);
  c.stuckTimeoutMs = constrain(c.stuckTimeoutMs, 10000UL, 86400000UL);

  c.mirrorEnabled = c.mirrorEnabled ? 1 : 0;
  c.mirrorOnCm = constrain(c.mirrorOnCm, 5.0f, 300.0f);
  c.mirrorOffCm = constrain(c.mirrorOffCm, c.mirrorOnCm + 1.0f, 400.0f);
  c.mirrorOnConfirmMs = constrain(c.mirrorOnConfirmMs, 50, 10000);
  c.mirrorOffConfirmMs = 0;
  c.mirrorInvalidHoldoffMs = 0;
  c.mirrorMaxOnMinutes = constrain(c.mirrorMaxOnMinutes, 0, 1440);

  c.buzzerEnabled = c.buzzerEnabled ? 1 : 0;
  c.buzzerIntensity = constrain(c.buzzerIntensity, 0, 100);
  c.buzzerConfirmReadings = constrain(c.buzzerConfirmReadings, 1, 20);
  c.buzzerFullPercent = constrain(c.buzzerFullPercent, 50.0f, 100.0f);
  c.buzzerRearmPercent = constrain(c.buzzerRearmPercent, 0.0f, c.buzzerFullPercent - 1.0f);
  c.buzzerDurationMs = constrain(c.buzzerDurationMs, 1000UL, 300000UL);
  c.buzzerBeepPeriodMs = constrain(c.buzzerBeepPeriodMs, 100, 5000);
  c.buzzerBeepOnMs = constrain(c.buzzerBeepOnMs, 10, c.buzzerBeepPeriodMs);

  c.oledShow = (c.oledShow & 0x3F) & ~OLED_SHOW_LITRES;
  c.oledPercentSize = constrain(c.oledPercentSize, 1, 6);
  c.oledLitresSize = constrain(c.oledLitresSize, 1, 6);
  c.oledBottomSize = constrain(c.oledBottomSize, 1, 4);
  c.oledContrast = constrain(c.oledContrast, 1, 255);
  c.oledInvert = c.oledInvert ? 1 : 0;

  c.historyIntervalSec = constrain(c.historyIntervalSec, HISTORY_MIN_INTERVAL_SEC, HISTORY_MAX_INTERVAL_SEC);
  c.historyRetentionDays = constrain(c.historyRetentionDays, 1, 30);
  c.historyEnabled = c.historyEnabled ? 1 : 0;

  if (c.aRevision == 0) c.aRevision = 1;
  if (c.revision == 0) c.revision = 1;
}

bool loadSettings() {
  prefsB.begin("water-b", false);
  size_t got = prefsB.getBytesLength("cfg");
  bool valid = false;
  if (got == sizeof(settings)) {
    prefsB.getBytes("cfg", &settings, sizeof(settings));
    valid = settings.magic == B_SETTINGS_MAGIC && settings.version == B_SETTINGS_VERSION;
  }
  if (!valid) {
    defaultSettings(settings);
    prefsB.putBytes("cfg", &settings, sizeof(settings));
  }
  clampSettings(settings);
  aDesiredRevision = settings.aRevision;
  return true;
}

void saveSettings() {
  prefsB.putBytes("cfg", &settings, sizeof(settings));
  settingsDirty = false;
  aDesiredRevision = settings.aRevision;
  DBG("[CFG] saved rev=%lu aRev=%lu\n",
      (unsigned long)settings.revision, (unsigned long)settings.aRevision);
}

bool aFieldsEqual(const AppSettings &a, const AppSettings &b) {
  return a.fullDistanceCm == b.fullDistanceCm &&
         a.emptyDistanceCm == b.emptyDistanceCm &&
         a.emptyDeadbandCm == b.emptyDeadbandCm &&
         a.tankCapacityL == b.tankCapacityL &&
         a.sensorMinCm == b.sensorMinCm &&
         a.sensorMaxCm == b.sensorMaxCm &&
         a.smoothAlpha == b.smoothAlpha &&
         a.jumpLimitCm == b.jumpLimitCm &&
         a.stuckTimeoutMs == b.stuckTimeoutMs &&
         a.measurePeriodMs == b.measurePeriodMs &&
         a.sampleSpacingMs == b.sampleSpacingMs &&
         a.jumpConfirm == b.jumpConfirm &&
         a.samplesPerCycle == b.samplesPerCycle;
}

void commitSettings(const AppSettings &old) {
  bool any = memcmp(&old, &settings, sizeof(settings)) != 0;
  if (!any) return;
  settings.revision = old.revision + 1;
  if (!aFieldsEqual(old, settings)) {
    settings.aRevision = old.aRevision + 1;
    aDesiredRevision = settings.aRevision;
    lastSettingsPushMs = 0;
  }
  saveSettings();
}

SettingsPacket makeSettingsPacket() {
  SettingsPacket p;
  memset(&p, 0, sizeof(p));
  p.magic = PKT_MAGIC;
  p.version = PROTOCOL_VERSION;
  p.type = PKT_SETTINGS;
  p.revision = settings.aRevision;
  p.fullDistanceCm = settings.fullDistanceCm;
  p.emptyDistanceCm = settings.emptyDistanceCm;
  p.emptyDeadbandCm = settings.emptyDeadbandCm;
  p.tankCapacityL = settings.tankCapacityL;
  p.sensorMinCm = settings.sensorMinCm;
  p.sensorMaxCm = settings.sensorMaxCm;
  p.smoothAlpha = settings.smoothAlpha;
  p.jumpLimitCm = settings.jumpLimitCm;
  p.stuckTimeoutMs = settings.stuckTimeoutMs;
  p.measurePeriodMs = settings.measurePeriodMs;
  p.sampleSpacingMs = settings.sampleSpacingMs;
  p.jumpConfirm = settings.jumpConfirm;
  p.samplesPerCycle = settings.samplesPerCycle;
  p.crc = crc16((const uint8_t *)&p, offsetof(SettingsPacket, crc));
  return p;
}

bool pushSettingsToA() {
  SettingsPacket p = makeSettingsPacket();
  esp_err_t err = esp_now_send(ESP32_A_MAC, (const uint8_t *)&p, sizeof(p));
  lastSettingsPushMs = millis();
  if (err != ESP_OK) DBG("[CFG] settings push failed: %d\n", (int)err);
  return err == ESP_OK;
}

void serviceSettingsSync() {
  if (aProtocolVersion < 2) return;
  if (haveASettingsRevision && aSeenRevision == aDesiredRevision) return;
  uint32_t waitMs = computeLinkOnline() ? 2000UL : 8000UL;
  if ((uint32_t)(millis() - lastSettingsPushMs) < waitMs) return;
  pushSettingsToA();
}

void sendCommandToA(uint8_t command, bool pressed) {
  CommandPacket cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.magic = PKT_MAGIC;
  cmd.version = aProtocolVersion;
  cmd.type = PKT_COMMAND;
  cmd.command = command;
  cmd.pressed = pressed ? 1 : 0;
  cmd.seq = ++statusSeq;
  cmd.uptimeMs = millis();
  cmd.crc = crc16((const uint8_t *)&cmd, offsetof(CommandPacket, crc));
  esp_now_send(ESP32_A_MAC, (const uint8_t *)&cmd, sizeof(cmd));
}

/* ==========================================================================
   HISTORY / EVENT LOG (LittleFS)
   ========================================================================== */

bool appendCsvLine(const char *path, const String &line) {
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f) return false;
  size_t written = f.print(line);
  f.print('\n');
  f.close();
  return written > 0;
}

uint32_t countCsvLines(const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  uint32_t n = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length()) n++;
  }
  f.close();
  return n;
}

void compactCsv(const char *path, uint32_t keepLines) {
  File src = LittleFS.open(path, "r");
  if (!src) return;
  uint32_t total = 0;
  while (src.available()) {
    String line = src.readStringUntil('\n');
    line.trim();
    if (line.length()) total++;
  }
  if (total <= keepLines + 20) { src.close(); return; }

  src.seek(0);
  File tmp = LittleFS.open("/tmpcsv.tmp", "w");
  if (!tmp) { src.close(); return; }
  uint32_t skip = total - keepLines;
  uint32_t idx = 0;
  while (src.available()) {
    String line = src.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    if (idx++ < skip) continue;
    tmp.print(line);
    tmp.print('\n');
  }
  tmp.close();
  src.close();
  LittleFS.remove(path);
  LittleFS.rename("/tmpcsv.tmp", path);
  DBG("[HIST] compacted %s to %lu lines\n", path, (unsigned long)keepLines);
}

void loadHistoryMeta() {
  File f = LittleFS.open(HISTORY_FILE, "r");
  historySeq = 0;
  historyLineCount = 0;
  if (f) {
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      historyLineCount++;
      int comma = line.indexOf(',');
      if (comma > 0) historySeq = strtoul(line.substring(0, comma).c_str(), nullptr, 10);
    }
    f.close();
  }
  eventSeq = 0;
  File e = LittleFS.open(EVENTS_FILE, "r");
  if (e) {
    while (e.available()) {
      String line = e.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      eventSeq++;
    }
    e.close();
  }
  DBG("[HIST] %lu samples, seq=%lu, events=%lu\n",
      (unsigned long)historyLineCount, (unsigned long)historySeq,
      (unsigned long)eventSeq);
}

void appendHistorySample(bool valid, float percent, uint16_t litres, float distanceCm, uint16_t flags) {
  if (!settings.historyEnabled) return;
  char buf[96];
  uint32_t sec = millis() / 1000UL;
  float p = valid ? percent : -1.0f;
  float d = valid ? distanceCm : -1.0f;
  snprintf(buf, sizeof(buf), "%lu,%lu,%.2f,%u,%.1f,%u",
           (unsigned long)(++historySeq), (unsigned long)sec,
           p, (unsigned)litres, d, (unsigned)flags);
  if (appendCsvLine(HISTORY_FILE, String(buf))) historyLineCount++;

  uint32_t maxInterval = settings.historyIntervalSec;
  uint32_t maxLines = ((uint32_t)settings.historyRetentionDays * 86400UL) / maxInterval;
  if (maxLines < 100) maxLines = 100;
  if (historyLineCount > maxLines + 20) {
    compactCsv(HISTORY_FILE, maxLines);
    historyLineCount = maxLines;
  }
}

void serviceHistory() {
  if (!settings.historyEnabled) return;
  uint32_t intervalMs = (uint32_t)settings.historyIntervalSec * 1000UL;
  if (historyLastSampleMs != 0 &&
      (uint32_t)(millis() - historyLastSampleMs) < intervalMs) return;
  historyLastSampleMs = millis();

  bool valid = computeOnline() && haveValidWater;
  appendHistorySample(valid,
                      valid ? lastValidPercent : 0.0f,
                      valid ? lastValidLitres : 0,
                      valid ? lastValidDistanceCm : 0.0f,
                      lastFlags);
}

void logEvent(uint8_t code, float value) {
  if (code >= 16) return;
  uint32_t now = millis();
  if (lastEventLogMs[code] != 0 && (uint32_t)(now - lastEventLogMs[code]) < 30000UL) return;
  lastEventLogMs[code] = now;
  char buf[64];
  snprintf(buf, sizeof(buf), "%lu,%lu,%u,%.1f",
           (unsigned long)(++eventSeq), (unsigned long)(now / 1000UL),
           (unsigned)code, value);
  if (appendCsvLine(EVENTS_FILE, String(buf)) && (eventSeq % 100UL) == 0) {
    compactCsv(EVENTS_FILE, EVENTS_MAX_LINES);
  }
}

String buildHistoryJson() {
  String j;
  j.reserve(2400);
  j = "{\"intervalSec\":";
  j += String((unsigned)settings.historyIntervalSec);
  j += ",\"retentionDays\":";
  j += String((unsigned)settings.historyRetentionDays);
  j += ",\"count\":";
  j += String((unsigned long)historyLineCount);
  j += ",\"seq\":";
  j += String((unsigned long)historySeq);
  j += ",\"points\":[";

  File f = LittleFS.open(HISTORY_FILE, "r");
  if (f) {
    uint32_t total = 0;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.length()) total++;
    }
    f.seek(0);
    uint32_t stride = 1;
    if (total > HISTORY_MAX_POINTS_RETURNED) {
      stride = (total + HISTORY_MAX_POINTS_RETURNED - 1) / HISTORY_MAX_POINTS_RETURNED;
    }
    uint32_t idx = 0;
    bool first = true;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      bool include = (idx % stride == 0) || (idx + 1 == total);
      idx++;
      if (!include) continue;

      unsigned long seq = 0, sec = 0, litres = 0, flags = 0;
      float p = -1.0f, d = -1.0f;
      int n = sscanf(line.c_str(), "%lu,%lu,%f,%lu,%f,%lu", &seq, &sec, &p, &litres, &d, &flags);
      if (n < 6) continue;
      if (!first) j += ',';
      first = false;
      uint32_t ago = 0;
      if (historySeq >= seq) ago = (uint32_t)(historySeq - seq) * settings.historyIntervalSec;
      j += "{\"t\":"; j += String(seq);
      j += ",\"ago\":"; j += String((unsigned long)ago);
      j += ",\"p\":"; j += String(p, 2);
      j += ",\"l\":"; j += String(litres);
      j += ",\"d\":"; j += String(d, 1);
      j += ",\"f\":"; j += String(flags);
      j += '}';
    }
    f.close();
  }
  j += "]}";
  return j;
}

String buildEventsJson() {
  String j;
  j.reserve(1800);
  j = "{\"events\":[";
  File f = LittleFS.open(EVENTS_FILE, "r");
  if (f) {
    const uint32_t maxReturn = 300;
    uint32_t total = 0;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.length()) total++;
    }
    f.seek(0);
    uint32_t skip = total > maxReturn ? total - maxReturn : 0;
    uint32_t idx = 0;
    bool first = true;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (!line.length()) continue;
      if (idx++ < skip) continue;
      unsigned long seq = 0, sec = 0, code = 0;
      float value = 0.0f;
      if (sscanf(line.c_str(), "%lu,%lu,%lu,%f", &seq, &sec, &code, &value) != 4) continue;
      if (!first) j += ',';
      first = false;
      j += "{\"t\":"; j += String(seq);
      j += ",\"sec\":"; j += String(sec);
      j += ",\"code\":"; j += String(code);
      j += ",\"v\":"; j += String(value, 1);
      j += '}';
    }
    f.close();
  }
  j += "]}";
  return j;
}

bool verifyAdmin() {
  // Local dashboard: no passcode / session gate.
  return true;
}

void requireAdminOrFail() {
  if (!verifyAdmin()) sendApiError(401, "admin login required");
}


/* ==========================================================================
   SAFE OUTPUT HELPERS
   ========================================================================== */

void writeRelayPin(uint8_t pin, bool on) {
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

void writeRfPin(uint8_t pin, bool pressed) {
  bool level = RF_ACTIVE_HIGH ? pressed : !pressed;
  digitalWrite(pin, level ? HIGH : LOW);
}

void writeIndicatorPin(uint8_t pin, bool on) {
  bool level = IND_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, level ? HIGH : LOW);
}

void setBuzzerOutput(bool on) {
  // Active buzzer (e.g. TMB12A05): GPIO13 -> 1k -> NPN base, emitter GND,
  // collector -> buzzer(-), buzzer(+) -> +5V. HIGH on GPIO turns NPN on => beep.
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  if (!BUZZER_IS_ACTIVE) {
    if (on) { tone(PIN_BUZZER, 2700); }
    else    { noTone(PIN_BUZZER); digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? LOW : HIGH); }
    return;
  }
  digitalWrite(PIN_BUZZER, (BUZZER_ACTIVE_HIGH == on) ? HIGH : LOW);
}

void initSafeOutputs() {
  // Relay pins first: force the INACTIVE level before making them outputs.
  digitalWrite(PIN_RELAY1, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_RELAY2, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);
  writeRelayPin(PIN_RELAY1, false);
  writeRelayPin(PIN_RELAY2, false);

  // RF remote switching outputs: released.
  digitalWrite(PIN_RF_START, RF_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_RF_STOP, RF_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(PIN_RF_START, OUTPUT);
  pinMode(PIN_RF_STOP, OUTPUT);
  writeRfPin(PIN_RF_START, false);
  writeRfPin(PIN_RF_STOP, false);

  // Indicators off.
  digitalWrite(PIN_IND_BLUE, IND_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(PIN_IND_RED, IND_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(PIN_IND_BLUE, OUTPUT);
  pinMode(PIN_IND_RED, OUTPUT);
  writeIndicatorPin(PIN_IND_BLUE, false);
  writeIndicatorPin(PIN_IND_RED, false);

  // Buzzer off (GPIO13 -> 1k -> NPN base).
  noTone(PIN_BUZZER);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? LOW : HIGH);
  setBuzzerOutput(false);

  // Buzzer-stop button, active low, internal pull-up.
  pinMode(PIN_BTN_BUZZER_STOP, INPUT_PULLUP);
  buttonLastReading = digitalRead(PIN_BTN_BUZZER_STOP);
  buttonStableState = buttonLastReading;
  buttonLastChangeMs = millis();
}

/* ==========================================================================
   WATER MODEL / LINK STATE
   ========================================================================== */

bool computeLinkOnline() {
  if (lastAnyRxMs == 0) return false;
  return (uint32_t)(millis() - lastAnyRxMs) <= LINK_TIMEOUT_MS;
}

bool computeOnline() {
  if (!haveValidWater || !lastPacketValid) return false;
  return (uint32_t)(millis() - lastValidRxMs) <= LINK_TIMEOUT_MS;
}

/* ==========================================================================
   BUZZER STATE MACHINE
   ========================================================================== */

const char *buzzerStateName() {
  if (buzzerTesting)  return "TESTING";
  if (buzzerAlerting) return "ALERTING";
  if (!buzzerArmed)   return "SILENCED";
  return "READY";
}

void stopBuzzerForLinkLoss() {
  // Keep manual/test beep independent of ESP-NOW/water link state.
  // Only stop automatic full-tank alert when water data is no longer valid.
  if (buzzerAlerting) {
    buzzerAlerting = false;
    buzzerArmed = false;      // re-arm only after valid level < 93 %
    setBuzzerOutput(false);
  }
}

void manualStopBuzzer() {
  bool wasActive = buzzerAlerting || buzzerTesting;
  buzzerTesting = false;
  if (buzzerAlerting) {
    buzzerAlerting = false;
    buzzerArmed = false;      // remains disarmed until level < 93 %
  }
  setBuzzerOutput(false);
  if (wasActive) DBGLN("[B] Buzzer stopped by user");
}

void startBuzzerAlert() {
  buzzerTesting = false;
  buzzerAlerting = true;
  buzzerArmed = false;
  buzzerStartMs = millis();
  fullConfirmCount = 0;
  DBGLN("[B] FULL TANK: buzzer alert started");
}

void startBuzzerTest() {
  buzzerTesting = true;
  buzzerTestStartMs = millis();
  DBG("[B] Buzzer test started (%lu s)\n", (unsigned long)(BUZZER_TEST_MS / 1000UL));
}

void updateBuzzerFromWater(float p, bool onlineNow) {
  if (buzzerTesting) return;
  if (!onlineNow) return;
  if (!settings.buzzerEnabled) {
    if (buzzerAlerting) {
      buzzerAlerting = false;
      setBuzzerOutput(false);
    }
    buzzerArmed = true;
    fullConfirmCount = 0;
    return;
  }

  if (p < settings.buzzerRearmPercent) {
    if (buzzerAlerting) {
      buzzerAlerting = false;
      setBuzzerOutput(false);
    }
    if (!buzzerArmed) DBG("[B] Buzzer re-armed (level < %.1f%%)\n", settings.buzzerRearmPercent);
    buzzerArmed = true;
    fullConfirmCount = 0;
    return;
  }

  if (buzzerArmed && p >= settings.buzzerFullPercent) {
    if (fullConfirmCount < 255) fullConfirmCount++;
    if (fullConfirmCount >= settings.buzzerConfirmReadings && !buzzerAlerting) {
      startBuzzerAlert();
    }
  } else if (p < settings.buzzerFullPercent) {
    fullConfirmCount = 0;
  }
}

void serviceBuzzer() {
  if (buzzerTesting) {
    uint32_t testElapsed = millis() - buzzerTestStartMs;
    if (testElapsed >= BUZZER_TEST_MS) {
      buzzerTesting = false;
      setBuzzerOutput(false);
      DBGLN("[B] Buzzer test finished");
      return;
    }
    uint32_t testPhase = testElapsed % settings.buzzerBeepPeriodMs;
    setBuzzerOutput(testPhase < settings.buzzerBeepOnMs);
    return;
  }

  if (!buzzerAlerting) {
    setBuzzerOutput(false);
    return;
  }

  uint32_t elapsed = millis() - buzzerStartMs;
  if (elapsed >= settings.buzzerDurationMs) {
    buzzerAlerting = false;
    buzzerArmed = false;
    setBuzzerOutput(false);
    DBG("[B] Buzzer alert timed out after %lu s\n",
        (unsigned long)(settings.buzzerDurationMs / 1000UL));
    return;
  }

  uint32_t phase = elapsed % settings.buzzerBeepPeriodMs;
  setBuzzerOutput(phase < settings.buzzerBeepOnMs);
}

/* ==========================================================================
   RELAY 1 / MIRROR LIGHT / RELAY 2
   ========================================================================== */

void applyRelay1() {
  bool desired = mirrorOverride ? true : autoRelay1On;
  writeRelayPin(PIN_RELAY1, desired);
  relay1On = desired;
}

void applyRelay2() {
  writeRelayPin(PIN_RELAY2, relay2On);
}

void toggleMirrorOverride() {
  mirrorOverride = !mirrorOverride;
  autoBlocked = false;
  if (!mirrorOverride) autoRelay1On = personDetected;
  applyRelay1();
  DBG("[B] Mirror manual override %s\n", mirrorOverride ? "ON" : "OFF (auto)");
}

void setMirrorRequest(bool on) {
  if (on) {
    mirrorOverride = true;
    autoBlocked = false;
  } else {
    mirrorOverride = false;
    autoBlocked = true;
    autoRelay1On = false;
  }
  applyRelay1();
  DBG("[B] Mirror request %s (override=%s)\n",
      on ? "ON" : "OFF", mirrorOverride ? "ON" : "OFF");
}

/* ==========================================================================
   HC-SR04 (object detection for Relay 1)
   ========================================================================== */

void IRAM_ATTR hcEchoISR() {
  uint32_t now = micros();
  if (digitalRead(PIN_HCSR_ECHO)) {
    hcRiseUs = now;
    hcEchoActive = true;
    hcEchoComplete = false;
  } else if (hcEchoActive) {
    hcFallUs = now;
    hcEchoActive = false;
    hcEchoComplete = true;
  }
}

float medianOfValidHc() {
  float vals[HC_FILTER_SAMPLES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < HC_FILTER_SAMPLES; i++) {
    if (hcRingValid[i]) vals[n++] = hcRing[i];
  }
  if (n == 0) return -1.0f;

  for (uint8_t i = 1; i < n; i++) {
    float key = vals[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }
  if (n & 1) return vals[n / 2];
  return (vals[n / 2 - 1] + vals[n / 2]) * 0.5f;
}

void pushHcSample(float cm, bool valid) {
  // No echo / out-of-range means nothing detected: clear immediately (no delay).
  if (!valid) {
    for (uint8_t i = 0; i < HC_FILTER_SAMPLES; i++) hcRingValid[i] = false;
    hcRingHead = 0;
    hcValid = false;
    personDetected = false;
    hcNearSinceMs = 0;
    hcFarSinceMs = millis();
    if (!mirrorOverride) {
      autoRelay1On = false;
      autoRelay1OnSinceMs = 0;
    }
    applyRelay1();
    return;
  }

  hcRing[hcRingHead] = cm;
  hcRingValid[hcRingHead] = true;
  hcRingHead = (hcRingHead + 1) % HC_FILTER_SAMPLES;

  float med = medianOfValidHc();
  if (med >= 0.0f) {
    hcDistanceCm = med;
    hcValid = true;
    haveHcReading = true;
    lastValidHcMs = millis();
  } else {
    hcValid = false;
  }
}

void startHcSample() {
  noInterrupts();
  hcEchoComplete = false;
  hcEchoActive = false;
  interrupts();

  digitalWrite(PIN_HCSR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_HCSR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_HCSR_TRIG, LOW);

  hcWaitDeadlineUs = micros() + HC_ECHO_TIMEOUT_US;
  hcWaiting = true;
}

void serviceHcSensor() {
  uint32_t now = millis();
  if (!settings.mirrorEnabled) {
    hcValid = false;
    hcWaiting = false;
    return;
  }

  if (hcWaiting) {
    bool done = false;
    bool valid = false;
    float cm = 0.0f;

    if (hcEchoComplete) {
      noInterrupts();
      uint32_t rise = hcRiseUs;
      uint32_t fall = hcFallUs;
      hcEchoComplete = false;
      interrupts();

      uint32_t width = fall - rise;
      if (width >= HC_MIN_ECHO_US && width <= HC_MAX_ECHO_US) {
        cm = (float)width * SOUND_CM_PER_US * 0.5f;
        valid = (cm >= 2.0f && cm <= 400.0f);
      }
      done = true;
    } else if ((int32_t)(micros() - hcWaitDeadlineUs) >= 0) {
      done = true;
    }

    if (done) {
      pushHcSample(cm, valid);
      hcWaiting = false;
      hcNextSampleMs = millis() + HC_SAMPLE_PERIOD_MS;
    }
    return;
  }

  if ((int32_t)(now - hcNextSampleMs) >= 0) {
    startHcSample();
  }
}

void updateAutoRelay1() {
  uint32_t now = millis();

  if (!settings.mirrorEnabled) {
    personDetected = false;
    autoRelay1On = false;
    autoRelay1OnSinceMs = 0;
    hcNearSinceMs = 0;
    hcFarSinceMs = 0;
    applyRelay1();
    return;
  }

  if (hcValid) {
    if (hcDistanceCm < settings.mirrorOnCm) {
      hcFarSinceMs = 0;
      if (hcNearSinceMs == 0) hcNearSinceMs = now;
      if (!personDetected &&
          (uint32_t)(now - hcNearSinceMs) >= settings.mirrorOnConfirmMs) {
        personDetected = true;
        DBG("[B] Person confirmed present (%.1f cm)\n", hcDistanceCm);
      }
    } else if (hcDistanceCm > settings.mirrorOffCm) {
      hcNearSinceMs = 0;
      hcFarSinceMs = now;
      if (personDetected) {
        personDetected = false;
        DBG("[B] Person clear - mirror off immediately (%.1f cm)\n", hcDistanceCm);
      }
    }
  } else {
    hcNearSinceMs = 0;
    hcFarSinceMs = 0;
    if (personDetected) {
      personDetected = false;
      logEvent(9, hcDistanceCm);
      DBG("[B] Person cleared immediately after sensor dropout\n");
    }
  }

  if (mirrorOverride) {
    autoRelay1On = personDetected;
  } else if (autoBlocked) {
    autoRelay1On = false;
    if (!personDetected) {
      autoBlocked = false;
      DBG("[B] Auto mirror re-enabled after clear\n");
    }
  } else {
    autoRelay1On = personDetected;
  }

  if (!mirrorOverride) {
    if (autoRelay1On) {
      if (autoRelay1OnSinceMs == 0) autoRelay1OnSinceMs = now;
      if (settings.mirrorMaxOnMinutes > 0 &&
          (uint32_t)(now - autoRelay1OnSinceMs) >=
              (uint32_t)settings.mirrorMaxOnMinutes * 60000UL) {
        autoRelay1On = false;
        autoBlocked = true;
        autoRelay1OnSinceMs = 0;
        logEvent(8, 0);
        DBGLN("[B] Mirror auto-on time limit reached");
      }
    } else {
      autoRelay1OnSinceMs = 0;
    }
  } else {
    autoRelay1OnSinceMs = 0;
  }

  applyRelay1();
}

/* ==========================================================================
   RF MOTOR MOMENTARY CONTROL
   ========================================================================== */

void releaseMotorOutput(bool startChannel) {
  if (startChannel) {
    if (motorStartPressed) {
      motorStartPressed = false;
      writeRfPin(PIN_RF_START, false);
      DBGLN("[B] RF START released");
    }
  } else {
    if (motorStopPressed) {
      motorStopPressed = false;
      writeRfPin(PIN_RF_STOP, false);
      DBGLN("[B] RF STOP released");
    }
  }
}

void pressMotor(bool startChannel) {
  if (startChannel) {
    if (motorStartPressed) return;   // do not restart auto-release timer
    releaseMotorOutput(false);       // arbitration: release STOP first
    motorStartPressed = true;
    motorStartMs = millis();
    writeRfPin(PIN_RF_START, true);
    DBGLN("[B] RF START pressed");
  } else {
    if (motorStopPressed) return;
    releaseMotorOutput(true);        // arbitration: release START first
    motorStopPressed = true;
    motorStopMs = millis();
    writeRfPin(PIN_RF_STOP, true);
    DBGLN("[B] RF STOP pressed");
  }
}

void serviceMotorSafety() {
  if (motorStartPressed &&
      (uint32_t)(millis() - motorStartMs) >= MOTOR_MAX_PRESS_MS) {
    DBGLN("[B] RF START auto-released (safety timeout)");
    releaseMotorOutput(true);
  }
  if (motorStopPressed &&
      (uint32_t)(millis() - motorStopMs) >= MOTOR_MAX_PRESS_MS) {
    DBGLN("[B] RF STOP auto-released (safety timeout)");
    releaseMotorOutput(false);
  }
  // Defensive arbitration: never allow both channels latched.
  if (motorStartPressed && motorStopPressed) {
    releaseMotorOutput(true);
    releaseMotorOutput(false);
  }
}

uint32_t motorRemainingMs(bool startChannel) {
  bool pressed = startChannel ? motorStartPressed : motorStopPressed;
  if (!pressed) return 0;
  uint32_t started = startChannel ? motorStartMs : motorStopMs;
  uint32_t elapsed = millis() - started;
  return (elapsed >= MOTOR_MAX_PRESS_MS) ? 0 : (MOTOR_MAX_PRESS_MS - elapsed);
}

/* ==========================================================================
   INDICATORS / LINK SERVICE
   ========================================================================== */

void updateIndicators(bool linkOnlineNow) {
  bool blue = linkOnlineNow && fillingActive;
  bool red  = linkOnlineNow && drainingActive;
  if (blue != prevBlue) {
    writeIndicatorPin(PIN_IND_BLUE, blue);
    prevBlue = blue;
  }
  if (red != prevRed) {
    writeIndicatorPin(PIN_IND_RED, red);
    prevRed = red;
  }
}

void serviceLink() {
  bool waterNow = computeOnline();
  bool linkNow = computeLinkOnline();

  if (prevOnline && !waterNow) {
    fullConfirmCount = 0;
    stopBuzzerForLinkLoss();
    DBGLN("[B] Sensor water data OFFLINE / invalid");
  } else if (!prevOnline && waterNow) {
    DBGLN("[B] Sensor water data ONLINE");
  }

  if (prevLinkOnline && !linkNow) {
    fillingActive = false;
    drainingActive = false;
    prevBlue = false;
    prevRed = false;
    writeIndicatorPin(PIN_IND_BLUE, false);
    writeIndicatorPin(PIN_IND_RED, false);
    logEvent(6, 0);
    DBGLN("[B] ESP-NOW link OFFLINE");
  } else if (!prevLinkOnline && linkNow) {
    logEvent(7, 0);
    DBGLN("[B] ESP-NOW link ONLINE");
  }

  prevOnline = waterNow;
  prevLinkOnline = linkNow;
  updateIndicators(linkNow);
}

/* ==========================================================================
   PHYSICAL BUTTON
   ========================================================================== */

void serviceBuzzerButton() {
  int reading = digitalRead(PIN_BTN_BUZZER_STOP);
  if (reading != buttonLastReading) {
    buttonLastReading = reading;
    buttonLastChangeMs = millis();
  }

  if ((uint32_t)(millis() - buttonLastChangeMs) >= BUTTON_DEBOUNCE_MS &&
      reading != buttonStableState) {
    buttonStableState = reading;
    bool pressed = BUTTON_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);

    if (pressed) {
      buttonPressActive = true;
      buttonLongHandled = false;
      buttonPressStartMs = millis();
    } else if (buttonPressActive) {
      buttonPressActive = false;
      if (!buttonLongHandled) manualStopBuzzer();
    }
  }

  if (buttonPressActive && !buttonLongHandled &&
      (uint32_t)(millis() - buttonPressStartMs) >= BUTTON_LONG_PRESS_MS) {
    buttonLongHandled = true;
    toggleMirrorOverride();
    if (buzzerAlerting) manualStopBuzzer();
  }

}

/* ==========================================================================
   OLED
   ========================================================================== */

void initOled() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  uint8_t address = OLED_I2C_ADDRESS;
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0) {
    address = 0x3D;
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      DBGLN("[B] OLED not found - dashboard continues without display");
      return;
    }
    DBGLN("[B] OLED found at 0x3D (config says 0x3C)");
  }

  display.setI2CAddress((uint8_t)(address << 1));
  display.setBusClock(400000);
  display.begin();
  oledReady = true;

  display.setContrast(settings.oledContrast);
  display.sendF("c", settings.oledInvert ? 0x0a7 : 0x0a6);
  display.clearBuffer();
  display.setDrawColor(1);
  display.setFontMode(1);
  display.sendBuffer();
}

void drawOled() {
  if (!oledReady) return;
  if ((uint32_t)(millis() - lastOledMs) < OLED_REFRESH_MS) return;
  lastOledMs = millis();

  bool linkNow = computeLinkOnline();
  uint8_t show = settings.oledShow;

  static uint8_t appliedContrast = 0;
  static int8_t appliedInvert = -1;
  if (appliedContrast != settings.oledContrast) {
    appliedContrast = settings.oledContrast;
    display.setContrast(appliedContrast);
  }
  if (appliedInvert != (int8_t)settings.oledInvert) {
    appliedInvert = (int8_t)settings.oledInvert;
    display.sendF("c", appliedInvert ? 0x0a7 : 0x0a6);
  }

  display.clearBuffer();
  display.setDrawColor(1);
  display.setFontMode(1);

  char pctNum[8];
  if (haveValidWater) snprintf(pctNum, sizeof(pctNum), "%d", (int)lroundf(lastValidPercent));
  else                snprintf(pctNum, sizeof(pctNum), "--");

  if (show & OLED_SHOW_PERCENT) {
    // Keep all content within 128x52 above footer (no top clipping).
    const uint8_t *numFont = u8g2_font_logisoso42_tn;
    int16_t baseline = 45;
    int16_t topY = 4, areaH = 46; // drawing area for custom % symbol
    if (settings.oledPercentSize <= 2) {
      numFont = u8g2_font_logisoso28_tn; baseline = 37; topY = 9; areaH = 28;
    } else if (settings.oledPercentSize <= 4) {
      numFont = u8g2_font_logisoso34_tn; baseline = 41; topY = 6; areaH = 36;
    } else {
      numFont = u8g2_font_logisoso42_tn; baseline = 45; topY = 4; areaH = 42;
    }

    display.setFont(numFont);
    int16_t numW = (int16_t)display.getStrWidth(pctNum);
    int16_t signW = (areaH * 11) / 16;            // custom percent symbol width
    int16_t gap = (areaH >= 40) ? 4 : 3;
    int16_t totalW = numW + gap + signW;
    int16_t x = (OLED_WIDTH - totalW) / 2;
    if (x < 0) x = 0;

    display.drawStr(x, baseline, pctNum);

    // Custom '%' symbol: diagonal slash + two hollow circular dots.
    int16_t sx = x + numW + gap;
    int16_t sy = baseline - areaH;               // top of percent area
    if (sy < 0) sy = 0;
    int16_t r = (areaH >= 40) ? 5 : (areaH >= 34 ? 4 : 3);
    int16_t x1 = sx + r + 1;
    int16_t y1 = sy + r + 1;
    int16_t x2 = sx + signW - r - 2;
    int16_t y2 = sy + areaH - r - 2;
    // Slash (thickened for bold look)
    display.drawLine(x1, y2, x2, y1);
    display.drawLine(x1 + 1, y2, x2 + 1, y1);
    if (areaH >= 36) display.drawLine(x1, y2 - 1, x2, y1 - 1);
    // Two hollow circles
    display.drawCircle(x1, y1, r, U8G2_DRAW_ALL);
    display.drawCircle(x2, y2, r, U8G2_DRAW_ALL);
  }

  char litres[10];
  if (haveValidWater) snprintf(litres, sizeof(litres), "%uL", (unsigned)lastValidLitres);
  else                snprintf(litres, sizeof(litres), "--L");

  // Slightly larger footer text, kept at bottom.
  const uint8_t *bottomFont = u8g2_font_6x10_tf;
  if (settings.oledBottomSize >= 4)      bottomFont = u8g2_font_9x15B_tf;
  else if (settings.oledBottomSize == 3) bottomFont = u8g2_font_8x13B_tf;
  else if (settings.oledBottomSize == 2) bottomFont = u8g2_font_7x13B_tf;
  display.setFont(bottomFont);

  if ((show & OLED_SHOW_DISTANCE) && haveValidWater) {
    char dist[16];
    snprintf(dist, sizeof(dist), "%.0fcm", lastValidDistanceCm);
    display.drawStr(0, 63, dist);
  }

  if (show & OLED_SHOW_LINK) {
    display.drawStr(0, 63, linkNow ? "CONNECTED" : "NO LINK");
  }
  if (show & OLED_SHOW_BOTTOM_L) {
    int16_t x = OLED_WIDTH - (int16_t)display.getStrWidth(litres);
    if (x < 0) x = 0;
    display.drawStr(x, 63, litres);
  }
  if ((show & OLED_SHOW_STATUS) && !(show & OLED_SHOW_LINK)) {
    display.drawStr(0, 63, linkNow ? "LIVE" : "STALE");
  }

  display.sendBuffer();
}

/* ==========================================================================
   JSON API
   ========================================================================== */

String jsonFloatOrNull(bool present, float value, uint8_t decimals) {
  if (!present) return String("null");
  if (!isfinite(value)) return String("null");
  return String((double)value, (unsigned int)decimals);
}

String buildStateJson() {
  bool onlineNow = computeOnline();
  bool linkNow = computeLinkOnline();
  String j;
  j.reserve(1200);

  j += "{";
  j += "\"online\":";        j += onlineNow ? "true" : "false";
  j += ",\"linkOnline\":";   j += linkNow ? "true" : "false";
  j += ",\"stale\":";        j += onlineNow ? "false" : "true";
  j += ",\"sensorValid\":";  j += lastPacketValid ? "true" : "false";
  j += ",\"haveData\":";     j += haveValidWater ? "true" : "false";
  j += ",\"percent\":";      j += jsonFloatOrNull(haveValidWater, lastValidPercent, 2);
  j += ",\"litres\":";
  if (haveValidWater) j += String((unsigned)lastValidLitres);
  else                j += "null";
  j += ",\"distance\":";     j += jsonFloatOrNull(haveValidWater, lastValidDistanceCm, 1);
  j += ",\"filling\":";      j += (linkNow && fillingActive) ? "true" : "false";
  j += ",\"draining\":";     j += (linkNow && drainingActive) ? "true" : "false";
  j += ",\"linkAgeMs\":";
  if (lastAnyRxMs != 0) j += String((uint32_t)(millis() - lastAnyRxMs));
  else                  j += "-1";

  j += ",\"hcsr04Valid\":";  j += hcValid ? "true" : "false";
  j += ",\"hcsr04Distance\":"; j += jsonFloatOrNull(haveHcReading, hcDistanceCm, 1);
  j += ",\"personDetected\":"; j += personDetected ? "true" : "false";
  j += ",\"relay1\":";       j += relay1On ? "true" : "false";
  j += ",\"mirrorOverride\":"; j += mirrorOverride ? "true" : "false";
  j += ",\"autoBlocked\":";  j += autoBlocked ? "true" : "false";
  j += ",\"relay2\":";       j += relay2On ? "true" : "false";

  j += ",\"buzzerActive\":"; j += (buzzerAlerting || buzzerTesting) ? "true" : "false";
  j += ",\"buzzerTest\":";   j += buzzerTesting ? "true" : "false";
  j += ",\"buzzerTestRemainingMs\":";
  j += buzzerTesting ? String((unsigned long)(BUZZER_TEST_MS - (millis() - buzzerTestStartMs))) : "0";
  j += ",\"buzzerArmed\":";  j += buzzerArmed ? "true" : "false";
  j += ",\"buzzerState\":\""; j += buzzerStateName(); j += "\"";

  j += ",\"motorStart\":";   j += motorStartPressed ? "true" : "false";
  j += ",\"motorStop\":";    j += motorStopPressed ? "true" : "false";
  j += ",\"motorStartRemainingMs\":";
  j += String((unsigned long)motorRemainingMs(true));
  j += ",\"motorStopRemainingMs\":";
  j += String((unsigned long)motorRemainingMs(false));

  j += ",\"aFlags\":";       j += String((unsigned)lastFlags);
  j += ",\"aValidEchoes\":";  j += String((unsigned)lastAValidEchoes);
  j += ",\"aTimeoutEchoes\":"; j += String((unsigned)lastATimeoutEchoes);
  j += ",\"aResetReason\":";  j += String((unsigned)lastAResetReason);
  j += ",\"aUptimeMs\":";     j += String((unsigned long)lastAUptimeMs);
  j += ",\"aProtocol\":";    j += String((unsigned)aProtocolVersion);
  j += ",\"aSettingsRevision\":"; j += String((unsigned long)aSeenRevision);
  j += ",\"settingsRevision\":"; j += String((unsigned long)settings.aRevision);
  j += ",\"settingsSynced\":"; j += (haveASettingsRevision && aSeenRevision == aDesiredRevision) ? "true" : "false";
  j += ",\"capacityL\":";    j += String(settings.tankCapacityL, 0);
  j += ",\"historyIntervalSec\":"; j += String((unsigned)settings.historyIntervalSec);
  j += ",\"historyRetentionDays\":"; j += String((unsigned)settings.historyRetentionDays);
  j += ",\"uptimeMs\":";     j += String((unsigned long)millis());
  j += ",\"apClients\":";    j += String((int)WiFi.softAPgetStationNum());
  j += "}";
  return j;
}

void sendJsonState() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "application/json", buildStateJson());
}

void sendApiError(int code, const char *message) {
  String j = "{\"error\":\"";
  for (const char *p = message; *p; ++p) {
    char c = *p;
    if (c == '"' || c == '\\') {
      j += '\\';
      j += c;
    } else if (c == '\n') {
      j += "\\n";
    } else if (c == '\r') {
      j += "\\r";
    } else {
      j += c;
    }
  }
  j += "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", j);
}

/* ==========================================================================
   WEB COMMAND PARSING
   ========================================================================== */

String lowerCopy(String s) {
  s.trim();
  s.toLowerCase();
  return s;
}

bool parseOnValue(const String &raw, bool &out) {
  String b = lowerCopy(raw);
  if (b == "on" || b == "1" || b == "true")  { out = true;  return true; }
  if (b == "off" || b == "0" || b == "false"){ out = false; return true; }

  int key = b.indexOf("\"on\"");
  if (key < 0) return false;
  int colon = b.indexOf(':', key + 4);
  if (colon < 0) return false;

  String v = b.substring(colon + 1);
  v.trim();
  if (v.startsWith("true") || v.startsWith("1"))  { out = true;  return true; }
  if (v.startsWith("false") || v.startsWith("0")) { out = false; return true; }
  return false;
}

bool parseBoolField(const String &raw, const char *name, bool &out) {
  String b = lowerCopy(raw);
  String key = String("\"") + name + "\"";
  int i = b.indexOf(key);
  if (i < 0) return false;
  int colon = b.indexOf(':', i + key.length());
  if (colon < 0) return false;

  String v = b.substring(colon + 1);
  v.trim();
  if (v.startsWith("true") || v.startsWith("1"))  { out = true;  return true; }
  if (v.startsWith("false") || v.startsWith("0")) { out = false; return true; }
  return false;
}

bool parseStringField(const String &raw, const char *name, String &out) {
  String b = lowerCopy(raw);
  String key = String("\"") + name + "\"";
  int i = b.indexOf(key);
  if (i < 0) return false;
  int colon = b.indexOf(':', i + key.length());
  if (colon < 0) return false;
  int q1 = b.indexOf('"', colon + 1);
  if (q1 < 0) return false;
  int q2 = b.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  out = b.substring(q1 + 1, q2);
  out.trim();
  return out.length() > 0;
}

bool jsonFindValueStart(const String &raw, const char *name, int &start) {
  String key = String("\"") + name + "\"";
  int i = raw.indexOf(key);
  if (i < 0) return false;
  int colon = raw.indexOf(':', i + key.length());
  if (colon < 0) return false;
  start = colon + 1;
  return true;
}

bool jsonFloatField(const String &raw, const char *name, float &out) {
  int start = 0;
  if (!jsonFindValueStart(raw, name, start)) return false;
  char *end = nullptr;
  float v = strtof(raw.c_str() + start, &end);
  if (end == raw.c_str() + start || !isfinite(v)) return false;
  out = v;
  return true;
}

bool jsonIntField(const String &raw, const char *name, long &out) {
  int start = 0;
  if (!jsonFindValueStart(raw, name, start)) return false;
  char *end = nullptr;
  long v = strtol(raw.c_str() + start, &end, 10);
  if (end == raw.c_str() + start) return false;
  out = v;
  return true;
}

bool jsonBoolField(const String &raw, const char *name, bool &out) {
  int start = 0;
  if (!jsonFindValueStart(raw, name, start)) return false;
  while (start < (int)raw.length() && isspace((unsigned char)raw[start])) start++;
  if (raw.startsWith("true", start) || raw.startsWith("1", start)) { out = true; return true; }
  if (raw.startsWith("false", start) || raw.startsWith("0", start)) { out = false; return true; }
  return false;
}

bool jsonStringField(const String &raw, const char *name, String &out) {
  int start = 0;
  if (!jsonFindValueStart(raw, name, start)) return false;
  int q1 = raw.indexOf('"', start);
  if (q1 < 0) return false;
  int q2 = raw.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  out = raw.substring(q1 + 1, q2);
  out.trim();
  return out.length() > 0;
}

/* ==========================================================================
   HTTP HANDLERS
   ========================================================================== */

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleState() {
  if (server.method() != HTTP_GET) {
    server.sendHeader("Allow", "GET");
    sendApiError(405, "use GET");
    return;
  }
  sendJsonState();
}

void handleMirrorLight() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  String body = server.arg("plain");
  bool on = false;
  if (body.length() > 256 || !parseOnValue(body, on)) {
    sendApiError(400, "invalid body; expected {\"on\":true|false}");
    return;
  }
  setMirrorRequest(on);
  sendJsonState();
}

void handleBulb() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  String body = server.arg("plain");
  bool on = false;
  if (body.length() > 256 || !parseOnValue(body, on)) {
    sendApiError(400, "invalid body; expected {\"on\":true|false}");
    return;
  }
  relay2On = on;
  applyRelay2();
  DBG("[B] Bulb relay2 -> %s\n", relay2On ? "ON" : "OFF");
  sendJsonState();
}

void handleBuzzerStop() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  if (server.arg("plain").length() > 256) {
    sendApiError(400, "body too large");
    return;
  }
  manualStopBuzzer();
  sendJsonState();
}

void handleBuzzerTest() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  if (server.arg("plain").length() > 256) {
    sendApiError(400, "body too large");
    return;
  }
  startBuzzerTest();
  sendJsonState();
}

void handleMotor() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  String body = server.arg("plain");
  String button;
  bool pressed = false;
  if (body.length() > 256 ||
      !parseStringField(body, "button", button) ||
      !parseBoolField(body, "pressed", pressed)) {
    sendApiError(400, "invalid body; expected {\"button\":\"start|stop\",\"pressed\":true|false}");
    return;
  }
  if (button != "start" && button != "stop") {
    sendApiError(400, "button must be start or stop");
    return;
  }

  bool startChannel = (button == "start");
  if (pressed) pressMotor(startChannel);
  else         releaseMotorOutput(startChannel);

  sendJsonState();
}


/* ==========================================================================
   ADMIN API
   ========================================================================== */

String macString(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

String buildAdminJson() {
  String j;
  j.reserve(4200);
  uint8_t apMac[6], staMac[6];
  WiFi.softAPmacAddress(apMac);
  WiFi.macAddress(staMac);

  j = "{\"settings\":{";
  j += "\"revision\":"; j += String((unsigned long)settings.revision);
  j += ",\"aRevision\":"; j += String((unsigned long)settings.aRevision);
  j += ",\"fullDistanceCm\":"; j += String(settings.fullDistanceCm, 1);
  j += ",\"emptyDistanceCm\":"; j += String(settings.emptyDistanceCm, 1);
  j += ",\"emptyDeadbandCm\":"; j += String(settings.emptyDeadbandCm, 1);
  j += ",\"tankCapacityL\":"; j += String(settings.tankCapacityL, 0);
  j += ",\"sensorMinCm\":"; j += String(settings.sensorMinCm, 1);
  j += ",\"sensorMaxCm\":"; j += String(settings.sensorMaxCm, 1);
  j += ",\"smoothAlpha\":"; j += String(settings.smoothAlpha, 2);
  j += ",\"jumpLimitCm\":"; j += String(settings.jumpLimitCm, 1);
  j += ",\"stuckTimeoutMs\":"; j += String((unsigned long)settings.stuckTimeoutMs);
  j += ",\"measurePeriodMs\":"; j += String((unsigned)settings.measurePeriodMs);
  j += ",\"sampleSpacingMs\":"; j += String((unsigned)settings.sampleSpacingMs);
  j += ",\"jumpConfirm\":"; j += String((unsigned)settings.jumpConfirm);
  j += ",\"samplesPerCycle\":"; j += String((unsigned)settings.samplesPerCycle);

  j += ",\"mirrorEnabled\":"; j += settings.mirrorEnabled ? "true" : "false";
  j += ",\"mirrorOnCm\":"; j += String(settings.mirrorOnCm, 1);
  j += ",\"mirrorOffCm\":"; j += String(settings.mirrorOffCm, 1);
  j += ",\"mirrorOnConfirmMs\":"; j += String((unsigned)settings.mirrorOnConfirmMs);
  j += ",\"mirrorOffConfirmMs\":"; j += String((unsigned)settings.mirrorOffConfirmMs);
  j += ",\"mirrorInvalidHoldoffMs\":"; j += String((unsigned)settings.mirrorInvalidHoldoffMs);
  j += ",\"mirrorMaxOnMinutes\":"; j += String((unsigned)settings.mirrorMaxOnMinutes);

  j += ",\"buzzerEnabled\":"; j += settings.buzzerEnabled ? "true" : "false";
  j += ",\"buzzerIntensity\":"; j += String((unsigned)settings.buzzerIntensity);
  j += ",\"buzzerConfirmReadings\":"; j += String((unsigned)settings.buzzerConfirmReadings);
  j += ",\"buzzerFullPercent\":"; j += String(settings.buzzerFullPercent, 1);
  j += ",\"buzzerRearmPercent\":"; j += String(settings.buzzerRearmPercent, 1);
  j += ",\"buzzerDurationMs\":"; j += String((unsigned long)settings.buzzerDurationMs);
  j += ",\"buzzerBeepPeriodMs\":"; j += String((unsigned)settings.buzzerBeepPeriodMs);
  j += ",\"buzzerBeepOnMs\":"; j += String((unsigned)settings.buzzerBeepOnMs);

  j += ",\"oledShow\":"; j += String((unsigned)settings.oledShow);
  j += ",\"oledPercentSize\":"; j += String((unsigned)settings.oledPercentSize);
  j += ",\"oledLitresSize\":"; j += String((unsigned)settings.oledLitresSize);
  j += ",\"oledBottomSize\":"; j += String((unsigned)settings.oledBottomSize);
  j += ",\"oledContrast\":"; j += String((unsigned)settings.oledContrast);
  j += ",\"oledInvert\":"; j += settings.oledInvert ? "true" : "false";

  j += ",\"historyIntervalSec\":"; j += String((unsigned)settings.historyIntervalSec);
  j += ",\"historyRetentionDays\":"; j += String((unsigned)settings.historyRetentionDays);
  j += ",\"historyEnabled\":"; j += settings.historyEnabled ? "true" : "false";
  j += "}";

  j += ",\"info\":{";
  j += "\"bMac\":\"" + macString(apMac) + "\"";
  j += ",\"bStaMac\":\"" + macString(staMac) + "\"";
  j += ",\"aMac\":\"" + macString(ESP32_A_MAC) + "\"";
  j += ",\"ip\":\"" + WiFi.softAPIP().toString() + "\"";
  j += ",\"ssid\":\"" + String(AP_SSID) + "\"";
  j += ",\"channel\":"; j += String(WIFI_CHANNEL);
  j += ",\"bUptimeMs\":"; j += String((unsigned long)millis());
  j += ",\"bResetReason\":"; j += String((unsigned)esp_reset_reason());
  j += ",\"freeHeap\":"; j += String((unsigned long)ESP.getFreeHeap());
  j += ",\"flashSize\":"; j += String((unsigned long)ESP.getFlashChipSize());
  j += ",\"fsTotal\":"; j += String((unsigned long)LittleFS.totalBytes());
  j += ",\"fsUsed\":"; j += String((unsigned long)LittleFS.usedBytes());
  j += ",\"historyLines\":"; j += String((unsigned long)historyLineCount);
  j += ",\"historySeq\":"; j += String((unsigned long)historySeq);
  j += ",\"eventSeq\":"; j += String((unsigned long)eventSeq);
  j += ",\"linkOnline\":"; j += computeLinkOnline() ? "true" : "false";
  j += ",\"waterOnline\":"; j += computeOnline() ? "true" : "false";
  j += ",\"aSettingsRevision\":"; j += String((unsigned long)aSeenRevision);
  j += ",\"settingsSynced\":"; j += (haveASettingsRevision && aSeenRevision == aDesiredRevision) ? "true" : "false";
  j += ",\"aUptimeMs\":"; j += String((unsigned long)lastAUptimeMs);
  j += ",\"aValidEchoes\":"; j += String((unsigned)lastAValidEchoes);
  j += ",\"aTimeoutEchoes\":"; j += String((unsigned)lastATimeoutEchoes);
  j += ",\"aResetReason\":"; j += String((unsigned)lastAResetReason);
  j += ",\"aFlags\":"; j += String((unsigned)lastFlags);
  j += ",\"pins\":{";
  j += "\"trig\":5,\"echo\":18,\"filling\":4,\"draining\":15";
  j += ",\"oledSda\":21,\"oledScl\":22,\"hcTrig\":25,\"hcEcho\":26";
  j += ",\"relay1\":32,\"relay2\":33,\"buzzer\":13,\"button\":4";
  j += ",\"indBlue\":16,\"indRed\":17,\"rfStart\":18,\"rfStop\":19";
  j += "}}";

  j += ",\"state\":";
  j += buildStateJson();
  j += "}";
  return j;
}

void sendAdminJson() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(200, "application/json", buildAdminJson());
}

void handleAdminSettings() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  if (server.method() == HTTP_GET) {
    sendAdminJson();
    return;
  }
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "GET, POST");
    sendApiError(405, "use GET or POST");
    return;
  }
  String body = server.arg("plain");
  if (body.length() > 4096) { sendApiError(413, "body too large"); return; }

  AppSettings old = settings;
  float f = 0.0f;
  long n = 0;
  bool b = false;

  if (jsonFloatField(body, "fullDistanceCm", f)) settings.fullDistanceCm = f;
  if (jsonFloatField(body, "emptyDistanceCm", f)) settings.emptyDistanceCm = f;
  if (jsonFloatField(body, "emptyDeadbandCm", f)) settings.emptyDeadbandCm = f;
  if (jsonFloatField(body, "tankCapacityL", f)) settings.tankCapacityL = f;
  if (jsonFloatField(body, "sensorMinCm", f)) settings.sensorMinCm = f;
  if (jsonFloatField(body, "sensorMaxCm", f)) settings.sensorMaxCm = f;
  if (jsonFloatField(body, "smoothAlpha", f)) settings.smoothAlpha = f;
  if (jsonFloatField(body, "jumpLimitCm", f)) settings.jumpLimitCm = f;
  if (jsonIntField(body, "stuckTimeoutMs", n)) settings.stuckTimeoutMs = (uint32_t)constrain(n, 10000L, 86400000L);
  if (jsonIntField(body, "measurePeriodMs", n)) settings.measurePeriodMs = (uint16_t)constrain(n, 200L, 10000L);
  if (jsonIntField(body, "sampleSpacingMs", n)) settings.sampleSpacingMs = (uint16_t)constrain(n, 20L, 500L);
  if (jsonIntField(body, "jumpConfirm", n)) settings.jumpConfirm = (uint8_t)constrain(n, 1L, 10L);
  if (jsonIntField(body, "samplesPerCycle", n)) settings.samplesPerCycle = (uint8_t)constrain(n, 3L, 9L);

  if (jsonBoolField(body, "mirrorEnabled", b)) settings.mirrorEnabled = b ? 1 : 0;
  if (jsonFloatField(body, "mirrorOnCm", f)) settings.mirrorOnCm = f;
  if (jsonFloatField(body, "mirrorOffCm", f)) settings.mirrorOffCm = f;
  if (jsonIntField(body, "mirrorOnConfirmMs", n)) settings.mirrorOnConfirmMs = (uint16_t)constrain(n, 50L, 10000L);
  if (jsonIntField(body, "mirrorOffConfirmMs", n)) settings.mirrorOffConfirmMs = (uint16_t)constrain(n, 50L, 10000L);
  if (jsonIntField(body, "mirrorInvalidHoldoffMs", n)) settings.mirrorInvalidHoldoffMs = (uint16_t)constrain(n, 0L, 60000L);
  if (jsonIntField(body, "mirrorMaxOnMinutes", n)) settings.mirrorMaxOnMinutes = (uint16_t)constrain(n, 0L, 1440L);

  if (jsonBoolField(body, "buzzerEnabled", b)) settings.buzzerEnabled = b ? 1 : 0;
  if (jsonIntField(body, "buzzerIntensity", n)) settings.buzzerIntensity = (uint8_t)constrain(n, 0L, 100L);
  if (jsonIntField(body, "buzzerConfirmReadings", n)) settings.buzzerConfirmReadings = (uint8_t)constrain(n, 1L, 20L);
  if (jsonFloatField(body, "buzzerFullPercent", f)) settings.buzzerFullPercent = f;
  if (jsonFloatField(body, "buzzerRearmPercent", f)) settings.buzzerRearmPercent = f;
  if (jsonIntField(body, "buzzerDurationMs", n)) settings.buzzerDurationMs = (uint32_t)constrain(n, 1000L, 300000L);
  if (jsonIntField(body, "buzzerBeepPeriodMs", n)) settings.buzzerBeepPeriodMs = (uint16_t)constrain(n, 100L, 5000L);
  if (jsonIntField(body, "buzzerBeepOnMs", n)) settings.buzzerBeepOnMs = (uint16_t)constrain(n, 10L, 5000L);

  if (jsonIntField(body, "oledShow", n)) settings.oledShow = (uint8_t)(n & 0x3F);
  if (jsonIntField(body, "oledPercentSize", n)) settings.oledPercentSize = (uint8_t)constrain(n, 1L, 6L);
  if (jsonIntField(body, "oledLitresSize", n)) settings.oledLitresSize = (uint8_t)constrain(n, 1L, 6L);
  if (jsonIntField(body, "oledBottomSize", n)) settings.oledBottomSize = (uint8_t)constrain(n, 1L, 4L);
  if (jsonIntField(body, "oledContrast", n)) settings.oledContrast = (uint8_t)constrain(n, 1L, 255L);
  if (jsonBoolField(body, "oledInvert", b)) settings.oledInvert = b ? 1 : 0;

  if (jsonIntField(body, "historyIntervalSec", n)) settings.historyIntervalSec = (uint16_t)constrain(n, (long)HISTORY_MIN_INTERVAL_SEC, (long)HISTORY_MAX_INTERVAL_SEC);
  if (jsonIntField(body, "historyRetentionDays", n)) settings.historyRetentionDays = (uint8_t)constrain(n, 1L, 30L);
  if (jsonBoolField(body, "historyEnabled", b)) settings.historyEnabled = b ? 1 : 0;

  clampSettings(settings);
  commitSettings(old);
  if (settings.aRevision != old.aRevision) {
    if (aProtocolVersion >= 2) pushSettingsToA();
    logEvent(11, (float)settings.aRevision);
  }
  sendAdminJson();
}

void handleAdminRebootA() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  sendCommandToA(CMD_REBOOT_A, true);
  logEvent(10, 1);
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"reboot sent to ESP32_A\"}");
}

void handleAdminRebootB() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  logEvent(10, 2);
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"ESP32_B rebooting\"}");
  delay(400);
  ESP.restart();
}

void handleHistory() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildHistoryJson());
}

void handleEvents() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildEventsJson());
}

void handleAdminClearData() {
  if (!verifyAdmin()) { sendApiError(401, "admin login required"); return; }
  if (server.method() != HTTP_POST) {
    server.sendHeader("Allow", "POST");
    sendApiError(405, "use POST");
    return;
  }
  String body = server.arg("plain");
  bool clearHistory = false, clearEvents = false;
  jsonBoolField(body, "history", clearHistory);
  jsonBoolField(body, "events", clearEvents);
  if (!clearHistory && !clearEvents) { sendApiError(400, "nothing to clear"); return; }

  if (clearHistory) {
    LittleFS.remove(HISTORY_FILE);
    historySeq = 0;
    historyLineCount = 0;
    historyLastSampleMs = millis();
  }
  if (clearEvents) {
    LittleFS.remove(EVENTS_FILE);
    for (uint8_t i = 0; i < 16; i++) lastEventLogMs[i] = 0;
    eventSeq = 0;
  }
  if (!clearEvents) logEvent(13, clearHistory ? 1 : 0);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  sendApiError(404, "not found");
}

/* ==========================================================================
   ESP-NOW
   ========================================================================== */

// Runs in the Wi-Fi driver task: only copy into the queue.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!rxQueue || !info || !data || len <= 0 || len > 64) return;
  RxItem item;
  memset(&item, 0, sizeof(item));
  item.len = (uint8_t)len;
  memcpy(item.data, data, len);
  memcpy(item.src, info->src_addr, 6);
  xQueueSend(rxQueue, &item, 0);
}

void sendStatus() {
  StatusPacket st;
  memset(&st, 0, sizeof(st));
  st.magic = PKT_MAGIC;
  st.version = aProtocolVersion;
  st.type = PKT_STATUS;
  st.seq = ++statusSeq;
  st.uptimeMs = millis();
  st.ackSeq = lastAckSeq;

  if (relay1On)      st.flags |= STATUS_RELAY1;
  if (relay2On)      st.flags |= STATUS_RELAY2;
  if (buzzerAlerting || buzzerTesting) st.flags |= STATUS_BUZZER;
  if (motorStartPressed) st.flags |= STATUS_MOTOR_START;
  if (motorStopPressed)  st.flags |= STATUS_MOTOR_STOP;
  if (personDetected) st.flags |= STATUS_PERSON;
  if (computeOnline()) st.flags |= STATUS_WATER_ONLINE;

  st.hcDistanceCm = haveHcReading ? hcDistanceCm : 0.0f;
  st.waterPercent = haveValidWater ? lastValidPercent : 0.0f;
  st.waterDistanceCm = haveValidWater ? lastValidDistanceCm : 0.0f;
  st.hcValid = hcValid ? 1 : 0;
  st.crc = crc16((const uint8_t *)&st, offsetof(StatusPacket, crc));

  lastStatusTxMs = millis();
  esp_now_send(ESP32_A_MAC, (const uint8_t *)&st, sizeof(st));
}

void handleCommandPacket(const RxItem &item) {
  if (item.len != (int)sizeof(CommandPacket)) return;

  CommandPacket cmd;
  memcpy(&cmd, item.data, sizeof(cmd));
  if (cmd.magic != PKT_MAGIC ||
      (cmd.version != 1 && cmd.version != PROTOCOL_VERSION) ||
      cmd.type != PKT_COMMAND) return;
  if (cmd.crc != crc16((const uint8_t *)&cmd, offsetof(CommandPacket, crc))) return;
  aProtocolVersion = cmd.version;

  bool pressed = (cmd.pressed != 0);
  if (cmd.command == CMD_MOTOR_START) {
    if (pressed) pressMotor(true);
    else         releaseMotorOutput(true);
  } else if (cmd.command == CMD_MOTOR_STOP) {
    if (pressed) pressMotor(false);
    else         releaseMotorOutput(false);
  } else {
    return;
  }

  lastAckSeq = cmd.seq;
  sendStatus();
  DBG("[B] A command=%u pressed=%u seq=%lu\n",
      (unsigned)cmd.command, (unsigned)cmd.pressed,
      (unsigned long)cmd.seq);
}

void handleWaterPacket(const RxItem &item) {
  uint8_t  version = 0;
  uint16_t flags = 0;
  uint32_t seq = 0;
  uint32_t uptimeMs = 0;
  uint32_t settingsRevision = 0;
  float    distanceCm = 0.0f;
  float    percent = 0.0f;
  uint16_t litres = 0;
  uint8_t  valid = 0;
  uint8_t  filling = 0;
  uint8_t  draining = 0;
  uint8_t  validEchoes = 0;
  uint8_t  timeoutEchoes = 0;
  uint8_t  resetReason = 0;

  if (item.len == (int)sizeof(WaterPacket)) {
    WaterPacket pkt;
    memcpy(&pkt, item.data, sizeof(pkt));
    if (pkt.magic != PKT_MAGIC || pkt.version != PROTOCOL_VERSION ||
        pkt.type != PKT_DATA) return;
    if (pkt.crc != crc16((const uint8_t *)&pkt, offsetof(WaterPacket, crc))) return;

    version = pkt.version;
    flags = pkt.flags;
    seq = pkt.seq;
    uptimeMs = pkt.uptimeMs;
    distanceCm = pkt.distanceCm;
    percent = pkt.percent;
    litres = pkt.litres;
    valid = pkt.valid;
    filling = pkt.filling;
    draining = pkt.draining;
    validEchoes = pkt.validEchoes;
    timeoutEchoes = pkt.timeoutEchoes;
    resetReason = pkt.resetReason;
    settingsRevision = pkt.settingsRevision;
  } else if (item.len == (int)sizeof(WaterPacketV1)) {
    WaterPacketV1 pkt;
    memcpy(&pkt, item.data, sizeof(pkt));
    if (pkt.magic != PKT_MAGIC || pkt.version != 1 ||
        pkt.type != PKT_DATA) return;
    if (pkt.crc != crc16((const uint8_t *)&pkt, offsetof(WaterPacketV1, crc))) return;

    version = pkt.version;
    flags = pkt.flags;
    seq = pkt.seq;
    uptimeMs = pkt.uptimeMs;
    distanceCm = pkt.distanceCm;
    percent = pkt.percent;
    litres = pkt.litres;
    valid = pkt.valid;
    filling = pkt.filling;
    draining = pkt.draining;
  } else {
    return;
  }

  uint32_t now = millis();
  aProtocolVersion = version;

  // Duplicate / out-of-order / restart handling.
  bool accept = true;
  if (haveSeq) {
    int32_t diff = (int32_t)(seq - lastSeq);
    bool restarted = (uptimeMs + 1000UL < lastAUptimeMs);
    if (diff == 0) accept = false;
    else if (diff < 0 && !restarted) accept = false;
  }
  if (!accept) return;

  uint16_t oldFlags = lastFlags;
  lastSeq = seq;
  lastAUptimeMs = uptimeMs;
  haveSeq = true;
  lastAnyRxMs = now;
  lastFlags = flags;
  fillingActive = (filling != 0);
  drainingActive = (draining != 0);

  if (version >= 2) {
    lastAValidEchoes = validEchoes;
    lastATimeoutEchoes = timeoutEchoes;
    lastAResetReason = resetReason;
    aSeenRevision = settingsRevision;
    haveASettingsRevision = true;
    if (aSeenRevision != aDesiredRevision) lastSettingsPushMs = 0;
  } else {
    lastAValidEchoes = 0;
    lastATimeoutEchoes = 0;
    lastAResetReason = 0;
    haveASettingsRevision = false;
  }

  uint16_t raised = flags & (uint16_t)~oldFlags;
  if (raised & FLAG_NO_ECHO) logEvent(1, distanceCm);
  if (raised & FLAG_TIMEOUT) logEvent(2, (float)timeoutEchoes);
  if (raised & FLAG_JUMP) logEvent(3, distanceCm);
  if (raised & FLAG_STUCK) logEvent(4, distanceCm);
  if (raised & FLAG_RANGE) logEvent(5, distanceCm);

  float p = percent;
  float d = distanceCm;
  lastPacketValid = (valid != 0) && isfinite(p) && isfinite(d);
  if (!lastPacketValid) fullConfirmCount = 0;

  if (lastPacketValid) {
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    if (d < 0.0f) d = 0.0f;

    lastValidPercent = p;
    lastValidDistanceCm = d;
    lastValidLitres = litres;
    long capLitres = lroundf(settings.tankCapacityL);
    if (lastValidLitres > capLitres) lastValidLitres = (uint16_t)capLitres;
    lastValidRxMs = now;
    haveValidWater = true;

    updateBuzzerFromWater(p, true);
  }

  lastAckSeq = seq;
  sendStatus();

  DBG("[B] RX v%u seq=%lu valid=%u d=%.1fcm p=%.1f%% L=%u fill=%u drain=%u flags=0x%04X\n",
      (unsigned)version, (unsigned long)seq, (unsigned)valid,
      distanceCm, percent, (unsigned)litres,
      (unsigned)filling, (unsigned)draining, (unsigned)flags);
}

void serviceEspNowQueue() {
  if (!rxQueue) return;
  RxItem item;
  while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
    if (item.len >= 6 && item.data[5] == PKT_COMMAND) {
      handleCommandPacket(item);
    } else {
      handleWaterPacket(item);
    }
  }
}

void setupEspNow() {
  if (esp_now_init() != ESP_OK) {
    DBGLN("[B] ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, ESP32_A_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_AP;

  esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
    DBG("[B] esp_now_add_peer failed: %d\n", (int)err);
  }
  DBG("[B] ESP-NOW ready on channel %d\n", WIFI_CHANNEL);
}

void serviceStatusTx() {
  if ((uint32_t)(millis() - lastStatusTxMs) >= STATUS_TX_PERIOD_MS) {
    sendStatus();
  }
}

/* ==========================================================================
   SETUP
   ========================================================================== */

void setup() {
  initSafeOutputs();

  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" Smart Water Tank - ESP32_B main controller");
  Serial.println("==============================================");

  loadSettings();
  if (!LittleFS.begin(true)) {
    Serial.println("[B] LittleFS mount FAILED - history disabled");
  } else {
    loadHistoryMeta();
  }

  // 1) HC-SR04 pins safe.
  digitalWrite(PIN_HCSR_TRIG, LOW);
  pinMode(PIN_HCSR_TRIG, OUTPUT);
  pinMode(PIN_HCSR_ECHO, INPUT);   // Echo MUST be divided to <= 3.3 V
  attachInterrupt(digitalPinToInterrupt(PIN_HCSR_ECHO), hcEchoISR, CHANGE);

  // 2) OLED.
  initOled();

  // 3) Wi-Fi access point with fixed channel (shared with ESP-NOW).
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(apIP, gateway, subnet)) {
    DBGLN("[B] softAPConfig FAILED");
  }
  if (!WiFi.softAP(AP_SSID, AP_WIFI_PASSWORD, WIFI_CHANNEL, false, AP_MAX_CLIENTS)) {
    DBGLN("[B] softAP start FAILED (check password length >= 8)");
  }

  // 4) ESP-NOW queue + radio.
  rxQueue = xQueueCreate(8, sizeof(RxItem));
  if (!rxQueue) DBGLN("[B] Could not create ESP-NOW queue");
  setupEspNow();
  lastStatusTxMs = millis();

  // 5) Web server.
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/api/state", HTTP_ANY, handleState);
  server.on("/api/mirror-light", HTTP_ANY, handleMirrorLight);
  server.on("/api/bulb", HTTP_ANY, handleBulb);
  server.on("/api/buzzer/stop", HTTP_ANY, handleBuzzerStop);
  server.on("/api/buzzer/test", HTTP_ANY, handleBuzzerTest);
  server.on("/api/motor", HTTP_ANY, handleMotor);
  server.on("/api/admin/settings", HTTP_ANY, handleAdminSettings);
  server.on("/api/admin/reboot-a", HTTP_ANY, handleAdminRebootA);
  server.on("/api/admin/reboot-b", HTTP_ANY, handleAdminRebootB);
  server.on("/api/admin/clear-data", HTTP_ANY, handleAdminClearData);
  server.on("/api/history", HTTP_ANY, handleHistory);
  server.on("/api/events", HTTP_ANY, handleEvents);
  server.onNotFound(handleNotFound);
  server.begin();

  // 5b) ArduinoOTA - wireless firmware uploads over ESP32_B AP (192.168.4.1).
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.printf("[B] OTA start: %s\n", type.c_str());
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[B] OTA end");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total) Serial.printf("[B] OTA progress: %u%%\r", (progress * 100U) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[B] OTA error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    else Serial.println("Unknown");
  });
  ArduinoOTA.begin();
  Serial.printf("OTA hostname: %s\n", OTA_HOSTNAME);
  Serial.println("OTA: ready (connect laptop to ESP32_B AP for wireless upload)");

  // 6) Timers.
  hcNextSampleMs = millis() + 300;
  lastOledMs = 0;
  historyLastSampleMs = millis();
  lastSettingsPushMs = 0;

  // 7) Diagnostics (always print MACs - needed for configuration).
  uint8_t apMac[6], staMac[6];
  WiFi.softAPmacAddress(apMac);
  WiFi.macAddress(staMac);
  Serial.printf("AP SSID: %s\n", AP_SSID);
  Serial.printf("AP password: %s  (change before deployment)\n", AP_WIFI_PASSWORD);
  Serial.printf("Dashboard: http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("Channel: %d (ESP-NOW + AP; must match ESP32_A)\n", WIFI_CHANNEL);
  Serial.printf("ESP32_B AP MAC  (use in ESP32_A): %02X:%02X:%02X:%02X:%02X:%02X\n",
                apMac[0], apMac[1], apMac[2], apMac[3], apMac[4], apMac[5]);
  Serial.printf("ESP32_B STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                staMac[0], staMac[1], staMac[2], staMac[3], staMac[4], staMac[5]);
  Serial.printf("Target ESP32_A MAC (must be A STA MAC): %02X:%02X:%02X:%02X:%02X:%02X\n",
                ESP32_A_MAC[0], ESP32_A_MAC[1], ESP32_A_MAC[2],
                ESP32_A_MAC[3], ESP32_A_MAC[4], ESP32_A_MAC[5]);
  Serial.printf("Relay active level: %s\n", RELAY_ACTIVE_HIGH ? "HIGH" : "LOW");
  Serial.printf("Motor max press: %lu ms\n", (unsigned long)MOTOR_MAX_PRESS_MS);
  Serial.printf("Buzzer: alert %.1f%%, max %lu s, re-arm below %.1f%%, intensity %u%%\n",
                settings.buzzerFullPercent,
                (unsigned long)(settings.buzzerDurationMs / 1000UL),
                settings.buzzerRearmPercent,
                (unsigned)settings.buzzerIntensity);
  Serial.printf("Mirror: %s, on < %.1f cm, off > %.1f cm, holdoff %u ms\n",
                settings.mirrorEnabled ? "enabled" : "disabled",
                settings.mirrorOnCm, settings.mirrorOffCm,
                (unsigned)settings.mirrorInvalidHoldoffMs);
  Serial.printf("History: %s, every %u s, retain %u days (%lu lines)\n",
                settings.historyEnabled ? "enabled" : "disabled",
                (unsigned)settings.historyIntervalSec,
                (unsigned)settings.historyRetentionDays,
                (unsigned long)historyLineCount);
  Serial.printf("Button GPIO %d: short=stop buzzer, hold %lu ms=mirror override\n",
                PIN_BTN_BUZZER_STOP,
                (unsigned long)(BUTTON_LONG_PRESS_MS / 1000));
  Serial.println("Reserved buttons: GPIO 2, 14, 23 (intentionally unassigned)");
  Serial.println("Safe state at boot: relays OFF, RF released, indicators/buzzer OFF");
#if DEBUG_SERIAL
  Serial.println("Diagnostics: ENABLED");
#else
  Serial.println("Diagnostics: disabled");
#endif
  Serial.println("Ready.");
}

/* ==========================================================================
   LOOP (non-blocking)
   ========================================================================== */

void loop() {
  ArduinoOTA.handle();
  serviceEspNowQueue();
  serviceLink();
  serviceHcSensor();
  updateAutoRelay1();
  serviceMotorSafety();
  serviceStatusTx();
  serviceBuzzer();
  serviceBuzzerButton();
  serviceSettingsSync();
  serviceHistory();
  drawOled();
  server.handleClient();
  delay(1);
}
