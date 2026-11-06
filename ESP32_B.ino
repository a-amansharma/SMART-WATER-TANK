/*
  ============================================================================
  SMART WATER TANK - ESP32_B (INDOOR MAIN CONTROLLER)
  ============================================================================
  Board target : ESP32 DevKit / ESP32-WROOM-32 (or equivalent)
  Arduino core : esp32:esp32 3.3.11 (tested build target; 3.x ESP-NOW API)
  Libraries    : Adafruit GFX Library, Adafruit SSD1306
  Web server   : built-in WebServer (HTTP polling, JSON; no CDN/cloud)

  Responsibilities:
    * receive water level and START/STOP commands over ESP-NOW from ESP32_A
    * send sensor/status information back to ESP32_A
    * 750 L tank percentage/volume model, trend state, full-tank buzzer
    * HC-SR04 person detection -> Relay 1 automatic mirror light
    * phone manual Mirror Light control, Relay 2 bulb, RF START/STOP
    * 0.96" I2C OLED, Wi-Fi AP "Smart Water Tank", local dashboard
    * four physical buttons: one buzzer-stop, three RESERVED/unassigned

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
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
   CONFIGURATION - ESP32_B
   ========================================================================== */

#define DEBUG_SERIAL 1                 // 1 = diagnostic serial output, 0 = quiet

// >>> CHANGE: ESP32_A station MAC (printed by ESP32_A at boot).
// PLACEHOLDER - the link will not work until this is replaced.
static const uint8_t ESP32_A_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};

// Wi-Fi AP + ESP-NOW channel. MUST exactly match ESP32_A.ino. Default 6.
#define WIFI_CHANNEL 6
static const char AP_SSID[] = "Smart Water Tank";          // exact SSID required
static const char AP_WIFI_PASSWORD[] = "WaterTank2026";    // EXAMPLE - change it
static const uint8_t AP_MAX_CLIENTS = 4;

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
static const uint32_t LINK_TIMEOUT_MS        = 5000;
static const uint32_t OLED_REFRESH_MS        = 250;
static const uint32_t HC_SAMPLE_PERIOD_MS    = 250;
static const uint32_t HC_ECHO_TIMEOUT_US     = 30000;
static const uint32_t HC_MIN_ECHO_US         = 100;
static const uint32_t HC_MAX_ECHO_US         = 25000;
static const float    HC_DETECT_ON_CM        = 100.0f;
static const float    HC_DETECT_OFF_CM       = 105.0f;  // hysteresis
static const uint32_t HC_INVALID_HOLDOFF_MS  = 2000;
static const uint8_t  HC_FILTER_SAMPLES      = 5;
static const float    SOUND_CM_PER_US        = 0.0343f;

static const uint32_t STATUS_TX_PERIOD_MS    = 1000;
static const uint32_t MOTOR_MAX_PRESS_MS     = 4000;
static const uint32_t BUTTON_DEBOUNCE_MS     = 50;

static const float    FULL_LEVEL_PERCENT     = 99.5f; // practical "full" trigger
static const uint8_t  FULL_CONFIRM_READINGS  = 2;
static const float    REARM_PERCENT          = 90.0f;
static const uint32_t BUZZER_ALERT_MAX_MS    = 30000;
static const uint32_t BUZZER_BEEP_PERIOD_MS  = 333;  // 3 beeps per second
static const uint32_t BUZZER_BEEP_ON_MS      = 120;

static const float    TREND_RISE_PCT         = 3.0f;
static const float    TREND_DROP_PCT         = 3.0f;
static const float    TREND_STABLE_BAND_PCT  = 1.0f;
static const uint32_t TREND_STABLE_MS        = 4000;
static const float    TREND_SMOOTH_ALPHA     = 0.35f;

// Protocol constants (must match ESP32_A.ino).
#define PROTOCOL_VERSION 1u
#define PKT_MAGIC 0x57544131UL   // "WTA1"

#define FLAG_NO_ECHO  0x0001u
#define FLAG_RANGE    0x0002u
#define FLAG_JUMP     0x0004u
#define FLAG_STUCK    0x0008u
#define FLAG_TIMEOUT  0x0010u

enum PktType : uint8_t {
  PKT_DATA    = 1,
  PKT_STATUS  = 2,
  PKT_COMMAND = 3
};

#define CMD_MOTOR_START 1u
#define CMD_MOTOR_STOP  2u

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
  uint8_t  valid;
  uint8_t  reserved[3];
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
<meta name="theme-color" content="#ffffff">
<title>Smart Water Tank</title>
<style>
:root{--ink:#14233b;--muted:#68778d;--line:#e4e9f0;--green:#159b63;--red:#df3c4f;--blue:#168bc9}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;width:100%;height:100%;overflow:hidden;overscroll-behavior:none}
body{font-family:Inter,system-ui,-apple-system,"Segoe UI",Arial,sans-serif;background:#fff;color:var(--ink);touch-action:manipulation}
button{font:inherit}
.app{width:min(100%,680px);height:100dvh;margin:auto;padding:clamp(8px,1.8vh,15px) 14px max(20px,env(safe-area-inset-bottom));display:flex;flex-direction:column;gap:clamp(7px,1.4vh,12px)}
header{min-height:clamp(48px,8vh,64px);flex:none;display:flex;align-items:center;justify-content:space-between;gap:8px}
.brand{display:flex;align-items:center;gap:11px;min-width:0}
.logo{width:clamp(40px,6vh,50px);height:clamp(40px,6vh,50px);flex:none;border-radius:14px;background:#f0f7f2;border:1px solid #d7e9dc;display:grid;place-items:center}
.logo svg{width:75%;height:75%}
.brand-title{font-size:clamp(19px,3.5vw,26px);font-weight:900;letter-spacing:-.65px;white-space:nowrap}
.credit{font-size:clamp(9px,1.6vh,11px);color:var(--muted);margin-top:2px;font-weight:650}
.connection{display:flex;align-items:center;gap:7px;padding:8px 11px;border-radius:22px;background:#edfaf3;border:1px solid #bde6cf;color:#087c4a;font-size:11px;font-weight:900;letter-spacing:.6px;flex:none}
.connection.offline{background:#fff0f1;border-color:#f0c2c8;color:#c52c40}
.dot{width:8px;height:8px;border-radius:50%;background:#16a66a;box-shadow:0 0 0 3px #16a66a20}
.offline .dot{background:#df3c4f;box-shadow:0 0 0 3px #df3c4f20}
.card{background:#fff;border:1px solid var(--line);border-radius:18px;box-shadow:0 5px 18px #142b470c}
.tank{padding:clamp(9px,1.6vh,14px) 16px;flex:none}
.card-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:8px}
.eyebrow{font-size:11px;font-weight:900;letter-spacing:1.1px;color:#748197;text-transform:uppercase}
.live{font-size:10px;color:var(--green);font-weight:900;letter-spacing:.6px}.tank.stale .live{opacity:.35}
.water-row{display:flex;align-items:flex-end;justify-content:space-between;gap:8px}
.percent{font-size:clamp(45px,8vh,68px);font-weight:900;letter-spacing:-3.5px;line-height:.88;color:#102039;font-variant-numeric:tabular-nums}
.percent small{font-size:.38em;color:#68778d;letter-spacing:0;margin-left:2px}
.volume{text-align:right;padding-bottom:2px}
.volume strong{font-size:clamp(32px,5.6vh,48px);font-weight:900;display:block;letter-spacing:-2.5px;line-height:.88;font-variant-numeric:tabular-nums;color:#102039}
.volume span{font-size:clamp(12px,1.9vh,15px);color:var(--muted);font-weight:900;letter-spacing:.7px}
.bar-wrap{height:10px;border-radius:12px;background:#edf1f5;border:1px solid #e3e9f0;overflow:hidden;margin:10px 0 5px}
.bar{height:100%;width:0%;border-radius:12px;background:linear-gradient(90deg,#168bc9,#58c7ed);transition:width .35s}
.scale{display:flex;justify-content:space-between;color:#8290a2;font-size:9px;font-weight:750}
.tank-bottom{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-top:7px}
.water-status{font-size:10px;font-weight:950;letter-spacing:.7px;border-radius:20px;padding:6px 11px;background:#eef2f6;color:#65748a}
.water-status.filling{background:#e9f8f0;color:#087d4d}
.water-status.draining{background:#fff0f1;color:#c92e42}
.water-status.stale{background:#eef2f6;color:#7a8799}
.distance{font-size:10px;color:#718096;font-weight:700}
.tank.stale .percent,.tank.stale .volume strong,.tank.stale .bar{opacity:.42}
.controls{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,1.5fr) minmax(0,1fr);gap:10px}
.motor-card{padding:10px;display:flex;flex-direction:column;align-items:center;min-height:0}
.motor-title{text-align:center;margin-bottom:4px}
.motor-title .control-state{margin-top:3px;font-size:clamp(8px,1.3vh,10px);letter-spacing:.4px}
.motor-buttons{display:flex;flex:1;min-height:0;width:100%;flex-direction:column;align-items:center;justify-content:center;gap:clamp(7px,1.2vh,12px)}
.motor-unit{display:flex;flex-direction:column;align-items:center;gap:4px;min-height:0}
.motor-btn{width:clamp(92px,17vh,150px);height:clamp(92px,17vh,150px);max-width:100%;aspect-ratio:1;border:0;border-radius:50%;display:grid;place-items:center;cursor:pointer;touch-action:none;box-shadow:inset 0 3px 3px #ffffffb5,inset 0 -7px 10px #0002,0 7px 0 #0002,0 12px 18px #14233920;transition:transform .09s,box-shadow .09s,filter .09s}
.motor-btn svg{width:43%;height:43%;filter:drop-shadow(0 1px 1px #0002)}
.motor-btn.start{background:linear-gradient(145deg,#3cdda0,#07945a);color:white}
.motor-btn.stop{background:linear-gradient(145deg,#ff7b87,#d92e47);color:white}
.motor-btn:active,.motor-btn.held{transform:translateY(5px);box-shadow:inset 0 3px 8px #0002,0 1px 0 #0001,0 3px 7px #1423391c;filter:brightness(1.05)}
.motor-caption{font-size:clamp(13px,2.2vh,17px);font-weight:950;letter-spacing:1.25px;color:#26364d}
.side-card{padding:9px;display:flex;flex-direction:column;justify-content:center;gap:clamp(6px,1.2vh,11px);min-height:0}
.control-tile{flex:1;min-height:0;border:1px solid #e7ebf1;background:#fbfcfe;border-radius:14px;padding:8px 6px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:clamp(5px,1vh,9px);text-align:center;box-shadow:0 2px 7px #142b4708}
.control-tile.light-on{background:#fff8df;border-color:#f0d98a;box-shadow:0 0 10px #f4d36a88,0 0 22px #f4d36a55,inset 0 0 12px #fff0b8}.control-tile.bulb-on{background:#eaf6ff;border-color:#9ed4f4;box-shadow:0 0 10px #75c6f588,0 0 22px #75c6f555,inset 0 0 12px #d7efff}
.control-name{font-size:clamp(12px,2vh,16px);font-weight:900;line-height:1.1}
.control-state{font-size:clamp(9px,1.4vh,11px);color:#8491a2;font-weight:800;line-height:1.1}
.timer{font-size:clamp(8px,1.25vh,10px);color:#60748b;font-weight:800;min-height:10px}
.toggle{width:clamp(68px,11vh,88px);height:clamp(39px,6.3vh,49px);flex:none;border:1px solid #cbd4df;background:#e9edf2;border-radius:30px;padding:4px;cursor:pointer;transition:.18s;box-shadow:inset 0 2px 4px #0000000b}
.toggle:after{content:"";display:block;width:clamp(29px,4.8vh,39px);height:clamp(29px,4.8vh,39px);border-radius:50%;background:#fff;box-shadow:0 2px 5px #0003;transition:.18s}
.toggle.on{background:#159b63;border-color:#159b63}
.toggle.on:after{transform:translateX(clamp(26px,4.2vh,37px))}
.buzzer-btn{width:min(100%,190px);border:1px solid #c9233c;background:linear-gradient(145deg,#ff6878,#d91f3d);color:#fff;border-radius:13px;padding:clamp(12px,2vh,17px) 10px;font-size:clamp(11px,1.9vh,14px);font-weight:950;letter-spacing:.35px;white-space:nowrap;box-shadow:0 5px 0 #a91c32,0 8px 13px #d91f3d25;cursor:pointer}
.buzzer-btn:active{transform:translateY(4px);box-shadow:0 1px 0 #a91c32}
footer{flex:none;text-align:center;font-size:clamp(9px,1.55vh,11px);line-height:1.2;color:#53647b;font-weight:900;letter-spacing:.45px;white-space:nowrap;padding-bottom:2px;margin-bottom:clamp(8px,2vh,20px)}
@media(max-height:620px){.app{gap:5px;padding-top:5px}.tank{padding:7px 12px}.bar-wrap{margin:6px 0 4px}.tank-bottom{margin-top:5px}.motor-btn{width:clamp(70px,15vh,100px);height:clamp(70px,15vh,100px)}.motor-buttons{gap:4px}.motor-caption{font-size:11px}.side-card{gap:5px;padding:6px}.control-tile{padding:5px}.toggle{width:58px;height:34px}.toggle:after{width:26px;height:26px}.toggle.on:after{transform:translateX(22px)}.buzzer-btn{padding:8px 6px}}
@media(orientation:landscape) and (max-height:520px){.app{width:min(100%,960px);padding:4px 12px 5px;gap:4px}header{min-height:34px}.logo{width:32px;height:32px}.brand-title{font-size:17px}.credit{font-size:8px}.tank{padding:5px 12px}.percent{font-size:40px}.volume strong{font-size:40px;letter-spacing:-2px}.volume span{font-size:10px}.controls{grid-template-columns:.95fr 1.05fr}.motor-card{padding:4px}.motor-buttons{flex-direction:row;justify-content:space-evenly;gap:5px}.motor-btn{width:min(21vh,105px);height:min(21vh,105px)}.motor-caption{font-size:10px}.side-card{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:5px;padding:5px}.control-tile{padding:4px;gap:4px}.control-name{font-size:11px}.toggle{width:48px;height:28px}.toggle:after{width:20px;height:20px}.toggle.on:after{transform:translateX(18px)}.buzzer-btn{font-size:8px;padding:7px 3px}footer{margin-bottom:5px}}
@media(max-width:430px){.app{padding-left:8px;padding-right:8px;gap:7px}.brand{gap:7px}.brand-title{font-size:clamp(16px,5vw,21px)}.connection{padding:6px 8px;font-size:10px}.tank{padding:9px 11px}.percent{font-size:clamp(39px,10vw,55px)}.volume strong{font-size:clamp(30px,7.2vw,40px);letter-spacing:-1.5px}.volume span{font-size:clamp(10px,2.8vw,12px)}.controls{grid-template-columns:minmax(0,1.5fr) minmax(0,1fr);gap:7px}.motor-card{padding:7px 5px}.side-card{padding:6px;gap:6px}.control-tile{padding:6px 3px;gap:5px}.control-name{font-size:clamp(10px,3vw,13px)}.control-state{font-size:9px}.timer{font-size:8px}.motor-btn{width:clamp(72px,20vw,112px);height:clamp(72px,20vw,112px)}.motor-caption{font-size:clamp(11px,3vw,14px)}.toggle{width:clamp(52px,15vw,68px);height:34px}.toggle:after{width:26px;height:26px}.toggle.on:after{transform:translateX(clamp(17px,6vw,29px))}.buzzer-btn{font-size:clamp(9px,2.6vw,11px);padding:10px 3px}footer{font-size:clamp(7px,2vw,9px);letter-spacing:.2px;margin-bottom:8px}}
</style>
</head>
<body>
<main class="app">
<header>
 <div class="brand">
  <div class="logo" aria-hidden="true">
   <svg viewBox="0 0 40 40" fill="none" stroke="#16804f" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M4 18 20 5l16 13M8 16v18h24V16M16 34V23h8v11"/>
    <path d="M12 21h3M25 21h3"/>
   </svg>
  </div>
  <div><div class="brand-title">Smart Water Tank</div><div class="credit">Designed by Aman Sharma</div></div>
 </div>
 <div class="connection offline" id="connection"><i class="dot"></i><span id="connectionText">OFFLINE</span></div>
</header>
<section class="card tank" id="tankCard">
 <div class="card-head"><span class="eyebrow">Tank overview</span><span class="live">● LIVE DATA</span></div>
 <div class="water-row"><div class="percent" id="percent">--<small>%</small></div><div class="volume"><strong id="litres">-- L</strong><span>OF 750.0 L</span></div></div>
 <div class="bar-wrap"><div class="bar" id="bar"></div></div>
 <div class="scale"><span>0 L · EMPTY</span><span>750 L · FULL</span></div>
 <div class="tank-bottom"><span class="water-status" id="waterStatus">WAITING</span><span class="distance" id="distance">Sensor distance: —</span></div>
</section>
<section class="controls">
 <section class="card motor-card">
  <div class="motor-title"><span class="eyebrow">Motor control · RF remote</span><div class="control-state" id="motorStatus">REMOTE STATE UNKNOWN</div></div>
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
   <div class="control-state" id="r1state">STATE UNKNOWN</div>
   <div class="timer" id="mirrorInfo">No connection</div>
  </div>
  <div class="control-tile" id="bulbTile">
   <div class="control-name">Bulb</div>
   <button class="toggle" id="relay2" aria-label="Toggle bulb" aria-pressed="false"></button>
   <div class="control-state" id="r2state">STATE UNKNOWN</div>
  </div>
  <div class="control-tile">
   <div class="control-name">Buzzer</div>
   <button class="buzzer-btn" id="buzzerStop">STOP BUZZER</button>
   <div class="control-state" id="buzzerState">STATE UNKNOWN</div>
  </div>
 </section>
</section>
<footer>SMART WATER TANK · LOCAL ESP32 DASHBOARD · DESIGNED BY AMAN SHARMA</footer>
</main>
<script>
(function(){
'use strict';
const $=id=>document.getElementById(id);
const tankCard=$('tankCard'), connection=$('connection'), connectionText=$('connectionText');
const percentEl=$('percent'), litresEl=$('litres'), barEl=$('bar'), distanceEl=$('distance');
const waterStatusEl=$('waterStatus'), motorStatusEl=$('motorStatus');
const relay1Btn=$('relay1'), relay2Btn=$('relay2'), mirrorTile=$('mirrorTile'), bulbTile=$('bulbTile');
const r1state=$('r1state'), r2state=$('r2state'), mirrorInfo=$('mirrorInfo'), buzzerStateEl=$('buzzerState');
const startBtn=$('motorStart'), stopBtn=$('motorStop');

const POLL_MS=500, POLL_TIMEOUT_MS=2500;
let current=null, reachable=false, failures=0, inFlight=false, pollTimer=null;
let mirrorBusy=false, bulbBusy=false, buzzerBusy=false;

const motor={
 start:{btn:startBtn, held:false, pointerId:null},
 stop:{btn:stopBtn, held:false, pointerId:null}
};

function clamp(v,lo,hi){return Math.max(lo,Math.min(hi,v));}
function num(v){return (typeof v==='number'&&isFinite(v))?v:null;}
function setOnline(online){
 connection.classList.toggle('offline',!online);
 connectionText.textContent=online?'ONLINE':'OFFLINE';
}
function setWaterStatus(status){
 waterStatusEl.textContent=status;
 waterStatusEl.className='water-status '+status.toLowerCase().replace(/[^a-z]/g,'');
}
function flashError(el,msg){
 if(!el) return;
 el.textContent=msg||'COMMAND FAILED';
 clearTimeout(el._errTimer);
 el._errTimer=setTimeout(()=>{ if(reachable&&current) render(current); },1400);
}
function renderUnreachable(){
 if(current) render(current);
 else {
  tankCard.classList.remove('stale');
  percentEl.innerHTML='--<small>%</small>';
  litresEl.textContent='-- L';
  barEl.style.width='0%';
  distanceEl.textContent='Device unreachable · reconnecting…';
  setWaterStatus('NO DATA');
  r1state.textContent='STATE UNKNOWN'; mirrorInfo.textContent='No connection';
  r2state.textContent='STATE UNKNOWN';
  buzzerStateEl.textContent='STATE UNKNOWN';
  motorStatusEl.textContent='REMOTE STATE UNKNOWN';
 }
}
function render(s){
 if(!s||typeof s!=='object') return;
 current=s;
 const online=(reachable&&s.online===true);
 setOnline(online);
 const pct=num(s.percent);
 const hasPct=(pct!==null);
 const shownPct=hasPct?clamp(pct,0,100):null;
 const stale=!online;

 tankCard.classList.toggle('stale',stale);
 if(hasPct){
  percentEl.innerHTML=Math.round(shownPct)+'<small>%</small>';
  const litres=num(s.litres);
  litresEl.textContent=(litres!==null?litres:shownPct*7.5).toFixed(1)+' L';
  barEl.style.width=shownPct.toFixed(1)+'%';
 }else{
  percentEl.innerHTML='--<small>%</small>';
  litresEl.textContent='-- L';
  barEl.style.width='0%';
 }
 const dist=num(s.distance);
 if(!reachable){
  distanceEl.textContent='Device unreachable · reconnecting…';
 }else if(dist!==null){
  distanceEl.textContent=(stale?'Last distance: ':'Sensor distance: ')+dist.toFixed(1)+' cm'+(stale?' · STALE':'');
 }else{
  distanceEl.textContent='Sensor distance: —';
 }
 if(!reachable||!online){
  setWaterStatus(hasPct?'STALE':'NO DATA');
 }else{
  const trend=String(s.trend||'STABLE').toUpperCase();
  setWaterStatus(trend==='FILLING'?'FILLING':trend==='DRAINING'?'DRAINING':'STABLE');
 }

 const mirrorOn=(reachable&&s.relay1===true);
 relay1Btn.classList.toggle('on',mirrorOn);
 relay1Btn.setAttribute('aria-pressed',mirrorOn?'true':'false');
 mirrorTile.classList.toggle('light-on',mirrorOn);
 if(!reachable){
  r1state.textContent='STATE UNKNOWN'; mirrorInfo.textContent='No connection';
 }else{
  const person=(s.personDetected===true);
  const manual=(s.mirrorManual===true);
  const hc=num(s.hcsr04Distance);
  r1state.textContent=mirrorOn?'ON':'OFF';
  let info=manual?'MANUAL':(person?'AUTO · PERSON':'AUTO · CLEAR');
  if(hc!==null) info=(manual?'MANUAL · ':person?'PERSON · ':'CLEAR · ')+(hc/100).toFixed(1)+' m';
  mirrorInfo.textContent=info;
 }

 const bulbOn=(reachable&&s.relay2===true);
 relay2Btn.classList.toggle('on',bulbOn);
 relay2Btn.setAttribute('aria-pressed',bulbOn?'true':'false');
 bulbTile.classList.toggle('bulb-on',bulbOn);
 r2state.textContent=reachable?(bulbOn?'ON':'OFF'):'STATE UNKNOWN';

 if(!reachable){
  buzzerStateEl.textContent='STATE UNKNOWN';
 }else{
  const b=String(s.buzzerState||'READY').toUpperCase();
  buzzerStateEl.textContent=(s.buzzerActive===true)?'ALERTING':b;
 }

 const startPressed=(reachable&&s.motorStart===true), stopPressed=(reachable&&s.motorStop===true);
 startBtn.setAttribute('aria-pressed',startPressed?'true':'false');
 stopBtn.setAttribute('aria-pressed',stopPressed?'true':'false');
 if(!reachable){
  motorStatusEl.textContent='REMOTE STATE UNKNOWN';
 }else if(startPressed){
  const rs=Math.ceil((num(s.motorStartRemainingMs)||0)/1000);
  motorStatusEl.textContent='REMOTE START PRESSED'+(rs>0?' · '+rs+'s':'');
 }else if(stopPressed){
  const rs=Math.ceil((num(s.motorStopRemainingMs)||0)/1000);
  motorStatusEl.textContent='REMOTE STOP PRESSED'+(rs>0?' · '+rs+'s':'');
 }else{
  motorStatusEl.textContent='REMOTE IDLE';
 }
}

async function sendCommand(path,body,errorEl){
 const ctrl=new AbortController();
 const to=setTimeout(()=>ctrl.abort(),3000);
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
  render(s);
  return true;
 }catch(e){
  flashError(errorEl);
  return false;
 }finally{
  clearTimeout(to);
 }
}

function schedulePoll(){
 clearTimeout(pollTimer);
 if(document.hidden) return;
 const delay=reachable?POLL_MS:Math.min(1000+failures*500,5000);
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
  render(s);
 }catch(e){
  failures++;
  reachable=false;
  setOnline(false);
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
$('buzzerStop').addEventListener('click',async()=>{
 if(buzzerBusy) return;
 buzzerBusy=true;
 try{ await sendCommand('/api/buzzer/stop',{},buzzerStateEl); }
 finally{buzzerBusy=false;}
});

setOnline(false);
pollState();
})();
</script>
</body>
</html>
)WTDASH";

/* ==========================================================================
   GLOBAL STATE
   ========================================================================== */

WebServer server(80);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledReady = false;

// --- ESP32_A water data ---
bool     haveValidWater = false;
bool     lastPacketValid = false;
float    lastValidPercent = 0.0f;
float    lastValidDistanceCm = 0.0f;
uint32_t lastValidRxMs = 0;
uint32_t lastAnyRxMs = 0;
uint32_t lastSeq = 0;
uint32_t lastAUptimeMs = 0;
bool     haveSeq = false;
uint16_t lastFlags = 0;

// --- Trend state machine ---
enum TrendState { TREND_STABLE = 0, TREND_FILLING = 1, TREND_DRAINING = 2 };
TrendState trendState = TREND_STABLE;
float    trendSmoothed = 0.0f;
float    trendRef = 0.0f;
bool     haveTrend = false;
bool     stableTimerActive = false;
uint32_t stableSinceMs = 0;

// --- Buzzer ---
bool     buzzerAlerting = false;
bool     buzzerArmed = true;
uint32_t buzzerStartMs = 0;
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
bool     autoRelay1On = false;
bool     personDetected = false;

bool     mirrorManual = false;
bool     mirrorManualOn = false;
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

// --- OLED/status ---
uint32_t lastOledMs = 0;
bool     prevOnline = false;
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
  if (!BUZZER_IS_ACTIVE) {
    if (on) tone(PIN_BUZZER, 2700);
    else    noTone(PIN_BUZZER);
    return;
  }
  bool level = BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_BUZZER, level ? HIGH : LOW);
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

  // Buzzer off.
  if (BUZZER_IS_ACTIVE) {
    digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(PIN_BUZZER, OUTPUT);
    setBuzzerOutput(false);
  } else {
    pinMode(PIN_BUZZER, OUTPUT);
    noTone(PIN_BUZZER);
  }

  // Buzzer-stop button, active low, internal pull-up.
  pinMode(PIN_BTN_BUZZER_STOP, INPUT_PULLUP);
  buttonLastReading = digitalRead(PIN_BTN_BUZZER_STOP);
  buttonStableState = buttonLastReading;
  buttonLastChangeMs = millis();
}

/* ==========================================================================
   WATER MODEL / TREND
   ========================================================================== */

const char *trendName(TrendState t) {
  switch (t) {
    case TREND_FILLING:  return "FILLING";
    case TREND_DRAINING: return "DRAINING";
    default:             return "STABLE";
  }
}

bool computeOnline() {
  if (!haveValidWater || !lastPacketValid) return false;
  return (uint32_t)(millis() - lastValidRxMs) <= LINK_TIMEOUT_MS;
}

void updateTrend(float p, uint32_t now) {
  if (!haveTrend) {
    trendSmoothed = p;
    trendRef = p;
    trendState = TREND_STABLE;
    haveTrend = true;
    stableTimerActive = true;
    stableSinceMs = now;
    return;
  }

  trendSmoothed = trendSmoothed * (1.0f - TREND_SMOOTH_ALPHA) +
                  p * TREND_SMOOTH_ALPHA;

  float diff = trendSmoothed - trendRef;

  switch (trendState) {
    case TREND_STABLE:
      if (diff >= TREND_RISE_PCT) {
        trendState = TREND_FILLING;
        trendRef = trendSmoothed;
        stableTimerActive = false;
      } else if (diff <= -TREND_DROP_PCT) {
        trendState = TREND_DRAINING;
        trendRef = trendSmoothed;
        stableTimerActive = false;
      }
      break;

    case TREND_FILLING:
      if (trendSmoothed > trendRef) trendRef = trendSmoothed;
      if (trendRef - trendSmoothed >= TREND_DROP_PCT) {
        trendState = TREND_DRAINING;
        trendRef = trendSmoothed;
        stableTimerActive = false;
      } else if (fabsf(trendSmoothed - trendRef) <= TREND_STABLE_BAND_PCT) {
        if (!stableTimerActive) {
          stableTimerActive = true;
          stableSinceMs = now;
        } else if ((uint32_t)(now - stableSinceMs) >= TREND_STABLE_MS) {
          trendState = TREND_STABLE;
          trendRef = trendSmoothed;
          stableTimerActive = false;
        }
      } else {
        stableTimerActive = false;
      }
      break;

    case TREND_DRAINING:
      if (trendSmoothed < trendRef) trendRef = trendSmoothed;
      if (trendSmoothed - trendRef >= TREND_RISE_PCT) {
        trendState = TREND_FILLING;
        trendRef = trendSmoothed;
        stableTimerActive = false;
      } else if (fabsf(trendSmoothed - trendRef) <= TREND_STABLE_BAND_PCT) {
        if (!stableTimerActive) {
          stableTimerActive = true;
          stableSinceMs = now;
        } else if ((uint32_t)(now - stableSinceMs) >= TREND_STABLE_MS) {
          trendState = TREND_STABLE;
          trendRef = trendSmoothed;
          stableTimerActive = false;
        }
      } else {
        stableTimerActive = false;
      }
      break;
  }
}

/* ==========================================================================
   BUZZER STATE MACHINE
   ========================================================================== */

const char *buzzerStateName() {
  if (buzzerAlerting) return "ALERTING";
  if (!buzzerArmed)   return "SILENCED";
  return "READY";
}

void stopBuzzerForLinkLoss() {
  if (buzzerAlerting) {
    buzzerAlerting = false;
    buzzerArmed = false;      // re-arm only after valid level < 90 %
    setBuzzerOutput(false);
  }
}

void manualStopBuzzer() {
  if (buzzerAlerting) {
    buzzerAlerting = false;
    buzzerArmed = false;      // remains disarmed until level < 90 %
    setBuzzerOutput(false);
    DBGLN("[B] Buzzer stopped by user");
  }
}

void startBuzzerAlert() {
  buzzerAlerting = true;
  buzzerArmed = false;
  buzzerStartMs = millis();
  fullConfirmCount = 0;
  DBGLN("[B] FULL TANK: buzzer alert started");
}

void updateBuzzerFromWater(float p, bool onlineNow) {
  if (!onlineNow) return;

  // Re-arm only after a valid reading falls below 90 %.
  if (p < REARM_PERCENT) {
    if (buzzerAlerting) {
      buzzerAlerting = false;
      setBuzzerOutput(false);
    }
    if (!buzzerArmed) DBGLN("[B] Buzzer re-armed (level < 90 %)");
    buzzerArmed = true;
    fullConfirmCount = 0;
    return;
  }

  if (buzzerArmed && p >= FULL_LEVEL_PERCENT) {
    if (fullConfirmCount < 255) fullConfirmCount++;
    if (fullConfirmCount >= FULL_CONFIRM_READINGS && !buzzerAlerting) {
      startBuzzerAlert();
    }
  } else if (p < FULL_LEVEL_PERCENT) {
    fullConfirmCount = 0;
  }
}

void serviceBuzzer() {
  if (!buzzerAlerting) {
    setBuzzerOutput(false);
    return;
  }

  uint32_t elapsed = millis() - buzzerStartMs;
  if (elapsed >= BUZZER_ALERT_MAX_MS) {
    buzzerAlerting = false;
    buzzerArmed = false;       // max-time stop also needs < 90 % to re-arm
    setBuzzerOutput(false);
    DBGLN("[B] Buzzer alert timed out after 30 s");
    return;
  }

  uint32_t phase = elapsed % BUZZER_BEEP_PERIOD_MS;
  setBuzzerOutput(phase < BUZZER_BEEP_ON_MS);
}

/* ==========================================================================
   RELAY 1 / MIRROR LIGHT / RELAY 2
   ========================================================================== */

void applyRelay1() {
  bool desired = mirrorManual ? mirrorManualOn : autoRelay1On;
  writeRelayPin(PIN_RELAY1, desired);
  relay1On = desired;
}

void applyRelay2() {
  writeRelayPin(PIN_RELAY2, relay2On);
}

void setMirrorManual(bool on) {
  mirrorManual = true;
  mirrorManualOn = on;
  applyRelay1();
  DBG("[B] Mirror Light manual %s\n", on ? "ON" : "OFF");
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
  hcRing[hcRingHead] = valid ? cm : 0.0f;
  hcRingValid[hcRingHead] = valid;
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
  bool newPerson = personDetected;

  if (hcValid) {
    if (!personDetected && hcDistanceCm < HC_DETECT_ON_CM) {
      newPerson = true;
    } else if (personDetected && hcDistanceCm > HC_DETECT_OFF_CM) {
      newPerson = false;
    }
  } else {
    // Invalid reading: hold briefly, then fail safe to "no person".
    if (personDetected &&
        (uint32_t)(millis() - lastValidHcMs) > HC_INVALID_HOLDOFF_MS) {
      newPerson = false;
    }
  }

  // A real person/no-person transition is the newest event, so it replaces
  // any earlier phone setting.  Phone setting otherwise persists indefinitely.
  if (newPerson != personDetected) {
    personDetected = newPerson;
    mirrorManual = false;
    autoRelay1On = personDetected;
    DBG("[B] Person %s - mirror auto %s\n",
        personDetected ? "detected" : "left",
        autoRelay1On ? "ON" : "OFF");
  } else if (!personDetected) {
    autoRelay1On = false;
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

void updateIndicators(bool onlineNow) {
  bool blue = onlineNow && (trendState == TREND_FILLING);
  bool red  = onlineNow && (trendState == TREND_DRAINING);
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
  bool onlineNow = computeOnline();
  if (prevOnline && !onlineNow) {
    fullConfirmCount = 0;
    stopBuzzerForLinkLoss();
    prevBlue = false;
    prevRed = false;
    writeIndicatorPin(PIN_IND_BLUE, false);
    writeIndicatorPin(PIN_IND_RED, false);
    DBGLN("[B] Sensor link OFFLINE / invalid");
  } else if (!prevOnline && onlineNow) {
    DBGLN("[B] Sensor link ONLINE");
  }
  prevOnline = onlineNow;
  updateIndicators(onlineNow);
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
    if (pressed) manualStopBuzzer();
  }
}

/* ==========================================================================
   OLED
   ========================================================================== */

void initOled() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  if (!oledReady) {
    // Common alternative address.
    oledReady = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
    if (oledReady) {
      DBGLN("[B] OLED found at 0x3D (config says 0x3C)");
    }
  }
  if (!oledReady) {
    DBGLN("[B] OLED not found - dashboard continues without display");
    return;
  }

  display.ssd1306_command(0x81);     // SETCONTRAST
  display.ssd1306_command(0xFF);     // maximum brightness
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.display();
}

void drawOled() {
  if (!oledReady) return;
  if ((uint32_t)(millis() - lastOledMs) < OLED_REFRESH_MS) return;
  lastOledMs = millis();

  bool onlineNow = computeOnline();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  const char *tr = trendName(trendState);
  display.setCursor(0, 0);
  display.print(tr);

  char vol[12];
  if (haveValidWater) snprintf(vol, sizeof(vol), "%.1fL", lastValidPercent * 7.5f);
  else                snprintf(vol, sizeof(vol), "--.-L");
  int16_t x = OLED_WIDTH - (int16_t)strlen(vol) * 6;
  if (x < 0) x = 0;
  display.setCursor(x, 0);
  display.print(vol);

  char pct[10];
  if (haveValidWater) {
    snprintf(pct, sizeof(pct), "%d%%%s",
             (int)lroundf(lastValidPercent), onlineNow ? "" : "*");
  } else {
    snprintf(pct, sizeof(pct), "--%%%s", onlineNow ? "" : "*");
  }
  display.setTextSize(4);
  x = (OLED_WIDTH - (int16_t)strlen(pct) * 24) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 18);
  display.print(pct);

  display.setTextSize(1);
  display.setCursor(0, 56);
  const char *linkText = onlineNow ? "Connected"
                        : ((lastAnyRxMs == 0) ? "Connecting" : "Not Connected");
  display.print(linkText);

  if (onlineNow) {
    const char *bz = buzzerAlerting ? "BUZ:ALERT"
                     : (buzzerArmed ? "BUZ:READY" : "BUZ:SILENT");
    x = OLED_WIDTH - (int16_t)strlen(bz) * 6;
    if (x < 0) x = 0;
    display.setCursor(x, 56);
    display.print(bz);
  }

  display.display();
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
  String j;
  j.reserve(1100);

  j += "{";
  j += "\"online\":";        j += onlineNow ? "true" : "false";
  j += ",\"stale\":";        j += onlineNow ? "false" : "true";
  j += ",\"sensorValid\":";  j += lastPacketValid ? "true" : "false";
  j += ",\"haveData\":";     j += haveValidWater ? "true" : "false";
  j += ",\"percent\":";      j += jsonFloatOrNull(haveValidWater, lastValidPercent, 2);
  j += ",\"litres\":";       j += jsonFloatOrNull(haveValidWater, lastValidPercent * 7.5f, 1);
  j += ",\"distance\":";     j += jsonFloatOrNull(haveValidWater, lastValidDistanceCm, 1);
  j += ",\"trend\":\"";      j += trendName(trendState); j += "\"";
  j += ",\"linkAgeMs\":";
  if (haveValidWater) j += String((uint32_t)(millis() - lastValidRxMs));
  else                j += "-1";

  j += ",\"hcsr04Valid\":";  j += hcValid ? "true" : "false";
  j += ",\"hcsr04Distance\":"; j += jsonFloatOrNull(haveHcReading, hcDistanceCm, 1);
  j += ",\"personDetected\":"; j += personDetected ? "true" : "false";
  j += ",\"relay1\":";       j += relay1On ? "true" : "false";
  j += ",\"mirrorManual\":"; j += mirrorManual ? "true" : "false";
  j += ",\"mirrorManualOn\":"; j += mirrorManualOn ? "true" : "false";
  j += ",\"relay2\":";       j += relay2On ? "true" : "false";

  j += ",\"buzzerActive\":"; j += buzzerAlerting ? "true" : "false";
  j += ",\"buzzerArmed\":";  j += buzzerArmed ? "true" : "false";
  j += ",\"buzzerState\":\""; j += buzzerStateName(); j += "\"";

  j += ",\"motorStart\":";   j += motorStartPressed ? "true" : "false";
  j += ",\"motorStop\":";    j += motorStopPressed ? "true" : "false";
  j += ",\"motorStartRemainingMs\":";
  j += String((unsigned long)motorRemainingMs(true));
  j += ",\"motorStopRemainingMs\":";
  j += String((unsigned long)motorRemainingMs(false));

  j += ",\"aFlags\":";       j += String((unsigned)lastFlags);
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
  setMirrorManual(on);
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
  st.version = PROTOCOL_VERSION;
  st.type = PKT_STATUS;
  st.seq = ++statusSeq;
  st.uptimeMs = millis();
  st.ackSeq = lastAckSeq;

  if (relay1On)      st.flags |= STATUS_RELAY1;
  if (relay2On)      st.flags |= STATUS_RELAY2;
  if (buzzerAlerting) st.flags |= STATUS_BUZZER;
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
  if (cmd.magic != PKT_MAGIC || cmd.version != PROTOCOL_VERSION ||
      cmd.type != PKT_COMMAND) return;
  if (cmd.crc != crc16((const uint8_t *)&cmd, offsetof(CommandPacket, crc))) return;

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
  if (item.len != (int)sizeof(WaterPacket)) return;

  WaterPacket pkt;
  memcpy(&pkt, item.data, sizeof(pkt));
  if (pkt.magic != PKT_MAGIC || pkt.version != PROTOCOL_VERSION ||
      pkt.type != PKT_DATA) return;
  if (pkt.crc != crc16((const uint8_t *)&pkt, offsetof(WaterPacket, crc))) return;

  uint32_t now = millis();

  // Duplicate / out-of-order / restart handling.
  bool accept = true;
  if (haveSeq) {
    int32_t diff = (int32_t)(pkt.seq - lastSeq);
    bool restarted = (pkt.uptimeMs + 1000UL < lastAUptimeMs);
    if (diff == 0) accept = false;
    else if (diff < 0 && !restarted) accept = false;
  }
  if (!accept) return;

  lastSeq = pkt.seq;
  lastAUptimeMs = pkt.uptimeMs;
  haveSeq = true;
  lastAnyRxMs = now;
  lastFlags = pkt.flags;

  float p = pkt.percent;
  float d = pkt.distanceCm;
  lastPacketValid = (pkt.valid != 0) && isfinite(p) && isfinite(d);
  if (!lastPacketValid) fullConfirmCount = 0;

  if (lastPacketValid) {
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    if (d < 0.0f) d = 0.0f;

    lastValidPercent = p;
    lastValidDistanceCm = d;
    lastValidRxMs = now;
    haveValidWater = true;

    updateTrend(p, now);
    updateBuzzerFromWater(p, true);
  }

  lastAckSeq = pkt.seq;
  sendStatus();

  DBG("[B] RX seq=%lu valid=%u d=%.1fcm p=%.1f%% flags=0x%04X\n",
      (unsigned long)pkt.seq, (unsigned)pkt.valid,
      pkt.distanceCm, pkt.percent, (unsigned)pkt.flags);
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
  server.on("/api/motor", HTTP_ANY, handleMotor);
  server.onNotFound(handleNotFound);
  server.begin();

  // 6) Timers.
  hcNextSampleMs = millis() + 300;
  lastOledMs = 0;

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
  serviceEspNowQueue();
  serviceLink();
  serviceHcSensor();
  updateAutoRelay1();
  serviceMotorSafety();
  serviceStatusTx();
  serviceBuzzer();
  serviceBuzzerButton();
  drawOled();
  server.handleClient();
  delay(1);
}
