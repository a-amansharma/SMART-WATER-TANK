/*
  ============================================================================
  SMART WATER TANK - ESP32_A (ROOFTOP SENSOR UNIT)
  ============================================================================
  Board target : ESP32 DevKit / ESP32-WROOM-32 (or equivalent)
  Arduino core : esp32:esp32 3.3.11 (tested build target; 3.x ESP-NOW API)
  Sensor       : JSN-SR04T / AJ-SR04M waterproof ultrasonic distance sensor
  Job          : measure distance -> validate/filter -> send water data over
                 ESP-NOW to ESP32_B; read two optional motor push buttons and
                 send START/STOP commands; receive B status for diagnostics.

  IMPORTANT HARDWARE NOTES (read README.md before wiring):
  * GPIO logic is 3.3 V.  The JSN-SR04T/AJ-SR04M Echo output is commonly 5 V.
    Use a resistor divider (1 k series + 2 k to GND) on Echo, or a proper
    level shifter.  Never feed 5 V directly into an ESP32 GPIO.
  * Confirm the trigger threshold of YOUR sensor revision.  Many 5 V
    modules do not reliably accept a 3.3 V trigger.  If yours does not,
    use a level shifter (e.g. 74HCT125 / transistor stage) on TRIG.
  * Power the sensor from its rated 5 V supply and share GND with the ESP32.
  * Check the actual pinout of your ESP32 board before wiring.

  USER-SPECIFIC VALUES TO CHANGE (marked ">>> CHANGE"):
  1. ESP32_B_MAC below = the AP interface MAC of ESP32_B.
  2. WIFI_CHANNEL below must equal the AP channel in ESP32_B.ino.
  3. Trigger/Echo pins if your board uses different safe GPIOs.
  4. PIN_BTN_START / PIN_BTN_STOP for your two motor buttons.  They are
     disabled (-1) until you assign real, safe GPIO numbers.

  Protocol (must match ESP32_B.ino):
    WaterPacket   : magic, version, type=1, flags, seq, uptimeMs,
                    distanceCm, percent, valid, reserved[3], crc16
    StatusPacket  : magic, version, type=2, flags, seq, uptimeMs, ackSeq,
                    hcDistanceCm, waterPercent, waterDistanceCm, hcValid,
                    reserved[3], crc16
    CommandPacket : magic, version, type=3, command, pressed, seq, uptimeMs,
                    reserved[4], crc16
    All structs are packed; CRC16-CCITT over all bytes except trailing crc.
  ============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==========================================================================
   CONFIGURATION - ESP32_A
   ========================================================================== */

#define DEBUG_SERIAL 1                 // 1 = diagnostic serial output, 0 = quiet

// >>> CHANGE: AP interface MAC address of ESP32_B (printed by ESP32_B at boot).
// The example below is a PLACEHOLDER and will not work until you replace it.
static const uint8_t ESP32_B_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Radio channel. MUST exactly match WIFI_CHANNEL in ESP32_B.ino.
// 1, 6 or 11 are the usual non-overlapping choices. Default 6.
#define WIFI_CHANNEL 6

// JSN-SR04T/AJ-SR04M pins (ESP32 DevKit). Echo needs a divider/level shifter.
#define PIN_TRIG 5
#define PIN_ECHO 18

// Optional motor push buttons, wired from GPIO to GND (internal pull-up).
// >>> CHANGE: assign real GPIOs after confirming they are safe on YOUR board.
// -1 = feature disabled; the firmware compiles and the water sensor still works.
#define PIN_BTN_START -1
#define PIN_BTN_STOP  -1
#define BUTTON_ACTIVE_LOW   true
#define BUTTON_DEBOUNCE_MS  40

// Calibration: sensor-to-water distance in centimetres.
static const float FULL_DISTANCE_CM  = 15.0f;   // 100 % (water near sensor)
static const float EMPTY_DISTANCE_CM = 120.0f;  // 0 %
static const float SENSOR_MIN_CM     = 10.0f;   // allow the 15 cm full point
static const float SENSOR_MAX_CM     = 150.0f;

// Measurement / filtering.
static const uint32_t MEASURE_PERIOD_MS  = 1000; // one burst per second
static const uint8_t  SAMPLES_PER_CYCLE  = 5;    // median of up to five echoes
static const uint32_t SAMPLE_SPACING_MS  = 60;   // avoids echo cross-talk
static const uint32_t ECHO_TIMEOUT_US    = 60000;// 60 ms no-echo timeout
static const uint32_t MIN_ECHO_US        = 100;  // ~1.7 cm
static const uint32_t MAX_ECHO_US        = 40000;// ~686 cm
static const float    SOUND_CM_PER_US    = 0.0343f; // ~343 m/s at 20 C

// Sanity / stuck-sensor handling.
static const float    JUMP_LIMIT_CM      = 30.0f;   // sudden impossible jump
static const uint8_t  JUMP_CONFIRM       = 3;       // accept only if repeated
static const float    STUCK_EPS_CM       = 0.15f;   // "same value" tolerance
static const uint32_t STUCK_TIMEOUT_MS   = 600000;  // 10 min bit-identical

// Retry.
static const uint8_t  SEND_RETRIES       = 2;
static const uint32_t SEND_RETRY_MS      = 250;

// Protocol constants (must match ESP32_B).
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

// Status flag bits (same meanings in both sketches).
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
   DEBUG MACROS
   ========================================================================== */

#if DEBUG_SERIAL
  #define DBG(...) Serial.printf(__VA_ARGS__)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(...) do {} while (0)
  #define DBGLN(x) do {} while (0)
#endif

/* ==========================================================================
   STATE
   ========================================================================== */

volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoFallUs = 0;
volatile bool     echoActive = false;
volatile bool     echoComplete = false;

volatile uint8_t  sendStatus = 0;      // 0 pending, 1 success, 2 failure
volatile uint32_t statusCount = 0;
volatile uint32_t lastStatusAckSeq = 0;
volatile uint32_t lastStatusMs = 0;
volatile uint32_t suppressSendCbUntilMs = 0;

WaterPacket pendingPacket;
bool     packetPending = false;
uint8_t  retriesLeft = 0;
uint32_t lastSendAttemptMs = 0;

// Received ESP32_B status (for diagnostics and delivery confirmation).
StatusPacket lastStatus;
volatile bool haveStatus = false;

// Optional START/STOP buttons on this board.
struct LocalButton {
  int      pin;
  uint8_t  command;
  bool     stable;
  bool     lastReading;
  uint32_t changedMs;
};
LocalButton buttons[2] = {
  {PIN_BTN_START, CMD_MOTOR_START, true, HIGH, 0},
  {PIN_BTN_STOP,  CMD_MOTOR_STOP,  true, HIGH, 0}
};

CommandPacket commandBurstPacket;
bool     commandBurstActive = false;
uint8_t  commandBurstRemaining = 0;
uint32_t commandBurstNextMs = 0;

// Measurement state machine.
bool     waitingEcho = false;
uint32_t waitDeadlineUs = 0;
uint32_t nextActionMs = 0;
uint8_t  samplesTaken = 0;
uint8_t  validCount = 0;
uint32_t timeoutCount = 0;
float    sampleBuf[SAMPLES_PER_CYCLE];

// Filtering state.
bool     haveAccepted = false;
float    lastAcceptedCm = 0.0f;
float    lastPercent = 0.0f;
uint32_t lastValidMs = 0;
uint8_t  jumpCount = 0;
uint32_t stuckSinceMs = 0;
uint32_t packetSeq = 0;

/* ==========================================================================
   CRC16-CCITT
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
   ULTRASONIC SENSOR
   ========================================================================== */

void IRAM_ATTR echoISR() {
  uint32_t now = micros();
  if (digitalRead(PIN_ECHO)) {
    echoRiseUs = now;
    echoActive = true;
    echoComplete = false;
  } else if (echoActive) {
    echoFallUs = now;
    echoActive = false;
    echoComplete = true;
  }
}

float medianOf(float *values, uint8_t count) {
  // Small insertion sort; count <= SAMPLES_PER_CYCLE.
  for (uint8_t i = 1; i < count; i++) {
    float key = values[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }
  if (count & 1) return values[count / 2];
  return (values[count / 2 - 1] + values[count / 2]) * 0.5f;
}

float percentFromDistance(float distanceCm) {
  float p = (EMPTY_DISTANCE_CM - distanceCm) /
            (EMPTY_DISTANCE_CM - FULL_DISTANCE_CM) * 100.0f;
  if (p < 0.0f)   p = 0.0f;
  if (p > 100.0f) p = 100.0f;
  return p;
}

void startSample() {
  noInterrupts();
  echoComplete = false;
  echoActive = false;
  interrupts();

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  waitDeadlineUs = micros() + ECHO_TIMEOUT_US;
  waitingEcho = true;
  samplesTaken++;
}

void finalizeBurst() {
  bool valid = true;
  uint16_t flags = 0;
  float median = 0.0f;

  if (validCount < 3) {
    valid = false;
    flags |= FLAG_NO_ECHO;
    if (timeoutCount > 0) flags |= FLAG_TIMEOUT;
  } else {
    median = medianOf(sampleBuf, validCount);

    if (median < SENSOR_MIN_CM || median > SENSOR_MAX_CM) {
      valid = false;
      flags |= FLAG_RANGE;
    }

    // Reject single/few sudden jumps; accept only if the new value repeats.
    if (valid && haveAccepted &&
        fabsf(median - lastAcceptedCm) > JUMP_LIMIT_CM) {
      jumpCount++;
      if (jumpCount < JUMP_CONFIRM) {
        valid = false;
        flags |= FLAG_JUMP;
      } else {
        jumpCount = 0;
      }
    } else if (valid) {
      jumpCount = 0;
    }

    // Stuck-sensor detection: bit-identical distance for a very long time.
    if (valid) {
      if (haveAccepted &&
          fabsf(median - lastAcceptedCm) < STUCK_EPS_CM) {
        if (stuckSinceMs == 0) {
          stuckSinceMs = millis();
        } else if ((uint32_t)(millis() - stuckSinceMs) >= STUCK_TIMEOUT_MS) {
          valid = false;
          flags |= FLAG_STUCK;
        }
      } else {
        stuckSinceMs = 0;
      }
    }
  }

  float reportCm = haveAccepted ? lastAcceptedCm : 0.0f;
  float reportPercent = haveAccepted ? lastPercent : 0.0f;

  if (valid) {
    lastAcceptedCm = median;
    lastPercent = percentFromDistance(median);
    haveAccepted = true;
    lastValidMs = millis();
    reportCm = lastAcceptedCm;
    reportPercent = lastPercent;
  }

  // Build and queue packet.
  memset(&pendingPacket, 0, sizeof(pendingPacket));
  pendingPacket.magic = PKT_MAGIC;
  pendingPacket.version = PROTOCOL_VERSION;
  pendingPacket.type = PKT_DATA;
  pendingPacket.flags = flags;
  pendingPacket.seq = ++packetSeq;
  pendingPacket.uptimeMs = millis();
  pendingPacket.distanceCm = reportCm;
  pendingPacket.percent = reportPercent;
  pendingPacket.valid = valid ? 1 : 0;
  pendingPacket.crc = crc16((const uint8_t *)&pendingPacket,
                            offsetof(WaterPacket, crc));

  packetPending = true;
  retriesLeft = SEND_RETRIES;
  sendStatus = 0;
  lastSendAttemptMs = millis();
  esp_err_t err = esp_now_send(ESP32_B_MAC,
                               (const uint8_t *)&pendingPacket,
                               sizeof(pendingPacket));
  if (err != ESP_OK) {
    sendStatus = 2;
    DBG("[A] esp_now_send immediate error: %d\n", (int)err);
  }

  DBG("[A] seq=%lu valid=%u d=%.1fcm p=%.1f%% flags=0x%04X samples=%u timeouts=%lu\n",
      (unsigned long)pendingPacket.seq, (unsigned)pendingPacket.valid,
      pendingPacket.distanceCm, pendingPacket.percent,
      (unsigned)pendingPacket.flags, (unsigned)validCount,
      (unsigned long)timeoutCount);
}

void serviceMeasurement() {
  uint32_t nowMs = millis();

  if (waitingEcho) {
    bool sampleDone = false;
    bool sampleValid = false;
    float cm = 0.0f;

    if (echoComplete) {
      noInterrupts();
      uint32_t rise = echoRiseUs;
      uint32_t fall = echoFallUs;
      echoComplete = false;
      interrupts();

      uint32_t width = fall - rise;
      if (width >= MIN_ECHO_US && width <= MAX_ECHO_US) {
        cm = (float)width * SOUND_CM_PER_US * 0.5f;
        sampleValid = (cm >= 10.0f && cm <= 400.0f);
      }
      sampleDone = true;
    } else if ((int32_t)(micros() - waitDeadlineUs) >= 0) {
      timeoutCount++;
      sampleDone = true;
    }

    if (sampleDone) {
      if (sampleValid && validCount < SAMPLES_PER_CYCLE) {
        sampleBuf[validCount++] = cm;
      }
      waitingEcho = false;

      if (samplesTaken >= SAMPLES_PER_CYCLE) {
        finalizeBurst();
        samplesTaken = 0;
        validCount = 0;
        timeoutCount = 0;
        nextActionMs = millis() + MEASURE_PERIOD_MS;
      } else {
        nextActionMs = millis() + SAMPLE_SPACING_MS;
      }
    }
    return;
  }

  if ((int32_t)(nowMs - nextActionMs) >= 0) {
    startSample();
  }
}

void serviceSendRetry() {
  if (!packetPending) return;

  // A B status carrying this sequence confirms end-to-end delivery.
  if (lastStatusAckSeq == pendingPacket.seq) {
    packetPending = false;
    return;
  }

  if (sendStatus == 1) {
    packetPending = false;
    return;
  }

  bool retryDue = false;
  if (sendStatus == 2) retryDue = true;
  else if (sendStatus == 0 &&
           (uint32_t)(millis() - lastSendAttemptMs) > 400) retryDue = true;

  if (retryDue) {
    if (retriesLeft > 0) {
      retriesLeft--;
      sendStatus = 0;
      lastSendAttemptMs = millis();
      esp_err_t err = esp_now_send(ESP32_B_MAC,
                                   (const uint8_t *)&pendingPacket,
                                   sizeof(pendingPacket));
      if (err != ESP_OK) sendStatus = 2;
    } else {
      packetPending = false;
      DBG("[A] send failed after retries (seq=%lu)\n",
          (unsigned long)pendingPacket.seq);
    }
  }
}

/* ==========================================================================
   OPTIONAL MOTOR BUTTONS + A -> B COMMANDS
   ========================================================================== */

void sendCommandBurstNow() {
  esp_err_t err = esp_now_send(ESP32_B_MAC,
                               (const uint8_t *)&commandBurstPacket,
                               sizeof(commandBurstPacket));
  suppressSendCbUntilMs = millis() + 400;   // keep water retry state separate
  commandBurstNextMs = millis() + 80;
  if (commandBurstRemaining > 0) commandBurstRemaining--;
  if (commandBurstRemaining == 0) commandBurstActive = false;

  DBG("[A] command=%u pressed=%u seq=%lu send=%d\n",
      (unsigned)commandBurstPacket.command,
      (unsigned)commandBurstPacket.pressed,
      (unsigned long)commandBurstPacket.seq, (int)err);
}

void queueCommand(uint8_t command, bool pressed) {
  memset(&commandBurstPacket, 0, sizeof(commandBurstPacket));
  commandBurstPacket.magic = PKT_MAGIC;
  commandBurstPacket.version = PROTOCOL_VERSION;
  commandBurstPacket.type = PKT_COMMAND;
  commandBurstPacket.command = command;
  commandBurstPacket.pressed = pressed ? 1 : 0;
  commandBurstPacket.seq = ++packetSeq;
  commandBurstPacket.uptimeMs = millis();
  commandBurstPacket.crc = crc16((const uint8_t *)&commandBurstPacket,
                                 offsetof(CommandPacket, crc));

  commandBurstRemaining = 3;                // small burst; command is idempotent
  commandBurstActive = true;
  sendCommandBurstNow();
}

void serviceCommandBurst() {
  if (!commandBurstActive) return;
  if ((int32_t)(millis() - commandBurstNextMs) >= 0) sendCommandBurstNow();
}

void setupButtons() {
  for (uint8_t i = 0; i < 2; i++) {
    if (buttons[i].pin < 0) continue;
    pinMode(buttons[i].pin, INPUT_PULLUP);
    buttons[i].lastReading = digitalRead(buttons[i].pin);
    buttons[i].stable = buttons[i].lastReading;
    buttons[i].changedMs = millis();
  }
}

void serviceButtons() {
  for (uint8_t i = 0; i < 2; i++) {
    LocalButton &b = buttons[i];
    if (b.pin < 0) continue;

    bool reading = digitalRead(b.pin);
    if (reading != b.lastReading) {
      b.lastReading = reading;
      b.changedMs = millis();
    }
    if ((uint32_t)(millis() - b.changedMs) >= BUTTON_DEBOUNCE_MS &&
        reading != b.stable) {
      b.stable = reading;
      bool pressed = BUTTON_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
      queueCommand(b.command, pressed);
    }
  }
}

/* ==========================================================================
   ESP-NOW
   ========================================================================== */

// ESP32 Arduino core 3.3.x send-callback signature.
void onDataSent(const esp_now_send_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  if ((int32_t)(millis() - suppressSendCbUntilMs) < 0) return;
  sendStatus = (status == ESP_NOW_SEND_SUCCESS) ? 1 : 2;
}

// ESP32 Arduino core 3.x signature.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(StatusPacket)) return;

  StatusPacket st;
  memcpy(&st, data, sizeof(st));
  if (st.magic != PKT_MAGIC || st.version != PROTOCOL_VERSION ||
      st.type != PKT_STATUS) return;
  if (st.crc != crc16((const uint8_t *)&st, offsetof(StatusPacket, crc))) return;

  noInterrupts();
  lastStatus = st;
  haveStatus = true;
  lastStatusAckSeq = st.ackSeq;
  lastStatusMs = millis();
  statusCount = statusCount + 1;
  interrupts();
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    DBGLN("[A] ESP-NOW init FAILED");
    return;
  }

  esp_err_t chErr = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chErr != ESP_OK) {
    DBG("[A] esp_wifi_set_channel failed: %d\n", (int)chErr);
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, ESP32_B_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  esp_err_t addErr = esp_now_add_peer(&peer);
  if (addErr != ESP_OK && addErr != ESP_ERR_ESPNOW_EXIST) {
    DBG("[A] esp_now_add_peer failed: %d\n", (int)addErr);
  }

  DBG("[A] ESP-NOW ready, channel %d, peer %02X:%02X:%02X:%02X:%02X:%02X\n",
      WIFI_CHANNEL, ESP32_B_MAC[0], ESP32_B_MAC[1], ESP32_B_MAC[2],
      ESP32_B_MAC[3], ESP32_B_MAC[4], ESP32_B_MAC[5]);
}

/* ==========================================================================
   SETUP / LOOP
   ========================================================================== */

void setup() {
  Serial.begin(115200);
  delay(250);                        // boot-time only, lets USB serial settle
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" Smart Water Tank - ESP32_A rooftop sensor");
  Serial.println("==============================================");

  // Safe output state first: trigger low; optional buttons are inputs.
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);          // Echo MUST be divided to <= 3.3 V
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);

  setupButtons();
  setupEspNow();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("ESP32_A STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("Target ESP32_B MAC: %02X:%02X:%02X:%02X:%02X:%02X (must be B AP MAC)\n",
                ESP32_B_MAC[0], ESP32_B_MAC[1], ESP32_B_MAC[2],
                ESP32_B_MAC[3], ESP32_B_MAC[4], ESP32_B_MAC[5]);
  Serial.printf("Channel: %d (must match ESP32_B AP channel)\n", WIFI_CHANNEL);
  Serial.printf("START button GPIO: %d, STOP button GPIO: %d (-1 = disabled)\n",
                PIN_BTN_START, PIN_BTN_STOP);
  Serial.printf("Calibration: full %.1f cm, empty %.1f cm\n",
                FULL_DISTANCE_CM, EMPTY_DISTANCE_CM);
#if DEBUG_SERIAL
  Serial.println("Diagnostics: ENABLED");
#else
  Serial.println("Diagnostics: disabled");
#endif

  delay(80);                         // JSN-SR04T power/warm-up at boot only
  nextActionMs = millis();
}

void loop() {
  serviceMeasurement();
  serviceSendRetry();
  serviceCommandBurst();
  serviceButtons();

  static uint32_t lastStatusPrintMs = 0;
  static uint32_t lastStatusSeen = 0;
  if (haveStatus && statusCount != lastStatusSeen) {
    lastStatusSeen = statusCount;
    if ((uint32_t)(millis() - lastStatusPrintMs) > 5000) {
      noInterrupts();
      StatusPacket st = lastStatus;
      interrupts();
      lastStatusPrintMs = millis();
      DBG("[A] B status: r1=%u r2=%u person=%u hc=%.1fcm water=%.1f%% age=%lums\n",
          (unsigned)((st.flags & STATUS_RELAY1) != 0),
          (unsigned)((st.flags & STATUS_RELAY2) != 0),
          (unsigned)((st.flags & STATUS_PERSON) != 0),
          st.hcDistanceCm, st.waterPercent,
          (unsigned long)(millis() - lastStatusMs));
    }
  }

  delay(1);
}
