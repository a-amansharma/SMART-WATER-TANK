# Smart Water Tank

Two ESP32 boards monitor a 750 L water tank and control the pump motor, mirror
light, bulb, buzzer, and indicators.

- **ESP32_A** is the rooftop sensor unit. It measures water level and reads the
  topmost (GPIO 4) and overflow (GPIO 15) water probes.
- **ESP32_B** is the indoor controller. It runs the OLED, relays, buzzer, motor
  remote outputs, Wi-Fi access point, and phone dashboard.
- The two boards talk wirelessly using **ESP-NOW**. No router or internet is
  required.
- The phone dashboard is served by ESP32_B at **http://192.168.4.1/**.

This folder intentionally contains only three files:

```text
Smart Water Tank/
  ESP32_A.ino    rooftop sensor firmware
  ESP32_B.ino    indoor controller firmware + embedded dashboard
  README.md      this document (all wiring and setup)
```

The dashboard HTML is embedded inside `ESP32_B.ino`. There is no separate
`index.html`.

> **Cleanup note:** this README replaces the old separate wiring/OTA documents.
> It also corrects several stale values: the current firmware uses 25 cm full /
> 84 cm empty calibration, a 15 s link timeout, 95 % buzzer-on / 92 % re-arm,
> and a 700 ms dashboard poll. See [Changes made](#14-changes-made-in-this-cleanup).

---

## 1. Quick wiring summary

Everything you need to connect is in this block. The detailed table and notes
follow immediately after it.

```text
ESP32_A  (rooftop sensor)
  JSN-SR04T / AJ-SR04M   VCC  -> 5 V
                         GND  -> GND
                         TRIG -> GPIO 5
                         ECHO -> 1k/2k divider -> GPIO 18
  FILLING probe          GPIO 4  -> probe/switch -> GND   (topmost level)
  DRAINING probe         GPIO 15 -> probe/switch -> GND   (overflow)
  START / STOP buttons   optional, disabled in firmware (-1)

ESP32_B  (indoor controller)
  OLED SSD1306           VCC  -> 3.3 V (check module)
                         GND  -> GND
                         SDA  -> GPIO 21
                         SCL  -> GPIO 22
  HC-SR04                VCC  -> 5 V
                         GND  -> GND
                         TRIG -> GPIO 25
                         ECHO -> 1k/2k divider -> GPIO 26
  Relay 1 (Mirror)       IN1  -> GPIO 32
  Relay 2 (Bulb)         IN2  -> GPIO 33
  Buzzer (via NPN)       base -> 1k -> GPIO 13
  Button                 GPIO 4 -> button -> GND
  Blue LED (via NPN)     base -> 1k -> GPIO 16
  Red LED  (via NPN)     base -> 1k -> GPIO 17
  RF remote START        GPIO 18 -> optocoupler/relay -> remote pads
  RF remote STOP         GPIO 19 -> optocoupler/relay -> remote pads
```

Never feed a 5 V signal directly into an ESP32 GPIO. Both ECHO lines need the
1 kΩ / 2 kΩ divider shown below.

---

## 2. Complete wiring table

| Board | Device / signal | Device terminal | ESP32 pin | Notes |
|---|---|---|---|---|
| **A** | Ultrasonic sensor | VCC | 5 V supply | Use the sensor's rated 5 V supply. |
| **A** | Ultrasonic sensor | GND | GND | Common ground with ESP32_A. |
| **A** | Ultrasonic sensor | TRIG | GPIO 5 | 3.3 V trigger. Add a level shifter if your 5 V module needs one. |
| **A** | Ultrasonic sensor | ECHO | Divider output → GPIO 18 | **5 V signal — divider required.** |
| **A** | Echo divider | R1 1 kΩ | Sensor ECHO → GPIO 18 | Series resistor. |
| **A** | Echo divider | R2 2 kΩ | GPIO 18 → GND | Lower resistor. |
| **A** | Topmost probe (GPIO 4) | Probe / switch | GPIO 4 ↔ GND | Active LOW, internal pull-up. Tank-full alert: buzzer + RED LED. |
| **A** | Overflow probe (GPIO 15) | Probe / switch | GPIO 15 ↔ GND | Active LOW, internal pull-up. BLUE LED + 2 s glow. GPIO 15 is a boot-strap pin; keep it HIGH at reset. |
| **A** | START button | Button | GPIO to assign | Optional. Firmware default is `-1` (disabled). |
| **A** | STOP button | Button | GPIO to assign | Optional. Firmware default is `-1` (disabled). |
| **B** | OLED SSD1306 | VCC | 3.3 V | Check your module; most 4-pin I2C modules are 3.3 V. |
| **B** | OLED SSD1306 | GND | GND | |
| **B** | OLED SSD1306 | SDA | GPIO 21 | I2C data. |
| **B** | OLED SSD1306 | SCL | GPIO 22 | I2C clock. |
| **B** | HC-SR04 | VCC | 5 V | 5 V module. |
| **B** | HC-SR04 | GND | GND | Common ground with ESP32_B. |
| **B** | HC-SR04 | TRIG | GPIO 25 | 3.3 V trigger normally works. |
| **B** | HC-SR04 | ECHO | Divider output → GPIO 26 | **5 V signal — divider required.** |
| **B** | HC-SR04 divider | R1 1 kΩ | ECHO → GPIO 26 | Series resistor. |
| **B** | HC-SR04 divider | R2 2 kΩ | GPIO 26 → GND | Lower resistor. |
| **B** | Relay 1 (Mirror Light) | IN1 | GPIO 32 | Module is assumed active LOW. |
| **B** | Relay 2 (Bulb) | IN2 | GPIO 33 | Module is assumed active LOW. |
| **B** | Relay module | VCC / GND | 5 V / GND | Check whether your module needs 5 V or 3.3 V. |
| **B** | Buzzer NPN base | 1 kΩ resistor | GPIO 13 | NPN drives the buzzer; GPIO never drives it directly. |
| **B** | Buzzer | + / − | +5 V / NPN collector | TMB12A05 5 V active buzzer. |
| **B** | Buzzer-stop button | One side | GPIO 4 | Other side to GND. Internal pull-up. |
| **B** | Blue indicator NPN base | 1 kΩ resistor | GPIO 16 | Overflow (GPIO 15) indicator. |
| **B** | Red indicator NPN base | 1 kΩ resistor | GPIO 17 | Topmost (GPIO 4) indicator. |
| **B** | RF remote START | Optocoupler / relay | GPIO 18 | Across the remote's START button pads only. |
| **B** | RF remote STOP | Optocoupler / relay | GPIO 19 | Across the remote's STOP button pads only. |
| **B** | Main power bulb | — | No GPIO | Wired directly to the main supply. |
| **B** | Reserved buttons | — | GPIO 2, 14, 23 | Not used by the firmware. |

### Echo divider (use on both boards)

```text
Sensor ECHO ---- R1 1 kΩ ----+---- ESP32 GPIO
                              |
                           R2 2 kΩ
                              |
                             GND
```

This gives approximately `5 × 2 / (1 + 2) = 3.33 V` at the GPIO.

### Probes on ESP32_A

```text
ESP32_A GPIO 4  ---- TOPMOST level probe / tilt switch ---- GND
ESP32_A GPIO 15 ---- OVERFLOW probe / switch ------------- GND
```

- Open circuit = inactive (internal pull-up holds GPIO HIGH).
- Closed circuit to GND = active (GPIO reads LOW). Both probes are submerged
  sensors, so a submerged probe reads LOW.
- **GPIO 4 (topmost):** when water reaches the topmost level, ESP32_B beeps the
  buzzer 5×/sec and blinks the RED LED in sync. There is no hold, so the alert
  stops the moment the probe leaves the water.
- **GPIO 15 (overflow):** when water reaches the overflow probe, ESP32_B lights
  the BLUE LED and keeps it on for an extra 2 s after the probe releases.
- For long probe leads, add a 10 kΩ pull-up from the GPIO to 3.3 V and a
  100 nF capacitor from GPIO to GND close to the ESP32.
- Keep probe wiring away from mains wiring.
- Use stainless-steel or corrosion-resistant electrodes.

### Relay polarity

The firmware assumes an **active-LOW** relay module:

```cpp
#define RELAY_ACTIVE_HIGH false
```

If your relay turns ON when its input is HIGH, change it to `true`.

At boot both relays are forced OFF before they become outputs. Add external
pull resistors so the relay inputs stay inactive while the ESP32 is resetting:

- Active-LOW module: 10 kΩ from each IN pin to **3.3 V**.
- Active-HIGH module: 10 kΩ from each IN pin to **GND**.

### Buzzer driver

```text
+5 V ---- TMB12A05 positive
TMB12A05 negative ---- NPN collector
                         |
GPIO 13 ---- 1 kΩ ---- NPN base
                         |
                      NPN emitter
                         |
                        GND
```

The firmware assumes an active buzzer with an active-HIGH GPIO output. Add a
1N4148 diode across the buzzer if it is inductive or noisy.

### Blue / red indicator driver

```text
+5 V or +12 V
     |
  indicator + series resistor
     |
  NPN collector
     |
GPIO 16 or 17 ---- 1 kΩ ---- NPN base
     |
  NPN emitter
     |
    GND
```

Use a MOSFET or relay for larger loads. Add a flyback diode across an inductive
load.

### RF remote START / STOP

Do not connect the ESP32 directly to unknown remote button pads.

```text
ESP32 GPIO 18 or 19 ---- 330 Ω ---- PC817 pin 1 (anode)
                                    PC817 pin 2 (cathode) ---- GND

Remote button pad A ---- PC817 pin 4 (collector)
Remote button pad B ---- PC817 pin 3 (emitter)
```

- Each optocoupler must connect across the two pads of **one** remote button.
- Verify the remote PCB with a multimeter before wiring.
- If the remote uses a matrix keypad, connect across the exact two pads that
  the physical button shorts.
- Both RF outputs are released at boot and auto-release after 4 seconds.
- START and STOP cannot be active at the same time.

---

## 3. Pin configuration

### ESP32_A

| Function | GPIO | Notes |
|---|---:|---|
| Ultrasonic TRIG | 5 | Output. |
| Ultrasonic ECHO | 18 | Input, must be divided to ≤ 3.3 V. |
| Topmost level probe | 4 | Active LOW, `INPUT_PULLUP`. Buzzer + RED LED when submerged. |
| Overflow probe | 15 | Active LOW, `INPUT_PULLUP`; boot-strap pin. BLUE LED + 2 s glow. |
| START button | -1 | Disabled. Assign a safe GPIO if needed. |
| STOP button | -1 | Disabled. Assign a safe GPIO if needed. |

### ESP32_B

| Function | GPIO | Notes |
|---|---:|---|
| OLED SDA | 21 | I2C. |
| OLED SCL | 22 | I2C. |
| HC-SR04 TRIG | 25 | Output. |
| HC-SR04 ECHO | 26 | Input, must be divided to ≤ 3.3 V. |
| Relay 1 (Mirror Light) | 32 | Active LOW by default. |
| Relay 2 (Bulb) | 33 | Active LOW by default. |
| Buzzer NPN base | 13 | Active HIGH. |
| Buzzer-stop / override button | 4 | Active LOW, `INPUT_PULLUP`. |
| Blue overflow indicator | 16 | Active HIGH. |
| Red topmost-level indicator | 17 | Active HIGH. |
| RF remote START | 18 | Active HIGH. |
| RF remote STOP | 19 | Active HIGH. |
| Reserved | 2, 14, 23 | Not initialized by the firmware. |

### Optional ESP32_A motor buttons

The two START/STOP button GPIOs are intentionally disabled:

```cpp
#define PIN_BTN_START -1
#define PIN_BTN_STOP  -1
```

To enable them, replace both `-1` values with safe output-capable GPIOs, wire
each button from the GPIO to GND, and re-upload ESP32_A. Do not use GPIO 34–39
(input-only, no internal pull-up) and do not use GPIO 4 or 15 (already used by
the probes).

---

## 4. Tank calibration

Defaults are stored in NVS and can be changed at runtime from the Admin API
(see [HTTP API](#11-http-api)) without recompiling:

| Setting | Default |
|---|---:|
| Full distance (100 %) | 25.0 cm |
| Empty distance (0 %) | 84.0 cm |
| Empty deadband | 0.0 cm |
| Sensor range | 2.0 – 200.0 cm |
| Tank capacity | 750 L |
| Samples per reading | 5 |
| Sample spacing | 60 ms |
| Measurement period | 500 ms |
| Rolling median window | 5 readings |
| Smoothing (EMA) | 0.50 |
| Jump limit | 15 cm per reading |
| Jump confirm | 2 readings |
| Stuck timeout | 24 h (off) |

Percentage formula (the only one in the firmware):

```text
percent = ((emptyDistance - measuredDistance) / (emptyDistance - fullDistance)) × 100
       = ((84 - measuredDistance) / 59) × 100
```

Result is clamped to 0–100 %. Reference points:

| Tank | Distance | Percent |
|---|---:|---:|
| Empty | 84.0 cm | 0 % |
| 25 % | 69.25 cm | 25 % |
| Half | 54.5 cm | 50 % |
| 75 % | 39.75 cm | 75 % |
| Full | 25.0 cm (or less) | 100 % |

The sensor faces down, so **water rising means distance falling and percentage
rising**. ESP32_A computes this value once per reading; the OLED and the web
dashboard display exactly that value and never re-map it.

Measure from the sensor face to the water surface. If you move the sensor or
change the tank, remeasure both points and save them through the Admin API.
ESP32_B forwards the new settings to ESP32_A over ESP-NOW.

The firmware now rejects a reversed calibration (full farther than empty)
instead of silently inverting the level, and it raises the range limit if
needed so a real empty reading is not discarded.

> **Flash both boards together.** ESP32_B stores the same calibration/filter
> values in NVS and pushes them to ESP32_A. If you flash only one board, the
> other can overwrite the new defaults with its old stored values. The settings
> version was bumped, so the first boot after flashing both boards resets the
> stored values to the table above. Re-enter any custom calibration afterwards.

### Important: ultrasonic blind zone

The JSN-SR04T / AJ-SR04M waterproof probe cannot measure closer than about
**20–25 cm**. The 25 cm full point sits right at the edge of that blind zone, so
the sensor can read a little short or drift when the tank is nearly full. That is
why the physical **GPIO 4 topmost probe** is the reliable "tank is full"
indicator: when water reaches it, ESP32_B beeps the buzzer 5×/sec and blinks the
RED LED, independently of the ultrasonic reading.

If you want the percentage itself to reach 100 % comfortably, the reliable fix is
mechanical and does **not** change any wiring:

1. Raise the sensor (or use a short standpipe) so the full water surface stays
   clearly beyond the blind zone, then measure the new full/empty distances and
   save them through the Admin API.
2. Or keep the sensor where it is and set the full calibration to the closest
   distance the sensor actually reads reliably, accepting that the last few
   centimetres are reported as 100 %.

There is no software formula that can recover a distance the sensor did not
measure. The firmware prints a boot-time warning when the full calibration is
inside this blind zone.

---

## 5. Water measurement logic

1. ESP32_A triggers the sensor 5 times per reading.
2. A 60 ms no-echo timeout and the 2–200 cm range reject bad samples.
3. **Burst median:** the median of the valid samples in that reading is taken.
4. **Jump gate:** a reading more than 15 cm from the previous accepted reading is
   treated as an echo artefact and not displayed. If two consecutive readings
   agree on the new value, it is accepted and the filter snaps to it. This gate
   is symmetric, so it delays (but never blocks) both filling and draining.
5. **Rolling median:** the median of the last 5 accepted readings removes
   multi-second spikes.
6. **EMA:** a 0.50 exponential moving average smooths the remaining noise. A
   genuine level change is visible in roughly 1–2 seconds.
7. The filtered distance is converted to percentage and litres **once**, in
   ESP32_A, using the calibration above.
8. ESP32_A sends a `WaterPacket` about every 0.8 s and immediately when a probe
   changes. The OLED and dashboard use the received `percent` unchanged.
9. A probe must be stable for 50 ms. The **overflow** probe (GPIO 15) keeps its
   state for 2 s after release so short contact bounces do not flicker the
   dashboard. The **topmost** probe (GPIO 4) has no hold — its full alert stops
   immediately when it leaves the water.

ESP32_B keeps the last valid water value, marks it stale if new valid data stops,
and shows `NO LINK` when no packet at all has arrived for 15 seconds. With
`DEBUG_SERIAL 1`, ESP32_B logs a warning if the received percentage disagrees
with the shared calibration, which catches an old or reversed calibration on A.

---

## 6. ESP32_B behaviour

### Mirror Light (Relay 1)

- HC-SR04 detects a person closer than 100 cm → Relay 1 ON.
- **Background rejection:** the firmware learns the empty-room background
  distance (floor, wall, or reflections, e.g. ~105 cm). A reading only counts as
  a person when it is at least **18 cm closer than that background**, so an
  ambient echo hovering around the 100 cm threshold no longer turns the light on
  by itself. The background tracks slow ambient drift but stops following as
  soon as a reading is clearly closer, and is frozen while a person is
  confirmed.
- **Noise rejection:** readings below 10 cm are treated as invalid spurious
  echoes; a sudden change of more than 35 cm from the last accepted distance is
  ignored until it repeats 3 times; and a detection needs 3 consecutive accepted
  near readings plus a minimum 600 ms confirm time. Short bursts and glitches no
  longer turn the light on.
- Person farther than 105 cm (or no longer clearly closer than the background)
  → Relay 1 OFF after a 1 s hold, so a brief loss of detection does not switch
  the mirror light off instantly. Moving back within range during that second
  cancels the turn-off.
- Invalid HC-SR04 reading → Relay 1 OFF after the same 1 s hold (sensor dropout
  is not treated as an immediate "person gone").
- The sensor keeps sampling continuously (every 250 ms); the 1 s delay only
  applies to switching Relay 1 off, not to detection speed.
- Phone ON/OFF works at any time.
- Phone OFF exits manual override and blocks automatic turn-on until the area is
  clear.
- Holding GPIO 4 for 3 seconds toggles manual override.
- Optional maximum auto-on time is available in settings; default is off.

### Bulb (Relay 2)

- Phone toggle only. The state is retained until changed.

### Buzzer

- **Topmost-probe alert (GPIO 4):** while the topmost probe is submerged, the
  buzzer beeps 5×/sec and the RED LED blinks in sync with it. This runs
  continuously (no time limit) and stops the moment the probe leaves the water.
  It is silenced by the buzzer-disabled setting or by STOP BUZZER, and re-arms
  when the probe clears.
- **Level-based full alert:** starts at **95 %** for 2 consecutive valid
  readings and beeps 5×/sec for up to **40 seconds**.
- Short press on GPIO 4 (ESP32_B button) stops it.
- A 3-second long press also stops an active alert while toggling mirror override.
- The dashboard TEST BUZZER button runs the same pattern for **5 seconds**.
- STOP BUZZER ends an alert or test immediately.
- The level alert re-arms only after the level falls below **92 %**.
- The level alert stops automatically when water data becomes invalid.

`buzzerIntensity` is stored in settings but is not applied by this build; the
output is plain on/off. Treat it as reserved.

### Motor control

- Phone START/STOP buttons are momentary: press sends `pressed`, release sends
  `released`.
- Optional ESP32_A buttons send the same commands over ESP-NOW.
- A press is released automatically after 4 seconds.
- Pressing START releases STOP first; pressing STOP releases START first.
- There is no motor feedback, so the dashboard shows commands, not proof that
  the motor is running.

### Indicators

| Input state | ESP32_B indicator | Dashboard capsule |
|---|---|---|
| Topmost probe active (GPIO 4) | Red LED, blinking 5×/sec with the buzzer | Red `TOP LEVEL` |
| Overflow probe active (GPIO 15) | Blue LED ON (+ 2 s glow) | Blue `OVERFLOWING` |
| Neither probe active | Both LEDs OFF | `STABLE` |
| ESP-NOW link lost | Both LEDs OFF | `NO A LINK` |

The two small **RED LED** / **BLUE LED** buttons at the bottom of the Motor
control box are hardware test toggles. Each one forces its indicator LED ON,
even when the ESP-NOW link is down or the matching probe is inactive. Tap again
to release the override and return the LED to automatic control. The override
is not saved across a reboot.

---

## 7. Network, MAC addresses, and OTA

Both boards must use the same ESP-NOW channel. The firmware default is
**channel 6**.

### ESP32_B access point

```text
SSID:      Smart Water Tank
Password:  dks12345
Dashboard: http://192.168.4.1/
Channel:   6
```

Change `AP_WIFI_PASSWORD` in `ESP32_B.ino` before deployment. The password must
be at least 8 characters.

### ESP32_A OTA access point

```text
SSID:      • TP-Link_8080
Password:  dks12345
Address:   192.168.10.1
Channel:   6
```

ESP32_A hosts this AP only for wireless firmware uploads. It shares channel 6
with ESP-NOW.

### MAC addresses

Only two values are required:

| Value needed | Comes from | Goes into |
|---|---|---|
| ESP32_A **AP MAC** | ESP32_A Serial Monitor: `ESP32_A AP MAC ...` | `ESP32_A_MAC` in `ESP32_B.ino` |
| ESP32_B **AP MAC** | ESP32_B Serial Monitor: `ESP32_B AP MAC ...` | `ESP32_B_MAC` in `ESP32_A.ino` |

Both boards run in AP mode, so use their **AP** MACs as ESP-NOW peers. The AP MAC
is normally the STA MAC + 1 on an ESP32, but always verify it from the serial
output.

Current values in the code:

```cpp
// ESP32_A.ino
static const uint8_t ESP32_B_MAC[6] = {0x30, 0x76, 0xF5, 0xE4, 0xFD, 0xD5};

// ESP32_B.ino
static const uint8_t ESP32_A_MAC[6] = {0x00, 0x70, 0x07, 0xA1, 0xDC, 0x81};
```

If either board is replaced, update these values and re-upload.

### OTA

After one USB flash, both boards can be updated wirelessly:

| Board | Connect laptop to | OTA hostname | OTA password |
|---|---|---|---|
| ESP32_A | `• TP-Link_8080` / `dks12345` | `ESP32-WATER-A` | `dks12345` |
| ESP32_B | `Smart Water Tank` / `dks12345` | `ESP32-WATER-B` | `dks12345` |

Use an OTA-capable partition scheme, for example **Default 4MB with spiffs**.
Keep power connected during upload.

---

## 8. Two-way ESP-NOW protocol

ESP32_A sends:

- **WaterPacket** — distance, percentage, litres, valid flag, sequence, uptime,
  fault flags, echo diagnostics, reset reason, and applied settings revision.
- **CommandPacket** — START/STOP/reboot, pressed/released, sequence, uptime.

ESP32_B sends:

- **StatusPacket** — HC-SR04 distance, person state, water values, relay states,
  buzzer state, motor states, link state, uptime, and acknowledged sequence.
- **SettingsPacket** — calibration, filtering, measurement timing, and revision.
- **CommandPacket** — reboot ESP32_A.

All packets are packed, use magic `WTA1`, and carry CRC16-CCITT over every byte
except the trailing CRC. Protocol version is **2**. ESP32_B also accepts legacy
v1 WaterPacket and CommandPacket frames from an older ESP32_A.

Settings flow: ESP32_B is the source of truth. It sends the settings packet to
ESP32_A, ESP32_A saves them to NVS and resets its measurement state, and the new
revision is reported in the next WaterPacket. ESP32_B retries until the
revisions match.

---

## 9. Compile and upload

### Arduino IDE

1. Install the ESP32 board package (target: `esp32:esp32` 3.3.11 or newer).
2. Install **U8g2** (used by the SSD1306 OLED).
3. Select **ESP32 Dev Module**.
4. Open `ESP32_A.ino` and upload it to the rooftop board.
5. Open `ESP32_B.ino` and upload it to the indoor board.
6. Confirm both boards use channel 6 and the correct peer MACs.

### Important Arduino sketch-folder rule

Arduino expects the folder name to match the sketch name. This folder is named
`Smart Water Tank`, so it cannot compile both `.ino` files directly. Copy each
sketch into its own matching folder before compiling:

```text
ESP32_A/ESP32_A.ino
ESP32_B/ESP32_B.ino
```

The three source files can still be stored together for delivery.

### Arduino CLI

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all ESP32_A
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all ESP32_B
```

### Verified build results (ESP32 core 3.3.11)

| Sketch | Flash | RAM |
|---|---:|---:|
| ESP32_A | 978,031 bytes (74 %) | 52,584 bytes (16 %) |
| ESP32_B | 1,182,362 bytes (90 %) | 55,992 bytes (17 %) |

Both sketches compile with `--warnings all` without warnings.

---

## 10. Dashboard

ESP32_B serves the embedded dashboard at `http://192.168.4.1/`.

- Polls `/api/state` every 700 ms.
- Request timeout is 2.5 s; failures back off up to 4 s.
- Shows percentage, litres, bar, sensor distance, and TOP LEVEL / OVERFLOWING /
  STABLE capsules.
- Mirror Light and Bulb toggles follow the actual relay state.
- Motor buttons are press-and-hold and release automatically when the page loses
  focus or closes.
- Buzzer TEST (5 s) and STOP buttons work without a page reload.

The dashboard has no separate Admin screen. Use the Admin API endpoints below
from a browser, `curl`, or a small script.

---

## 11. HTTP API

| Method | Path | Body | Purpose |
|---|---|---|---|
| GET | `/api/state` | — | Current water, relay, buzzer, motor, and sensor state. |
| POST | `/api/mirror-light` | `{"on":true}` | Mirror Light ON/OFF. |
| POST | `/api/bulb` | `{"on":true}` | Bulb ON/OFF. |
| POST | `/api/buzzer/test` | `{}` | 5-second buzzer test. |
| POST | `/api/buzzer/stop` | `{}` | Stop alert or test. |
| POST | `/api/led/test` | `{"led":"blue","on":true}` | Force Blue/Red indicator LED ON/OFF for testing. |
| POST | `/api/motor` | `{"button":"start","pressed":true}` | Press or release the RF motor output. |
| GET | `/api/admin/settings` | — | Read all settings and diagnostics. |
| POST | `/api/admin/settings` | JSON fields | Change settings. |
| POST | `/api/admin/reboot-a` | `{}` | Reboot ESP32_A. |
| POST | `/api/admin/reboot-b` | `{}` | Reboot ESP32_B. |
| POST | `/api/admin/clear-data` | `{"history":true}` and/or `{"events":true}` | Clear stored data. |
| GET | `/api/history` | — | Water history CSV-derived JSON. |
| GET | `/api/events` | — | Event log JSON. |

There is **no passcode** on the local AP. Anyone connected to `Smart Water Tank`
can use these endpoints. Change the AP password before deployment.

### Setting examples

```bash
curl -X POST http://192.168.4.1/api/mirror-light \
  -H 'Content-Type: application/json' -d '{"on":true}'

curl -X POST http://192.168.4.1/api/admin/settings \
  -H 'Content-Type: application/json' \
  -d '{"tankCapacityL":750,"fullDistanceCm":25,"emptyDistanceCm":84}'
```

Settings are saved to NVS on ESP32_B. Water settings are forwarded to ESP32_A
and also persisted in ESP32_A NVS.

---

## 12. Storage and history

ESP32_B uses LittleFS for:

- `/history.csv` — water samples, default every 300 s, 4 days retained.
- `/events.csv` — fault/link/reboot/settings events, capped at 1,200 lines.

There is no RTC. Graph ages are approximate and derived from the configured
sampling interval, not wall-clock time.

---

## 13. Test checklist and troubleshooting

### First power-up

1. Power ESP32_B with no mains load connected. Confirm relays are OFF.
2. Connect to `Smart Water Tank` and open `http://192.168.4.1/`.
3. Confirm the OLED shows `NO LINK` before ESP32_A starts.
4. Power ESP32_A and confirm the OLED changes to `CONNECTED`.
5. Move the water sensor and watch the percentage change.
6. Short ESP32_A GPIO 4 to GND: the buzzer should beep 5×/sec and the RED LED
   should blink with it; the dashboard shows a red `TOP LEVEL` capsule. Short
   GPIO 15 to GND: the BLUE LED lights and the dashboard shows blue
   `OVERFLOWING`, staying on for 2 s after you release the probe.
7. With no probe active, tap **BLUE LED** then **RED LED** under the Motor
   control buttons. Each LED should light and stay lit until tapped again.
8. Hold a hand within 100 cm of the HC-SR04. Mirror Light should turn ON.
9. Move beyond 105 cm. Mirror Light should turn OFF after about 1 second.
10. Test the Bulb toggle.
11. Press TEST BUZZER, then stop it early with STOP BUZZER and with the physical
    button.
12. Test phone START/STOP. Each press should release after 4 seconds.

### Troubleshooting

| Problem | Check |
|---|---|
| OLED blank | 3.3 V, GND, SDA 21, SCL 22, address 0x3C/0x3D. |
| Dashboard does not open | Join `Smart Water Tank`; use `http://192.168.4.1/`. |
| Water shows no data | Both AP MACs, both channels, ESP32_A power, sensor wiring. |
| Distance always invalid | Echo divider, sensor supply, calibration values, 60 ms timeout. |
| Level falls while filling / tops out at 75–85 % | Sensor is inside its 20–25 cm blind zone; see [Important: ultrasonic blind zone](#important-ultrasonic-blind-zone). |
| Buzzer never reaches full alert | If the tank tops out below 95 %, the sensor cannot see the real full level; fix the mounting or lower the full calibration. |
| Percentage disagrees with distance | Check the serial log for `percent/distance disagree`; A is running an old calibration. |
| Mirror Light flickers | Keep HC-SR04 away from reflective surfaces; check its 5 V supply. |
| Mirror Light turns on with nobody there | Check `mirrorBackgroundCm` and `hcsr04Distance` in `/api/state`; if the sensor reads a static echo only slightly closer than the background, move it away from reflections or lower `mirrorOnCm`. |
| Mirror Light stays on | A person is within 105 cm, phone manual ON is active, or override is on. |
| Relay works backwards | Change `RELAY_ACTIVE_HIGH`. |
| Buzzer never sounds | NPN pinout, 1 kΩ base resistor, buzzer polarity, `BUZZER_IS_ACTIVE`. |
| Motor does not respond | RF remote wiring and battery; verify optocoupler orientation. |
| ESP-NOW send fails | Same channel, correct **AP** MACs, peer registered. |
| ESP32_A buttons do nothing | Assign `PIN_BTN_START` / `PIN_BTN_STOP`. |
| ESP32_A does not boot | GPIO 15 may be held LOW by the overflow probe; add a 10 kΩ pull-up. |

---

## 14. Changes made in this cleanup

### Latest: calibration + topmost/overflow probe rework

- **Recalibrated the tank:** full = **25 cm**, empty = **84 cm** (was 10 / 100).
  With the tank empty the sensor's ~84 cm reading now shows **0 %** instead of
  the old ~18 %. Bumped the settings version on **both** boards so any stored
  10/100 calibration is discarded on first boot.
- **Reworked the ESP32_A probes:** GPIO 4 is now the **topmost level** probe and
  GPIO 15 stays the **overflow** probe.
- **ESP32_A GPIO 4:** when water reaches the topmost probe, ESP32_B beeps the
  buzzer **5×/sec** and blinks the **RED LED** in sync with the buzzer. No hold —
  it stops as soon as the probe leaves the water.
- **ESP32_A GPIO 15:** overflow lights the **BLUE LED** and keeps it on for an
  extra **2 s** after release.
- Swapped the ESP32_B indicator roles accordingly (blue = overflow, red =
  topmost) and relabelled the dashboard capsule to red `TOP LEVEL` (the blue
  `OVERFLOWING` capsule is unchanged). The ESP-NOW field names `filling`/
  `draining` are kept for protocol compatibility.
- Added a probe-driven buzzer path that runs independently of the 40 s level
  alert; STOP BUZZER silences it until the probe clears.

### Earlier cleanup

- Removed the separate `index.html` (it was a duplicate of the HTML already
  embedded in `ESP32_B.ino`).
- Removed `ADDITIONAL_FEATURES_AND_WIRING.md`; its useful wiring is now in this
  README.
- Removed `WIRELESS_UPLOAD_ESP32_A.md` and `WIRELESS_UPLOAD_ESP32_B.md`; OTA
  setup is now in [Network, MAC addresses, and OTA](#7-network-mac-addresses-and-ota).
- Corrected the README to match the current firmware: 25/84 cm calibration,
  2–200 cm sensor range, 15 s link timeout, 95/92 % buzzer thresholds, 700 ms
  dashboard polling, and 3.5 s command timeout.
- Fixed the ESP32_A ↔ ESP32_B peer address documentation: both boards use their
  **AP** MAC for ESP-NOW, not the station MAC.
- Kept ESP32_A's OTA SSID unchanged as `• TP-Link_8080`.
- Moved ESP32_A settings application out of the ESP-NOW receive callback into
  the main loop, so NVS writes and measurement resets cannot stall the Wi-Fi
  task.
- Restored the jump/stuck settings in ESP32_A so the existing settings fields
  actually work; the jump filter now defaults to an active 15 cm symmetric gate.
- Re-applied the empty deadband in the percentage calculation (default 0 cm).
- **Fixed the level calculation:** ESP32_A now converts distance to percentage
  once with `(100 - d) / 90 × 100`, and ESP32_B/OLED/dashboard display that value
  unchanged. Removed the ambiguous "either sensor orientation" branch.
- **Blocked reversed calibration:** both sketches now swap/reject a pair where
  "full" is farther than "empty", which was the one configuration that made the
  percentage fall while the tank filled.
- **Fixed the jump filter:** the old code cleared the confirm counter on every
  rejected reading, so once a reading exceeded the jump limit it could be
  rejected forever. Confirmed changes are now accepted in both directions.
- **Replaced filtering:** burst median → 15 cm symmetric jump gate → 5-reading
  rolling median → EMA 0.50. Rising and falling levels are treated identically.
- Bumped the stored settings version on both boards so any old 29/76 or
  smoothing-off calibration in NVS is discarded on first boot after flashing.
- Added a boot-time warning and README section for the JSN-SR04T ~20–25 cm
  blind zone, which is the physical reason a full tank can read below 100 %.
- Fixed the OLED footer so distance, link status, and litres no longer draw on
  top of each other.
- Removed hard-coded ESP32_B pin numbers from the admin JSON; it now reports the
  real configured pins.
- **ESP32_B OLED:** the percentage now uses FreeSans Bold (`fub42`/`fub35`/
  `fub25`) with a wider custom `%` sign, and the number width is measured from
  real glyph advances so `100%` still fits.
- **ESP32_B mirror light:** Relay 1 now holds ON for 1 s after the person leaves
  (or after an echo dropout) before switching off; returning within 100 cm
  cancels the turn-off.
- **ESP32_B indicator LEDs:** added RED LED / BLUE LED test toggles at the bottom
  of the Motor control card, backed by `POST /api/led/test`.
- **ESP32_B person detection:** added background-echo rejection — a reading must
  be at least 18 cm closer than the learned empty-room distance — plus a jump
  gate, a minimum valid distance, 3 consecutive near readings and a 600 ms
  confirm floor, so ambient echoes near 1 m and short noise bursts no longer
  turn the mirror light on.
- Verified both sketches compile with `arduino-cli` on ESP32 core 3.3.11 with
  `--warnings all`.

---

## 15. Safety notes

- Mains voltage can kill. Use a proper enclosure, fuse, strain relief, and rated
  terminals.
- Do not touch the relay board or wiring while mains power is connected.
- ESP32 GPIOs are 3.3 V. Never feed 5 V directly into an ECHO or button input.
- Use the 1 kΩ / 2 kΩ divider on **both** ultrasonic sensors.
- Do not power relay coils or buzzers from an ESP32 GPIO.
- Verify the RF remote PCB before connecting it.
- Use a flyback diode across inductive loads.
- Keep the water sensor cable's waterproof joint out of standing water.
- Do not exceed relay contact ratings. Motors have high inrush current.
- The project has no motor feedback. The dashboard shows commands, not physical
  motor state.
