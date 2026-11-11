# Smart Water Tank

A two-board ESP32 water-tank monitor and controller.

- **ESP32_A** sits on the roof, measures the water level, and sends the reading to ESP32_B.
- **ESP32_B** is indoors. It runs the OLED, relays, buzzer, motor buttons, Wi-Fi access point, and phone dashboard.
- The two boards talk wirelessly with **ESP-NOW**. No router or internet is needed.
- The phone dashboard is served by ESP32_B at **http://192.168.4.1/**.
- The dashboard includes an open **Admin** control centre for calibration, filtering, mirror, buzzer, OLED, history, diagnostics, and reboot controls. No passcode is required on the local access point.

This folder contains these files:

```text
Smart Water Tank/
  ESP32_A.ino
  ESP32_B.ino
  index.html
  README.md
  ADDITIONAL_FEATURES_AND_WIRING.md
```

> The probe, long-press button, buzzer-threshold, and OLED additions are documented in
> [`ADDITIONAL_FEATURES_AND_WIRING.md`](ADDITIONAL_FEATURES_AND_WIRING.md). Where this
> file and that document differ, the additional document and the current firmware are
> authoritative.

---

## 1. What each board does

### ESP32_A - rooftop sensor

- Measures distance to the water with a JSN-SR04T or AJ-SR04M ultrasonic sensor.
- Takes 5 samples and uses the median to reject bad echoes.
- Rejects timeouts, impossible distances, sudden jumps, and a stuck sensor.
- Converts distance to water percentage and litres.
- Sends water data to ESP32_B once per second and immediately on touch-state change.
- Reads FILLING and DRAINING probes on GPIO 4 and GPIO 15.
- Receives status back from ESP32_B.
- Supports two optional START/STOP push buttons for the motor.
- Has no relay, OLED, buzzer, or web server.

### ESP32_B - indoor controller

- Receives water data, touch states, and motor commands from ESP32_A.
- Sends sensor and status information back to ESP32_A.
- Shows water percentage, litres, and connection status on the OLED.
- Drives the blue/red indicators from the received touch states.
- Sounds the full-tank buzzer.
- Switches the Mirror Light from a person sensor and from the phone.
- Supports a 3-second button manual override for the Mirror Light.
- Switches the Bulb from the phone.
- Presses and releases the RF remote START/STOP buttons.
- Hosts the Wi-Fi access point and dashboard.
- Has one physical buzzer-stop / mirror-override button.

### Tank model

- Tank capacity: **750 litres**.
- Water volume is reported as integer litres (`percentage x 7.5`, clamped to 750 L).
- Full distance: **10 cm or less** from the sensor face to the water = **100 %**.
- Empty distance: **76 cm** = **0 %**, calibrated to the installed sensor and empty tank.
- A **2 cm empty deadband** keeps a true empty tank at 0 % despite small echo jitter.
- **94 %** is the requested maximum usable filling/overflow point; the 0-100 scale is not
  remapped to make 94 % equal 100 %.
- Percentage formula:

```text
percent = (76 - distanceCm) / (76 - 10) x 100
```

- Examples: 10 cm or less = 100 %, 43 cm = 50 %, 76 cm or more = 0 %.

---

## 2. Parts

### ESP32_A

| Qty | Part |
|---|---|
| 1 | ESP32 development board |
| 1 | JSN-SR04T or AJ-SR04M waterproof ultrasonic sensor |
| 1 | 5 V power supply with enough current for the ESP32 and sensor |
| 1 | 1 kOhm resistor for the Echo divider |
| 1 | 2 kOhm resistor for the Echo divider |
| 2 | Momentary push buttons for START and STOP, optional |
| 1 | Weatherproof enclosure, cable gland, strain relief |

### ESP32_B

| Qty | Part |
|---|---|
| 1 | ESP32 development board |
| 1 | HC-SR04 ultrasonic sensor |
| 1 | 1 kOhm resistor for the Echo divider |
| 1 | 2 kOhm resistor for the Echo divider |
| 1 | 0.96 inch I2C OLED, SSD1306, address 0x3C or 0x3D |
| 1 | 2-channel 5 V relay module with optocouplers |
| 2 | 2N2222 or BC547 NPN transistors for the blue/red indicators |
| 1 | 2N2222 or BC547 NPN transistor for the buzzer |
| 3 | 1 kOhm base resistors |
| 1 | Blue indicator and 1 red indicator, with suitable series resistors |
| 1 | TMB12A05 5 V active buzzer |
| 1 | 1N4148 diode, optional, across an inductive buzzer |
| 4 | Momentary push buttons: 1 buzzer-stop, 3 reserved |
| 1 | RF remote with separate START and STOP buttons |
| 2 | PC817 optocouplers, or 2 small signal relays, for the RF remote |
| 2 | 330 Ohm resistors for the optocoupler LEDs |
| 1 | 5 V power supply |
| 1 | Mains-rated enclosure, fuse, terminals, and cable clamps for mains loads |

> The main power indicator bulb is wired directly to the main supply. It is not controlled by the ESP32.

---

## 3. ESP32_A wiring

This is the complete ESP32_A connection list.

| Device | Device pin | ESP32_A pin | Notes |
|---|---|---|---|
| Ultrasonic sensor | VCC | 5 V supply | Use the sensor's rated voltage. |
| Ultrasonic sensor | GND | GND | Common ground with the ESP32. |
| Ultrasonic sensor | TRIG | GPIO 5 | 3.3 V trigger. Add a level shifter if your 5 V sensor needs it. |
| Ultrasonic sensor | ECHO | Divider output to GPIO 18 | Never connect a 5 V Echo directly. |
| Echo divider | R1 1 kOhm | Sensor ECHO to GPIO 18 | Series resistor. |
| Echo divider | R2 2 kOhm | GPIO 18 to GND | Lower resistor. |
| START button | One side | GPIO to be assigned | Other side to GND. Internal pull-up. |
| STOP button | One side | GPIO to be assigned | Other side to GND. Internal pull-up. |

### Echo divider

```text
Sensor ECHO ---- R1 1 kOhm ----+---- ESP32_A GPIO 18
                                |
                             R2 2 kOhm
                                |
                               GND
```

This gives about `5 x 2 / (1 + 2) = 3.33 V` at the GPIO.

### ESP32_A buttons

The button GPIO numbers are **not supplied yet**. The firmware uses:

```cpp
#define PIN_BTN_START -1
#define PIN_BTN_STOP  -1
```

`-1` means disabled. The water sensor still works normally.

When the GPIO numbers are confirmed:

1. Replace both `-1` values with safe output-capable GPIOs.
2. Do not use GPIO 34-39: they are input-only and have no internal pull-up.
3. Wire each button from its GPIO to GND.
4. Buttons use `INPUT_PULLUP`, so released = HIGH and pressed = LOW.
5. Recompile and upload ESP32_A.

A button can connect directly from GPIO to GND. The motor is switched by the existing two transistor/RF circuits on ESP32_B.

If you want to use the two supplied 2N2222 transistors as button input buffers, wire each one like this:

```text
3.3 V ---- push button ---- 1 kOhm ---- base
                                          |
                                      NPN 2N2222
                                          |
GPIO (INPUT_PULLUP) ---- collector        | emitter ---- GND
                                          |
                                    10 kOhm to GND
```

- Pressing the button turns the NPN ON and pulls the GPIO LOW.
- The firmware still sees the same active-low signal, so no code change is needed.
- This transistor stage is optional. It does not switch the motor.
- Check the 2N2222 pinout from its datasheet before soldering.

### ESP32_A calibration

The defaults are stored in ESP32_A NVS and can be changed from the dashboard **Admin**
page without recompiling:

```text
Full distance:   10.0 cm  -> 100 %
Empty distance:  76.0 cm  -> 0 %
Empty deadband:   2.0 cm
Sensor range:     2.0 to 150.0 cm
Tank capacity:  750 L
```

Measure from the sensor face to the water surface. Distances of 10 cm or less are clamped
to 100 %; distances within 2 cm of the 76 cm empty calibration are clamped to 0 %. If the
sensor is moved or the tank changes, remeasure both points and save them from the Admin
page. ESP32_B sends the changed settings over ESP-NOW and ESP32_A applies them
automatically.

---

## 4. ESP32_B wiring

This is the complete ESP32_B connection list.

| Device | Device pin | ESP32_B pin | Notes |
|---|---|---|---|
| HC-SR04 | VCC | 5 V | 5 V module. |
| HC-SR04 | GND | GND | Common ground. |
| HC-SR04 | TRIG | GPIO 25 | 3.3 V trigger normally works. |
| HC-SR04 | ECHO | Divider output to GPIO 26 | Never connect 5 V Echo directly. |
| HC-SR04 divider | R1 1 kOhm | ECHO to GPIO 26 | Series resistor. |
| HC-SR04 divider | R2 2 kOhm | GPIO 26 to GND | Lower resistor. |
| OLED | VCC | 3.3 V, check your module | Most 4-pin modules are 3.3 V. |
| OLED | GND | GND | |
| OLED | SDA | GPIO 21 | I2C data. |
| OLED | SCL | GPIO 22 | I2C clock. |
| Relay module | VCC | 5 V or 3.3 V as required | Check the module. |
| Relay module | GND | GND | |
| Relay 1 input | IN1 | GPIO 32 | Mirror Light. |
| Relay 2 input | IN2 | GPIO 33 | Bulb. |
| Buzzer driver | 1 kOhm to NPN base | GPIO 13 | NPN drives the buzzer. |
| Blue indicator driver | 1 kOhm to NPN base | GPIO 16 | ON while filling. |
| Red indicator driver | 1 kOhm to NPN base | GPIO 17 | ON while draining. |
| Buzzer-stop button | One side | GPIO 4 | Other side to GND. |
| RF remote START | Optocoupler/relay input | GPIO 18 | Across the remote START pads. |
| RF remote STOP | Optocoupler/relay input | GPIO 19 | Across the remote STOP pads. |
| Main power bulb | - | No GPIO | Wired directly to the main supply. |
| Reserved button | - | GPIO 2, 14, 23 | Not used by firmware. |

### HC-SR04 Echo divider

```text
HC-SR04 ECHO ---- R1 1 kOhm ----+---- ESP32_B GPIO 26
                                |
                             R2 2 kOhm
                                |
                               GND
```

### Relay polarity

The firmware assumes the relay module is **active LOW**:

```cpp
#define RELAY_ACTIVE_HIGH false
```

If your relay turns ON when its input is HIGH, change it to `true`.

At boot both relays are forced OFF before they become outputs.

GPIO pins can float briefly during reset. Fit external resistors so the relay inputs stay inactive:

- Active-LOW module: 10 kOhm from each IN pin to **3.3 V**.
- Active-HIGH module: 10 kOhm from each IN pin to **GND**.

Do not rely on the ESP32 alone for safe relay state at power-up.

### OLED

The firmware tries address `0x3C` first and then `0x3D`.

The default OLED shows:

- A large water percentage with a smaller `%` sign. U8g2 Logisoso fonts are rendered at their native size with 1 px curves, so the digits do not show scaled 8×8 blocks.
- Bottom-left link status: `CONNECTED` or `NO LINK` (font size 1).
- Bottom-right integer litres (font size 1).

The Admin page can show/hide percent, link status, bottom litres, live/stale status,
and distance. It can also change the percentage scale and bottom font size, contrast,
and inversion. The live preview approximates the 128×64 OLED.

---

## 5. Transistor and RF wiring

### Blue/red indicator NPN driver

```text
5 V or 12 V positive
        |
   indicator load and series resistor
        |
     collector
        |
GPIO 16 or 17 ---- 1 kOhm ---- base   NPN 2N2222/BC547
        |
     emitter
        |
       GND
```

- GPIO 16 drives the blue indicator.
- GPIO 17 drives the red indicator.
- The ESP32 GPIO drives only the base resistor, never the load.
- Check the transistor pinout from its datasheet before soldering.
- Small 2N2222/BC547 transistors are only for small loads. Use a MOSFET or relay for larger loads.
- Add a flyback diode across an inductive load.

### Buzzer NPN driver

```text
5 V positive ---- TMB12A05 positive
TMB12A05 negative ---- collector
                         |
GPIO 13 ---- 1 kOhm ---- base   NPN 2N2222/BC547
                         |
                      emitter
                         |
                        GND
```

- The firmware assumes an active buzzer.
- Buzzer output is active HIGH at the GPIO.
- Add a 1N4148 diode across the buzzer if needed.

### RF remote START/STOP

Do not connect the ESP32 directly to unknown remote button pads.

Use two PC817 optocouplers or two small signal relays. Each one must connect across the two pads of **one** remote button only.

```text
ESP32 GPIO 18 or 19 ---- 330 Ohm ---- PC817 pin 1, anode
                                      PC817 pin 2, cathode ---- GND

Remote button pad A ---- PC817 pin 4, collector
Remote button pad B ---- PC817 pin 3, emitter
```

- GPIO 18 is START.
- GPIO 19 is STOP.
- Verify the remote PCB with a multimeter before wiring.
- If the remote uses a matrix circuit, connect each optocoupler across the exact two pads that its physical button shorts.
- The firmware releases both RF outputs at boot.
- A press is released automatically after 4 seconds.
- START and STOP cannot be active at the same time.
- There is no motor feedback, so the dashboard only shows the command state, not proof that the motor is running.

---

## 6. How the system behaves

### Water reading

1. ESP32_A triggers the sensor 5 times (configurable in Admin).
2. Invalid echoes are discarded.
3. The median of the valid echoes is used.
4. Readings outside 2 to 150 cm are rejected by default; 10 cm or less reports 100 %, and 74 cm or more reports 0 %.
5. A sudden jump over 30 cm must repeat 3 times before it is accepted.
6. A bit-identical reading for 10 minutes is treated as a stuck sensor.
7. The accepted distance is converted to percentage and sent to ESP32_B.

All thresholds above are editable in Admin. Use a stable calibration first, then adjust
filtering only if the installation is noisy.

### Link status

- ESP32_B has an 8-second ESP-NOW link timeout and a separate valid-water timeout.
- Touch states remain usable while the ESP-NOW link is online even if the water reading is invalid.
- If the link is lost, the buzzer is stopped and both touch indicators turn off.
- The OLED shows `CONNECTED` while ESP-NOW packets are arriving and `NO LINK` after the link is lost.
- ESP32_B accepts both current protocol v2 packets and legacy v1 packets from an older ESP32_A, so a B-only update keeps the roof sensor online.

### Mirror Light

- The HC-SR04 on ESP32_B looks for a person.
- Person closer than 100 cm for about 0.4 seconds: Mirror Light ON.
- Person farther than 105 cm: Mirror Light OFF immediately.
- The gap between 100 and 105 cm rejects brief false triggers.
- If the person sensor reading becomes invalid, the light turns OFF immediately.
- ON/OFF distances, the ON confirmation time, and an optional maximum auto-on time are
  editable in Admin. Auto detection can also be disabled.
- The phone can switch the Mirror Light ON or OFF at any time.
- Holding the GPIO 4 button for 3 seconds toggles manual override ON/OFF.
- Phone **OFF** immediately exits manual override and keeps the relay off until the
  detection area is clear.
- There is **no 30-second timer**.

### Bulb

- The phone switches Relay 2 ON or OFF.
- It stays in that state until changed.

### Full-tank buzzer

- At 97 % or higher for 2 readings, the buzzer starts.
- It beeps 5 times per second for up to 40 seconds.
- A short GPIO 4 button press stops it.
- A 3-second GPIO 4 long press also acknowledges an active buzzer while toggling mirror override.
- The phone STOP BUZZER button also stops it.
- The phone TEST BUZZER (5s) button plays the same beep pattern for 5 seconds, then stops automatically. STOP BUZZER also ends a test immediately.
- It re-arms only after the water level falls below 93 %.
- The buzzer also stops if the water link is lost.
- Start threshold, re-arm threshold, confirmation count, duration, beeps per second,
  beep ON time, and intensity are editable in Admin. Intensity uses PWM and is
  approximate on an active buzzer; some modules only click instead of changing volume.

### Motor control

- The phone START/STOP buttons press the RF remote while held.
- The optional ESP32_A buttons send START/STOP commands over ESP-NOW.
- ESP32_B receives the command and drives the RF remote.
- Any press is released automatically after 4 seconds.
- Pressing START releases STOP first, and pressing STOP releases START first.

### Two-way ESP-NOW

ESP32_A sends:

- WaterPacket: distance, percentage, valid flag, sequence number, uptime, fault flags,
  echo diagnostics, reset reason, and the settings revision currently applied.
- CommandPacket: START/STOP or reboot, pressed/released, sequence number, uptime.

ESP32_B sends:

- StatusPacket: HC-SR04 distance, person state, water percentage/distance, relay states, buzzer state, motor states, link state, uptime, and the acknowledged sequence number.
- SettingsPacket: calibration, filtering, measurement timing, and ESP32_A settings revision.
- Status is also sent once per second for diagnostics.

ESP32_A prints the received B status to the serial monitor. ESP32_A saves a valid
SettingsPacket to NVS, resets its measurement state, and reports the new revision in its
next WaterPacket. ESP32_B keeps retrying until the revisions match.

Current protocol version is **2**. ESP32_B also accepts legacy v1 WaterPacket and CommandPacket frames from an older ESP32_A, so a B-only update is supported. Runtime settings sync to ESP32_A requires v2 on both boards.

---

## 7. Firmware logic, function by function

### `ESP32_A.ino`

| Function | What it does |
|---|---|
| `crc16()` | Calculates CRC16-CCITT for packet checking. |
| `echoISR()` | Records the Echo pulse rise and fall times. |
| `medianOf()` | Sorts samples and returns the median. |
| `percentFromDistance()` | Converts centimetres to percentage with the 10/76 cm calibration and 2 cm empty deadband. |
| `startSample()` | Sends one 10 microsecond trigger pulse. |
| `finalizeBurst()` | Validates samples, filters jumps/stuck readings, builds a WaterPacket, and sends it. |
| `serviceMeasurement()` | Runs the non-blocking sample state machine every second. |
| `serviceSendRetry()` | Resends a water packet until the send succeeds or the B status acknowledges it. |
| `sendCommandBurstNow()` | Sends one copy of a command packet. |
| `queueCommand()` | Builds a START/STOP CommandPacket and starts a 3-copy burst. |
| `serviceCommandBurst()` | Sends the remaining command copies without blocking. |
| `setupButtons()` | Configures assigned START/STOP GPIOs as inputs with pull-ups. |
| `serviceButtons()` | Debounces both buttons and queues press/release commands. |
| `onDataSent()` | Records the result of a water-packet send. |
| `onDataRecv()` | Validates a StatusPacket from ESP32_B and saves it. |
| `setupEspNow()` | Starts ESP-NOW, sets the channel, and adds ESP32_B as a peer. |
| `setup()` | Starts serial, sensor pins, buttons, ESP-NOW, and prints diagnostics. |
| `loop()` | Runs measurement, retry, command, button, and status-report tasks. |

### `ESP32_B.ino`

| Function | What it does |
|---|---|
| `crc16()` | Same CRC16-CCITT as ESP32_A. |
| `writeRelayPin()` | Applies the relay active-high/active-low setting. |
| `writeRfPin()` | Applies the RF remote output polarity. |
| `writeIndicatorPin()` | Applies the indicator output polarity. |
| `setBuzzerOutput()` | Turns the active buzzer on/off, or generates a tone for a passive buzzer. |
| `initSafeOutputs()` | Forces relays, RF outputs, indicators, and buzzer OFF at boot. |
| `computeLinkOnline()` | Checks for any valid ESP-NOW packet newer than 8 seconds. |
| `computeOnline()` | Checks for a valid water packet newer than 8 seconds. |
| `buzzerStateName()` | Returns TESTING, ALERTING, SILENCED, or READY. |
| `stopBuzzerForLinkLoss()` | Stops the buzzer if the water link is lost. |
| `manualStopBuzzer()` | Stops an alert or test from a button or the dashboard. |
| `startBuzzerAlert()` | Starts a 40-second full-tank alert. |
| `startBuzzerTest()` | Starts the same beep pattern as a 5-second test. |
| `updateBuzzerFromWater()` | Starts and re-arms the full-tank buzzer at 97 % / 93 %. |
| `serviceBuzzer()` | Produces the configured beep pattern for alerts and tests without blocking. |
| `applyRelay1()` | Chooses manual override or automatic Mirror Light state and writes Relay 1. |
| `applyRelay2()` | Writes Relay 2. |
| `toggleMirrorOverride()` | Toggles the 3-second button Mirror Light override. |
| `setMirrorRequest()` | Applies a phone ON/OFF request and exits override on OFF. |
| `hcEchoISR()` | Records the HC-SR04 Echo pulse. |
| `medianOfValidHc()` | Returns the median of valid HC-SR04 samples. |
| `pushHcSample()` | Adds an HC-SR04 sample to the filter ring. |
| `startHcSample()` | Sends one HC-SR04 trigger pulse. |
| `serviceHcSensor()` | Runs the non-blocking person-sensor sampling. |
| `updateAutoRelay1()` | Confirms person ON, turns OFF immediately on clear, respects manual override, and blocks auto until clear after phone OFF. |
| `releaseMotorOutput()` | Releases one RF output. |
| `pressMotor()` | Presses one RF output and releases the other. |
| `serviceMotorSafety()` | Releases a press after 4 seconds and prevents both outputs being held. |
| `motorRemainingMs()` | Returns the remaining auto-release time for the dashboard. |
| `updateIndicators()` | Drives the blue/red indicators from the received touch states. |
| `serviceLink()` | Detects link/water online changes and updates indicators. |
| `serviceBuzzerButton()` | Debounces short press and detects the 3-second long press. |
| `initOled()` | Starts I2C, finds the OLED, sets maximum brightness, and clears it. |
| `drawOled()` | Draws the large percentage, link status, and bottom litres. |
| `buildStateJson()` | Builds the `/api/state` JSON response. |
| `sendJsonState()` | Sends the current state to the dashboard. |
| `sendApiError()` | Sends a JSON error response. |
| `parseOnValue()` | Parses `{"on":true}` or `{"on":false}`. |
| `parseBoolField()` | Parses a named boolean JSON field. |
| `parseStringField()` | Parses a named string JSON field. |
| `handleRoot()` | Serves the embedded dashboard. |
| `handleState()` | Handles `GET /api/state`. |
| `handleMirrorLight()` | Handles manual Mirror Light commands. |
| `handleBulb()` | Handles Bulb commands. |
| `handleBuzzerStop()` | Handles buzzer-stop commands. |
| `handleBuzzerTest()` | Handles the 5-second buzzer-test command. |
| `handleMotor()` | Handles phone START/STOP press and release. |
| `onDataRecv()` | Copies incoming ESP-NOW data into a queue. |
| `sendStatus()` | Builds and sends a StatusPacket to ESP32_A. |
| `handleCommandPacket()` | Applies a START/STOP command from ESP32_A. |
| `handleWaterPacket()` | Validates and applies a WaterPacket, then sends status. |
| `serviceEspNowQueue()` | Processes queued ESP-NOW packets safely in the main loop. |
| `setupEspNow()` | Starts ESP-NOW and adds ESP32_A as a peer. |
| `serviceStatusTx()` | Sends status to ESP32_A once per second. |
| `setup()` | Starts safe outputs, sensors, OLED, access point, ESP-NOW, web server, and diagnostics. |
| `loop()` | Runs all non-blocking services and the web server. |

### Dashboard logic

| Feature | Behaviour |
|---|---|
| Polling | Reads `/api/state` every 500 ms. |
| Timeout | Gives up after 2.5 seconds and backs off to 5 seconds. |
| Water display | Shows percentage, integer litres, bar, distance, and FILLING/OVERFLOWING/STABLE touch capsules. |
| Stale data | Keeps the last water percentage and draws a thin underline while link/data is stale. |
| Mirror Light | Toggle follows the actual Relay 1 state and shows person/manual mode; auto-off is immediate. |
| Bulb | Toggle follows Relay 2. |
| Motor | Press and hold sends pressed; release sends released. |
| Safety | Releases held motor buttons when the page loses focus or closes. |
| Buzzer | TEST BUZZER plays the alert pattern for 5 seconds; STOP BUZZER ends an alert or test. |
| Admin | Top-right ADMIN button opens the Admin page directly; no passcode. |

### Admin control centre

The Admin page is part of the same `index.html` and is served at `http://192.168.4.1/`.
It is a full-screen overlay; the original dashboard controls remain unchanged behind it.

- **Access:** no passcode. Any phone on the local ESP32_B access point can open Admin.
- **Live diagnostics:** ESP32_A online/link state, uptime, reset reason, valid/timeout
  echo counts, flags, settings-sync state, current level, ESP-NOW link.
- **History graph:** stored water samples with rising/filling and falling/draining
  colours, range buttons, and automatic old-sample trimming.
- **Noise/event log:** no-echo, timeout, jump, stuck, out-of-range, link loss/restore,
  mirror dropouts, reboot, and settings events.
- **Water settings:** full/empty/deadband, capacity, sensor limits, smoothing, jump
  limit/confirmations, stuck timeout, measure period, sample spacing, samples/cycle.
- **Mirror settings:** enable, ON/OFF distances, ON confirm time, max auto-on. OFF is immediate.
- **Buzzer settings:** enable, start/re-arm thresholds, confirmations, duration, beeps
  per second, beep ON time, intensity.
- **OLED settings:** show/hide fields, percentage scale, bottom font size, contrast, inversion, live 128×64 preview.
- **History settings:** enable, sample interval, retention days, clear history/events.
- **Maintenance:** reboot ESP32_A, reboot ESP32_B, clear history, clear events.
- **Technical information:** MACs, IP, channel, uptime, reset reasons, free heap, flash,
  LittleFS usage, stored line counts, and pin assignments.

Settings are saved to NVS on ESP32_B. Water settings are forwarded to ESP32_A over
ESP-NOW and also persisted in ESP32_A NVS, so they survive reboots. History and events
are stored in LittleFS on ESP32_B. There is no RTC, so history ages are approximate and
are based on the configured sampling interval.

---

## 8. Wi-Fi and dashboard

ESP32_B creates this access point:

```text
SSID:     Smart Water Tank
Password: dks12345
Address:  http://192.168.4.1/
Channel:  6
```

Change these in `ESP32_B.ino` before deployment:

```cpp
static const char AP_WIFI_PASSWORD[] = "dks12345";
#define WIFI_CHANNEL 6
```

The password must be at least 8 characters.

The ESP-NOW channel on both boards must be the same.

---

## 9. MAC addresses

ESP-NOW needs the other board's MAC address. Only these two values are used:

| Value needed | Comes from | Goes into |
|---|---|---|
| ESP32_A **STA MAC** | ESP32_A Serial Monitor | `ESP32_A_MAC` in `ESP32_B.ino` |
| ESP32_B **AP MAC** | ESP32_B Serial Monitor | `ESP32_B_MAC` in `ESP32_A.ino` |

ESP32_A does not create an access point, so an A AP MAC is not used. ESP32_B uses its AP interface for ESP-NOW, so a B STA MAC is not used.

### Find ESP32_A station MAC

Upload ESP32_A and open Serial Monitor at 115200 baud. It prints:

```text
ESP32_A STA MAC: 00:70:07:A1:DC:80
```

Put that value in `ESP32_B.ino`:

```cpp
static const uint8_t ESP32_A_MAC[6] = {0x00, 0x70, 0x07, 0xA1, 0xDC, 0x80};
```

### Find ESP32_B access-point MAC

Upload ESP32_B and open Serial Monitor. It prints:

```text
ESP32_B AP MAC  (use in ESP32_A): 30:76:F5:E4:FD:D5
```

Put that value in `ESP32_A.ino`:

```cpp
static const uint8_t ESP32_B_MAC[6] = {0x30, 0x76, 0xF5, 0xE4, 0xFD, 0xD5};
```

These are the addresses already configured in this project. Verify them against the Serial Monitor after flashing.

---

## 10. Compile and upload

### Arduino IDE

1. Install the ESP32 board package.
2. Select board **ESP32 Dev Module**.
3. Install **U8g2** (used for the SSD1306 OLED).
4. Open `ESP32_A.ino` and upload it to the rooftop board.
5. Open `ESP32_B.ino` and upload it to the indoor board.
6. Set both boards to the same ESP-NOW channel.
7. Enter both MAC addresses.
8. Upload both boards again.

### Important sketch-folder rule

Arduino expects the folder name to match the `.ino` name.

If Arduino refuses to open the file, make a temporary folder for each board:

```text
ESP32_A/ESP32_A.ino
ESP32_B/ESP32_B.ino
```

This project folder can still hold both files for delivery, but compile them from matching temporary folders.

### Command-line compile used for this project

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all ESP32_A.ino
arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all ESP32_B.ino
```

If the sketches are not inside matching folders, copy each one into its own folder first.

---

## 11. Dashboard file

`index.html` is the readable dashboard source.

The same HTML is embedded inside `ESP32_B.ino` between:

```text
R"WTDASH(
...
)WTDASH"
```

If you edit `index.html`, copy the new HTML into that block in `ESP32_B.ino`. Keep the two copies identical or the browser and serial monitor will serve different versions.

Do not move the dashboard to an external file or CDN. ESP32_B must serve everything locally.

---

## 12. HTTP API

| Method | Path | Body | Purpose |
|---|---|---|---|
| GET | `/api/state` | none | Returns current water, relay, buzzer, motor, and sensor state. |
| POST | `/api/mirror-light` | `{"on":true}` | Manual Mirror Light ON/OFF. |
| POST | `/api/bulb` | `{"on":true}` | Bulb ON/OFF. |
| POST | `/api/buzzer/test` | `{}` | Play the alert beep pattern for 5 seconds. |
| POST | `/api/buzzer/stop` | `{}` | Stop an alert or test immediately. |
| POST | `/api/motor` | `{"button":"start","pressed":true}` | Press or release the RF motor button. |

The dashboard polls state every 500 ms. Command requests time out after 3 seconds.

---

## 13. Test checklist

1. Power ESP32_B with no mains load connected. Confirm relays are OFF at boot.
2. Connect to the `Smart Water Tank` Wi-Fi and open `http://192.168.4.1/`.
3. Confirm the OLED shows `NO LINK` before ESP32_A starts.
4. Power ESP32_A and confirm the OLED changes to `CONNECTED`.
5. Move the water sensor and watch the percentage change.
6. Short ESP32_A GPIO 4 / GPIO 15 to GND in turn and confirm the blue/red LEDs and dashboard capsules change immediately.
7. Hold a hand within 100 cm of the HC-SR04. Mirror Light should turn ON.
8. Remove your hand beyond 105 cm. Mirror Light should turn OFF immediately.
9. Use the phone Mirror Light toggle while no person is detected. It should stay ON or OFF with no timer.
10. Test the Bulb toggle.
11. Press TEST BUZZER. Confirm it uses the same pattern as a full-tank alert and stops automatically after 5 seconds. Press TEST BUZZER again and stop it early with STOP BUZZER and with the physical button.
12. Test the buzzer at full water, then press the physical button and the dashboard button.
13. Test phone START/STOP. Each press should release after 4 seconds.
14. Assign ESP32_A button GPIOs and test START/STOP from the rooftop board.
15. Check that the dashboard updates after a page reload.

---

## 14. Safety notes

- Mains voltage can kill. Use a proper enclosure, fuse, strain relief, and rated terminals.
- Do not touch the relay board or wiring while mains power is connected.
- The ESP32 GPIOs are 3.3 V. Never feed 5 V directly into an Echo or button input.
- Use the Echo divider on both ultrasonic sensors.
- Do not power relay coils from an ESP32 GPIO.
- Verify the RF remote PCB before connecting it.
- Use a flyback diode across inductive loads.
- Keep the water sensor cable's waterproof joint out of standing water.
- Do not exceed relay contact ratings. Motors have high inrush current.
- The project has no motor feedback. The dashboard shows commands, not physical motor state.

---

## 15. Configuration summary

### ESP32_A

```cpp
static const uint8_t ESP32_B_MAC[6] = {0x30, 0x76, 0xF5, 0xE4, 0xFD, 0xD5};
#define WIFI_CHANNEL 6
#define PIN_TRIG 5
#define PIN_ECHO 18
#define PIN_BTN_START -1
#define PIN_BTN_STOP  -1
static const float FULL_DISTANCE_CM  = 10.0f;
static const float EMPTY_DISTANCE_CM = 76.0f;
static const float EMPTY_DEADBAND_CM = 2.0f;
static const float SENSOR_MIN_CM     = 2.0f;
```

### ESP32_B

```cpp
static const uint8_t ESP32_A_MAC[6] = {0x00, 0x70, 0x07, 0xA1, 0xDC, 0x80};
#define WIFI_CHANNEL 6
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_HCSR_TRIG 25
#define PIN_HCSR_ECHO 26
#define PIN_RELAY1 32
#define PIN_RELAY2 33
#define PIN_BUZZER 13
#define PIN_BTN_BUZZER_STOP 4
#define PIN_IND_BLUE 16
#define PIN_IND_RED 17
#define PIN_RF_START 18
#define PIN_RF_STOP 19
#define RELAY_ACTIVE_HIGH false
```

---

## 16. Current compile results

Checked with Arduino-ESP32 core 3.3.11:

| Sketch | Flash | RAM |
|---|---:|---:|
| ESP32_A | 899,236 bytes (68 %) | 46,304 bytes (14 %) |
| ESP32_B | 1,169,115 bytes (89 %) | 51,068 bytes (15 %) |

Both sketches compile with `--warnings all` without warnings.

---

## 17. Troubleshooting

| Problem | Check |
|---|---|
| OLED blank | Check 3.3 V, GND, SDA 21, SCL 22, and address 0x3C/0x3D. |
| Dashboard does not open | Join the `Smart Water Tank` Wi-Fi and use `http://192.168.4.1/`. |
| Water shows no data | Check both MAC addresses, both channels, ESP32_A power, and sensor wiring. |
| Distance is always invalid | Check the Echo divider, sensor supply, and calibration values. |
| Mirror Light flickers | Keep the HC-SR04 away from reflective surfaces and check its 5 V supply. |
| Mirror Light stays on | Check for a person within 105 cm, or a phone manual ON command. |
| Relay works backwards | Change `RELAY_ACTIVE_HIGH`. |
| Buzzer never sounds | Check transistor pinout, base resistor, buzzer polarity, and `BUZZER_IS_ACTIVE`. |
| Motor does not respond | Verify the RF remote wiring and that the remote battery is installed. |
| ESP-NOW send fails | Confirm both boards use the same channel and the correct peer MAC. |
| ESP32_A buttons do nothing | Set `PIN_BTN_START` and `PIN_BTN_STOP` to real GPIO numbers. |
