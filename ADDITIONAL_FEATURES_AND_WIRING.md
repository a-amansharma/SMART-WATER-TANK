# Additional Features and Wiring

This document covers only the features added to the existing Smart Water Tank project:
FILLING/DRAINING probes, the short/long ESP32_B button behavior, the buzzer driver, and
verification of the untouched wiring. The original wiring document,
[`README.md`](README.md), remains the main reference.

> The firmware in this folder now implements these additions. Do not connect 5 V logic
> directly to any ESP32 GPIO. Power off both boards before changing wiring.

---

## 1. ESP32_A FILLING and DRAINING probes

### Assigned pins

| Function | ESP32_A GPIO | Firmware behavior |
| --- | ---: | --- |
| FILLING probe | **4** | Active LOW; blue FILLING indicator and blue dashboard capsule |
| DRAINING probe | **15** | Active LOW; red indicator and red dashboard capsule |

The firmware configures both pins as `INPUT_PULLUP`, debounces them for 50 ms, and sends
the current state to ESP32_B immediately when it changes. The normal one-second water
packet also carries both states.

### Contact / tilt-switch wiring

Use a dry-contact or tilt switch, or a pair of water-contact electrodes. The simplest
and recommended arrangement is a closure to ESP32_A GND:

```text
ESP32_A GPIO 4  ---- FILLING probe / tilt switch ---- ESP32_A GND
ESP32_A GPIO 15 ---- DRAINING probe / tilt switch --- ESP32_A GND
```

- Open circuit = inactive (internal pull-up holds the GPIO HIGH).
- Closed circuit to GND = active (GPIO reads LOW).
- If your contact instead switches to 3.3 V, change `TOUCH_ACTIVE_LOW` in
  `ESP32_A.ino` to `false`; do **not** switch it to 5 V.
- The probe wiring and ESP32_A must share a common GND reference.
- For longer probe leads or a noisy tank, add a **10 kOhm external pull-up from the GPIO
  to 3.3 V** and a **100 nF capacitor from the GPIO to GND** close to the ESP32. These are
  optional filters, not substitutes for the internal pull-up.

### Important GPIO 15 warning

GPIO 15 is an ESP32 boot-strapping pin. It must be HIGH while the board resets or the
board may enter the wrong boot mode.

- Keep the DRAINING probe open, or use the external 10 kOhm pull-up to 3.3 V, during
  reset/power-up.
- Do not hold GPIO 15 LOW while resetting or flashing ESP32_A.
- If the tank wiring can pull GPIO 15 LOW at power-up, add the external 10 kOhm pull-up.

### Probe material and safety

- Use stainless-steel or otherwise corrosion-resistant electrodes in water.
- Keep the probe current very low; the internal pull-up is sufficient for detection.
- Keep water-probe wiring physically separated from mains wiring.
- Do not use the tank water or pipe as an electrical return unless the installation is
  isolated, dry, and safe. A dedicated low-voltage probe return to ESP32_A GND is safer.
- ESP32_B's GPIO **4** is its existing buzzer-stop button. It is a different board and a
  different function from ESP32_A's GPIO 4 FILLING probe. Do not cross-wire them.

---

## 2. ESP32_B button: short press and long press

The existing button remains on **ESP32_B GPIO 4**, wired from GPIO 4 to GND. It uses the
internal pull-up and the firmware's 50 ms debounce.

| Action | Result |
| --- | --- |
| Short press (released before 3 s) | Stops/silences the full-tank buzzer, if active |
| Hold for at least 3 s | Toggles the Mirror Light manual override |
| Release after a long press | No additional short-press action |

Long-press details:

- First long press: Mirror Light manual override turns ON and stays ON; automatic person
  detection is ignored while the override is active.
- Next long press: manual override turns OFF and normal automatic person detection
  resumes.
- If the buzzer is active when the 3 s long press is recognized, the buzzer is also
  silenced/acknowledged. The release does not trigger a second stop command.
- A phone **OFF** command for the Mirror Light immediately exits manual override. The
  firmware also suppresses automatic turn-on until the person-detection area is clear, so
  the phone OFF command is not immediately overridden.

No new button or GPIO is required for this feature.

---

## 3. ESP32_B buzzer driver

The supplied build list identifies **TMB12A05 5 V active buzzer** and allows either a
**2N2222 or BC547** small NPN transistor. It does not uniquely identify which transistor
is fitted, so verify the actual part before soldering.

### Wiring

```text
+5 V ---- TMB12A05 positive
TMB12A05 negative ---- NPN collector
                                   |
ESP32_B GPIO 13 ---- 1 kOhm ---- NPN base
                                   |
                               NPN emitter
                                   |
                                  GND
```

- ESP32_B GND, ESP32_A GND, the 5 V supply GND, and the transistor emitter must share a
  common ground as required by the existing supply arrangement.
- GPIO 13 drives only the 1 kOhm base resistor; it never drives the buzzer directly.
- The firmware assumes an active buzzer and an active-HIGH GPIO output.
- Add a **1N4148** diode across the buzzer if the buzzer is inductive or if switching
  noise is present. Connect the diode cathode to the buzzer positive/5 V side and the
  anode to the buzzer negative/collector side.
- Do not power the buzzer from a GPIO.

### Transistor pinout

TO-92 packages, flat side facing you, leads pointing down:

| Transistor | Left | Middle | Right |
| --- | --- | --- | --- |
| 2N2222 / 2N2222A (common TO-92) | Emitter | Base | Collector |
| BC547 | Collector | Base | Emitter |

- The metal-can 2N2222 package has a different pinout; check its datasheet.
- Always confirm the actual part and package with a multimeter or datasheet.
- The buzzer sounds at 3 beeps per second for up to **40 seconds** when the water level
  reaches **97%**. It re-arms only after the level falls below **93%**, or after a short
  press acknowledges it and the level falls below 93%.

Threshold note: the requested maximum usable/overflow level is **94%**, while the
requested buzzer threshold is **97%**. The firmware implements both values exactly as
requested. Because a level capped at 94% may never reach 97%, confirm whether the buzzer
threshold should instead be below 94% in the final installation.

---

## 4. Probe-driven LED and dashboard behavior

The old automatic trend-based LED logic has been removed.

| Input state | ESP32_B indicator | Dashboard capsule |
| --- | --- | --- |
| FILLING probe active | Blue LED ON | Blue glowing `FILLING` |
| DRAINING probe active | Red LED ON | Red glowing `OVERFLOWING` |
| Neither probe active | Both LEDs OFF | `STABLE` capsule |
| ESP-NOW link lost | Both LEDs OFF | `NO SIGNAL` capsule |

The dashboard capsule named `OVERFLOWING` is driven by the probe the firmware calls
`DRAINING` (ESP32_A GPIO 15), matching the requested dashboard label. The LEDs are
controlled only by the received probe states, not by buzzer acknowledgement or water
trend.

---

## 5. Existing wiring verification

Do not change these connections as part of this addition. Verify them against the main
README and your board pinout:

| Board | Connection | Pin |
| --- | --- | --- |
| ESP32_A | JSN-SR04T TRIG | GPIO 5 |
| ESP32_A | JSN-SR04T ECHO through divider | GPIO 18 |
| ESP32_A | FILLING probe | GPIO 4 |
| ESP32_A | DRAINING probe | GPIO 15 |
| ESP32_B | OLED SDA / SCL | GPIO 21 / 22 |
| ESP32_B | HC-SR04 TRIG / ECHO through divider | GPIO 25 / 26 |
| ESP32_B | Relay 1 (Mirror Light) | GPIO 32 |
| ESP32_B | Relay 2 (Bulb) | GPIO 33 |
| ESP32_B | Buzzer NPN base resistor | GPIO 13 |
| ESP32_B | Buzzer-stop / long-press button | GPIO 4 to GND |
| ESP32_B | Blue / red indicators | GPIO 16 / 17 |
| ESP32_B | RF START / STOP | GPIO 18 / 19 |

- Both Echo inputs are 5 V signals and must use the existing resistor dividers.
- Relay 1 and Relay 2 use the existing active-LOW assumption.
- The ESP32_A calibration is full **10 cm**, empty **76 cm**, with a **2 cm empty
  deadband**; an empty tank should read 0 %. Re-measure both points if the sensor moves.
- The ESP32_B Wi-Fi AP and ESP-NOW channel remain channel **6**.
- Do not assign ESP32_A's optional START/STOP buttons to GPIO 4 or 15.
- Keep the two boards' grounds separate except where the existing power arrangement
  already requires a common reference.

---

## 6. Verification checklist

1. Disconnect power before adding the two probes or changing the buzzer transistor.
2. Check GPIO 15 is HIGH at ESP32_A reset; add the 10 kOhm pull-up if it is not.
3. Power ESP32_A and confirm the serial log prints `FILLING GPIO 4` and
   `DRAINING GPIO 15`.
4. Short each probe to GND in turn and confirm the matching LED and dashboard capsule
   change immediately.
5. Confirm the opposite LED does not turn on.
6. Confirm the dashboard shows `STABLE` only when neither probe is active and the
   ESP-NOW link is online.
7. Short-press the ESP32_B button with the buzzer active; confirm it goes quiet.
8. Hold the ESP32_B button for 3 s; confirm the Mirror Light manual override toggles
   once, and releasing does not send a second action.
9. Send Mirror Light OFF from the phone while override is active; confirm override exits
   and the relay stays off until the person-detection area becomes clear.
10. Verify the buzzer sounds at 97% and stops at 40 s or after acknowledgement, and
    re-arms only below 93%.
11. Confirm the OLED shows a large percentage, litres below it, and
    `CONNECTED`/`NOT CONNECTED` plus litres on the bottom row.
12. Confirm the dashboard and embedded HTML are still byte-identical after any edits.
