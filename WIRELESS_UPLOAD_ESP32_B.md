# Wireless Firmware Upload for ESP32_B (OTA)

This guide explains how to upload new firmware to **ESP32_B only** wirelessly (ArduinoOTA), without opening the enclosure again after a one-time USB setup.

- **ESP32_A is not changed** and does **not** need any upload for ESP32_B OTA changes.
- ESP32_B keeps all existing functions (WiFi AP, dashboard, ESP-NOW, relays, buzzer, OLED, motor controls, admin, history).
- Wireless upload works when your computer can connect to ESP32_B’s WiFi access point (SSID `Smart Water Tank`).

---

## 1. How it works

- ESP32_B runs as a WiFi Access Point: **SSID `Smart Water Tank`** (see `AP_SSID` in `ESP32_B.ino`), IP **`192.168.4.1`**, channel **6**.
- OTA service is enabled in `ESP32_B.ino`:
  - **Hostname:** `ESP32-WATER-B`
  - **OTA password:** `dks12345` (change in code before deployment if needed)
- To upload wirelessly, your laptop must be connected to **`Smart Water Tank`** and reach **`192.168.4.1`**.
- During upload the dashboard/ESP-NOW may pause briefly; ESP32_B reboots automatically after a successful upload.

> **Important:** You must perform **one last USB upload** to flash the OTA-enabled firmware. After that, future ESP32_B uploads can be done wirelessly.

---

## 2. Prerequisites

- Arduino IDE 2.x (or Arduino CLI) with **ESP32 board support** installed (tested target: `esp32:esp32` 3.3.11).
- OTA-enabled `ESP32_B.ino` (includes `ArduinoOTA.h`, OTA hostname/password, `ArduinoOTA.begin()`, `ArduinoOTA.handle()`).
- USB data cable for the one-time flash.
- Stable power for ESP32_B during every upload (USB or normal supply). **Do not power off mid-upload.**
- Laptop/PC WiFi able to join `Smart Water Tank` (ESP32_B AP).
- OTA-capable partition scheme (default ESP32 schemes usually work, e.g. **Default 4MB with spiffs**).

---

## 3. One-time USB setup (required once)

1. **Open ESP32_B enclosure once** and connect ESP32_B to your computer via USB.
2. Open `ESP32_B.ino` in Arduino IDE.
3. Select board/port:
   - **Tools → Board:** your ESP32 board (e.g. *ESP32 Dev Module*).
   - **Tools → Port:** USB serial port of ESP32_B.
   - **Tools → Partition Scheme:** an OTA-capable scheme (e.g. *Default 4MB with spiffs* / scheme with OTA app slots).
4. (Recommended) Change `OTA_PASSWORD` in `ESP32_B.ino` if desired, then save.
5. Click **Upload** and wait for success.
6. Open **Serial Monitor** at **115200 baud** and press reset if needed. Confirm you see:
   - `AP SSID: Smart Water Tank`
   - `Dashboard: http://192.168.4.1/`
   - `OTA hostname: ESP32-WATER-B`
   - `OTA: ready (wireless upload over 192.168.4.1 when connected to AP)`
7. Disconnect USB and reassemble/place ESP32_B in its normal powered location.

---

## 4. Wireless upload steps (after one-time USB setup)

### A. Using Arduino IDE

1. **Power on ESP32_B** normally and wait ~10–20 seconds for AP/OTA to start.
2. On your laptop, connect WiFi to **`Smart Water Tank`** (password: value of `AP_WIFI_PASSWORD` in `ESP32_B.ino`, default `dks12345`).
3. Confirm connectivity:
   - Open `http://192.168.4.1/` in a browser (dashboard should load), or
   - Ping/check reachability to `192.168.4.1`.
4. Open the updated `ESP32_B.ino` in Arduino IDE.
5. Go to **Tools → Port** and select the **network port** for `ESP32-WATER-B` / `192.168.4.1` (it may take a few seconds to appear after joining the AP).
6. Keep the same board settings and OTA-capable partition scheme used in the USB flash.
7. Click **Upload**.
8. If prompted, enter the **OTA password**: `dks12345` (or your changed password).
9. Wait for upload progress and “Done uploading”. ESP32_B will reboot automatically.
10. Reconnect to `Smart Water Tank` if needed and verify at `http://192.168.4.1/` and/or Serial logs (if accessible).

### B. Using Arduino CLI (example)

1. Power on ESP32_B and connect your computer to **`Smart Water Tank`**.
2. Compile (optional but recommended):
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32 "ESP32_B.ino"
   ```
   *(If compiling from a folder with multiple `.ino` files causes sketch-name conflicts, compile from a B-only sketch folder/copy.)*
3. Upload over network (port/password flags may vary by core/CLI version):
   ```bash
   arduino-cli upload -p 192.168.4.1 --fqbn esp32:esp32:esp32 --upload-field password=dks12345 "ESP32_B.ino"
   ```
   If your CLI/core exposes a network protocol flag, use the appropriate one (e.g. `--protocol network`) and provide the OTA password as required.
4. Wait for completion and automatic reboot, then verify dashboard/behavior.

---

## 5. Verification checklist

- [ ] Laptop connected to `Smart Water Tank` during upload.
- [ ] ESP32_B stayed powered for the entire upload.
- [ ] Upload completed without OTA auth/connect/receive errors.
- [ ] ESP32_B rebooted and dashboard is reachable at `http://192.168.4.1/`.
- [ ] Relays, buzzer, OLED, ESP-NOW link behavior, and controls work as expected.

---

## 6. Troubleshooting

- **Network port not visible in IDE:** Ensure laptop is connected to `Smart Water Tank`, wait 10–30s, restart Arduino IDE, or use IP `192.168.4.1` with CLI. Firewalls/VPNs can block mDNS/OTA discovery.
- **Auth Failed:** OTA password in IDE/CLI must match `OTA_PASSWORD` in the currently running ESP32_B firmware.
- **Connect/Receive/End Failed:** Move laptop closer to ESP32_B, ensure stable power, retry. Avoid heavy dashboard use during upload.
- **Upload starts but device misbehaves after:** Re-upload wirelessly if reachable; otherwise use USB recovery.
- **ESP32_A affected?** No—ESP32_A firmware and wiring are unchanged by ESP32_B OTA.
- **Need USB again:** Required if OTA never flashed, wrong partition scheme, or a bad/incomplete upload makes OTA unreachable. Plan a serviceable access point if possible.

---

## 7. Security notes

- OTA and dashboard are available to devices connected to the local AP `Smart Water Tank`.
- Change `AP_WIFI_PASSWORD` and `OTA_PASSWORD` from defaults before deployment.
- Keep ESP32_B physically secure; anyone on the AP with OTA access/password could upload firmware.
