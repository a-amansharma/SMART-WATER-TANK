# Wireless Firmware Upload for ESP32_A (OTA)

This guide explains how to upload new firmware to **ESP32_A only** wirelessly (ArduinoOTA), without opening the rooftop/enclosure again after a one-time USB setup.

- **ESP32_B is unchanged by this process** except it must remain powered/on channel 6 so ESP-NOW communication works normally.
- ESP32_A now hosts its **own WiFi AP for OTA uploads**: SSID **`• TP-Link_8080`** (bullet character), password **`dks12345`**, IP **`192.168.10.1`**, channel **6** (same as ESP32_B/ESP-NOW).
- OTA credentials: **Hostname `ESP32-WATER-A`**, **OTA password `dks12345`** (same password style as ESP32_B).

> **Important:** You must perform **one last USB upload** to flash this OTA-enabled `ESP32_A.ino`. After that, future ESP32_A uploads can be done wirelessly.

## 1. Prerequisites
- Arduino IDE 2.x (or Arduino CLI) with ESP32 board support (target tested: `esp32:esp32` 3.3.11).
- `ESP32_A.ino` with ArduinoOTA enabled.
- USB data cable for the one-time flash.
- Stable power for ESP32_A during every upload. **Do not power off mid-upload.**
- Laptop WiFi able to join **`• TP-Link_8080`** (note the leading bullet `•` in SSID).
- OTA-capable partition scheme (e.g. **Default 4MB with spiffs**).

## 2. One-time USB setup (required once)
1. Connect ESP32_A to computer via USB once.
2. Open `ESP32_A.ino` in Arduino IDE.
3. **Tools → Board:** your ESP32 board. **Tools → Port:** USB serial port. **Tools → Partition Scheme:** OTA-capable (e.g. Default 4MB with spiffs).
4. Click **Upload** and wait for success.
5. Open **Serial Monitor** at **115200** and confirm logs show:
   - `OTA AP SSID: • TP-Link_8080  IP: 192.168.10.1`
   - `OTA hostname: ESP32-WATER-A ...`
   - ESP-NOW ready on channel 6 and target ESP32_B MAC.
6. Disconnect USB and install/power ESP32_A normally.

## 3. Wireless upload steps (after one-time USB setup)
1. **Power on ESP32_A** (and ESP32_B, if you want normal operation/ESP-NOW active during test) and wait ~10–20s.
2. On laptop, connect WiFi to **`• TP-Link_8080`** (password **`dks12345`**).
3. Confirm reachability to **`192.168.10.1`** (ping or network port discovery).
4. Open updated `ESP32_A.ino` in Arduino IDE.
5. **Tools → Port:** select network port **`ESP32-WATER-A` / `192.168.10.1`**.
6. Keep same board/OTA partition settings. Click **Upload**.
7. If prompted, enter OTA password: **`dks12345`**.
8. Wait for completion; ESP32_A will reboot automatically.
9. Reconnect to `• TP-Link_8080` if needed and verify Serial/dashboard behavior via ESP32_B.

### Arduino CLI (example)
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 "ESP32_A.ino"
arduino-cli upload -p 192.168.10.1 --fqbn esp32:esp32:esp32 --upload-field password=dks12345 "ESP32_A.ino"
```
*(Run from a context where sketch path/name resolves correctly; flags may vary by core/CLI version.)*

## 4. Notes for stable A↔B communication
- Both ESPs use **channel 6**, WiFi power-save off, and max TX power setting in firmware for stronger link.
- ESP32_A **hourly disruptive auto-refresh removed**; continuous ESP-NOW peer/channel maintenance runs in loop.
- ESP32_A OTA AP and ESP-NOW share channel 6. Keep ESP32_A within reliable range of ESP32_B and use stable power (antenna improvement on A is recommended).
- During OTA upload, sensor/ESP-NOW activity may be briefly affected; device reboots after successful upload.
- If OTA becomes unreachable, recover via USB.

## 5. Quick reference
- ESP32_A OTA WiFi: **`• TP-Link_8080`** / **`dks12345`** / **192.168.10.1**
- ESP32_B OTA WiFi: **`Smart Water Tank`** / **`dks12345`** / **192.168.4.1** (OTA host `ESP32-WATER-B`, pass `dks12345`)
- ESP32_A does not need to join ESP32_B’s AP for ESP-NOW in this setup; both operate on channel 6.
