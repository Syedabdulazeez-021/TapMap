# TapMap

**An autonomous percussive defect-mapping rover.** TapMap drives over a tiled or
layered surface, taps each spot with a solenoid, listens to the tap with a
microphone, classifies the sound as **hollow** (debonded / delaminated) or
**solid**, and builds a live defect **heat-map** plus a PDF report.

It automates the manual "tap testing" that inspectors do by ear, into an
objective, repeatable robotic survey.

---

## Architecture

- **Raspberry Pi 4** — the brain. Runs ROS 2 (Jazzy): navigation, odometry,
  acoustic classification, mapping, and reporting.
- **ESP32-WROOM-32** — the I/O board. Drives the motors + solenoid, reads the
  mic (I2S) and the VL53L0X distance sensor, shows status on an OLED.
- **Link** — the ESP32 and Pi talk over **WiFi (TCP)**. The ESP32 runs a TCP
  server; the Pi's `wifi_bridge` connects as a client. (A USB-serial variant is
  included as a fallback.)
- **Chassis** — 4-wheel skid-steer (TT motors via an L298N), encoder-less; pose
  is estimated by open-loop dead-reckoning from commanded velocity + IMU heading.

Protocol: the ESP32 streams `ENC,l,r` (50 Hz) and `DIST,<mm>` (10 Hz); it obeys
`M,<l>,<r>` (motor PWM) and `TAP` (fire solenoid + capture audio → `AUDIO,...`).

Classification is a **band-energy-ratio** DSP: energy in 150–1000 Hz (hollow
"drum") vs 1000–4000 Hz (solid "tick"); ratio > threshold ⇒ hollow.

---

## Repository layout

```
pi/
  ros2_tapmap/        ROS 2 package  (copy to ~/tapmap_ws/src/tapmap/)
    tapmap/           nodes: wifi_bridge, serial_bridge, deadreckon_controller,
                      coverage_planner, audio_classifier, defect_mapper,
                      report_generator, base_controller, sim_firmware
    launch/           robot_wifi.launch.py, robot.launch.py, sim.launch.py
    config/tapmap.yaml
  scripts/
    wander.py         autonomous march-and-tap explorer (obstacle-aware)
    live_map.py       live web heat-map on :8080
    chassis_test.py   direct Pi+L298N wheel test (gpiozero)
esp32/
  src/main.cpp        WiFi firmware (set WIFI_SSID / WIFI_PASS at the top)
  variants/           USB-serial firmware variant
  tests/              solenoid_test, oled_test, selftest sketches
  platformio.ini
docs/
  TapMap_Full_Explanation.md
  TapMap_Complete_Build_Plan.pdf
  TapMap_Engineering_Blueprint.md
  TapMap_Mid_Review.pptx
```

---

## Hardware

Raspberry Pi 4 · ESP32-WROOM-32 · L298N motor driver · 4× TT gear motors ·
INMP441 I2S microphone · VL53L0X ToF sensor · MPU6050 IMU · SSD1306 OLED ·
push-pull solenoid (tapper) · IRLZ44N MOSFET + 1N5822 flyback diode + 1000 µF cap ·
12 V battery (motors) · buck converter / power bank (5 V logic).

Locked pins: `ENC 34/35/36/39`, `ENA 32 IN1 25 IN2 26 IN3 27 IN4 13 ENB 33`,
`SOLENOID 4`, `I2S SCK 18 WS 19 SD 23` (mic L/R → GND), `I2C SDA 21 SCL 22`
(OLED 0x3C, MPU 0x68, VL53 0x29).

---

## Running it

### 1. ESP32 firmware
Open `esp32/src/main.cpp`, set your 2.4 GHz WiFi:
```cpp
const char* WIFI_SSID = "YOUR_HOTSPOT_NAME";
const char* WIFI_PASS = "YOUR_HOTSPOT_PASSWORD";
```
Flash with PlatformIO (`esp32/platformio.ini`). The OLED shows the ESP32's IP on
boot — put that IP into `pi/ros2_tapmap/config/tapmap.yaml` → `wifi_bridge: esp32_ip`.

### 2. Pi workspace
```bash
# copy the package in, then:
cd ~/tapmap_ws && colcon build --packages-select tapmap && source install/setup.bash
```

### 3. Launch + run (same WiFi as the ESP32)
```bash
# terminal 1 — robot
ros2 launch tapmap robot_wifi.launch.py            # wait for "connected to ESP32"

# terminal 2 — live map (open http://<pi-ip>:8080)
python3 ~/live_map.py

# terminal 3 — autonomous run (drive + tap + avoid, prints HOLLOW/SOLID)
python3 ~/wander.py 120

# after it finishes — PDF report
ros2 service call /generate_report std_srvs/srv/Trigger   # -> ~/tapmap_report.pdf
```

A fixed-area grid scan is also available via the ScanArea action:
```bash
ros2 action send_goal --feedback /scan_area tapmap_interfaces/action/ScanArea \
  "{width: 1.0, height: 1.0, spacing: 0.1, return_home: true}"
```

---

## Notes

- Encoder-less chassis → open-loop dead-reckoning (stopwatch-calibrated); best on
  a flat surface.
- The classifier is DSP (band-energy ratio), not deep learning.
- The ESP32's WiFi IP can change when the hotspot reassigns it — read it off the
  OLED and update `esp32_ip` in `tapmap.yaml` if needed.
- WiFi credentials are placeholders in `main.cpp` — fill in your own locally.
