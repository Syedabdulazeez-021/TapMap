# Install & Setup

## 1. Raspberry Pi — ROS 2 workspace

Requires **Ubuntu 24.04** + **ROS 2 Jazzy** on the Pi 4.

```bash
# create the workspace and copy both packages in
mkdir -p ~/tapmap_ws/src
cp -r pi/ros2_tapmap        ~/tapmap_ws/src/tapmap            # python nodes (package name: tapmap)
cp -r pi/tapmap_interfaces  ~/tapmap_ws/src/tapmap_interfaces # custom msgs + action

# install dependencies
cd ~/tapmap_ws
rosdep install --from-paths src --ignore-src -r -y

# python libs used by the nodes / scripts
pip3 install numpy matplotlib --break-system-packages
sudo apt install -y python3-gpiozero        # only for chassis_test.py (direct GPIO drive)

# build
colcon build
source install/setup.bash
```

> Build `tapmap_interfaces` first if colcon complains:
> `colcon build --packages-select tapmap_interfaces && colcon build`

## 2. ESP32 — firmware (laptop, PlatformIO)

- Open `esp32/` in PlatformIO (uses `esp32/platformio.ini`).
- Flash `esp32/src/main.cpp`. **Set your 2.4 GHz WiFi first:**
  ```cpp
  const char* WIFI_SSID = "YOUR_HOTSPOT_NAME";
  const char* WIFI_PASS = "YOUR_HOTSPOT_PASSWORD";
  ```
- Libraries (auto-installed by PlatformIO): Adafruit SSD1306, Adafruit GFX, pololu/VL53L0X.
- On boot the OLED shows the ESP32's IP — put it in
  `pi/ros2_tapmap/config/tapmap.yaml` → `wifi_bridge: esp32_ip`, then rebuild.

## 3. Run

See the main `README.md` → **Running it**.
