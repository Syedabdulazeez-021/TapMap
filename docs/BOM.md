# Bill of Materials

Approximate build cost (₹, INR). The Raspberry Pi 4 was lab-provided and is
not counted in the build total.

| # | Component | Qty | Role | Approx. ₹ |
|---|-----------|-----|------|-----------|
| 1 | Raspberry Pi 4 | 1 | Onboard brain (ROS 2) | lab-provided |
| 2 | ESP32-WROOM-32 DevKit | 1 | I/O board | 400 |
| 3 | L298N motor driver | 1 | Dual H-bridge for motors | 150 |
| 4 | TT gear motor + wheel | 4 | 4-wheel skid-steer drive | 400 |
| 5 | INMP441 I2S microphone | 1 | Tap-sound capture | 250 |
| 6 | VL53L0X ToF sensor | 1 | Front obstacle ranging | 200 |
| 7 | MPU6050 IMU | 1 | Heading | 100 |
| 8 | SSD1306 OLED 0.96" | 1 | Status display | 180 |
| 9 | Push-pull solenoid (12 V) | 1 | Percussive tapper | 200 |
| 10 | IRLZ44N MOSFET | 1 | Solenoid low-side switch | 40 |
| 11 | 1N5822 diode | 1 | Flyback protection | 10 |
| 12 | 1000 µF capacitor | 1 | Supply buffering | 20 |
| 13 | LM2596 buck converter | 1 | 12 V → 5 V logic | 80 |
| 14 | 12 V Li-ion battery pack | 1 | Motor + solenoid power | 500 |
| 15 | Chassis, wiring, breadboard, fasteners | — | Structure & interconnect | 400 |
|   | **Build total (excl. Pi)** | | | **~₹2,930** |

Locked pin map: `ENC 34/35/36/39` · `ENA 32 IN1 25 IN2 26 IN3 27 IN4 13 ENB 33` ·
`SOLENOID 4` · `I2S SCK 18 WS 19 SD 23` (mic L/R → GND) ·
`I2C SDA 21 SCL 22` (OLED 0x3C, MPU 0x68, VL53 0x29).
