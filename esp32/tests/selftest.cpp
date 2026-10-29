// ============================================================
//  TapMap — SELF TEST (4-wheel chassis)
//  Checks: I2C bus (OLED, MPU6050, VL53L0X presence), INMP441 mic,
//          and the L298N motors (drives all 4 wheels FORWARD in pulses).
//  Shows results on Serial (115200) AND the OLED.
//
//  PlatformIO -> platformio.ini  [env:esp32dev]:
//    lib_deps =
//      adafruit/Adafruit SSD1306
//      adafruit/Adafruit GFX Library
//
//  NOTE: motors only actually spin when 12V is connected to the L298N.
//  Without 12V the control signals still pulse (harmless).
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>

// ---- pins (locked) ----
#define I2C_SDA 21
#define I2C_SCL 22
#define I2S_SCK 18
#define I2S_WS  19
#define I2S_SD  23
#define ENA 32
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 13
#define ENB 33
#define SOLENOID 4

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOK = false, mpuOK = false, micOK = false, loxSeen = false;
const uint8_t MPU_ADDR = 0x68;

bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return (Wire.endTransmission() == 0);
}

void i2cScan() {
  Serial.println("\n--- I2C scan ---");
  for (uint8_t a = 1; a < 127; a++)
    if (i2cPresent(a)) { Serial.print("found 0x"); Serial.println(a, HEX); }
  Serial.println("----------------");
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return (Wire.read() == 0x68);
}

void mpuRead(float &ax, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  int16_t x = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();            // skip ay
  int16_t z = Wire.read() << 8 | Wire.read();
  ax = x / 16384.0; az = z / 16384.0;
}

void micInit() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = 16000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;   // <-- the fix (LEFT gave mic=0)
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_SCK;
  pins.ws_io_num  = I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = I2S_SD;
  micOK = (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) == ESP_OK &&
           i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK);
}

int micLevel() {
  static int32_t raw[256];
  size_t n;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &n, 20 / portTICK_PERIOD_MS);
  int cnt = n / 4;
  int16_t mn = 32767, mx = -32768;
  for (int i = 0; i < cnt; i++) {
    int16_t s = raw[i] >> 16;
    if (s < mn) mn = s;
    if (s > mx) mx = s;
  }
  return (cnt > 0) ? (mx - mn) : 0;
}

// drive both sides: dir +1 = forward, 0 = stop
void motors(int dir) {
  bool fwd = (dir > 0);
  digitalWrite(IN1, fwd ? HIGH : LOW); digitalWrite(IN2, LOW);   // left side
  digitalWrite(IN3, fwd ? HIGH : LOW); digitalWrite(IN4, LOW);   // right side
  digitalWrite(ENA, dir != 0 ? HIGH : LOW);
  digitalWrite(ENB, dir != 0 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n==== TapMap self test (4-wheel) ====");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  i2cScan();

  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.println(oledOK ? "OLED  : OK" : "OLED  : NOT FOUND (try 0x3D)");
  if (oledOK) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1); oled.setCursor(0, 0);
    oled.println("TapMap self test"); oled.display();
  }

  loxSeen = i2cPresent(0x29);
  Serial.println(loxSeen ? "VL53  : seen on bus (0x29)" : "VL53  : NOT FOUND");

  mpuOK = mpuInit();
  Serial.println(mpuOK ? "MPU   : OK" : "MPU   : NOT FOUND");

  micInit();
  Serial.println(micOK ? "MIC   : OK (I2S started)" : "MIC   : FAIL");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(SOLENOID, OUTPUT); digitalWrite(SOLENOID, LOW);
  motors(0);
  Serial.println("L298N : pulses all 4 wheels FORWARD every 3s (needs 12V to spin)");
}

void loop() {
  float ax = 0, az = 0;
  if (mpuOK) mpuRead(ax, az);
  int lvl = micOK ? micLevel() : 0;
  bool tap = lvl > 4000;

  Serial.print("MPU ax="); Serial.print(ax, 2);
  Serial.print(" az="); Serial.print(az, 2);
  Serial.print(" | VL53="); Serial.print(loxSeen ? "on" : "--");
  Serial.print(" | mic="); Serial.print(lvl);
  if (tap) Serial.print("  <== TAP!");
  Serial.println();

  // ---- every 3 s: fire the SOLENOID (120 ms tap) then pulse all 4 wheels ----
  static unsigned long t = 0;
  static bool driving = false;
  unsigned long now = millis();
  if (!driving && now - t > 3000) {
    digitalWrite(SOLENOID, HIGH); delay(120); digitalWrite(SOLENOID, LOW);  // TAP
    motors(1); driving = true; t = now;
  }
  if (driving && now - t > 600)   { motors(0); driving = false; t = now; }

  if (oledOK) {
    oled.clearDisplay(); oled.setCursor(0, 0);
    oled.println("TapMap self test");
    oled.print("MPU ax:"); oled.println(ax, 2);
    oled.print("VL53  :"); oled.println(loxSeen ? " on bus" : " --");
    oled.print("Mic   :"); oled.println(lvl);
    oled.print("Motors:"); oled.println(driving ? " FWD" : " stop");
    oled.setTextSize(2); oled.setCursor(0, 48);
    oled.println(tap ? "TAP!" : "...");
    oled.setTextSize(1); oled.display();
  }
  delay(100);
}
