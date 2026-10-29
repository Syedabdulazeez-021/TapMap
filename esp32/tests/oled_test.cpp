// ============================================================
//  OLED TEST  (ESP32 + SSD1306 128x64, I2C)
//  Flash from the laptop, open Serial Monitor @ 115200.
//  - scans the I2C bus and prints what it finds
//  - starts the OLED (tries 0x3C, then 0x3D)
//  - shows a steady screen with a live counter
//
//  PlatformIO -> platformio.ini  [env:esp32dev]:
//    lib_deps =
//      adafruit/Adafruit SSD1306
//      adafruit/Adafruit GFX Library
//
//  Wiring: OLED VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOK = false;
uint8_t addr = 0x3C;

void i2cScan() {
  Serial.println("\n--- I2C scan ---");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("found 0x"); Serial.println(a, HEX);
      n++;
    }
  }
  if (n == 0) Serial.println("NOTHING found -> check SDA/SCL/VCC/GND wiring");
  Serial.println("----------------");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n==== OLED test ====");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  i2cScan();

  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOK) {                 // some modules are at 0x3D
    addr = 0x3D;
    oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  Serial.println(oledOK ? "OLED: OK" : "OLED: begin() FAILED (wrong addr or wiring/power)");

  if (oledOK) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 0);  oled.println("OLED OK");
    oled.setTextSize(1);
    oled.setCursor(0, 24); oled.print("addr 0x"); oled.println(addr, HEX);
    oled.setCursor(0, 36); oled.println("ESP32 alive");
    oled.display();
    delay(1200);
  }
}

void loop() {
  static uint32_t n = 0;
  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(2);  oled.setCursor(0, 0);  oled.println("OLED OK");
    oled.setTextSize(1);
    oled.setCursor(0, 24); oled.print("addr 0x"); oled.println(addr, HEX);
    oled.setCursor(0, 38); oled.print("count: "); oled.println(n);
    oled.setCursor(0, 52); oled.print("uptime: "); oled.print(millis() / 1000); oled.println("s");
    oled.display();
  }
  Serial.print("alive, count="); Serial.println(n);
  n++;
  delay(500);
}
