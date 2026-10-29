// ============================================================
//  TapMap — ESP32 ROBOT firmware  (WiFi / TCP link to the Pi)
//  Board: ESP32-WROOM-32 30-pin DevKit
//
//  Same behaviour as the USB firmware, but the link to the Pi is a
//  TCP socket over WiFi instead of USB serial:
//    * ESP32 joins your hotspot and runs a TCP SERVER on port 8266
//    * the Pi (wifi_bridge) connects to the ESP32's IP as a client
//    * protocol is IDENTICAL: streams "ENC,l,r" @50Hz; obeys "M,l,r"
//      and "TAP"; replies "AUDIO,.."; 400ms motor deadman
//    * OLED shows the WiFi IP + whether the Pi is connected
//
//  >>> SET YOUR HOTSPOT NAME + PASSWORD BELOW <<<
//  The ESP32's IP is shown on the OLED + Serial on boot; put that IP
//  into the Pi's tapmap.yaml (wifi_bridge: esp32_ip).
//
//  PlatformIO -> platformio.ini  [env:esp32dev]:
//    lib_deps =
//      adafruit/Adafruit SSD1306
//      adafruit/Adafruit GFX Library
//      pololu/VL53L0X
//  Streams "ENC,l,r" @50Hz and "DIST,<mm>" @10Hz; obeys "M,l,r" and "TAP".
// ============================================================
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VL53L0X.h>
#include <driver/i2s.h>

// ===================== SET THESE =====================
const char* WIFI_SSID = "YOUR_HOTSPOT_NAME";
const char* WIFI_PASS = "YOUR_HOTSPOT_PASSWORD";
const uint16_t TCP_PORT = 8266;
// =====================================================

// ---------------- PIN MAP (locked) ----------------
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 36
#define R_ENC_B 39
#define ENA 32
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 13
#define ENB 33
#define SOLENOID 4
#define I2S_SCK 18
#define I2S_WS  19
#define I2S_SD  23
#define I2C_SDA 21
#define I2C_SCL 22

// ---------------- OLED ----------------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOK = false;

// ---------------- VL53L0X distance (front obstacle sensor) ----------------
VL53L0X lox;
bool loxOK = false;
int  lastDist = -1;            // most recent distance in mm (-1 = no reading)
unsigned long lastDistOut = 0; // last time we streamed DIST

// ---------------- WiFi / TCP ----------------
WiFiServer server(TCP_PORT);
WiFiClient client;

// ---------------- PWM (LEDC) ----------------
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int CH_A = 0;
const int CH_B = 1;

// ---------------- Encoders ----------------
volatile long leftTicks  = 0;
volatile long rightTicks = 0;
void IRAM_ATTR leftISR()  { if (digitalRead(L_ENC_B)) leftTicks++;  else leftTicks--; }
void IRAM_ATTR rightISR() { if (digitalRead(R_ENC_B)) rightTicks++; else rightTicks--; }

// ---------------- Motors ----------------
void setSide(int inA, int inB, int ch, int pwm) {
  bool fwd = (pwm >= 0);
  int duty = abs(pwm); if (duty > 255) duty = 255;
  digitalWrite(inA, fwd ? HIGH : LOW);
  digitalWrite(inB, fwd ? LOW  : HIGH);
  ledcWrite(ch, duty);
}
void drive(int left, int right) {
  setSide(IN1, IN2, CH_A, left);
  setSide(IN3, IN4, CH_B, right);
}
void stopMotors() { drive(0, 0); }

// ---------------- I2S audio ----------------
const int SAMPLE_RATE = 16000;
const int TAP_MS      = 100;
const int TAP_SAMPLES = SAMPLE_RATE * TAP_MS / 1000;
const int SOL_MS      = 400;     // solenoid strike length (PROVEN value — taps firmly)
bool i2sReady = false;

void initI2S() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;   // INMP441 delivers on RIGHT on this ESP32
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_SCK;
  pins.ws_io_num  = I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = I2S_SD;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) == ESP_OK &&
      i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK) {
    i2sReady = true;
  }
}

int16_t window[TAP_SAMPLES];

// live peak-to-peak level so the OLED can show the mic actually working.
// returns -1 if the driver isn't ready, 0 if no samples arrived at all.
int micLevel() {
  if (!i2sReady) return -1;
  static int32_t raw[128];
  size_t nbytes = 0;
  i2s_read(I2S_NUM_0, raw, sizeof(raw), &nbytes, 20 / portTICK_PERIOD_MS);
  int cnt = nbytes / 4;
  if (cnt == 0) return 0;
  int16_t mn = 32767, mx = -32768;
  for (int i = 0; i < cnt; i++) {
    int16_t s = (int16_t)(raw[i] >> 16);
    if (s < mn) mn = s;
    if (s > mx) mx = s;
  }
  return (int)(mx - mn);
}

void fireTapAndCapture() {
  int collected = 0;

  // Fire the strike and RECORD FROM THE SAME INSTANT. Reading immediately keeps the
  // I2S DMA draining (a long blind delay overflows it and returns nothing) AND
  // captures the real impact ring. The solenoid is still held the full SOL_MS.
  digitalWrite(SOLENOID, HIGH);
  unsigned long t0 = millis();

  if (i2sReady) {
    static int32_t raw[256];
    size_t bytesRead;

    // FLUSH stale/overflowed DMA first. The idle OLED meter reads slowly and lets
    // the buffer overflow, which can leave i2s_read returning nothing. Drain it so
    // the real grab starts on fresh, flowing audio.
    unsigned long df = millis();
    while ((millis() - df) < 15) {
      if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, 5 / portTICK_PERIOD_MS) != ESP_OK) break;
      if (bytesRead == 0) break;
    }

    // collect the ring, bounded at 300 ms so a dead mic can never hang us
    while (collected < TAP_SAMPLES && (millis() - t0) < 300) {
      if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, 10 / portTICK_PERIOD_MS) == ESP_OK) {
        int n = bytesRead / 4;
        for (int i = 0; i < n && collected < TAP_SAMPLES; i++)
          window[collected++] = (int16_t)(raw[i] >> 16);
      }
    }
  }

  // hold the solenoid for the full strike duration, then release
  while (millis() - t0 < SOL_MS) delay(2);
  digitalWrite(SOLENOID, LOW);

  // DEBUG: report how many samples we grabbed (appears in the launch log as "ESP32: ...")
  if (client && client.connected()) { client.print("TapMap taps="); client.println(collected); }

  // reply to the Pi — build the whole line and send it in ONE write (sending 1900
  // numbers as individual prints floods the TCP stack and can stall/drop).
  if (client && client.connected()) {
    if (collected > 64) {
      String out; out.reserve(collected * 6 + 8);
      out = "AUDIO";
      for (int i = 0; i < collected; i++) { out += ','; out += window[i]; }
      out += '\n';
      client.print(out);
    } else {
      client.println("AUDIO,0");
    }
  }
}

// ---------------- command parsing ----------------
String buf;
unsigned long lastMotorCmd = 0;
unsigned long lastEncOut   = 0;
unsigned long lastOled     = 0;
unsigned long lastRx       = 0;

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  lastRx = millis();
  if (line == "TAP") {
    fireTapAndCapture();
  } else if (line.startsWith("M,")) {
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 > 0) {
      int l = line.substring(c1 + 1, c2).toInt();
      int r = line.substring(c2 + 1).toInt();
      drive(l, r);
      lastMotorCmd = millis();
    }
  }
}

void drawOled(long l, long r, int mic) {
  if (!oledOK) return;
  bool linkUp = client && client.connected();
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.println("TapMap  WiFi");
  oled.setCursor(0, 12);
  if (WiFi.status() == WL_CONNECTED) { oled.print("IP:"); oled.println(WiFi.localIP()); }
  else oled.println("WiFi: connecting");
  oled.setCursor(0, 26); oled.print("Pi : ");
  oled.println(linkUp ? "connected" : "waiting");
  oled.setCursor(0, 40); oled.print("VL53: ");
  if (loxOK && lastDist >= 0) { oled.print(lastDist); oled.println(" mm"); } else oled.println("--");
  oled.setCursor(0, 54); oled.print("Mic: ");
  if (!i2sReady) oled.println("--"); else oled.println(mic);
  oled.display();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400); Serial.print(".");
    if (oledOK) {
      oled.clearDisplay(); oled.setCursor(0, 0);
      oled.println("TapMap WiFi");
      oled.println("connecting...");
      oled.display();
    }
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK  IP = "); Serial.println(WiFi.localIP());
    server.begin();
    server.setNoDelay(true);
  } else {
    Serial.println("WiFi FAILED (check SSID/PASS, 2.4GHz hotspot)");
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // VL53L0X continuous ranging (non-blocking reads at ~20 Hz)
  lox.setTimeout(100);
  loxOK = lox.init();
  if (loxOK) lox.startContinuous(50);

  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(CH_A, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES); ledcAttachPin(ENB, CH_B);
  stopMotors();

  pinMode(SOLENOID, OUTPUT); digitalWrite(SOLENOID, LOW);

  initI2S();
  connectWiFi();
  Serial.println("TapMap ESP32 (WiFi) ready");
}

void loop() {
  // keep WiFi up
  if (WiFi.status() != WL_CONNECTED) {
    stopMotors();
    connectWiFi();
    return;
  }

  // accept / refresh the Pi client
  if (!client || !client.connected()) {
    WiFiClient incoming = server.available();
    if (incoming) { client = incoming; client.setNoDelay(true); lastRx = millis(); }
  }

  // read incoming commands line by line
  while (client && client.available()) {
    char c = client.read();
    if (c == '\n') { handleLine(buf); buf = ""; }
    else if (c != '\r') { buf += c; if (buf.length() > 64) buf = ""; }
  }

  unsigned long now = millis();
  long l, r;
  noInterrupts(); l = leftTicks; r = rightTicks; interrupts();

  if (now - lastEncOut >= 20) {            // 50 Hz encoder stream
    lastEncOut = now;
    if (client && client.connected()) {
      client.print("ENC,"); client.print(l); client.print(','); client.println(r);
    }
  }

  if (now - lastDistOut >= 100) {          // 10 Hz distance stream
    lastDistOut = now;
    if (loxOK) {
      int d = lox.readRangeContinuousMillimeters();
      lastDist = lox.timeoutOccurred() ? -1 : d;
    }
    if (client && client.connected()) {
      client.print("DIST,"); client.println(lastDist);
    }
  }

  if (now - lastOled >= 250) {             // 4 Hz OLED refresh
    lastOled = now;
    drawOled(l, r, micLevel());
  }

  if (now - lastMotorCmd > 400) stopMotors();   // deadman
}
