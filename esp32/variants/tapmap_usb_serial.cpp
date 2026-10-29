// ============================================================
//  TapMap — ESP32 ROBOT firmware  (USB link, OLED status)
//  Board: ESP32-WROOM-32 30-pin DevKit
//
//  CLEAN build — NO VL53 library (its ranging call was stalling the
//  loop and blocking incoming Pi commands). Serial RX is serviced
//  first every loop so M / TAP always get through.
//
//  Does: streams "ENC,l,r" @50Hz; obeys "M,l,r" (motor PWM) and "TAP"
//  (guaranteed solenoid strike + audio capture -> "AUDIO,.."); 400ms
//  motor deadman. OLED shows Link / Enc / Mic.
//
//  PlatformIO -> platformio.ini  [env:esp32dev]:
//    lib_deps =
//      adafruit/Adafruit SSD1306
//      adafruit/Adafruit GFX Library
//  (pololu/VL53L0X is NOT needed here.)
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/i2s.h>

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

// ---------------- PWM ----------------
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
const int TAP_MS      = 120;     // ring window captured after the strike
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

void fireTapAndCapture() {
  // STEP 1 — guaranteed simple strike (runs no matter what the mic does)
  digitalWrite(SOLENOID, HIGH);
  delay(SOL_MS);
  digitalWrite(SOLENOID, LOW);

  // STEP 2 — capture the ring (bounded; a loose mic can never hang us)
  int collected = 0;
  if (i2sReady) {
    static int32_t raw[256];
    size_t bytesRead;
    unsigned long c0 = millis();
    while (collected < TAP_SAMPLES && (millis() - c0) < 200) {
      if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, 10 / portTICK_PERIOD_MS) == ESP_OK) {
        int n = bytesRead / 4;
        for (int i = 0; i < n && collected < TAP_SAMPLES; i++)
          window[collected++] = (int16_t)(raw[i] >> 16);
      }
    }
  }

  // STEP 3 — reply
  if (collected > 64) {
    Serial.print("AUDIO");
    for (int i = 0; i < collected; i++) { Serial.print(','); Serial.print(window[i]); }
    Serial.println();
  } else {
    Serial.println("AUDIO,0");
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

void drawOled(long l, long r) {
  if (!oledOK) return;
  bool linkUp = (lastRx != 0) && (millis() - lastRx) < 2000;
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);  oled.print("TapMap  "); oled.println(linkUp ? "Pi" : "USB");
  oled.setCursor(0, 16); oled.print("EncL: "); oled.println(l);
  oled.setCursor(0, 30); oled.print("EncR: "); oled.println(r);
  oled.setCursor(0, 44); oled.print("Mic : "); oled.println(i2sReady ? "ready" : "--");
  oled.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOK) {
    oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0); oled.println("TapMap booting..."); oled.display();
  }

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
  Serial.println("TapMap ESP32 ready (clean, no VL53)");
}

void loop() {
  // 1) ALWAYS service incoming commands first
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleLine(buf); buf = ""; }
    else if (c != '\r') { buf += c; if (buf.length() > 64) buf = ""; }
  }

  unsigned long now = millis();
  long l, r;
  noInterrupts(); l = leftTicks; r = rightTicks; interrupts();

  if (now - lastEncOut >= 20) {            // 50 Hz ENC stream
    lastEncOut = now;
    Serial.print("ENC,"); Serial.print(l); Serial.print(','); Serial.println(r);
  }

  if (now - lastOled >= 300) {             // OLED refresh (light, non-blocking)
    lastOled = now;
    drawOled(l, r);
  }

  if (now - lastMotorCmd > 400) stopMotors();   // deadman
}
