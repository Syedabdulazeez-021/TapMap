// ============================================================
//  TapMap — SOLENOID-ONLY TEST
//  Pulses GPIO4 (the solenoid gate) HIGH for 400 ms every 2 s.
//  Watch the solenoid: it should tap firmly once every ~2 seconds.
//
//  This tests the whole path:  GPIO4 (D4) -> gate -> MOSFET -> solenoid.
//  Serial prints ON / off so you can match the tap to the pulse.
// ============================================================
#include <Arduino.h>

#define SOLENOID 4      // GPIO4 = the pin labelled D4 / 4

void setup() {
  Serial.begin(115200);
  pinMode(SOLENOID, OUTPUT);
  digitalWrite(SOLENOID, LOW);
  Serial.println("Solenoid test: pulsing GPIO4 every 2s");
}

void loop() {
  digitalWrite(SOLENOID, HIGH);
  Serial.println("ON  -> solenoid should pull in");
  delay(400);
  digitalWrite(SOLENOID, LOW);
  Serial.println("off");
  delay(1600);
}
