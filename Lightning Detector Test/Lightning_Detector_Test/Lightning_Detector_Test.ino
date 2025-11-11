#include <SPI.h>
#include "SparkFun_AS3935.h"

// ----------------- Pin configuration -----------------
const int AS3935_CS  = 5;   // SPI Chip Select
const int AS3935_INT = 19;  // Interrupt pin from sensor

// ----------------- Sensor object -----------------
SparkFun_AS3935 lightning;

// ----------------- Mock control -----------------
bool mockLightning = false;  // toggled via serial command

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SparkFun AS3935 Mock Lightning Test ===");
  Serial.println("Type 'L' in Serial Monitor to simulate a lightning strike.");
  Serial.println("Type 'D' to simulate a disturber (false positive).");

  SPI.begin();
  if (!lightning.beginSPI(AS3935_CS, AS3935_INT)) {
    Serial.println("Sensor not found. Check wiring.");
    while (1);
  }

  lightning.calibrateOsc();
  lightning.setNoiseLevel(2);
  lightning.spikeRejection(2);

  pinMode(AS3935_INT, INPUT);
}

void loop() {
  // ---- Check for serial input to trigger mock event ----
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'L' || cmd == 'l') {
      mockLightning = true;
      Serial.println("[MOCK] Triggering fake lightning event...");
    } else if (cmd == 'D' || cmd == 'd') {
      Serial.println("[MOCK] Triggering fake disturber event...");
      Serial.println("Disturber detected (false positive)");
    }
  }

  // ---- Normal AS3935 interrupt handling ----
  byte event = lightning.readInterruptReg();

  if (event == 2 || event == 3) {
    Serial.print("Lightning detected! Estimated distance (km): ");
    Serial.println(lightning.distanceToStorm());
  } else if (event == 1) {
    Serial.println("Disturber detected (false positive)");
  }

  // ---- Mock lightning simulation ----
  if (mockLightning) {
    Serial.print("Lightning detected! Estimated distance (km): ");
    Serial.println(random(1, 40));  // fake random distance
    mockLightning = false;
  }

  // ---- Debug: show INT pin state ----
  Serial.print("INT pin state: ");
  Serial.println(digitalRead(AS3935_INT));

  delay(500);
}
