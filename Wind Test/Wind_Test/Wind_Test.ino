const int windPin = 4;         // wire from red -> GPIO4
volatile unsigned long windCount = 0;

void IRAM_ATTR onWind() {
  // very small ISR: just increment
  windCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(windPin, INPUT_PULLUP);              // internal pull-up
  attachInterrupt(digitalPinToInterrupt(windPin), onWind, FALLING);
}

void loop() {
  delay(5000);                                 // sample window (5s)
  noInterrupts();
  unsigned long ticks = windCount;
  windCount = 0;
  interrupts();

  Serial.print("Pulses in 5s: ");
  Serial.println(ticks);

  float pulses_per_sec = ticks / 5.0;
  float mph = pulses_per_sec * 1.492;          // kit: 1 rotation/sec = 1.492 MPH
  float mps = mph * 0.44704;                   // mph -> m/s
  Serial.print("Pulses/s: "); Serial.println(pulses_per_sec);
  Serial.print("Wind speed: "); Serial.print(mph); Serial.print(" MPH, ");
  Serial.print(mps); Serial.println(" m/s");
}
