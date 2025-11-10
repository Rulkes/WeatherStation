const int rainPin = 27;       // your chosen GPIO
volatile unsigned long rainCount = 0;

void IRAM_ATTR onRain() {
  rainCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), onRain, FALLING);
}

void loop() {
  delay(5000);  // check every 5 seconds
  noInterrupts();
  unsigned long tips = rainCount;
  rainCount = 0;
  interrupts();

  Serial.print("Bucket tips in 5s: ");
  Serial.println(tips);

  float inches = tips * 0.011;       // kit spec: 0.011" per tip
  float mm = inches * 25.4;
  Serial.print("Rainfall: ");
  Serial.print(mm);
  Serial.println(" mm");
}
