int ledPin = 2;  // Usually GPIO2 on ESP32 boards with expansion

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);   // Turn the LED on
  delay(500);                   // Wait 500 milliseconds
  digitalWrite(ledPin, LOW);    // Turn the LED off
  delay(500);                   // Wait 500 milliseconds
}
