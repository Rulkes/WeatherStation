// Wind vane test sketch for ESP32
int windPin = 34; // ADC pin connected to yellow wire

struct WindDir {
  int adcMin;
  int adcMax;
  const char* direction;
};

// Lookup table based on your measured ADC values, with E ~3950
WindDir windMap[] = {
  {950, 1050, "N"},
  {1051, 2300, "NE"},   // Approximate range halfway to E
  {3900, 4000, "E"},    // Adjusted E to ~3950
  {2301, 3899, "SE"},   // Adjusted SE to fill the gap to E
  {2950, 3050, "S"},
  {3051, 3400, "SW"},   // Approximate range halfway to W
  {150, 250, "W"},
  {251, 949, "NW"}      // Approximate range halfway back to N
};

void setup() {
  Serial.begin(115200);
}

void loop() {
  int adcVal = analogRead(windPin);

  const char* direction = "Unknown";

  for (int i = 0; i < 8; i++) {
    if (adcVal >= windMap[i].adcMin && adcVal <= windMap[i].adcMax) {
      direction = windMap[i].direction;
      break;
    }
  }

  Serial.print("ADC: ");
  Serial.print(adcVal);
  Serial.print(" | Direction: ");
  Serial.println(direction);

  delay(500); // Read twice per second
}
