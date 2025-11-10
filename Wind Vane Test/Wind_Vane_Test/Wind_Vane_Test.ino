// Wind vane test sketch for ESP32
int windPin = 34; // ADC pin connected to yellow wire

struct WindDir {
  int adcMin;
  int adcMax;
  const char* direction;
};

// Example lookup table — adjust adcMin/adcMax after testing
WindDir windMap[] = {
  {0, 200, "N"},
  {201, 400, "NNE"},
  {401, 600, "NE"},
  {601, 800, "ENE"},
  {801, 1000, "E"},
  {1001, 1200, "ESE"},
  {1201, 1400, "SE"},
  {1401, 1600, "SSE"},
  {1601, 1800, "S"},
  {1801, 2000, "SSW"},
  {2001, 2200, "SW"},
  {2201, 2400, "WSW"},
  {2401, 2600, "W"},
  {2601, 2800, "WNW"},
  {2801, 3000, "NW"},
  {3001, 4095, "NNW"}
};

void setup() {
  Serial.begin(115200);
}

void loop() {
  int adcVal = analogRead(windPin);

  const char* direction = "Unknown";

  for (int i = 0; i < 16; i++) {
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
