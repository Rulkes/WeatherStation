// ESP32 Wind Vane 8-Point Direction Test
// Reads ADC on GPIO 32 and maps to 8 compass points using nearest-neighbor

const int windDirPin = 32; // ADC input

// Updated calibrated ADC values from your latest test
const char* directions[8] = {"N","NE","E","SE","S","SW","W","NW"};
const int adcValues[8]   = {3022, 1700, 216, 586, 995, 2381, 3800, 3500};

// Store last valid ADC reading
int lastValidADC = adcValues[0];

// Read ADC with averaging, ignoring near-zero spikes
int readWindDir() {
  long sum = 0;
  int validSamples = 0;
  const int samples = 5;

  for(int i=0;i<samples;i++){
    int val = analogRead(windDirPin);
    if(val>10){  // ignore 0 / floating spikes
      sum += val;
      validSamples++;
    }
    delay(10);
  }

  if(validSamples == 0) return lastValidADC; // keep last reading if none valid
  lastValidADC = sum / validSamples;
  return lastValidADC;
}

// Map ADC reading to nearest compass direction
String getDirection(int adc) {
  int closestIndex = 0;
  int smallestDiff = abs(adc - adcValues[0]);

  for(int i=1;i<8;i++){
    int diff = abs(adc - adcValues[i]);
    if(diff < smallestDiff){
      smallestDiff = diff;
      closestIndex = i;
    }
  }

  return String(directions[closestIndex]);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Wind vane direction test starting...");
}

void loop() {
  int rawADC = readWindDir();
  String dir = getDirection(rawADC);

  Serial.print("Raw ADC: ");
  Serial.print(rawADC);
  Serial.print(" → Direction: ");
  Serial.println(dir);

  delay(500);
}
