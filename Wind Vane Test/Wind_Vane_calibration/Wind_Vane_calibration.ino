/*
  Wind Vane Interactive Calibration
  - Point vane to heading shown on serial, press a key to record.
  - Collects N_SAMPLES samples and computes median for accuracy.
  - At the end it prints medians and suggested ADC ranges (midpoints).
*/

const int windPin = 34;
const int N_DIRECTIONS = 8;
const int N_SAMPLES = 200;        // samples per heading
const int SAMPLE_DELAY_MS = 8;    // between samples

const char* headings[N_DIRECTIONS] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
int medians[N_DIRECTIONS];

int readMedianSamples(int count) {
  static int buf[1024];
  if (count > 1024) count = 1024;
  for (int i = 0; i < count; i++) {
    buf[i] = analogRead(windPin);
    delay(SAMPLE_DELAY_MS);
  }
  // simple nth_element-style median (partial sort by qsort for clarity)
  // Using qsort (std C) for simplicity
  qsort(buf, count, sizeof(int), [](const void* a, const void* b) -> int {
    int aa = *(const int*)a;
    int bb = *(const int*)b;
    return (aa < bb) ? -1 : (aa > bb) ? 1 : 0;
  });
  return buf[count/2];
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Wind Vane Interactive Calibration ===");
  Serial.println("You will be prompted to point the vane to each heading.");
  Serial.println("After pointing and stabilizing, press ENTER in Serial Monitor to record.");
  Serial.println();
  pinMode(windPin, INPUT);
}

void loop() {
  for (int i = 0; i < N_DIRECTIONS; i++) {
    Serial.print("Point vane to ");
    Serial.print(headings[i]);
    Serial.println(" and press ENTER to record...");
    // wait for newline
    while (!Serial.available()) { delay(50); }
    // consume input
    while (Serial.available()) Serial.read();

    Serial.println("Recording samples...");
    int m = readMedianSamples(N_SAMPLES);
    medians[i] = m;
    Serial.print("Recorded median for ");
    Serial.print(headings[i]);
    Serial.print(" = ");
    Serial.println(m);
    delay(300);
  }

  // Print medians and compute boundaries
  Serial.println("\nCalibration results:");
  for (int i = 0; i < N_DIRECTIONS; i++) {
    Serial.print(headings[i]);
    Serial.print(" median = ");
    Serial.println(medians[i]);
  }

  // compute boundaries between adjacent medians (circular)
  Serial.println("\nSuggested ADC ranges:");
  for (int i = 0; i < N_DIRECTIONS; i++) {
    int a = medians[i];
    int b = medians[(i+1)%N_DIRECTIONS];
    // midpoint circularly: if b < a, treat as wrap-around
    int mid;
    if (b >= a) mid = (a + b) / 2;
    else { // wrap
      // convert to 0..4095 linear by adding 4096 to b
      mid = (a + (b + 4096)) / 2;
      if (mid >= 4096) mid -= 4096;
    }
    int rangeStart = (i==0) ? ((medians[(i-1+N_DIRECTIONS)%N_DIRECTIONS] + a + 4096)/2 % 4096) : 0;
    // We'll print min..max by using adjacent mids:
    int prev = medians[(i-1+N_DIRECTIONS)%N_DIRECTIONS];
    int prevMid;
    if (a >= prev) prevMid = (prev + a) / 2;
    else prevMid = (prev + (a + 4096)) / 2 % 4096;
    int start = prevMid;
    int end = mid;
    Serial.print(headings[i]);
    Serial.print(": ");
    Serial.print(start);
    Serial.print("  ->  ");
    Serial.println(end);
  }

  Serial.println("\nCalibration complete. Restart board to run normal code.");
  while (1) delay(1000);
}
