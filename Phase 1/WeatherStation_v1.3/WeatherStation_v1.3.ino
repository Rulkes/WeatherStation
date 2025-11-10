#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "AdafruitIO_WiFi.h"
#include "secrets.h"
#include <Wire.h>
#include <SD.h>
#include <time.h>

// ----------------- Configuration -----------------
#define SEALEVELPRESSURE_HPA 1013.25
#define SD_CS 5                 
#define LOG_FILE "/system.log"
#define LOG_MAX_LINES 5000
#define SENSOR_INTERVAL_MS 15000
#define PRUNE_CHECK_INTERVAL 3600000
#define MAX_UPLOAD_RETRIES 3
#define SENSOR_WARMUP_READINGS 5

// SparkFun Weather Meter Kit specifications
#define WIND_FACTOR 2.4         // km/h per pulse per second (1.492 MPH = 2.4 km/h)
#define RAIN_PER_TIP 0.2794     // mm per tip (0.011" = 0.2794mm)
#define DEBOUNCE_MS 10          // Debounce time for switches

// ----------------- Adafruit IO -----------------
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
Adafruit_BME680 bme;

// Feeds
AdafruitIO_Feed *temperature = io.feed("Temperature");
AdafruitIO_Feed *humidity    = io.feed("Humidity");
AdafruitIO_Feed *pressure    = io.feed("Pressure");
AdafruitIO_Feed *gas         = io.feed("Gas");
AdafruitIO_Feed *windSpeed   = io.feed("WindSpeed");
AdafruitIO_Feed *rainGauge   = io.feed("RainGauge");
AdafruitIO_Feed *windDir     = io.feed("WindDirection");
AdafruitIO_Feed *windDirText = io.feed("WindDirectionText");


// Wind direction data structure
struct WindDirData {
  float degrees;
  const char* direction;
};

// ----------------- State variables -----------------
bool sdCardAvailable = false;
bool sensorAvailable = false;
unsigned long lastSensorTime = 0;
unsigned long lastPruneCheck = 0;
int gasWarmupCounter = 0;

// Latest sensor values
float t = 0, h = 0, p = 0, g = 0;
float wind_kmh = 0, rain_mm = 0, wind_deg = 0;
String wind_direction = "N";

// ----------------- Wind & Rain Pins -----------------
const int windPin = 4;
const int rainPin = 27;
const int windDirPin = 34;

volatile unsigned long windCount = 0;
volatile unsigned long rainCount = 0;
volatile unsigned long lastWindTime = 0;
volatile unsigned long lastRainTime = 0;

// Interrupt handlers with debouncing
void IRAM_ATTR onWind() {
  unsigned long now = millis();
  if (now - lastWindTime > DEBOUNCE_MS) {
    windCount++;
    lastWindTime = now;
  }
}

void IRAM_ATTR onRain() {
  unsigned long now = millis();
  if (now - lastRainTime > DEBOUNCE_MS) {
    rainCount++;
    lastRainTime = now;
  }
}


// Wind direction lookup table for SparkFun Weather Meter Kit
// Based on resistor network values (16 positions)
// Reference voltage divider creates these ADC values at 3.3V
WindDirData getWindDirection(int analogValue) {
  // Lookup table: ADC value ranges -> degrees & cardinal direction
  struct WindEntry {
    int adcMin;
    int adcMax;
    float degrees;
    const char* direction;
  };
  
  // 16-position wind vane ADC ranges (for 3.3V reference, 12-bit ADC)
  WindEntry directions[] = {
    {3143, 3243, 0.0, "N"},
    {1624, 1724, 22.5, "NNE"},
    {1845, 1945, 45.0, "NE"},
    {335, 435, 67.5, "ENE"},
    {372, 472, 90.0, "E"},
    {264, 364, 112.5, "ESE"},
    {738, 838, 135.0, "SE"},
    {506, 606, 157.5, "SSE"},
    {1149, 1249, 180.0, "S"},
    {979, 1079, 202.5, "SSW"},
    {2030, 2130, 225.0, "SW"},
    {1493, 1593, 247.5, "WSW"},
    {3780, 3880, 270.0, "W"},
    {3309, 3409, 292.5, "WNW"},
    {3548, 3648, 315.0, "NW"},
    {2815, 2915, 337.5, "NNW"}
  };
  
  // Find matching direction
  for (int i = 0; i < 16; i++) {
    if (analogValue >= directions[i].adcMin && analogValue <= directions[i].adcMax) {
      WindDirData result = {directions[i].degrees, directions[i].direction};
      return result;
    }
  }
  
  // If no exact match, find closest
  int closestIdx = 0;
  int minDiff = abs(analogValue - (directions[0].adcMin + directions[0].adcMax) / 2);
  
  for (int i = 1; i < 16; i++) {
    int centerValue = (directions[i].adcMin + directions[i].adcMax) / 2;
    int diff = abs(analogValue - centerValue);
    if (diff < minDiff) {
      minDiff = diff;
      closestIdx = i;
    }
  }
  
  WindDirData result = {directions[closestIdx].degrees, directions[closestIdx].direction};
  return result;
}

// ----------------- Logging helper -----------------
void logMessage(const char *msg) {
  char prefix[32];
  char fullMsg[256];
  
  if (WiFi.status() == WL_CONNECTED) {
    time_t now; 
    time(&now);
    struct tm *t = localtime(&now);
    snprintf(prefix, sizeof(prefix), "[%02d:%02d:%02d]", t->tm_hour, t->tm_min, t->tm_sec);
  } else {
    snprintf(prefix, sizeof(prefix), "[%lu ms]", millis());
  }

  snprintf(fullMsg, sizeof(fullMsg), "%s %s", prefix, msg);

  if (sdCardAvailable) {
    File logFile = SD.open(LOG_FILE, FILE_APPEND);
    if (logFile) {
      logFile.println(fullMsg);
      logFile.close();
    } else {
      sdCardAvailable = false;
      Serial.println("SD card write failed - disabling SD logging");
    }
  }

  Serial.println(fullMsg);
}

// Log pruning function
void pruneLog() {
  if (!sdCardAvailable) return;
  if (millis() - lastPruneCheck < PRUNE_CHECK_INTERVAL) return;
  
  lastPruneCheck = millis();

  File file = SD.open(LOG_FILE, FILE_READ);
  if (!file) return;

  int lineCount = 0;
  while (file.available()) {
    if (file.read() == '\n') lineCount++;
  }
  file.close();

  if (lineCount <= LOG_MAX_LINES) {
    logMessage("Log size check: within limits");
    return;
  }

  int linesToKeep = (LOG_MAX_LINES * 6) / 10;
  int linesToSkip = lineCount - linesToKeep;

  File src = SD.open(LOG_FILE, FILE_READ);
  File tmp = SD.open("/tmp.log", FILE_WRITE);
  
  if (!src || !tmp) {
    if (src) src.close();
    if (tmp) tmp.close();
    logMessage("Log pruning failed");
    return;
  }

  int currentLine = 0;
  while (src.available() && currentLine < linesToSkip) {
    if (src.read() == '\n') currentLine++;
  }

  while (src.available()) {
    tmp.write(src.read());
  }
  
  src.close();
  tmp.close();
  SD.remove(LOG_FILE);
  SD.rename("/tmp.log", LOG_FILE);

  char msg[64];
  snprintf(msg, sizeof(msg), "Log pruned: %d -> %d lines", lineCount, linesToKeep);
  logMessage(msg);
}

// Validate sensor readings
bool validateReading(float temp, float hum, float press, float gasRes) {
  if (isnan(temp) || temp < -40 || temp > 85) return false;
  if (isnan(hum) || hum < 0 || hum > 100) return false;
  if (isnan(press) || press < 300 || press > 1100) return false;
  if (isnan(gasRes) || gasRes < 0) return false;
  return true;
}

// Initialize sensor with retry
bool initSensor() {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (bme.begin()) {
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150);
      
      logMessage("BME680 sensor initialized");
      return true;
    }
    delay(1000);
  }
  logMessage("BME680 initialization failed");
  return false;
}

// Upload with retry logic
bool uploadToAdafruitIO() {
  for (int attempt = 0; attempt < MAX_UPLOAD_RETRIES; attempt++) {
    if (io.status() < AIO_CONNECTED) {
      logMessage("Reconnecting to Adafruit IO");
      io.connect();
      delay(2000);
      continue;
    }

    bool success = true;
    success &= temperature->save(t);
    success &= humidity->save(h);
    success &= pressure->save(p);
    success &= gas->save(g);
    success &= windSpeed->save(wind_kmh);
    success &= rainGauge->save(rain_mm);
    success &= windDir->save(wind_deg);
    success &= windDirText->save(wind_direction);

    if (success) {
      logMessage("Uploaded all readings to Adafruit IO");
      return true;
    }
    
    delay(1000);
  }
  
  logMessage("Upload failed after retries");
  return false;
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Weather Station with SparkFun Meter Kit ===");

  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (SD.begin(SD_CS)) {
    sdCardAvailable = true;
    logMessage("SD card initialized");
  } else {
    Serial.println("SD card failed - continuing without logging");
  }

  // Connect to Adafruit IO
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  
  int connectAttempts = 0;
  while (io.status() < AIO_CONNECTED && connectAttempts < 30) {
    Serial.print(".");
    delay(500);
    connectAttempts++;
  }
  
  if (io.status() >= AIO_CONNECTED) {
    Serial.println("\nConnected to Adafruit IO!");
    logMessage("Connected to Adafruit IO");
    
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    logMessage("NTP time sync configured");
  } else {
    Serial.println("\nFailed to connect - will retry in loop");
    logMessage("Initial connection failed");
  }

  // Initialize BME680
  sensorAvailable = initSensor();
  if (!sensorAvailable) {
    logMessage("WARN: Starting without BME680 - will retry");
  }

  // Setup wind sensor (anemometer)
  pinMode(windPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(windPin), onWind, FALLING);
  logMessage("Anemometer initialized (pin 4)");

  // Setup rain gauge
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), onRain, FALLING);
  logMessage("Rain gauge initialized (pin 27)");

  // Setup wind vane (analog direction sensor)
  pinMode(windDirPin, INPUT);
  logMessage("Wind vane initialized (pin 34)");

  if (sdCardAvailable) {
    pruneLog();
  }
  
  logMessage("=== System startup complete ===");
  logMessage("Wind speed in km/h, Rain in mm");
}

// ----------------- Main loop -----------------
void loop() {
  io.run();
  
  unsigned long now = millis();

  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;

    // Retry sensor if needed
    if (!sensorAvailable) {
      sensorAvailable = initSensor();
      if (!sensorAvailable) return;
    }

    // Read BME680
    if (!bme.performReading()) {
      logMessage("Sensor read failed");
      sensorAvailable = false;
      return;
    }

    t = bme.temperature;
    h = bme.humidity;
    p = bme.pressure / 100.0;
    g = bme.gas_resistance / 1000.0;

    // Validate BME680 readings
    if (!validateReading(t, h, p, g)) {
      logMessage("Invalid sensor readings - skipping");
      return;
    }

    if (gasWarmupCounter < SENSOR_WARMUP_READINGS) {
      gasWarmupCounter++;
      char msg[64];
      snprintf(msg, sizeof(msg), "Gas warmup %d/%d", gasWarmupCounter, SENSOR_WARMUP_READINGS);
      logMessage(msg);
    }

    // Read wind and rain with interrupt safety
    noInterrupts();
    unsigned long windTicks = windCount;
    windCount = 0;
    unsigned long rainTicks = rainCount;
    rainCount = 0;
    interrupts();

    // Calculate wind speed in km/h
    // SparkFun spec: 1.492 MPH = 1 pulse/sec = 2.4 km/h per pulse/sec
    float intervalSeconds = SENSOR_INTERVAL_MS / 1000.0;
    wind_kmh = (windTicks / intervalSeconds) * WIND_FACTOR;
    
    // Calculate rain in mm
    // SparkFun spec: 0.011" per tip = 0.2794mm per tip
    rain_mm = rainTicks * RAIN_PER_TIP;
    
    // Read wind direction
    int windDirAnalog = analogRead(windDirPin);
    WindDirData windData = getWindDirection(windDirAnalog);
    wind_deg = windData.degrees;
    wind_direction = String(windData.direction);

    // Log all readings
    char msg[256];
snprintf(msg, sizeof(msg), 
         "T=%.2fC H=%.2f%% P=%.2f hPa G=%.2f kOhm | Wind=%.2f km/h Dir=%s (%.1f deg) Rain=%.2f mm",
         t, h, p, g, wind_kmh, wind_direction.c_str(), wind_deg, rain_mm);
    logMessage(msg);

    // Upload to cloud
    uploadToAdafruitIO();

    // Periodic maintenance
    if (sdCardAvailable) {
      pruneLog();
    }
  }

  delay(10);
}
