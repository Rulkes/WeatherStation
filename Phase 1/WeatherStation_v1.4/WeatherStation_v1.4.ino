#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "AdafruitIO_WiFi.h"
#include "secrets.h"
#include <Wire.h>
#include <SPI.h>
#include "SparkFun_AS3935.h"
#include <SD.h>
#include <time.h>
#include <WiFi.h>

// --- Build Info ---
#define BUILD_VERSION "1.0"

// --- CONFIGURATION ---
#define SEALEVELPRESSURE_HPA 1013.25
#define SD_CS 5
#define LOG_FILE "/system.log"
#define LOG_MAX_LINES 5000
#define SENSOR_INTERVAL_MS 30000       // 30 seconds
#define PRUNE_CHECK_INTERVAL 3600000   // 1 hour
#define MAX_UPLOAD_RETRIES 3
#define SENSOR_WARMUP_READINGS 5

// Lightning Detector Pins
#define AS3935_CS 15
#define AS3935_INT 19

// SparkFun Weather Meter Kit specifications
#define WIND_FACTOR 2.4
#define RAIN_PER_TIP 0.2794
#define DEBOUNCE_MS 10
#define WIND_DIR_SAMPLES 10

// Wind direction buffer for non-blocking averaging
#define WIND_DIR_BUFFER_SIZE 10

// Gust calculation config
#define WIND_GUST_WINDOW_MS 2000      // Gust window (2 seconds)

// --- Adafruit IO ---
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS); // Place BEFORE feeds!

Adafruit_BME680 bme;
SparkFun_AS3935 lightning;

// --- Feeds ---
AdafruitIO_Feed *temperature   = io.feed("Temperature");
AdafruitIO_Feed *humidity      = io.feed("Humidity");
AdafruitIO_Feed *pressure      = io.feed("Pressure");
AdafruitIO_Feed *gas           = io.feed("Gas");
AdafruitIO_Feed *windSpeed     = io.feed("WindSpeed");
AdafruitIO_Feed *rainGauge     = io.feed("RainGauge");
AdafruitIO_Feed *windDir       = io.feed("WindDirection");
AdafruitIO_Feed *windDirText   = io.feed("WindDirectionText");
AdafruitIO_Feed *lightningFeed = io.feed("Lightning");
AdafruitIO_Feed *windGust      = io.feed("WindGust"); // Gust feed

// --- Wind direction struct ---
struct WindDirData {
  float degrees;
  const char* direction;
};

// --- State Variables ---
bool sdCardAvailable = false;
bool sensorAvailable = false;
bool lightningAvailable = false;
unsigned long lastSensorTime = 0;
unsigned long lastPruneCheck = 0;
int gasWarmupCounter = 0;

float t = 0, h = 0, p = 0, g = 0;
float wind_kmh = 0, rain_mm = 0, wind_deg = 0, gust_kmh = 0;
String wind_direction = "N";

// Wind/rain hardware pins
const int windPin = 4;
const int rainPin = 27;
const int windDirPin = 34;

// --- Interrupt variables ---
volatile unsigned long windCount = 0;
volatile unsigned long rainCount = 0;
volatile unsigned long lastWindTime = 0;
volatile unsigned long lastRainTime = 0;

// --- Wind Gust State ---
unsigned long gustWindowStart = 0;
volatile unsigned long gustWindowCount = 0; // Updated by interrupt

// --- Wind direction averaging buffer ---
int windDirBuffer[WIND_DIR_BUFFER_SIZE];
int windDirIndex = 0;
bool windDirBufferReady = false;

// --- Interrupt Handlers ---
void IRAM_ATTR onWind() {
  unsigned long now = millis();
  if (now - lastWindTime > DEBOUNCE_MS) {
    windCount++;
    gustWindowCount++; // For gust calculation
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

// --- Logging ---
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

void tryMountSDCard() {
  if (!sdCardAvailable) {
    if (SD.begin(SD_CS)) {
      sdCardAvailable = true;
      logMessage("SD card re-mounted after failure");
    }
  }
}

// --- Log Maintenance ---
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
    tryMountSDCard();
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

// --- Sensor Logic ---
bool validateReading(float temp, float hum, float press, float gasRes) {
  if (isnan(temp) || temp < -40 || temp > 85) return false;
  if (isnan(hum) || hum < 0 || hum > 100) return false;
  if (isnan(press) || press < 300 || press > 1100) return false;
  if (isnan(gasRes) || gasRes < 0) return false;
  return true;
}

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

// --- Wind Direction ADC Mapping ---
WindDirData getWindDirectionFromADC(int adc) {
  if (adc < 150) return {112.5, "ESE"};
  if (adc < 300) return {67.5, "ENE"};
  if (adc < 400) return {90.0, "E"};
  if (adc < 600) return {157.5, "SSE"};
  if (adc < 900) return {135.0, "SE"};
  if (adc < 1100) return {202.5, "SSW"};
  if (adc < 1500) return {180.0, "S"};
  if (adc < 1700) return {22.5, "NNE"};
  if (adc < 2000) return {45.0, "NE"};
  if (adc < 2400) return {247.5, "WSW"};
  if (adc < 2800) return {225.0, "SW"};
  if (adc < 3200) return {337.5, "NNW"};
  if (adc < 3600) return {0.0, "N"};
  if (adc < 3900) return {292.5, "WNW"};
  if (adc < 4000) return {315.0, "NW"};
  return {270.0, "W"};
}

// --- Non-blocking Wind Direction Buffer ---
void updateWindDirBuffer() {
  windDirBuffer[windDirIndex++] = analogRead(windDirPin);
  if (windDirIndex >= WIND_DIR_BUFFER_SIZE) {
    windDirBufferReady = true;
    windDirIndex = 0;
  }
}

WindDirData getAveragedWindDirection() {
  float sumX = 0, sumY = 0;
  for (int i = 0; i < WIND_DIR_BUFFER_SIZE; i++) {
    WindDirData wd = getWindDirectionFromADC(windDirBuffer[i]);
    float radians = wd.degrees * PI / 180.0;
    sumX += cos(radians);
    sumY += sin(radians);
  }
  float avgRadians = atan2(sumY, sumX);
  float avgDegrees = avgRadians * 180.0 / PI;
  if (avgDegrees < 0) avgDegrees += 360.0;

  if (avgDegrees < 11.25 || avgDegrees >= 348.75) return {avgDegrees, "N"};
  if (avgDegrees < 33.75) return {avgDegrees, "NNE"};
  if (avgDegrees < 56.25) return {avgDegrees, "NE"};
  if (avgDegrees < 78.75) return {avgDegrees, "ENE"};
  if (avgDegrees < 101.25) return {avgDegrees, "E"};
  if (avgDegrees < 123.75) return {avgDegrees, "ESE"};
  if (avgDegrees < 146.25) return {avgDegrees, "SE"};
  if (avgDegrees < 168.75) return {avgDegrees, "SSE"};
  if (avgDegrees < 191.25) return {avgDegrees, "S"};
  if (avgDegrees < 213.75) return {avgDegrees, "SSW"};
  if (avgDegrees < 236.25) return {avgDegrees, "SW"};
  if (avgDegrees < 258.75) return {avgDegrees, "WSW"};
  if (avgDegrees < 281.25) return {avgDegrees, "W"};
  if (avgDegrees < 303.75) return {avgDegrees, "WNW"};
  if (avgDegrees < 326.25) return {avgDegrees, "NW"};
  return {avgDegrees, "NNW"};
}

// --- Gust Tracking ---
void updateWindGust() {
  unsigned long now = millis();
  if (gustWindowStart == 0) gustWindowStart = now;
  if (now - gustWindowStart >= WIND_GUST_WINDOW_MS) {
    noInterrupts();
    unsigned long count = gustWindowCount;
    gustWindowCount = 0;
    gustWindowStart = now;
    interrupts();
    // For this window, get the speed
    float windowKmh = (count / (WIND_GUST_WINDOW_MS / 1000.0)) * WIND_FACTOR;
    if (windowKmh > gust_kmh) gust_kmh = windowKmh;
  }
}

// --- Upload Data ---
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
    success &= windGust->save(gust_kmh);
    if (success) {
      logMessage("Uploaded all readings (including gust) to Adafruit IO");
      return true;
    }
    delay(1000);
  }
  logMessage("Upload failed after retries");
  return false;
}

void logIPAddress() {
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    char ipMsg[64];
    snprintf(ipMsg, sizeof(ipMsg), "Device IP Address: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    logMessage(ipMsg);
  }
}

// --- Lightning Event Handler ---
void handleLightningEvents() {
  if (lightningAvailable && digitalRead(AS3935_INT) == HIGH) {
    delay(2); // Debounce
    byte event = lightning.readInterruptReg();

    if (event == 1) {
      logMessage("⚠️ Disturber detected (not lightning)");
    } else if (event == 2 || event == 3) {
      int dist = lightning.distanceToStorm();
      char msg[64];
      snprintf(msg, sizeof(msg), "⚡⚡⚡ LIGHTNING STRIKE at %d km ⚡⚡⚡", dist);
      logMessage(msg);

      if (io.status() >= AIO_CONNECTED) {
        lightningFeed->save(dist);
        logMessage("Lightning event uploaded to Adafruit IO");
      } else {
        logMessage("WARNING: Could not upload lightning - no connection");
      }
    }
  }
}

// --- Sensor Read & Upload ---
void readAndUploadSensors() {
  if (!sensorAvailable) {
    sensorAvailable = initSensor();
    if (!sensorAvailable) {
      tryMountSDCard();
      return;
    }
  }

  // Read BME680
  if (!bme.performReading()) {
    logMessage("BME680 read failed - will retry");
    sensorAvailable = false;
    tryMountSDCard();
    return;
  }

  t = bme.temperature;
  h = bme.humidity;
  p = bme.pressure / 100.0;
  g = bme.gas_resistance / 1000.0;

  if (!validateReading(t, h, p, g)) {
    logMessage("Invalid sensor readings - skipping");
    tryMountSDCard();
    return;
  }

  // Gas sensor warmup period
  if (gasWarmupCounter < SENSOR_WARMUP_READINGS) {
    gasWarmupCounter++;
    char msg[64];
    snprintf(msg, sizeof(msg), "Gas sensor warmup %d/%d", gasWarmupCounter, SENSOR_WARMUP_READINGS);
    logMessage(msg);
    return;
  }

  // Wind, rain
  noInterrupts();
  unsigned long windTicks = windCount;
  windCount = 0;
  unsigned long rainTicks = rainCount;
  rainCount = 0;
  interrupts();
  float intervalSeconds = SENSOR_INTERVAL_MS / 1000.0;
  wind_kmh = (windTicks / intervalSeconds) * WIND_FACTOR;
  rain_mm = rainTicks * RAIN_PER_TIP;

  // Non-blocking wind direction averaging
  if (windDirBufferReady) {
    WindDirData wd = getAveragedWindDirection();
    wind_deg = wd.degrees;
    wind_direction = String(wd.direction);
    windDirBufferReady = false;
  }

  // --- Gust value is accumulated by updateWindGust() and reset below ---

  // Log all readings
  char msg[256];
  snprintf(msg, sizeof(msg),
    "T=%.2f°C H=%.2f%% P=%.2f hPa G=%.2f kΩ | Wind=%.2f km/h Gust=%.2f km/h %.1f° (%s) Rain=%.2f mm",
    t, h, p, g, wind_kmh, gust_kmh, wind_deg, wind_direction.c_str(), rain_mm);
  logMessage(msg);

  uploadToAdafruitIO();

  gust_kmh = 0; // Reset gust for next interval

  if (sdCardAvailable) pruneLog();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Weather Station with Lightning Detector ===");
  logMessage("BUILD VERSION: " BUILD_VERSION);

  // SD Card
  Serial.println("Initializing SD card...");
  delay(100);
  if (SD.begin(SD_CS)) {
    sdCardAvailable = true;
    delay(50);
    logMessage("SD card initialized");
  } else {
    Serial.println("SD card failed - continuing without logging");
  }

  // Adafruit IO
  io.connect();
  Serial.print("Connecting to Adafruit IO");
  int connectAttempts = 0;
  while (io.status() < AIO_CONNECTED && connectAttempts < 30) {
    Serial.print(".");
    delay(500);
    connectAttempts++;
  }
  if (io.status() >= AIO_CONNECTED) {
    Serial.println("\nConnected to Adafruit IO!");
    logMessage("Connected to Adafruit IO");
    logIPAddress();
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    logMessage("NTP time sync configured");
  } else {
    Serial.println("\nFailed to connect");
    logMessage("Initial connection failed");
  }

  // BME680
  sensorAvailable = initSensor();

  // Wind and Rain
  pinMode(windPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(windPin), onWind, FALLING);
  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), onRain, FALLING);
  pinMode(windDirPin, INPUT);
  logMessage("Wind and rain sensors initialized");

  // Lightning Detector
  SPI.begin();
  if (!lightning.beginSPI(AS3935_CS, AS3935_INT)) {
    logMessage("Lightning sensor not found - check wiring");
    lightningAvailable = false;
  } else {
    lightning.calibrateOsc();
    lightning.setNoiseLevel(2);
    lightning.spikeRejection(2);
    pinMode(AS3935_INT, INPUT);
    lightningAvailable = true;
    logMessage("Lightning detector initialized and monitoring");
  }

  if (sdCardAvailable) pruneLog();
  logMessage("=== System startup complete ===");

  // Fill wind direction buffer to start
  for (int i = 0; i < WIND_DIR_BUFFER_SIZE; i++) {
    windDirBuffer[i] = analogRead(windDirPin);
    delay(20);
  }
  windDirBufferReady = true;
  gustWindowStart = millis();
}

// --- Main Loop ---
void loop() {
  io.run();
  updateWindDirBuffer();
  updateWindGust();
  handleLightningEvents();

  unsigned long now = millis();
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;
    readAndUploadSensors();
  }
  delay(10);
}
