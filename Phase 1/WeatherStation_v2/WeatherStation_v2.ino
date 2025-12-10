/*
 Simplified wind-direction to 8-point compass (N, NE, E, SE, S, SW, W, NW)
 Uses ADC ranges you measured to map ADC -> coarse direction (degrees = center of sector)
 Averaging still used (vector average) but final output is one of 8 directions.
*/

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
#define SD_CS 26
#define LOG_FILE "/system.log"
#define LOG_MAX_LINES 5000
#define SENSOR_INTERVAL_MS 30000       // 30 seconds
#define PRUNE_CHECK_INTERVAL 3600000   // 1 hour
#define MAX_UPLOAD_RETRIES 3
#define SENSOR_WARMUP_READINGS 0

// Lightning Detector Pins
#define AS3935_CS 5
#define AS3935_INT 27

// SparkFun Weather Meter Kit specifications
#define WIND_FACTOR 2.4
#define RAIN_PER_TIP 0.2794
#define DEBOUNCE_MS 10
#define WIND_DIR_SAMPLES 10

// Wind direction buffer for non-blocking averaging
#define WIND_DIR_BUFFER_SIZE 10

// Gust calculation config
#define WIND_GUST_WINDOW_MS 2000      // Gust window (2 seconds)

// --- Wind direction calibration / deadzone ---
#define WIND_DIR_ROTATION_OFFSET 112.5f
#define WIND_DIR_DEADZONE_DEG    5.0f

// --- Adafruit IO ---
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

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
AdafruitIO_Feed *windGust      = io.feed("WindGust");

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
// Use a fixed C-string to avoid dynamic allocation
char wind_direction[8] = "N";

// Wind/rain hardware pins
const int windPin = 4;
const int rainPin = 13;
const int windDirPin = 34;

// --- Interrupt variables ---
volatile unsigned long windCount = 0;
volatile unsigned long rainCount = 0;
volatile unsigned long lastWindTime = 0;
volatile unsigned long lastRainTime = 0;

// --- 24h rain tracking ---
volatile unsigned long rainCount24h = 0;
time_t lastRainReset = 0;

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
    gustWindowCount++;
    lastWindTime = now;
  }
}

void IRAM_ATTR onRain() {
  unsigned long now = millis();
  if (now - lastRainTime > DEBOUNCE_MS) {
    rainCount++;
    rainCount24h++;
    lastRainTime = now;
  }
}

// --- Logging ---
void logMessage(const char *msg) {
  char prefix[64];
  char fullMsg[256];

  if (WiFi.status() == WL_CONNECTED) {
    time_t now;
    time(&now);
    struct tm *tinfo = localtime(&now);
    if (tinfo) {
      snprintf(prefix, sizeof(prefix), "[%04d-%02d-%02d %02d:%02d:%02d]",
               tinfo->tm_year + 1900, tinfo->tm_mon + 1, tinfo->tm_mday,
               tinfo->tm_hour, tinfo->tm_min, tinfo->tm_sec);
    } else {
      snprintf(prefix, sizeof(prefix), "[%lu ms]", millis());
    }
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
// Simplified mapping to 8 compass points based on the ranges you measured.
// Returns center angle for that direction plus the 8-point name.
WindDirData getWindDirectionFromADC(int adc) {
  // Bound the ADC (esp32 typically 0-4095)
  if (adc < 0) adc = 0;
  if (adc > 4095) adc = 4095;

  // Thresholds chosen from your reported clusters:
  // W ~ 200, NW ~ 510, N ~ 850, SW ~ 1360, NE ~ 1850,
  // S ~ 2290, SE ~ 2590, E ~ 2770
  if (adc < 300)           return {270.0f, "W"};   // ~200
  else if (adc < 650)      return {315.0f, "NW"};  // ~510
  else if (adc < 1100)     return {0.0f,   "N"};   // ~850
  else if (adc < 1600)     return {225.0f, "SW"};  // ~1360
  else if (adc < 2050)     return {45.0f,  "NE"};  // ~1850
  else if (adc < 2450)     return {180.0f, "S"};   // ~2290
  else if (adc < 2700)     return {135.0f, "SE"};  // ~2590
  else if (adc < 3000)     return {90.0f,  "E"};   // ~2770
  else                     return {270.0f, "W"};   // fallback
}

// --- Non-blocking Wind Direction Buffer ---
void updateWindDirBuffer() {
  int val = analogRead(windDirPin);
  windDirBuffer[windDirIndex++] = val;
  if (windDirIndex >= WIND_DIR_BUFFER_SIZE) {
    windDirBufferReady = true;
    windDirIndex = 0;
  }
}

// --- Averaging with rotation + deadzone ---
// Vector-average the directions (using degrees returned by ADC mapping), apply rotation offset,
// then coarse-map to 8-point compass: N, NE, E, SE, S, SW, W, NW.
WindDirData getAveragedWindDirection() {
  float sumX = 0.0f, sumY = 0.0f;
  for (int i = 0; i < WIND_DIR_BUFFER_SIZE; i++) {
    WindDirData wd = getWindDirectionFromADC(windDirBuffer[i]);
    float radians = wd.degrees * PI / 180.0f;
    sumX += cosf(radians);
    sumY += sinf(radians);
  }

  float avgRadians = atan2f(sumY, sumX);
  float avgDegrees = avgRadians * 180.0f / PI;
  if (avgDegrees < 0.0f) avgDegrees += 360.0f;

  // Apply your rotation offset
  avgDegrees -= WIND_DIR_ROTATION_OFFSET;
  if (avgDegrees < 0.0f) avgDegrees += 360.0f;
  if (avgDegrees >= 360.0f) avgDegrees -= 360.0f;

  // Deadzone/hysteresis: keep last if small change
  static float lastReportedDeg = -1.0f;
  if (lastReportedDeg < 0.0f) lastReportedDeg = avgDegrees;
  else {
    float diff = fabsf(avgDegrees - lastReportedDeg);
    if (diff > 180.0f) diff = 360.0f - diff;
    if (diff < WIND_DIR_DEADZONE_DEG) avgDegrees = lastReportedDeg;
    else lastReportedDeg = avgDegrees;
  }

  // Map avgDegrees to 8 sectors (each sector center at multiples of 45°)
  // Boundaries are at +/-22.5° around each center.
  if (avgDegrees < 22.5f || avgDegrees >= 337.5f) return {0.0f,   "N"};
  if (avgDegrees < 67.5f)  return {45.0f,  "NE"};
  if (avgDegrees < 112.5f) return {90.0f,  "E"};
  if (avgDegrees < 157.5f) return {135.0f, "SE"};
  if (avgDegrees < 202.5f) return {180.0f, "S"};
  if (avgDegrees < 247.5f) return {225.0f, "SW"};
  if (avgDegrees < 292.5f) return {270.0f, "W"};
  return {315.0f, "NW"};
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
      // let background tasks run
      unsigned long start = millis();
      while (millis() - start < 2000) {
        io.run();
        delay(10);
      }
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
    // convert C-string to String temporarily for AdafruitIO
    success &= windDirText->save(String(wind_direction));
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
    delay(2);
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

  if (gasWarmupCounter < SENSOR_WARMUP_READINGS) {
    gasWarmupCounter++;
    char msg[64];
    snprintf(msg, sizeof(msg), "Gas sensor warmup %d/%d", gasWarmupCounter, SENSOR_WARMUP_READINGS);
    logMessage(msg);
    return;
  }

  // --- Reset 24h rain at midnight ---
  time_t nowTime;
  time(&nowTime);
  if (lastRainReset == 0) lastRainReset = nowTime;
  struct tm *tmNow = localtime(&nowTime);
  struct tm *tmLastReset = localtime(&lastRainReset);
  if (tmNow && tmLastReset && tmNow->tm_mday != tmLastReset->tm_mday) {
    rainCount24h = 0;
    lastRainReset = nowTime;
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
  float intervalRainMm = rainTicks * RAIN_PER_TIP;
  (void)intervalRainMm; // currently not used per-interval
  rain_mm = rainCount24h * RAIN_PER_TIP;

  if (windDirBufferReady) {
    WindDirData wd = getAveragedWindDirection();
    wind_deg = wd.degrees;
    // safe copy to C-string
    strncpy(wind_direction, wd.direction, sizeof(wind_direction) - 1);
    wind_direction[sizeof(wind_direction) - 1] = '\0';
    windDirBufferReady = false;
  }

  char msg[256];
  snprintf(msg, sizeof(msg),
    "T=%.2f°C H=%.2f%% P=%.2fhPa G=%.2f kohms Wind=%.2f km/h Dir=%s Rain24h=%.2f mm Gust=%.2f km/h",
    t, h, p, g, wind_kmh, wind_direction, rain_mm, gust_kmh);
  logMessage(msg);

  uploadToAdafruitIO();

  gust_kmh = 0;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  logMessage("System booting...");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  logMessage("Connecting to WiFi...");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    // NTP time sync for accurate timestamps
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    unsigned long start = millis();
    // wait up to ~10 seconds for time to sync (time() > 1 Jan 2017 as a heuristic)
    while (time(nullptr) < 1600000000 && millis() - start < 10000) {
      delay(200);
    }
  }

  logIPAddress();

  if (!SD.begin(SD_CS)) {
    sdCardAvailable = false;
    logMessage("SD card not detected");
  } else {
    sdCardAvailable = true;
    logMessage("SD card initialized");
  }

  initSensor();

  pinMode(windPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(windPin), onWind, RISING);

  pinMode(rainPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainPin), onRain, RISING);

  io.connect();

  // Pre-fill wind direction buffer so averaging has valid data on first run
  for (int i = 0; i < WIND_DIR_BUFFER_SIZE; i++) {
    windDirBuffer[i] = analogRead(windDirPin);
  }
  windDirBufferReady = true;

  logMessage("Setup complete");
}

// --- Main loop ---
void loop() {
  io.run();
  updateWindDirBuffer();
  updateWindGust();
  handleLightningEvents();

  if (millis() - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = millis();
    readAndUploadSensors();
    pruneLog();
  }
}
