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
#define PRUNE_CHECK_INTERVAL 3600000  // Check log size once per hour
#define MAX_UPLOAD_RETRIES 3
#define SENSOR_WARMUP_READINGS 5      // Discard first N gas readings

// ----------------- Adafruit IO -----------------
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
Adafruit_BME680 bme;

// Feeds for dashboard
AdafruitIO_Feed *temperature = io.feed("Temperature");
AdafruitIO_Feed *humidity    = io.feed("Humidity");
AdafruitIO_Feed *pressure    = io.feed("Pressure");
AdafruitIO_Feed *gas         = io.feed("Gas");

// ----------------- State variables -----------------
bool sdCardAvailable = false;
bool sensorAvailable = false;
unsigned long lastSensorTime = 0;
unsigned long lastPruneCheck = 0;
int gasWarmupCounter = 0;

// Latest sensor values
float t = 0, h = 0, p = 0, g = 0;

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

  // Write to SD card if available
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

  // Always write to serial
  Serial.println(fullMsg);
}

// Improved log pruning - only checks periodically
void pruneLog() {
  if (!sdCardAvailable) return;
  if (millis() - lastPruneCheck < PRUNE_CHECK_INTERVAL) return;
  
  lastPruneCheck = millis();

  File file = SD.open(LOG_FILE, FILE_READ);
  if (!file) return;

  // Count lines efficiently
  int lineCount = 0;
  while (file.available()) {
    if (file.read() == '\n') lineCount++;
  }
  file.close();

  if (lineCount <= LOG_MAX_LINES) {
    logMessage("Log size check: within limits");
    return;
  }

  // Keep last 60% of lines to avoid frequent pruning
  int linesToKeep = (LOG_MAX_LINES * 6) / 10;
  int linesToSkip = lineCount - linesToKeep;

  File src = SD.open(LOG_FILE, FILE_READ);
  File tmp = SD.open("/tmp.log", FILE_WRITE);
  
  if (!src || !tmp) {
    if (src) src.close();
    if (tmp) tmp.close();
    logMessage("Log pruning failed - file error");
    return;
  }

  // Skip lines efficiently
  int currentLine = 0;
  while (src.available() && currentLine < linesToSkip) {
    if (src.read() == '\n') currentLine++;
  }

  // Copy remaining lines
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
      
      logMessage("BME680 sensor initialized successfully");
      return true;
    }
    delay(1000);
  }
  logMessage("BME680 initialization failed after 3 attempts");
  return false;
}

// Upload with retry logic
bool uploadToAdafruitIO() {
  for (int attempt = 0; attempt < MAX_UPLOAD_RETRIES; attempt++) {
    if (io.status() < AIO_CONNECTED) {
      logMessage("Adafruit IO disconnected - attempting reconnection");
      io.connect();
      delay(2000);
      continue;
    }

    bool success = true;
    success &= temperature->save(t);
    success &= humidity->save(h);
    success &= pressure->save(p);
    success &= gas->save(g);

    if (success) {
      logMessage("Uploaded readings to Adafruit IO");
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
  Serial.println("\n=== Weather Station Booting ===");

  // Initialize SD card
  Serial.println("Initializing SD card...");
  if (SD.begin(SD_CS)) {
    sdCardAvailable = true;
    logMessage("SD card initialized successfully");
  } else {
    Serial.println("SD card initialization failed - continuing without SD logging");
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
    
    // Configure NTP for accurate timestamps
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    logMessage("NTP time sync configured");
  } else {
    Serial.println("\nFailed to connect to Adafruit IO - will retry in loop");
    logMessage("Initial Adafruit IO connection failed");
  }

  // Initialize BME680 sensor
  sensorAvailable = initSensor();
  
  if (!sensorAvailable) {
    logMessage("WARN: Starting without sensor - will retry");
  }

  if (sdCardAvailable) {
    pruneLog();
  }
  
  logMessage("=== System startup complete ===");
}

// ----------------- Main loop -----------------
void loop() {
  io.run(); // Maintain Adafruit IO connection - call frequently
  
  unsigned long now = millis();

  // --- Sensor reading & upload every 15 seconds ---
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;

    // Retry sensor initialization if it failed
    if (!sensorAvailable) {
      sensorAvailable = initSensor();
      if (!sensorAvailable) return;
    }

    // Perform sensor reading
    if (!bme.performReading()) {
      logMessage("Sensor reading failed - will retry next cycle");
      sensorAvailable = false;
      return;
    }

    // Get readings
    t = bme.temperature;
    h = bme.humidity;
    p = bme.pressure / 100.0;
    g = bme.gas_resistance / 1000.0;

    // Validate readings
    if (!validateReading(t, h, p, g)) {
      logMessage("Invalid sensor readings detected - skipping");
      return;
    }

    // Discard initial gas readings during warmup
    if (gasWarmupCounter < SENSOR_WARMUP_READINGS) {
      gasWarmupCounter++;
      char msg[64];
      snprintf(msg, sizeof(msg), "Gas sensor warmup %d/%d", gasWarmupCounter, SENSOR_WARMUP_READINGS);
      logMessage(msg);
    }

    // Log readings
    char msg[128];
    snprintf(msg, sizeof(msg), "Readings: T=%.2f°C H=%.2f%% P=%.2f hPa G=%.2f kΩ", t, h, p, g);
    logMessage(msg);

    // Upload to Adafruit IO
    uploadToAdafruitIO();

    // Periodic log pruning check
    if (sdCardAvailable) {
      pruneLog();
    }
  }

  // Small delay to prevent tight looping
  delay(10);
}
