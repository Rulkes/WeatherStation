#include "AdafruitIO_WiFi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// ===== Include secrets (API keys & WiFi credentials) =====
#include "secrets.h" // contains IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS

// Create Adafruit IO instance
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

// Create feeds for each sensor
AdafruitIO_Feed *tempFeed     = io.feed("temperature");
AdafruitIO_Feed *pressureFeed = io.feed("pressure");
AdafruitIO_Feed *humidityFeed = io.feed("humidity");
AdafruitIO_Feed *gasFeed      = io.feed("gas");

// BME680 sensor setup
Adafruit_BME680 bme; // default I2C address 0x76

void setup() {
  Serial.begin(115200);
  
  // Connect to WiFi + Adafruit IO
  Serial.print("Connecting to Adafruit IO");
  io.connect();
  
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to Adafruit IO!");
  
  // Initialize BME680
  if(!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while(1);
  }
  
  // Set up oversampling and filter (optional)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
}

void loop() {
  io.run(); // keep connection alive

  // Perform a reading
  if(!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Read values
  float temperature = bme.temperature; // °C
  float pressure    = bme.pressure / 100.0; // hPa
  float humidity    = bme.humidity;    // %
  float gas         = bme.gas_resistance / 1000.0; // KOhms

  // Print to Serial
  Serial.print("Temp: "); Serial.print(temperature); Serial.print(" *C, ");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa, ");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.print(" %, ");
  Serial.print("Gas: "); Serial.print(gas); Serial.println(" KOhms");

  // Push to Adafruit IO
  tempFeed->save(temperature);
  pressureFeed->save(pressure);
  humidityFeed->save(humidity);
  gasFeed->save(gas);

  delay(10000); // wait 10 seconds
}
