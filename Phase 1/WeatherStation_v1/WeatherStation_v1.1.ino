#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "AdafruitIO_WiFi.h"
#include "secrets.h"
#include <Wire.h>

// Reference pressure at sea level (used for relative readings)
#define SEALEVELPRESSURE_HPA 1013.25

// Set up Adafruit IO connection using creds from secrets.h
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

// BME680 handles temp, humidity, pressure, and gas
Adafruit_BME680 bme;

// Feeds for the dashboard (names must match your Adafruit IO feeds)
AdafruitIO_Feed *temperature = io.feed("Temperature");
AdafruitIO_Feed *humidity    = io.feed("Humidity");
AdafruitIO_Feed *pressure    = io.feed("Pressure");
AdafruitIO_Feed *gas         = io.feed("Gas");

void setup() {
  Serial.begin(115200);
  Serial.println("Booting weather station...");

  // Connect to Adafruit IO
  io.connect();
  Serial.print("Connecting to Adafruit IO");
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected!");

  // Initialize BME680 sensor
  if (!bme.begin()) {
    Serial.println("BME680 not found – check wiring or I2C address.");
    while (1);
  }

  // Sensor configuration for better stability
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // Heater temp in °C, duration in ms
}

void loop() {
  io.run(); // Keeps the Adafruit IO connection alive

  // Try reading the sensor
  if (!bme.performReading()) {
    Serial.println("Sensor read failed.");
    return;
  }

  // Grab sensor data
  float t = bme.temperature;             // °C
  float h = bme.humidity;                // %
  float p = bme.pressure / 100.0;        // hPa
  float g = bme.gas_resistance / 1000.0; // kΩ

  // Show it locally
  Serial.printf("Temp: %.2f°C | Hum: %.2f%% | Press: %.2f hPa | Gas: %.2f kΩ\n", t, h, p, g);

  // Send to dashboard
  temperature->save(t);
  humidity->save(h);
  pressure->save(p);
  gas->save(g);

  // Update interval – 10 seconds
  delay(10000);
}
