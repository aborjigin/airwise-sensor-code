/*
 * AirWise Sensor Test
 * 
 * Board: SparkFun RedBoard Turbo (SAMD21)
 * 
 * Sensors:
 * - BMV080: Air Quality PM1, PM2.5, PM10 Sensor (Qwiic/I2C)
 * - INA3221: Triple-Channel Power Monitor (I2C)
 * - BME680: Temperature, Humidity, Pressure, Gas Sensor (I2C)
 * 
 * This sketch reads all sensors and outputs a formatted data packet over Serial
 */

#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h"
#include <Adafruit_INA3221.h>
#include <Adafruit_BME680.h>

// Sensor objects
BMV080 bmv080;
Adafruit_INA3221 ina3221;
Adafruit_BME680 bme680;

// Sensor status flags
bool bmv080_available = false;
bool ina3221_available = false;
bool bme680_available = false;

// Timing
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 2000; // Change Read Interval here


// Data structure for sensor packet
struct SensorPacket {
  unsigned long timestamp;
  
  // BMV080 - Air Quality
  float pm1_0;
  float pm2_5;
  float pm10;
  
  // INA3221 - Power monitoring (3 channels)
  float voltage_ch1;
  float current_ch1;
  float voltage_ch2;
  float current_ch2;
  float voltage_ch3;
  float current_ch3;
  
  // BME680 - Environmental
  float temperature;
  float humidity;
  float pressure;
  float gas_resistance;
  float altitude;
};

SensorPacket packet;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial to be ready
  }
  
  delay(2000); // Give user time to open Serial Monitor
  
  Serial.println("=== AirWise Sensor Test ===");
  Serial.println("Board: SparkFun RedBoard Turbo");
  Serial.println();
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize BMV080
  Serial.print("Initializing BMV080... ");
  if (bmv080.begin()) {
    bmv080_available = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }
  
  // Initialize INA3221
  Serial.print("Initializing INA3221... ");
  if (ina3221.begin()) {
    ina3221_available = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }
  
  // Initialize BME680
  Serial.print("Initializing BME680... ");
  if (bme680.begin()) {
    bme680_available = true;
    
    // Configure BME680 oversampling and filter
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320°C for 150 ms, CHECK HERE IF GOOD
    
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }
  
  Serial.println();
  Serial.println("Setup complete. Starting sensor readings...");
  Serial.println();
  
  // Print header
  printHeader();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Read all sensors
    readSensors();
    
    // Print data packet
    printPacket();
  }
}

void readSensors() {
  packet.timestamp = millis();
  
  // Read BMV080 - Air Quality Sensor
  if (bmv080_available) {
    if (bmv080.dataAvailable()) {
      packet.pm1_0 = bmv080.getPM1_0();
      packet.pm2_5 = bmv080.getPM2_5();
      packet.pm10 = bmv080.getPM10();
    }
  } else {
    packet.pm1_0 = -1;
    packet.pm2_5 = -1;
    packet.pm10 = -1;
  }
  
  // Read INA3221 - Power Monitor
  if (ina3221_available) {
    packet.voltage_ch1 = ina3221.getBusVoltage_V(1);
    packet.current_ch1 = ina3221.getCurrent_mA(1);
    packet.voltage_ch2 = ina3221.getBusVoltage_V(2);
    packet.current_ch2 = ina3221.getCurrent_mA(2);
    packet.voltage_ch3 = ina3221.getBusVoltage_V(3);
    packet.current_ch3 = ina3221.getCurrent_mA(3);
  } else {
    packet.voltage_ch1 = -1;
    packet.current_ch1 = -1;
    packet.voltage_ch2 = -1;
    packet.current_ch2 = -1;
    packet.voltage_ch3 = -1;
    packet.current_ch3 = -1;
  }
  
  // Read BME680 - Environmental Sensor
  if (bme680_available) {
    if (bme680.performReading()) {
      packet.temperature = bme680.temperature;
      packet.humidity = bme680.humidity;
      packet.pressure = bme680.pressure / 100.0; // Convert Pa to hPa
      packet.gas_resistance = bme680.gas_resistance / 1000.0; // Convert to KOhms
      packet.altitude = bme680.readAltitude(1013.25); // Sea level pressure in hPa
    }
  } else {
    packet.temperature = -999;
    packet.humidity = -1;
    packet.pressure = -1;
    packet.gas_resistance = -1;
    packet.altitude = -999;
  }
}

void printHeader() {
  Serial.println("--- Sensor Data Header ---");
  Serial.println("Format: CSV");
  Serial.println("Timestamp(ms),PM1.0(ug/m3),PM2.5(ug/m3),PM10(ug/m3),V_CH1(V),I_CH1(mA),V_CH2(V),I_CH2(mA),V_CH3(V),I_CH3(mA),Temp(C),Humidity(%),Pressure(hPa),GasRes(KOhm),Altitude(m)");
  Serial.println();
}

void printPacket() {
  // CSV format for easy parsing
  Serial.print(packet.timestamp); Serial.print(",");
  
  // BMV080
  Serial.print(packet.pm1_0, 2); Serial.print(",");
  Serial.print(packet.pm2_5, 2); Serial.print(",");
  Serial.print(packet.pm10, 2); Serial.print(",");
  
  // INA3221
  Serial.print(packet.voltage_ch1, 3); Serial.print(",");
  Serial.print(packet.current_ch1, 2); Serial.print(",");
  Serial.print(packet.voltage_ch2, 3); Serial.print(",");
  Serial.print(packet.current_ch2, 2); Serial.print(",");
  Serial.print(packet.voltage_ch3, 3); Serial.print(",");
  Serial.print(packet.current_ch3, 2); Serial.print(",");
  
  // BME680
  Serial.print(packet.temperature, 2); Serial.print(",");
  Serial.print(packet.humidity, 2); Serial.print(",");
  Serial.print(packet.pressure, 2); Serial.print(",");
  Serial.print(packet.gas_resistance, 2); Serial.print(",");
  Serial.print(packet.altitude, 2);
  
  Serial.println();
}
