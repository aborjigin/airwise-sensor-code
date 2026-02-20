# AirWise Sensor Code

Arduino project for testing air quality and environmental sensors with the SparkFun RedBoard Turbo.

## Hardware

### Board
- **SparkFun RedBoard Turbo** (SAMD21 based)
  - Product Link: [DigiKey 14812](https://www.digikey.com/en/products/detail/sparkfun-electronics/14812/9866252)
  - Microcontroller: ATSAMD21G18
  - Operating Voltage: 3.3V
  - I/O Pins: 3.3V logic

### Sensors

1. **BMV080** - Air Quality Sensor (PM1, PM2.5, PM10)
   - Product: [SparkFun Air Quality Sensor](https://www.sparkfun.com/sparkfun-air-quality-pm1-pm2-5-pm10-sensor-bmv080-qwiic.html)
   - Interface: I2C (Qwiic)
   - Library: [SparkFun BMV080 Arduino Library](https://github.com/sparkfun/SparkFun_BMV080_Arduino_Library)

2. **INA3221** - Triple-Channel Power Monitor
   - Product: [Adafruit INA3221](https://www.adafruit.com/product/6062)
   - Interface: I2C
   - Library: [Adafruit INA3221 Library](https://github.com/adafruit/Adafruit_INA3221)

3. **BME680** - Environmental Sensor (Temperature, Humidity, Pressure, Gas)
   - Datasheet: [Bosch BME680](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf)
   - Interface: I2C
   - Library: Adafruit BME680 Library

## Installation

### Arduino IDE Setup

1. **Install Arduino IDE** 
2. **Add SparkFun Board Support**
   - Open Arduino IDE
   - Go to `File` → `Preferences`
   - Add this URL to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
     ```
   - Go to `Tools` → `Board` → `Boards Manager`
   - Search for "SparkFun SAMD Boards"
   - Install the package

3. **Install Required Libraries**

### Data Format

The output is CSV formatted with the following columns:

```
Timestamp(ms), PM1.0(ug/m3), PM2.5(ug/m3), PM10(ug/m3), 
V_CH1(V), I_CH1(mA), V_CH2(V), I_CH2(mA), V_CH3(V), I_CH3(mA), 
Temp(C), Humidity(%), Pressure(hPa), GasRes(KOhm), Altitude(m)
```

Example output:
```
2000,12.50,15.30,18.20,5.000,125.50,3.300,45.20,12.000,200.00,22.50,45.30,1013.25,150.45,10.25
```

Example using screen on Mac/Linux:
```bash
screen -L /dev/ttyACM0 115200
```

