# Multi-Metal-Detector-and-Classification-System
## Features

- Ability to detect ferromagnetic, non-ferromagnetic, copper, brass, and gold metals.
- Display the detected metal type on the LCD screen.
- Provide audible notifications.
- Send alarms via email.
- Illuminate different LED lights for different metal types.

## Required Components

- ESP32 Microcontroller
- Magnetometer (DFRobot BMX160)
- Hall Sensor
- LCD Screen (LiquidCrystal I2C)
- Audio Output (I2S module)
- Buzzer
- LEDs (Red, Green, Yellow)
- RTC Module (DS3231)
- WiFi connection for SMTP

## Libraries

To run the project, the following libraries need to be installed in the Arduino IDE:

- `Wire.h`
- `WiFi.h`
- `ESP_Mail_Client.h`
- `LiquidCrystal_I2C.h`
- `ESP8266SAM.h`
- `AudioOutputI2S.h`
- `DFRobot_BMX160.h`
- `RTClib.h`
