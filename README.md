# GloveXR

## Overview
GloveXR is a motion-controlled VR input system using ESP32 microcontrollers with motion sensors. The system consists of two ESP32 devices: one functioning as a mouse controller (right hand) and the other as a WASD keyboard controller (left hand).

## Components
- 2× ESP32 microcontrollers
- 1× BNO055 9-axis sensor (right hand)
- 1× MPU6050 6-axis sensor (left hand)
- 2× Calibration buttons
- 1× Hall effect sensor (for mouse clicks)

## Features
- Motion-based mouse control with the right hand
- WASD keyboard control with the left hand
- Calibration system for both controllers
- Left-click functionality using hall effect sensor
- Bluetooth connectivity to PC/mobile devices

## Dependencies
To successfully compile and run the GloveXR project, ensure you have the following libraries installed in your Arduino environment:

- **BleCombo**: Enables Bluetooth keyboard and mouse functionality for ESP32.
- **Wire**: Facilitates I2C communication between the ESP32 and the sensors.
- **MPU6050**: Enables communication with the MPU6050 6-axis sensor.
- **math.h**: Provides mathematical functions for calculations.
- **Adafruit Sensor**: Provides common sensor functions used by Adafruit libraries.
- **Adafruit BNO055**: Provides support for the BNO055 9-axis sensor.
- **BleMouse**: Enables Bluetooth mouse functionality for ESP32.
- **EEPROM**: Allows reading from and writing to the EEPROM of the ESP32.

Ensure these libraries are installed and up to date to avoid compilation issues.

## Calibration
- Both controllers have a calibration button on pin 18.
- Press the calibration button to set the current orientation as neutral.
- Calibration settings are saved to EEPROM and persist between power cycles.
- Keep the controllers still during calibration.

## Connection Table

### Right Hand (Mouse Controller)

| Component             | ESP32 Pin | Description                          |
|-----------------------|-----------|--------------------------------------|
| BNO055 SDA            | GPIO 21   | I2C Data                             |
| BNO055 SCL            | GPIO 22   | I2C Clock                            |
| BNO055 VCC            | 3.3V      | Power                                |
| BNO055 GND            | GND       | Ground                               |
| Calibration Button    | GPIO 18   | Connect to GND when pressed          |
| Hall Effect Sensor    | GPIO 36   | Digital input for mouse clicks       |

### Left Hand (Keyboard Controller)

| Component             | ESP32 Pin | Description                          |
|-----------------------|-----------|--------------------------------------|
| MPU6050 SDA           | GPIO 21   | I2C Data                             |
| MPU6050 SCL           | GPIO 22   | I2C Clock                            |
| MPU6050 VCC           | 3.3V      | Power                                |
| MPU6050 GND           | GND       | Ground                               |
| Calibration Button    | GPIO 18   | Connect to GND when pressed          |

## Usage
1. Upload the respective code to each ESP32.
2. Power on both devices.
3. Connect to them via Bluetooth from your computer.
4. Press calibration buttons to set the neutral position.
5. Use right-hand tilting for mouse movement.
6. Use the hall effect sensor for mouse clicks.
7. Use left-hand tilting for WASD keyboard controls.

https://github.com/arahe-dev/GloveXR

