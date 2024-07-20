# ESP32 MPU6050 and LED Blink Example

This project demonstrates the use of an ESP32 microcontroller to interface with an MPU6050 accelerometer/gyroscope and control an LED. It utilizes FreeRTOS for task management, enabling concurrent operation of the MPU6050 data reading and LED blinking.

## Features

- LED blinking at 5 Hz
- MPU6050 data reading and printing at 500 Hz
- I2C communication with MPU6050
- FreeRTOS task management
- BLE functionality to send device status messages

## Hardware Requirements

- ESP32 development board
- MPU6050 module
- LED (I'm using hte built-in one at GPIO 2)

## Pin Configuration

- SDA (I2C Data): GPIO 21
- SCL (I2C Clock): GPIO 22
- LED: GPIO 2 (built-in LED on most ESP32 dev boards)

## Software Dependencies

- ESP-IDF (Espressif IoT Development Framework)
- FreeRTOS
- MPU6050 library (ensure this is properly installed in your project)
- NimBLE Library (and appropriate SDKConfig)

## Setup and Configuration

1. Set up your ESP-IDF development environment.
2. Clone this repository or copy the provided code into your project.
3. Ensure all necessary libraries are installed and properly linked.
4. Configure your project to use the correct ESP32 board and toolchain.

## Building and Flashing

1. Navigate to your project directory.
2. Build the project: 
```idf.py build```
3. Flash the project to your ESP32:
```idf.py -p (PORT) flash```
Replace (PORT) with your ESP32's serial port (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux).

## How the System Works

### Configurations

SDA (MPU6050) -> GPIO 21 
SCL (MPU6050) -> GPIO 22 
LED           -> GPIO 2 

#### Bluetooth:

BLE Initialization: The BLE server initializes with a unique name and starts advertising.
Connection Handling: When a client connects, the server sends "Device On" or "Device Off" messages depending on the device's state.
Device Status: The device status is maintained through a variable and updated during the device's operation.