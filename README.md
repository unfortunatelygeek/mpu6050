# ESP32 MPU6050 and LED Blink Example

This project demonstrates the use of an ESP32 microcontroller to interface with an MPU6050 accelerometer/gyroscope and control an LED. It utilizes FreeRTOS for task management, enabling concurrent operation of the MPU6050 data reading and LED blinking.

## Features

- LED blinking at 5 Hz
- MPU6050 data reading and printing at 500 Hz
- I2C communication with MPU6050
- FreeRTOS task management

## Hardware Requirements

- ESP32 development board
- MPU6050 module
- LED (built-in or external)
- Appropriate connections and power supply

## Pin Configuration

- SDA (I2C Data): GPIO 21
- SCL (I2C Clock): GPIO 22
- LED: GPIO 2 (built-in LED on most ESP32 dev boards)

## Software Dependencies

- ESP-IDF (Espressif IoT Development Framework)
- FreeRTOS
- MPU6050 library (ensure this is properly installed in your project)

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