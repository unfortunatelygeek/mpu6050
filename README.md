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
5. Install nRF Connect by Nordic Semiconductors on your mobile phone.

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

#### MPU6050

- The code first initializes the I2C communication protocol to interact with the MPU6050 sensor. This is done by the function i2c_master_init(), which sets up the necessary configuration for I2C communication like the data and clock lines.

- The code creates a handle for the MPU6050 sensor using mpu6050_create(). This handle is used to interact with the sensor.

- The sensor is then configured with specific settings for the accelerometer and gyroscope ranges using mpu6050_config(). These settings determine the sensitivity of the measurements.

- The sensor is "woken up" using the function mpu6050_wake_up(), making it ready to start taking measurements.

- The code enters an infinite loop where it periodically reads accelerometer and gyroscope data from the sensor.

- Functions mpu6050_get_acce() and mpu6050_get_gyro() are used to fetch the current acceleration and gyroscope values.

- The mpu_task function runs as a FreeRTOS task, meaning it operates independently and concurrently with other tasks in the system. It uses vTaskDelayUntil() to ensure it runs at a regular interval.


#### LED and Bluetooth:

- The LED Blinking aspect is self explanatory. However, the status variable, which is globally defined, is used to determine the message to be sent to the Moblie App.

- The Bluetooth stack is initialized using functions like nimble_port_init() and ble_svc_gap_init().

- The device name is set to "BLE-Server-Aditi" using ble_svc_gap_device_name_set().

- A GATT service is defined with characteristics that the Bluetooth clients can read.

- The ble_device_read() function is a callback that provides the current status ("Device On" or "Device Off") to the client when requested.

- The ble_app_advertise() function sets up the advertising parameters and starts advertising the device so that other Bluetooth devices can discover and connect to it.

- The host_task function runs the Bluetooth stack.

- This task is initialized using nimble_port_freertos_init(), which sets up the Bluetooth stack to run as a FreeRTOS task.

- The connect_ble() function is simply a wrapper for all the previously mentioned functions.