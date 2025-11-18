# STM32 IoT Proximity Monitoring System

## 📌 Project Overview
An embedded IoT proximity monitoring system developed using an STM32 Nucleo-L552ZE-Q microcontroller. The system uses ultrasonic ranging, LCD feedback, buzzer alerts, and optional WiFi telemetry via ESP8266 to support real-time sensing and monitoring.

## 🧩 Features
- Real-time distance measurement using ultrasonic sensor
- LCD system status and distance output
- Buzzer alerts when distance threshold is met
- UART communication for debugging/telemetry
- Modular firmware structure using HAL drivers
- Expandable to cloud platforms (ThingSpeak / MQTT)

## 🛠️ Hardware Used
| Component | Description |
|-----------|-------------|
| STM32 Nucleo-L552ZE-Q | ARM Cortex-M33 MCU |
| HC-SR04 | Ultrasonic distance sensor |
| 16x2 LCD | I2C display |
| ESP8266 | WiFi telemetry module |
| Buzzer | Audible alert feedback |

## 💻 Software and Tools
- C (HAL Drivers)
- STM32CubeIDE
- UART Serial Communication
- ThingSpeak Cloud Dashboard (optional)

## 📁 Repository Structure
