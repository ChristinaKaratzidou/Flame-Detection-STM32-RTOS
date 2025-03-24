# Flame-Detection-STM32-RTOS
STM32 Flame Detection System with RGB LED and FreeRTOS

## Project Overview
This project implements a flame detection system using an STM32 microcontroller, an infrared flame sensor, and an RGB LED module. The system is designed to detect the presence of a flame and provide real-time feedback through an RGB LED and serial communication. The project uses FreeRTOS to manage multiple tasks efficiently, ensuring reliable and responsive performance.

## Why FreeRTOS?
- Task Management: The system performs multiple tasks simultaneously without blocking each other. Reading the flame sensor (ADC). Controlling the RGB LED. Sending status updates via UART.
- Real-Time Responsiveness: FreeRTOS ensures that critical tasks (e.g., flame detection) are executed promptly, providing real-time feedback.
- Scalability: FreeRTOS makes it easy to add new features or tasks (e.g., additional sensors or communication protocols) without disrupting the existing system.

## Features
- Flame detection: The infrared flame sensor detects the presence of a flame and outputs an analog signal. The STM32 reads this signal using its ADC (Analog-to-Digital Converter) and converts it to a digital value.
- Threshold Comparison: The system compares the ADC value to a predefined threshold (FLAME_THRESHOLD). If the ADC value exceeds the threshold, a flame is detected.
- RGB LED Feedback:
  - Green: No flame detected.
  - Red: Flame detected.
- Serial Communication: The system sends real-time status updates to a PC via UART:
  - System Normal: No flame detected.
  - FLAME DETECTED!: Flame detected.
- FreeRTOS Tasks: The system uses three FreeRTOS tasks: Flame Sensor Task:
  - Reads the ADC value and updates the flame status.
  - Alert Task: Controls the RGB LED based on the flame status.
  - Serial Task: Sends status updates to the PC via UART.

## Components
- **STM32F401RE**: MCU.
- **Infrared Flame Sensor**: Flame sensor module.
- **RGB LED Module** 
- Jumper wires.

## Hardware Connections
| STM32 Pin | Component      | Connection |
|-----------|----------------|------------|
| PA0       | Flame Sensor   | Analog Out |
| GND       | Flame Sensor   | GND        |
| VCC       | Flame Sensor   | 5V         |
| PA6       | RGB LED (Red)  | Anode      |
| PA7       | RGB LED (Green)| Anode      |
| PB0       | RGB LED (Blue) | Anode      |
| GND       | RGB LED        | GND        |
| PA2       | USB-UART TX    | RX         |
| PA3       | USB-UART RX    | TX         |

## Software Setup
1. Install **STM32CubeMX**.
2. Install **PuTTY**. (for serial monitoring)

