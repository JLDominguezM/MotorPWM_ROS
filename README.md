# Motor DC Control with ROS 2 and ESP32

This project implements a control system for a DC motor using an **ESP32**, an **L298N** driver, and **micro-ROS** running on ROS 2 Humble. It includes features for open-loop control, encoder feedback, and PID tuning.

## Prerequisites

Before cloning this repository, ensure you have ROS 2 Humble installed on Ubuntu 22.04 and the micro-ROS tools configured.

1.  **Install micro-ROS setup:**
    Follow the official instructions: [micro-ROS Setup](https://github.com/micro-ROS/micro_ros_setup).

## Project Structure

-   **motorControl/**: Contains the Arduino sketch (`motorControl.ino`) for the ESP32.
-   **src/pc_motor_control/**: ROS 2 package for PC-side control and interfacing.
-   **experiments/**: Includes CSV logs of experiments and Python scripts for plotting and Ziegler-Nichols tuning.

## Hardware Configuration

The following pinout is used in the `motorControl.ino` sketch:

| Component | ESP32 Pin |
| :--- | :--- |
| **L298N EN (PWM)** | 27 |
| **L298N IN1** | 26 |
| **L298N IN2** | 25 |
| **Encoder Phase A**| 33 |
| **Encoder Phase B**| 32 |

-   **PWM Frequency**: 980 Hz
-   **Resolution**: 8-bit

## Getting Started

### 1. ESP32 Setup
1.  Open `motorControl/motorControl.ino` in the Arduino IDE.
2.  Install the `micro_ros_arduino` library.
3.  Configure your WiFi credentials if using a wireless transport, or use Serial.
4.  Upload the code to your ESP32.
