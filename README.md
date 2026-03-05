# Mini Reto 3: Control de Motor DC con ROS 2 y ESP32

Este proyecto implementa un sistema de control de lazo abierto para un motor DC utilizando un **ESP32**, el driver **L298N** y **micro-ROS** sobre ROS 2 Humble.

## 🛠️ Requisitos Previos

Antes de clonar este repositorio, asegúrate de tener instalado ROS 2 Humble en Ubuntu 22.04 y las herramientas de micro-ROS:

1. **Instalar micro-ROS setup:**
   Sigue las instrucciones del repositorio oficial:
   [micro-ROS Setup](https://github.com/micro-ROS/micro_ros_setup)

2. **Librería de Arduino:**
   Descarga e instala en tu Arduino IDE la librería:
   `micro_ros_arduino` (Versión Humble).

## 🚀 Instalación y Uso

1. **Clonar este repositorio en tu espacio de trabajo:**
   ```bash
   cd ~/microros_ws/src
   git clone https://github.com/JLDominguezM/MotorPWM_ROS.git
   cd ..
   colcon build --packages-select pc_motor_control
   source install/local_setup.bash

2. **Cargar el Firmware:**
Abre el archivo .ino ubicado en la carpeta esp32_code/ y súbelo a tu ESP32.

3. **Ejecutar el Agente y el Nodo:**

- Terminal 1 (Agente): ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

- Terminal 2 (Controlador): ros2 run pc_motor_control motor_controller

## 🔌 Conexiones (Hardware)
IN1: GPIO 18

IN2: GPIO 15

ENA (PWM): GPIO 13

GND: Común entre ESP32 y L298N