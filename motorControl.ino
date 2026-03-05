#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// Pines indicados en la presentación del reto
#define IN1_PIN 25
#define IN2_PIN 27
#define EN_PIN 26

// Configuración de la señal PWM (frecuencia y resolución)
const int freq = 980;
const int resolution = 8; // 8 bits = rango de 0 a 255

rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

// Esta función se ejecuta cada vez que llega un mensaje de ROS 2 en /cmd_pwm
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float cmd = msg->data;

  // 1. Limitar (saturar) el comando al rango [-1.0, 1.0] por seguridad
  if (cmd > 1.0) cmd = 1.0;
  if (cmd < -1.0) cmd = -1.0;

  // 2. Mapear la velocidad (valor absoluto) de 0.0 - 1.0 a PWM de 0 - 255
  int duty_cycle = (int)(abs(cmd) * 255.0);

  // 3. Determinar la dirección de giro con los pines IN1 e IN2
  if (cmd > 0.0) {
    // Giro hacia adelante
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (cmd < 0.0) {
    // Giro hacia atrás
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    // Freno (cmd == 0.0)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  // 4. Mandar la potencia al pin Enable (Nueva sintaxis ESP32 Core v3)
  ledcWrite(EN_PIN, duty_cycle);
}

void setup() {
  set_microros_transports();

  // Configurar pines de dirección como salida
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Configurar el pin de Enable con PWM (Nueva sintaxis ESP32 Core v3)
  ledcAttach(EN_PIN, freq, resolution);

  // Asegurar que el motor arranque apagado
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(EN_PIN, 0);

  delay(2000); // Darle un respiro al ESP32 para conectar el USB

  allocator = rcl_get_default_allocator();

  // Inicializar micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crear el nodo llamado "motor"
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // Crear la suscripción al tópico "/cmd_pwm"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm"));

  // Inicializar el ejecutor para escuchar los callbacks
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(15);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
