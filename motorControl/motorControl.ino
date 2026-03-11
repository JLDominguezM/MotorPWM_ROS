#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

// --- LEFT MOTOR PINS ---
#define EN_PIN_L  27
#define IN1_PIN_L 26
#define IN2_PIN_L 25

#define ENCODER_A_PIN_L 33
#define ENCODER_B_PIN_L 32

// --- RIGHT MOTOR PINS 
#define EN_PIN_R  13
#define IN1_PIN_R 14
#define IN2_PIN_R 12

#define ENCODER_A_PIN_R 21
#define ENCODER_B_PIN_R 19

const int freq = 980;
const int resolution = 8;
#define DEBOUNCE_US 50

volatile long encoder_count_l = 0;
volatile unsigned long last_isr_time_A_l = 0;
volatile unsigned long last_isr_time_B_l = 0;

volatile long encoder_count_r = 0;
volatile unsigned long last_isr_time_A_r = 0;
volatile unsigned long last_isr_time_B_r = 0;

rcl_subscription_t subscriber_l;
rcl_subscription_t subscriber_r;
rcl_publisher_t    encoder_publisher_l;
rcl_publisher_t    encoder_publisher_r;

std_msgs__msg__Float32 msg_l;
std_msgs__msg__Float32 msg_r;
std_msgs__msg__Int32   encoder_msg_l;
std_msgs__msg__Int32   encoder_msg_r;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) { delay(100); }
}

void IRAM_ATTR encoder_isr_A_l() {
  unsigned long now = micros();
  if (now - last_isr_time_A_l < DEBOUNCE_US) return;
  last_isr_time_A_l = now;
  if (digitalRead(ENCODER_A_PIN_L) == digitalRead(ENCODER_B_PIN_L)) {
    encoder_count_l++;
  } else {
    encoder_count_l--;
  }
}

void IRAM_ATTR encoder_isr_B_l() {
  unsigned long now = micros();
  if (now - last_isr_time_B_l < DEBOUNCE_US) return;
  last_isr_time_B_l = now;
  if (digitalRead(ENCODER_A_PIN_L) == digitalRead(ENCODER_B_PIN_L)) {
    encoder_count_l--;
  } else {
    encoder_count_l++;
  }
}
void IRAM_ATTR encoder_isr_A_r() {
  unsigned long now = micros();
  if (now - last_isr_time_A_r < DEBOUNCE_US) return;
  last_isr_time_A_r = now;
  if (digitalRead(ENCODER_A_PIN_R) == digitalRead(ENCODER_B_PIN_R)) {
    encoder_count_r++;
  } else {
    encoder_count_r--;
  }
}

void IRAM_ATTR encoder_isr_B_r() {
  unsigned long now = micros();
  if (now - last_isr_time_B_r < DEBOUNCE_US) return;
  last_isr_time_B_r = now;
  if (digitalRead(ENCODER_A_PIN_R) == digitalRead(ENCODER_B_PIN_R)) {
    encoder_count_r--;
  } else {
    encoder_count_r++;
  }
}

void subscription_callback_l(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float cmd = m->data;

  if (cmd >  1.0f) cmd =  1.0f;
  if (cmd < -1.0f) cmd = -1.0f;

  int duty_cycle = (int)(fabsf(cmd) * 255.0f);

  if (cmd > 0.0f) {
    digitalWrite(IN1_PIN_L, HIGH);
    digitalWrite(IN2_PIN_L, LOW);
  } else if (cmd < 0.0f) {
    digitalWrite(IN1_PIN_L, LOW);
    digitalWrite(IN2_PIN_L, HIGH);
  } else {
    digitalWrite(IN1_PIN_L, LOW);
    digitalWrite(IN2_PIN_L, LOW);
  }

  ledcWrite(EN_PIN_L, duty_cycle);
}

void subscription_callback_r(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float cmd = m->data;

  if (cmd >  1.0f) cmd =  1.0f;
  if (cmd < -1.0f) cmd = -1.0f;

  int duty_cycle = (int)(fabsf(cmd) * 255.0f);

  if (cmd > 0.0f) {
    digitalWrite(IN1_PIN_R, HIGH);
    digitalWrite(IN2_PIN_R, LOW);
  } else if (cmd < 0.0f) {
    digitalWrite(IN1_PIN_R, LOW);
    digitalWrite(IN2_PIN_R, HIGH);
  } else {
    digitalWrite(IN1_PIN_R, LOW);
    digitalWrite(IN2_PIN_R, LOW);
  }

  ledcWrite(EN_PIN_R, duty_cycle);
}

void setup() {
  set_microros_transports();

  // --- SETUP LEFT MOTOR ---
  pinMode(IN1_PIN_L, OUTPUT);
  pinMode(IN2_PIN_L, OUTPUT);
  ledcAttach(EN_PIN_L, freq, resolution);
  digitalWrite(IN1_PIN_L, LOW);
  digitalWrite(IN2_PIN_L, LOW);
  ledcWrite(EN_PIN_L, 0);

  pinMode(ENCODER_A_PIN_L, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_L), encoder_isr_A_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_L), encoder_isr_B_l, CHANGE);

  // --- SETUP RIGHT MOTOR ---
  pinMode(IN1_PIN_R, OUTPUT);
  pinMode(IN2_PIN_R, OUTPUT);
  ledcAttach(EN_PIN_R, freq, resolution);
  digitalWrite(IN1_PIN_R, LOW);
  digitalWrite(IN2_PIN_R, LOW);
  ledcWrite(EN_PIN_R, 0);

  pinMode(ENCODER_A_PIN_R, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN_R), encoder_isr_A_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN_R), encoder_isr_B_r, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robot_motors", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_l, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm_left"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher_l, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/encoder_left"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_r, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm_right"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher_r, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/encoder_right"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_l, &msg_l,
                                          &subscription_callback_l, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_r, &msg_r,
                                          &subscription_callback_r, ON_NEW_DATA));
}

void loop() {
  delay(15);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  encoder_msg_l.data = (int32_t)encoder_count_l;
  RCSOFTCHECK(rcl_publish(&encoder_publisher_l, &encoder_msg_l, NULL));

  encoder_msg_r.data = (int32_t)encoder_count_r;
  RCSOFTCHECK(rcl_publish(&encoder_publisher_r, &encoder_msg_r, NULL));
}