#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#define EN_PIN  27
#define IN1_PIN 26
#define IN2_PIN 25

#define ENCODER_A_PIN 33
#define ENCODER_B_PIN 32

const int freq = 980;
const int resolution = 8;

volatile long encoder_count = 0;
volatile unsigned long last_isr_time_A = 0;
volatile unsigned long last_isr_time_B = 0;
#define DEBOUNCE_US 50

rcl_subscription_t subscriber;
rcl_publisher_t    encoder_publisher;
std_msgs__msg__Float32 msg;
std_msgs__msg__Int32   encoder_msg;
rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) { delay(100); }
}

void IRAM_ATTR encoder_isr_A() {
  unsigned long now = micros();
  if (now - last_isr_time_A < DEBOUNCE_US) return;
  last_isr_time_A = now;
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

void IRAM_ATTR encoder_isr_B() {
  unsigned long now = micros();
  if (now - last_isr_time_B < DEBOUNCE_US) return;
  last_isr_time_B = now;
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    encoder_count--;
  } else {
    encoder_count++;
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float cmd = m->data;

  if (cmd >  1.0f) cmd =  1.0f;
  if (cmd < -1.0f) cmd = -1.0f;

  int duty_cycle = (int)(fabsf(cmd) * 255.0f);

  if (cmd > 0.0f) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (cmd < 0.0f) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  ledcWrite(EN_PIN, duty_cycle);
}

void setup() {
  set_microros_transports();

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcAttach(EN_PIN, freq, resolution);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(EN_PIN, 0);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoder_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoder_isr_B, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/cmd_pwm"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/encoder"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                          &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(15);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  encoder_msg.data = (int32_t)encoder_count;
  RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
}
