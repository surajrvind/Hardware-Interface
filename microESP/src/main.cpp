#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_system.h>
#include "servo_commands.h"
#include "motor_commands.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const char* kSubscriberTopic = "esp_commands";
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray state_msg;
std_msgs__msg__Float32MultiArray cmd_msg;
float cmd_buffer[2];
float state_buffer [2];
char debug_buffer [50];

//debug publisher
rcl_publisher_t debug_pub;
rcl_publisher_t state_pub;
std_msgs__msg__String debug_msg;

// initialise motor values
float target_throttle = 0.0;
float target_steering = 0.0;

float current_throttle = 0.0;
float current_steering = 0.0;

void subscription_callback(const void*msgin);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void subscription_callback(const void*msgin){
  const std_msgs__msg__Float32MultiArray*msg = (const std_msgs__msg__Float32MultiArray*) msgin;

  float throttle = constrain(msg->data.data[0], -1.0, 1.0);
  float steering = constrain(msg->data.data[1], -1.0, 1.0);

  target_throttle = throttle;
  target_steering = steering;

  snprintf(debug_buffer, sizeof(debug_buffer),
          "Throttle: %.2f Steering: %.2f",
          throttle, steering);

  debug_msg.data.size = strlen(debug_buffer);

  RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));
}

void setup() {
  // Setup motor and servo
  setupMotor();
  setupServo();
  setMotor(0);
  setSteering(0);
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  debug_msg.data.data = debug_buffer;
  debug_msg.data.capacity = sizeof(debug_buffer);
  debug_msg.data.size = 0;

  state_msg.data.capacity = 2;
  state_msg.data.size = 2;
  state_msg.data.data = state_buffer;

  cmd_msg.data.capacity = 2;
  cmd_msg.data.size = 2;
  cmd_msg.data.data = cmd_buffer;

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &debug_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "debug_topic"
  ));

  RCCHECK(rclc_publisher_init_default(
    &state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "esp_state"
  ));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    kSubscriberTopic));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &cmd_msg,
    &subscription_callback,
    ON_NEW_DATA));
}

void loop() {
  float dt = 0.01; // 10 ms loop
  float throttle_rate = 2.0; // per second
  float steering_rate = 5.0;

  if (current_throttle < target_throttle)
    current_throttle += throttle_rate * dt;
  else if (current_throttle > target_throttle)
    current_throttle -= throttle_rate * dt;

  if (current_steering < target_steering)
    current_steering += steering_rate * dt;
  else if (current_steering > target_steering)
    current_steering -= steering_rate * dt;
  
  setMotor(current_throttle);
  setSteering(current_steering);

  state_buffer[0] = current_throttle;
  state_buffer[1] = current_steering;
  RCSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));

  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}