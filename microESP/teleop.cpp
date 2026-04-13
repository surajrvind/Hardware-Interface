#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <esp_system.h>
#include "servo_commands.h"
#include "motor_commands.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
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

const char* kSubscriberTopic = "cmd_vel";
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_msg;
//char received_buffer [50];
char debug_buffer [50];

//debug publisher
rcl_publisher_t debug_pub;
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
  const geometry_msgs__msg__Twist*cmd = (const geometry_msgs__msg__Twist*) msgin;

  float throttle = constrain(cmd->linear.x, -1.0, 1.0);
  float steering = constrain(cmd->angular.z, -1.0, 1.0);

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

  // Initialise messages
  cmd_msg.linear.x = 0.0;
  //cmd_msg.linear.y = 0.0;
  cmd_msg.linear.z = 0.0;
  cmd_msg.angular.x = 0.0;
  //cmd_msg.angular.y = 0.0;
  cmd_msg.angular.z = 0.0;

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
  // create subscriber 
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
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
  float throttle_step = 0.02;
  float steering_step = 0.1;
  // Simple ramping logic for throttle and steering
  if (current_throttle < target_throttle) {
    current_throttle += throttle_step;} 
  else if (current_throttle > target_throttle) {
    current_throttle -= throttle_step;
  }
  // Steering logic
  if (current_steering < target_steering) {
    current_steering += steering_step;
  } 
  else if (current_steering > target_steering) {
    current_steering -= steering_step;
  }
  
  setMotor(current_throttle);
  setSteering(current_steering);

  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}