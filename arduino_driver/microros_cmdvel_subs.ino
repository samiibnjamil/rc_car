#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <rmw/qos_profiles.h>

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

int leftSpeed, rightSpeed;
int speedIndex = 0;
int maxSpeed = 160;
float angular_z, linear_x;

// ROS 2 variables
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#define LED_PIN 2

void error_loop() {
  pinMode(LED_PIN, OUTPUT);
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
void moveWithTurn(int speed, int turn, int reverse) {
  if (reverse == 0 && speed == 0) {
    // Handle stationary turning
    leftSpeed = turn;
    rightSpeed = -turn;
  } else {
    leftSpeed = (reverse == 0) ? speed - turn : reverse + turn;
    rightSpeed = (reverse == 0) ? speed + turn : reverse - turn;
  }

  int maxConstraint = (reverse == 0) ? 250 : 150;
  leftSpeed = constrain(leftSpeed, -maxConstraint, maxConstraint);
  rightSpeed = constrain(rightSpeed, -maxConstraint, maxConstraint);

  frontLeftMotor->setSpeed(abs(leftSpeed));
  backLeftMotor->setSpeed(abs(leftSpeed));
  frontRightMotor->setSpeed(abs(rightSpeed));
  backRightMotor->setSpeed(abs(rightSpeed));

  frontLeftMotor->run(leftSpeed > 0 ? FORWARD : BACKWARD);
  backLeftMotor->run(leftSpeed > 0 ? FORWARD : BACKWARD);
  frontRightMotor->run(rightSpeed > 0 ? FORWARD : BACKWARD);
  backRightMotor->run(rightSpeed > 0 ? FORWARD : BACKWARD);

  Serial.printf("LeftSpeed: %d RightSpeed: %d\n", leftSpeed, rightSpeed);
}

// Callback function for the subscription
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extracting linear_x and angular_z values
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;

  // Print the received values to the Serial Monitor
//  Serial.printf("linear_x: %f angular_z: %f\n", linear_x, angular_z);

  // Map linear_x and angular_z to motor speeds
  int speed = map(linear_x*100, 0, 200, 0, maxSpeed);
  int reverse = map(linear_x*100, 0, -200, 0, -maxSpeed);
  int turn = map(angular_z*1000, -1000, 1000, maxSpeed, -maxSpeed);


   //Serial.printf("speed: %d reverse: %d turn: %d\n", speed, reverse,turn);
  moveWithTurn(speed, turn, reverse);
}

void setup() {
  Serial.begin(115200);

  // Initialize the micro-ROS transport for Wi-Fi
  set_microros_wifi_transports("BELL777", "Sohul_0302", "192.168.4.23", 8888);
  allocator = rcl_get_default_allocator();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Create a support instance
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create a node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create a subscriber with volatile QoS
  RCCHECK(rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel2",
    &rmw_qos_profile_sensor_data));  // Volatile QoS

  // Create an executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Initialize the motor shield
  AFMS.begin();
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(8))); // Reduced delay for maximum processing speed
}
