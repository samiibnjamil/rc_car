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
#include <sensor_msgs/msg/battery_state.h>

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

int leftSpeed, rightSpeed;
int speedIndex = 0;
int maxSpeed = 200;
float angular_z, linear_x;

// ROS 2 variables
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Battery publisher variables
rcl_publisher_t battery_publisher;
sensor_msgs__msg__BatteryState battery_msg;

#define LED_PIN 2
#define BATTERY_ADC_PIN 34
#define R1 9.9
#define R2 5.06
#define REFERENCE_VOLTAGE 3.3
#define ADC_MAX_VALUE 4095

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Task handle for core 0
TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

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

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extracting linear_x and angular_z values
  linear_x = msg->linear.x;
  angular_z = msg->angular.z;

  // Map linear_x and angular_z to motor speeds
  int speed = map(linear_x*100, 0, 200, 0, maxSpeed);
  int reverse = map(linear_x*100, 0, -200, 0, -maxSpeed);
  int turn = map(angular_z*1000, -1000, 1000, maxSpeed, -maxSpeed);

  moveWithTurn(speed, turn, reverse);
}

void publish_battery_status() {
  // Read ADC value and calculate battery voltage
  int adc_value = analogRead(BATTERY_ADC_PIN);
  float voltageOut = adc_value * (REFERENCE_VOLTAGE / ADC_MAX_VALUE);
  float batteryVoltage = voltageOut * (R1 + R2) / R2;
  float batteryPercentage = constrain((batteryVoltage - 6.0) / (8.4 - 6.0) * 100, 0.0, 100);

  // Update the BatteryState message
  battery_msg.header.stamp.sec = millis() / 1000;
  battery_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  battery_msg.voltage = batteryVoltage;
  battery_msg.percentage = batteryPercentage;

  // Publish the battery status
  RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
   // vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second delay
}

// Function to blink LED on Core 0
void bateryPub(void *parameter) {

  while (true) {
    publish_battery_status();
 Serial.print("Free heap memory: ");
    Serial.println(ESP.getFreeHeap());
    //chekc using space 
    // Serial.print("BatteryPub Stack High Water Mark: ");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

// Function to print message on Core 1
void motor(void *parameter) {
  while (true) {
    // Process any pending ROS callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(8))); 

      vTaskDelay(4/ portTICK_PERIOD_MS);  // Delay for 10ms
}
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

  // Create a publisher for BatteryState message
  RCCHECK(rclc_publisher_init_best_effort(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "battery_status"));

  // Create an executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Initialize the motor shield
  AFMS.begin();
  
  // Create tasks to run on Core 0 and Core 1
  xTaskCreatePinnedToCore(bateryPub, "bateryPub ", 6500, NULL, 1, &TaskCore0, 0);  // Core 0
  xTaskCreatePinnedToCore(motor, "Motor control", 7000, NULL, 2, &TaskCore1, 1);  // Core 1
}

void loop() {
  // Nothing to do in loop since tasks are running on different cores
}
