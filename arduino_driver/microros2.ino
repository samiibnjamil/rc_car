#include <micro_ros_arduino.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include <MPU6050_light.h>
#include <Adafruit_MotorShield.h>
#include <geometry_msgs/msg/twist.h>

// MotorShield and Motor Setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

// Motor control variables
int maxSpeed = 255;  // Adjust as necessary
int leftSpeed = 0, rightSpeed = 0;

void moveWithTurn(int speed, int turn, int reverse) {
  int error = 0;
  reverse = map(reverse, 0, 1020, 0, -maxSpeed);
  speed = map(speed, 0, 1020, 0, maxSpeed);
  turn = map(turn, -509, 509, maxSpeed / 1.2, -maxSpeed / 1.2);

  if (reverse == 0) {
    leftSpeed = speed - turn;
    rightSpeed = speed + turn;
    leftSpeed = constrain(leftSpeed, -250, 250);
    rightSpeed = constrain(rightSpeed, -250, 250);

    frontLeftMotor->setSpeed(abs(leftSpeed));
    backLeftMotor->setSpeed(abs(leftSpeed));
    frontRightMotor->setSpeed(abs(rightSpeed));
    backRightMotor->setSpeed(abs(rightSpeed));

    if (leftSpeed > 0) {
      frontLeftMotor->run(FORWARD);
      backLeftMotor->run(FORWARD);
    } else {
      frontLeftMotor->run(BACKWARD);
      backLeftMotor->run(BACKWARD);
    }

    if (rightSpeed > 0) {
      frontRightMotor->run(FORWARD);
      backRightMotor->run(FORWARD);
    } else {
      frontRightMotor->run(BACKWARD);
      backRightMotor->run(BACKWARD);
    }
  } else {
    leftSpeed = reverse + turn;
    rightSpeed = reverse - turn;
    leftSpeed = constrain(leftSpeed, -150, 150);
    rightSpeed = constrain(rightSpeed, -150, 150);

    frontLeftMotor->setSpeed(abs(leftSpeed) - error);
    backLeftMotor->setSpeed(abs(leftSpeed) - error);
    frontRightMotor->setSpeed(abs(rightSpeed));
    backRightMotor->setSpeed(abs(rightSpeed));

    if (leftSpeed > 0) {
      frontLeftMotor->run(FORWARD);
      backLeftMotor->run(FORWARD);
    } else {
      frontLeftMotor->run(BACKWARD);
      backLeftMotor->run(BACKWARD);
    }

    if (rightSpeed > 0) {
      frontRightMotor->run(FORWARD);
      backRightMotor->run(FORWARD);
    } else {
      frontRightMotor->run(BACKWARD);
      backRightMotor->run(BACKWARD);
    }
  }

  Serial.print("leftSpeed: ");
  Serial.print(abs(leftSpeed) - error);
  Serial.print(" rightSpeed: ");
  Serial.println(rightSpeed);
}

// Micro-ROS Node Handles
rcl_publisher_t ultrasonic_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t cmd_vel_subscription;

// ROS 2 Messages
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Float32 distance_msg;
std_msgs__msg__Float32MultiArray imu_msg;

// Timing Variables
unsigned long last_publish_time = 0;
const unsigned long publish_interval = 8;  // 8.33 ms for 120 Hz

// Micro-ROS Support
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Extract linear and angular velocities
  int speed = msg->linear.x * maxSpeed;   // Linear velocity (forward/backward)
  int turn = msg->angular.z * maxSpeed;   // Angular velocity (turning)
  
  // Call moveWithTurn with the received speed and turn values
  moveWithTurn(speed, turn, 0); // Reverse is set to 0 for simplicity (adjust as needed)
}

void setup() {
  // Initialize Serial and Micro-ROS
  Serial.begin(115200);
  set_microros_transports();

  // Micro-ROS Setup
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Ultrasonic Publisher
  rclc_publisher_init_default(
    &ultrasonic_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ultrasonic/distance"
  );

  // IMU Publisher
  rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/imu/data"
  );

  // Subscribe to the /motor/cmd_vel topic to receive linear and angular velocity commands
  rclc_subscription_init_default(
    &cmd_vel_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/motor/cmd_vel"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  Serial.println("Micro-ROS Node Initialized");
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_publish_time >= publish_interval) {
    last_publish_time = current_time;

    // Publish Ultrasonic Sensor Data
    float distance = ultrasonic.read();
    distance_msg.data = distance;
    rcl_publish(&ultrasonic_publisher, &distance_msg, NULL);

    // Publish Gyroscope Data
    mpu.update();
    imu_msg.data.capacity = 2;
    imu_msg.data.data[0] = mpu.getAngleX();
    imu_msg.data.data[1] = mpu.getAngleY();
    imu_msg.data.size = 2;
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    // Spin Executor and Handle Callback for Control Inputs
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  // Handle incoming subscriptions
  }
}
