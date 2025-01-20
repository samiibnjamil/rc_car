#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Motor definitions
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

// Gamepad variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int leftSpeed, rightSpeed;
int speedIndex = 0;
int maxSpeed = 160;

// Semaphore for synchronization
SemaphoreHandle_t controllerSemaphore;

// Callback for connected controllers
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      break;
    }
  }
}

// Callback for disconnected controllers
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      break;
    }
  }
}

// Move motors with speed and turn
void moveWithTurn(int speed, int turn, int reverse) {
  reverse = map(reverse, 0, 1020, 0, -maxSpeed);
  speed = map(speed, 0, 1020, 0, maxSpeed);
  turn = map(turn, -509, 509, maxSpeed, -maxSpeed);

  leftSpeed = (reverse == 0) ? speed - turn : reverse + turn;
  rightSpeed = (reverse == 0) ? speed + turn : reverse - turn;

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

// Process input from a gamepad
void processGamepad(ControllerPtr ctl) {
  if (ctl->a()) {
    Serial.println("A is pressed");
  }

  if (ctl->b()) {
    Serial.println("B is pressed");
  }

  if (ctl->x()) {
    ctl->playDualRumble(0, 250, 0x80, 0x40);
    Serial.println("X is pressed");
  }

  if (ctl->y()) {
    speedIndex = (speedIndex + 1) % 3;
    switch (speedIndex) {
      case 0:
        maxSpeed = 80;
        ctl->playDualRumble(0, 200, 0x80, 0x40);
        break;
      case 1:
        maxSpeed = 160;
        ctl->playDualRumble(0, 450, 0x80, 0x40);
        break;
      case 2:
        maxSpeed = 250;
        ctl->playDualRumble(0, 900, 0x80, 0x40);
        break;
    }
    Serial.printf("Speed set to: %d\n", maxSpeed);
  }

  moveWithTurn(ctl->throttle(), ctl->axisX(), ctl->brake());
}

// Process all connected controllers
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// Bluetooth task: handles controller updates
void taskBluetooth(void *pvParameters) {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  while (true) {
    if (BP32.update()) {
      xSemaphoreTake(controllerSemaphore, portMAX_DELAY);
      processControllers();
      xSemaphoreGive(controllerSemaphore);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// Motor control task: handles motor updates
void taskMotorControl(void *pvParameters) {
  AFMS.begin();

  while (true) {
    // Additional motor control logic can be added here
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Setup tasks
void setup() {
  Serial.begin(115200);
  controllerSemaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskBluetooth, "BluetoothTask", 10000, NULL, 2, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(taskMotorControl, "MotorControlTask", 10000, NULL, 1, NULL, 1); // Core 1
}

void loop() {
  // Empty loop as all logic is in tasks
}
