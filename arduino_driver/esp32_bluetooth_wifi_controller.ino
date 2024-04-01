#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

AsyncWebServer server(80);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int leftSpeed, rightSpeed;
const char *ssid = "Sami";           // Your WiFi SSID
const char *password = "bhulegesi";  // Your WiFi Password

int speedIndex = 0; 
int maxSpeed=160;
TaskHandle_t core0TaskHandle = NULL;
TaskHandle_t core1TaskHandle = NULL;

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      WebSerial.println("Controller Connected");
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      WebSerial.println("Controller disconnected");
      myControllers[i] = nullptr;
      break;
    }
  }
}

void moveWithTurn(int speed, int turn, int reverse) {

  int error = 0;
  reverse = map(reverse, 0, 1020, 0, -maxSpeed);
  speed = map(speed, 0, 1020, 0, maxSpeed);
  turn = map(turn, -509, 509, maxSpeed, -maxSpeed);

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

void processGamepad(ControllerPtr ctl) {
  if (ctl->a()) {
    static int colorIdx = 0;
    Serial.println("a is pressed");
    WebSerial.println("a is pressed");
  }

  if (ctl->b()) {
    Serial.println("b is pressed");
    WebSerial.println("b is pressed");
  }

  if (ctl->x()) {
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                        0x40 /* strongMagnitude */);
    Serial.println("x is pressed");
    WebSerial.println("x is pressed");
  }
 if (ctl->y()) {
    // Toggle between three speeds: low, medium, high
    speedIndex = (speedIndex + 1) % 3; // Cycle through 0, 1, 2 (low, medium, high)
    switch(speedIndex) {
      case 0:
        maxSpeed = 80; // Set speed to low
        ctl->playDualRumble(0 /* delayedStartMs */, 200 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */); // Short rumble for low speed
        break;
      case 1:
        maxSpeed = 160; // Set speed to medium
        ctl->playDualRumble(0 /* delayedStartMs */, 450 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */); // Two short rumbles for medium speed
        
        break;
      case 2:
        maxSpeed = 250; // Set speed to high
        ctl->playDualRumble(0 /* delayedStartMs */, 900 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */); // Three short rumbles for high speed
        
        break;
    }
    Serial.print("Speed set to: ");
    Serial.println(maxSpeed);
    WebSerial.print("Speed set to: ");
    WebSerial.println(maxSpeed);
  }


  moveWithTurn(ctl->throttle(), ctl->axisX(), ctl->brake());
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
        WebSerial.println("Unsupported controller");
      }
    }
  }
}

void recvMsg(uint8_t *data, size_t len) {
  Serial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  }
  Serial.println(d);
}

void core0Task(void *pvParameters) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();

bool movingRightPrinted = false;
bool movingLeftPrinted = false;
bool movingFWDPrinted = false;
bool movingBWDPrinted = false;
bool notMovingPrinted = false;

while (true) {
    if (leftSpeed > 5 || rightSpeed > 5) {
        if (leftSpeed > rightSpeed && !movingRightPrinted) {
            WebSerial.println("Moving Right");
            movingRightPrinted = true;
            movingLeftPrinted = false;
            movingFWDPrinted = false;
            movingBWDPrinted = false;
            notMovingPrinted = false;
        } else if (leftSpeed < rightSpeed && !movingLeftPrinted) {
            WebSerial.println("Moving Left");
            movingLeftPrinted = true;
            movingRightPrinted = false;
            movingFWDPrinted = false;
            movingBWDPrinted = false;
            notMovingPrinted = false;
        } else if (leftSpeed == rightSpeed && !movingFWDPrinted) {
            WebSerial.println("Moving FWD");
            movingFWDPrinted = true;
            movingRightPrinted = false;
            movingLeftPrinted = false;
            movingBWDPrinted = false;
            notMovingPrinted = false;
        }
    } else if (leftSpeed < -1 || rightSpeed < 0) {
        if (!movingBWDPrinted) {
            WebSerial.println("Moving BWD");
            movingBWDPrinted = true;
            movingRightPrinted = false;
            movingLeftPrinted = false;
            movingFWDPrinted = false;
            notMovingPrinted = false;
        }
    } else if (!notMovingPrinted) {
        WebSerial.println("Not moving");
        notMovingPrinted = true;
        movingRightPrinted = false;
        movingLeftPrinted = false;
        movingFWDPrinted = false;
        movingBWDPrinted = false;
    }
    delay(50);
}
}

void core1Task(void *pvParameters) {
  //Serial.println("Adafruit Motorshield v2 - Differential Drive Car Control");
  AFMS.begin();
  const uint8_t *addr = BP32.localBdAddress();
  // Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  while (true) {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
      processControllers();
  }
}

void setup() {
  xTaskCreatePinnedToCore(
    core0Task,
    "core0Task",
    10000,
    NULL,
    1,
    &core0TaskHandle,
    0);

  xTaskCreatePinnedToCore(
    core1Task,
    "core1Task",
    10000,
    NULL,
    1,
    &core1TaskHandle,
    1);
}

void loop() {
  // This function will not be executed as everything is running on tasks
  delay(50);
}
