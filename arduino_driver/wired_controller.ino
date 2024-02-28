
#include <Adafruit_MotorShield.h>


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

int speed1;
int LT_Button, RT_Button, Joystick1_X, Joystick1_Y, Joystick2_X, Joystick2_Y;
void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
 moveWithTurn(50, 0, 1000);
stopMotors();
 Serial.println("Ready");
}
unsigned long lastInputTime = 0;
unsigned long debounceDelay = 80;  // Adjust this value based on your needs
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseInput(input);

    // Check for debounce before processing the next input
    unsigned long currentTime = millis();
    if (currentTime - lastInputTime > debounceDelay) {
      moveBasedOnInput();
      lastInputTime = currentTime;
    }
  }
}

void parseInput(String input) {
  // Parse the input string using sscanf
  int parsedValues = sscanf(input.c_str(), "LT_Button,%d,RT_Button,%d,Joystick1_X,%d,Joystick1_Y,%d,Joystick2_X,%d,Joystick2_Y,%d",
                            &LT_Button, &RT_Button, &Joystick1_X, &Joystick1_Y, &Joystick2_X, &Joystick2_Y);

  // Print the number of successfully parsed values for debugging

while (Serial.available() > 0) {
    Serial.read();
  }
  // Print the parsed values
//  Serial.print("LT_Button: ");
//  Serial.print(LT_Button);
//  Serial.print(", RT_Button: ");
//  Serial.print(RT_Button);
//  Serial.print(", Joystick1_X: ");
//  Serial.print(Joystick1_X);
//  Serial.print(", Joystick1_Y: ");
//  Serial.print(Joystick1_Y);
//  Serial.print(", Joystick2_X: ");
//  Serial.print(Joystick2_X);
//  Serial.print(", Joystick2_Y: ");
//  Serial.println(Joystick2_Y);
}
void moveWithTurn(int speed, int turn, int t) {
  Serial.print("Moving with speed=");
  Serial.print(speed);
  Serial.print(", turn=");
  Serial.print(turn);
  Serial.print(", time=");
  Serial.println(t);

  speed = map(speed, 0, 100, 0, 150);  // Map speed to motor range
  turn = map(turn, -100, 100, -150, 150);    // Map turn to motor range

  int leftSpeed = speed - turn;
  int rightSpeed = speed + turn;

  // Constrain motor speeds to allowable range
  leftSpeed = constrain(leftSpeed, -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);

  // Set motor speeds
  frontLeftMotor->setSpeed(abs(leftSpeed));
  backLeftMotor->setSpeed(abs(leftSpeed));
  frontRightMotor->setSpeed(abs(rightSpeed));
  backRightMotor->setSpeed(abs(rightSpeed));

  // Set motor directions
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

  //delay(t);
  
}
void moveBasedOnInput() {
  // Map RT_Button to speed and Joystick1_X to turn
  int speed = map(Joystick1_Y, 0, -4767, 0, 100);
  int turn = map(Joystick1_X, 4768, -4767, -100, 100);

  moveWithTurn(speed, turn, 1);
}
void stopMotors() {
  for (speed1=200; speed1 != 0; speed1--) {
    frontRightMotor->setSpeed(speed1);
    frontLeftMotor->setSpeed(speed1);
    backRightMotor->setSpeed(speed1);
    backLeftMotor->setSpeed(speed1);
    // delay(5);
  }
  frontRightMotor->run(RELEASE);
  frontLeftMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
 // Serial.println("Motor stopped");
}
