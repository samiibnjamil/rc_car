#include <Wire.h>
#include <Adafruit_MotorShield.h>
int speed = 0;
int turn = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *frontRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *frontLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(4);

int button_x, button_y, button_a, button_b, joystickL_x, joystickL_y, joystickR_x, joystickR_y, L2_Button, R2_Button;

void setup() {
  Serial.begin(115200); // Increased baud rate
  Serial.println("Adafruit Motorshield v2 - Differential Drive Car Control");

  AFMS.begin();  // Create with the default frequency 1.6KHz
}

void loop() {
  readSerialData();
  // Your other loop code goes here
}

void readSerialData() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseInput(data);
  }
}

void parseInput(String input) {
  // Parse the input string using sscanf
  int parsedValues = sscanf(input.c_str(), "X,%d,Y,%d,A,%d,B,%d,JoystickL_X,%d,JoystickL_Y,%d,JoystickR_X,%d,JoystickR_Y,%d,L2,%d,R2,%d",
                            &button_x, &button_y, &button_a, &button_b,
                            &joystickL_x, &joystickL_y, &joystickR_x, &joystickR_y,
                            &L2_Button, &R2_Button);
  //Serial.println(input);
  moveWithTurn(R2_Button, joystickL_x,L2_Button);


}


void moveWithTurn(int speed, int turn, int reverse) {

  int leftSpeed, rightSpeed;
   int error=1;
  reverse= map(reverse, 0, 255, 0, -150);
  speed = map(speed, 0, 255, 0, 250);  // Map speed to motor range
  turn = map(turn, 1, 255, -250, 250);    // Map turn to motor range

  if (reverse ==0){

  leftSpeed = speed - turn;
  rightSpeed = speed + turn;

  // Constrain motor speeds to allowable range
  leftSpeed = constrain(leftSpeed, -250, 250);
  rightSpeed = constrain(rightSpeed, -250, 250);

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
    
  }
  else{
    
   leftSpeed = reverse + turn;
   rightSpeed = reverse - turn;

  // Constrain motor speeds to allowable range
  leftSpeed = constrain(leftSpeed, -150, 150);
  rightSpeed = constrain(rightSpeed, -150, 150);
  

  // Set motor speeds
  frontLeftMotor->setSpeed(abs(leftSpeed)-error);
  backLeftMotor->setSpeed(abs(leftSpeed)-error);
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
  }


  Serial.print("leftSpeed: ");
  Serial.print(abs(leftSpeed)-error);
  Serial.print(" rightSpeed: ");
  Serial.println(rightSpeed);
  

}
