#include <Adafruit_MotorShield.h>
#include <IRremote.h>
#include <QMC5883LCompass.h>

#this is a test

double error ;
int RECV_PIN = A3;
uint8_t speed1;
char serialData;
unsigned long previousCommand = 0; // To store the previous valid command
IRrecv irrecv(RECV_PIN);
decode_results results;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// Initialize the compass
QMC5883LCompass compass;

#define FORWD 1
#define BACKWD 0
#define LEFT 2
#define RIGHT 3
int x, y, z, a, b;

double targetAngle = 45.0; // Target angle (in degrees)
double input, output;

const unsigned long sampleTime = 100; // PID loop interval in milliseconds


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  irrecv.enableIRIn(); // Start the receiver

  //compass.setCalibrationOffsets(-1563.00, -2390.00, -1143.00);
  //compass.setCalibrationScales(1.09, 0.71, 1.49);

  // Initialize the compass
  compass.init();
  compass.setCalibrationOffsets(-605.00, -713.00, -808.00);
  compass.setCalibrationScales(0.92, 1.03, 1.06);
  

}
void loop() {
  if (Serial.available() > 0) {
    serialData = Serial.read();
    processSerialCommand(serialData);
  }

  // Continue with the rest of your loop logic as needed
}
void loop2() {
 // getDir();
  if (irrecv.decode(&results)) {
    Serial.print("Before moving: ");
    getDir();
    Serial.print("Received IR signal: 0x");
    Serial.println(results.value, HEX);

    if (results.value == 0xFF48B7) {
      Serial.println("fwd");
      previousCommand = results.value;
      move(FORWD,1000, 100);

    } else if (results.value == 0xFF7887) {
      Serial.println("bck");
      previousCommand = results.value;
      move(BACKWD,500, 100);
    } else if (results.value == 0xFF02FD) {
      Serial.println("lft");
      previousCommand = results.value;
      move(LEFT,700, 100);
    } else if (results.value == 0xFF32CD) {
      Serial.println("stp");
      previousCommand = results.value;
      stopMotors();
    } else if (results.value == 0xFF20DF) {
      Serial.println("rgt");
      previousCommand = results.value;
      move(RIGHT,0, 700);
    } else if (results.value == 0xFF708F) {
      Serial.println("rgt oh yeahj 90");
      previousCommand = results.value;
      moveInCircle(LEFT, 30 , 3000);
    } else if (results.value == 0xFF30CF) {
      Serial.println("looping");
      looping();
    } else if (results.value == 0xFFFFFFFF && previousCommand != 0) {
      // Continue the previous task
      if (previousCommand == 0xFF48B7) {
        Serial.println("fwd (continue)");
        move(FORWD,0, 100);
      } else if (previousCommand == 0xFF7887) {
        Serial.println("bck (continue)");
        move(BACKWD,0, 100);
      } else if (previousCommand == 0xFF02FD) {
        Serial.println("lft (continue)");
        move(LEFT,0, 100);
      } else if (previousCommand == 0xFF32CD) {
        Serial.println("stp (continue)");
        stopMotors();
      } else if (previousCommand == 0xFF20DF) {
        Serial.println("rgt (continue)");
        move(RIGHT,0, 0);
      } else if (previousCommand == 0xFF708F) {
        Serial.println("rgt 90");
       // rotateToTargetAngleNoPID();
      }
    }
    irrecv.resume(); // Receive the next value
    Serial.print(" After moving: ");
    getDir();
  }

  //  if (error >10){
  //    rotateToTargetAngleNoPID();
  //
  //  }
  // Now, integrate compass functionality

}

void processSerialCommand(char command) {
  switch (command) {
    case 'W':
      move(FORWD,1000, 100);
      break;
    case 'X':
      move(BACKWD,1000, 100);
      break;
    case 'A':
      move(RIGHT,1000, 100);
      break;
    case 'D':
      move(LEFT,1000, 100);
      break;
    case 'S':
      stopMotors();
      break;
    case 'C':
      moveInCircle(LEFT, 30, 3000);
      break;
    default:
      // Handle unknown command or do nothing
      break;
  }
}
void getDir() {

  char myArray[3];

  compass.read();
  a = map(compass.getAzimuth(), -180, 180, 0, 360);
  b = compass.getBearing(a);
  compass.getDirection(myArray, a);


  Serial.print(" Azimuth: ");
  Serial.print(a);

  Serial.print(" Bearing: ");
  Serial.print(b);

  Serial.print(" Direction: ");
  Serial.print(myArray[0]);
  Serial.print(myArray[1]);
  Serial.print(myArray[2]);

  Serial.println();

  delay(5);
}


void move(uint8_t direction,int time,int speed) {
  unsigned long startTime = millis(); // Record the start time
  //uint8_t i;
  if (speed==0){
    speed=155;
  }
  for (speed1 = 20; speed1 < speed; speed1++) {

    if (direction == FORWD){
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);
      
    }
    else if(direction == BACKWD){
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(BACKWARD);
      
    }
    else if(direction == LEFT){
      //speed1=50;
      //speed=0;
   myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(BACKWARD);
      
    }
        else if(direction == RIGHT){
          //speed=150;
         // speed1=50;
   myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(FORWARD);
      
    }
    else{
       Serial.println("Error! Not Enough Argument to move");
    }


    myMotor1->setSpeed(speed1);
    myMotor2->setSpeed(speed1);
    myMotor3->setSpeed(speed1);
    myMotor4->setSpeed(speed1);
    // delay(10);
    // Serial.println(speed1);
  }
 // Serial.println("Moving Forward");
  if (time > 0) {

    while (millis() - startTime < time) {
      irrecv.resume();

    }

    // Continue to run motors for 1 second
    // This loop won't block the program execution
    stopMotors();
  }
  irrecv.resume();
}



void stopMotors() {
  // int i = 0;
  for (speed1; speed1 != 0; speed1--) {
    myMotor1->setSpeed(speed1);
    myMotor2->setSpeed(speed1);
    myMotor3->setSpeed(speed1);
    myMotor4->setSpeed(speed1);
    // delay(5);
  }
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
  Serial.println("Motor stopped");
}


void rotateToTargetAngle2() {
  compass.read();
  a = map(compass.getAzimuth(), -180, 180, 0, 360);
  int target_azimuth = a + 15;  // Calculate the target azimuth

  int rotation_tolerance = 5;
  getDir();
  Serial.println("target_azimuth");
  Serial.println(target_azimuth);
  Serial.println(target_azimuth - a);
move(RIGHT,0, 2550);
  while (target_azimuth - a > 5) {


    compass.read();
    a = map(compass.getAzimuth(), -180, 180, 0, 360);
    Serial.println("target_azimuth");
    Serial.println(target_azimuth);
    Serial.println("a");
    Serial.println(a);
    Serial.println("target_azimuth-a");
    Serial.println(target_azimuth - a);
    if (target_azimuth - a > 360) {
      target_azimuth = target_azimuth - 360;
      Serial.println("extra ran target_azimuth=target_azimuth-360; ");
    }
    irrecv.resume();
  }
stopMotors();
}


void rotateToTargetAngle() {
  compass.read();
  a = map(compass.getAzimuth(), -180, 180, 0, 360);
  
  // Map azimuth from -180 to 180 to 0 to 360
  int target_azimuth = a + 90;

  int rotation_tolerance = 2;
  getDir();
  Serial.print("target_azimuth: ");
  Serial.print(target_azimuth);
  Serial.print(", Difference: ");
  Serial.print(target_azimuth - a);
  move(RIGHT,0, 25);
int previous_difference = 600;
  while (true) {
    compass.read();
    a =  map(compass.getAzimuth(), -180, 180, 0, 360);
   // target_azimuth = map(a, -180, 180, 0, 360) + 15;

      // Handle azimuth rollover from 0 to 360
    if (target_azimuth > 360) {
      target_azimuth -= 360;
    }

  if (irrecv.decode(&results)) {
      // Process IR signal if needed
      irrecv.resume(); // Receive the next value
      break;
    }
    Serial.print(" | target_azimuth: ");
    Serial.print(target_azimuth);
    Serial.print(", Current azimuth: ");
    Serial.print(a);
    int absolute_difference = min((target_azimuth - a + 360) % 360, (a - target_azimuth + 360) % 360);

    Serial.print(", Difference: ");
    Serial.println(absolute_difference);

  
    // Check if within tolerance
 if (absolute_difference <= rotation_tolerance ) {
      break;  // Exit the loop when the target is reached and the difference is decreasing
    }
previous_difference=absolute_difference;
    irrecv.resume();
    //delay(10);  // Add a small delay to control the loop update rate
  }
  stopMotors();
}



void looping() {
  move(FORWD,1500, 100);
  delay(2500);
  move(LEFT,1700, 100);
  delay(500);
  move(FORWD,1500, 100);
  delay(1500);
  move(LEFT,1700, 100);
  delay(500);
  move(FORWD,1500, 100);
  delay(1500);
  move(LEFT,1700, 100);
  delay(500);
  move(FORWD,1500, 100);
  delay(1500);
  move(LEFT,1700, 100);
  delay(500);
}

void moveInCircle(int direction, int radius, int time) {
  unsigned long startTime = millis(); // Record the start time

  int speed = 100; // Adjust this speed as needed

  // Calculate speeds for left or right turn based on the radius
  int leftSpeed = speed;
  int rightSpeed = (radius - 5) * speed / (radius + 5); // Adjust this formula based on your robot's behavior

  if (direction == LEFT) {
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(BACKWARD);
  } else if (direction == RIGHT) {
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(FORWARD);
  }

  myMotor1->setSpeed(leftSpeed);
  myMotor2->setSpeed(leftSpeed);
  myMotor3->setSpeed(rightSpeed);
  myMotor4->setSpeed(rightSpeed);

  Serial.println("Moving in a circular path");

  if (time > 0) {
    while (millis() - startTime < time) {
      // Continue to run motors for the specified time duration
      // This loop won't block the program execution
      irrecv.resume();
    }
    stopMotors();
  }
  irrecv.resume();
}
