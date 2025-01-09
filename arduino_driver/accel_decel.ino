void moveWithTurn(int speed, int turn, int reverse) {
    int error = 0;

    // Map input values
    reverse = map(reverse, 0, 1020, 0, -MAX_SPEED);
    speed = map(speed, 0, 1020, 0, MAX_SPEED);
    turn = map(turn, -509, 509, MAX_SPEED / 1.2, -MAX_SPEED / 1.2);

    static int currentLeftSpeed = 0;
    static int currentRightSpeed = 0;
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    int step = (currentTime - lastTime) / RAMP_TIME;  // Incremental step based on time
    lastTime = currentTime;

    // Apply quadratic acceleration or deceleration
    if (reverse == 0 && (speed != 0 || turn != 0)) {
        int targetLeftSpeed = speed - turn;
        int targetRightSpeed = speed + turn;

        // Constrain target speed
        targetLeftSpeed = constrain(targetLeftSpeed, MIN_SPEED, MAX_SPEED);
        targetRightSpeed = constrain(targetRightSpeed, MIN_SPEED, MAX_SPEED);

        // Quadratic acceleration curve for speeding up
        if (currentLeftSpeed < targetLeftSpeed) {
            currentLeftSpeed += (targetLeftSpeed - currentLeftSpeed) * (step / 255.0) * (step / 255.0);
            if (currentLeftSpeed > targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
        } else if (currentLeftSpeed > targetLeftSpeed) {
            currentLeftSpeed -= (currentLeftSpeed - targetLeftSpeed) * (step / 255.0) * (step / 255.0);
            if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
        }

        if (currentRightSpeed < targetRightSpeed) {
            currentRightSpeed += (targetRightSpeed - currentRightSpeed) * (step / 255.0) * (step / 255.0);
            if (currentRightSpeed > targetRightSpeed) currentRightSpeed = targetRightSpeed;
        } else if (currentRightSpeed > targetRightSpeed) {
            currentRightSpeed -= (currentRightSpeed - targetRightSpeed) * (step / 255.0) * (step / 255.0);
            if (currentRightSpeed < targetRightSpeed) currentRightSpeed = targetRightSpeed;
        }

        setMotorSpeed(currentLeftSpeed, currentRightSpeed);
    } else {
        // If the controller is released (speed == 0, turn == 0), decelerate to zero gradually
        int targetLeftSpeed = 0;
        int targetRightSpeed = 0;

        // Apply quadratic deceleration curve to slow down
        if (currentLeftSpeed > targetLeftSpeed) {
            currentLeftSpeed -= (currentLeftSpeed) * (1 - (step / 255.0)) * (1 - (step / 255.0));
            if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
        }

        if (currentRightSpeed > targetRightSpeed) {
            currentRightSpeed -= (currentRightSpeed) * (1 - (step / 255.0)) * (1 - (step / 255.0));
            if (currentRightSpeed < targetRightSpeed) currentRightSpeed = targetRightSpeed;
        }

        setMotorSpeed(currentLeftSpeed - error, currentRightSpeed);
    }

    // Print motor speeds for debugging
    Serial.print("leftSpeed: ");
    Serial.print(abs(currentLeftSpeed) - error);
    Serial.print(" rightSpeed: ");
    Serial.println(currentRightSpeed);
}
