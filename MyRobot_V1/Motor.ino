/**
 * @file Motor.ino
 * @brief H-bridge motor control helpers.
 * @author Paul Bucci
 * @date 2026
 */

void drive() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(enA, HIGH);
}

void stop() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
}

void moveBackward() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 100);  // Left motor — max, it needs it due to stiffer gearbox

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 80);  // Right motor — tuned to match left encoder counts
}

void moveForward() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 100);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 80);
}

void moveLeft() {
    // Left motor drives forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 100);

    // Right motor coasts — both pins LOW avoids stall hum
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
}

void moveRight() {
    // Left motor coasts — both pins LOW avoids stall hum
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    // Right motor drives forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 100);
}

void turnRobotInPlace() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 155);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 155);
}

// Variable-speed versions used by the P-controller (Follow Me mode)
void moveBackwardPWM(int speed) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, speed);
}

void moveForwardPWM(int speed) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);
}

// Slow pivot for tape following — less overshoot on corners
void turnRightSlow() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);        // left motor coast

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 70);      // right motor slow
}

void turnLeftSlow() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 55);      // left motor slow

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);        // right motor coast
}

// Aggressive pivot for sharp corners — both motors active, opposite directions
void turnRight() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 80);      // left motor backward

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 80);      // right motor forward
}

void turnLeft() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 80);      // left motor forward

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 80);      // right motor backward
}