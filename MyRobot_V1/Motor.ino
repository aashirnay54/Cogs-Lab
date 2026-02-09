/**
 * @file Motor.ino
 * @brief Simple H-bridge motor control helpers.
 *
 * Provides a minimal interface for driving a DC motor using
 * digital GPIO pins (e.g. Arduino-style platforms).
 *
 * The motor direction is controlled via two input pins,
 * while a separate enable pin turns the motor on or off.
 *
 * @author Paul Bucci
 * @date 2026
 */

/**
 * @brief Drives a DC motor in a fixed direction using an H-bridge.
 *
 * @param in1 GPIO pin connected to motor driver input 1 (direction control)
 * @param in2 GPIO pin connected to motor driver input 2 (direction control)
 * @param enA GPIO pin connected to motor driver enable pin (motor on/off)
 */



void drive() {
    digitalWrite(in1, LOW);   // Direction control: IN1
    digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
    digitalWrite(enA, HIGH);  // Enable motor driver
}

void stop() {
    digitalWrite(in1, LOW);   // Direction control: IN1
    digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
    analogWrite(enA, 0);   // Disable motor driver
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 0);   // Disable motor driver

}

// TODO: add your own driving functions here

void moveForward() {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    analogWrite(enA, 255);
    
    delay(40);
    
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    analogWrite(enB, 240);  
}


void moveBackward() {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 200);  

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 200);  

}

void moveRight() {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 180);  

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 0);  

}

void moveLeft() {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 0);  

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 180);  

}

void turnRobotInPlace() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 255);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 255);

}
