// MyRobot-V1.ino
// This file must be named the same as your sketch folder
int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A


int enB = 5;     // Enable pin for Motor B
int in3 = 4;   // Direction control pin 1 for motor B
int in4 = 2;    // Direction control pin 2 for motor B

void setup() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);
    Serial.println("Use arrow keys to control the car")
}

void loop() {
    // put your main code here, to run repeatedly:

    // // Drive for 1 second
    // logInfo(Serial, "Driving");
    // drive(in1, in2, enA);
    // delay(1000);

    // // Stop for 1 second
    // logInfo(Serial, "Stopping");
    // stop(in1, in2, enA);
    // delay(1000);

    void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 27) {
            Serial.read();      
            char arrow = Serial.read();

            switch (arrow) {
                case 'A':  // UP
                    moveForward(in1, in2, enA, in3, in4, enB);
                    Serial.println("Forward");
                    break;

                case 'B':  // DOWN
                    moveBackward(in1, in2, enA, in3, in4, enB);
                    Serial.println("Backward");
                    break;

                case 'C':  // RIGHT
                    moveRight(in1, in2, enA, in3, in4, enB);
                    Serial.println("Right");
                    break;

                case 'D':  // LEFT
                    moveLeft(in1, in2, enA, in3, in4, enB);
                    Serial.println("Left");
                    break;
            }
        }
    }
}

}
