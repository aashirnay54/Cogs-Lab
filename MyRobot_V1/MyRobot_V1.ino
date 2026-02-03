// MyRobot-V1.ino
// This file must be named the same as your sketch folder
int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A

int enB = 5;     // Enable pin for Motor B
int in3 = 4;   // Direction control pin 1 for motor B
int in4 = 2;    // Direction control pin 2 for motor B

const 
int D1 = 11;
int D2 = 3;


void setup() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(D1, INPUT_PULLUP);   // use pullups; pressed/LOW = 0
    pinMode(D2, INPUT_PULLUP);
    Serial.begin(115200);

    Serial.begin(9600);

}

void loop() {
    // put your main code here, to run repeatedly:

    // Encoder Code

    int v1 = digitalRead(D1);    // 0 or 1
    int v2 = digitalRead(D2);    // 0 or 1
    Serial.print(v1); Serial.print(',');
    Serial.println(v2);          // newline-terminated
    delay(5);                    // ~200 Hz (adjust as needed)

    if (Serial.available()) {
        char c = Serial.read();

        switch (c) {
            case 'w':
                moveForward(in1, in2, enA, in3, in4, enB);
                Serial.println("FORWARD");
                break;

            case 's':
                moveBackward(in1, in2, enA, in3, in4, enB);
                Serial.println("BACKWARD");
                break;

            case 'a':
                moveLeft(in1, in2, enA, in3, in4, enB);
                Serial.println("LEFT");
                break;

            case 'd':
                moveRight(in1, in2, enA, in3, in4, enB);
                Serial.println("RIGHT");
                break;

            case 'q':
                turnRobotInPlace(in1, in2, enA, in3, in4, enB);
                Serial.println("IN PLACE");
                break;

            case 'e':
                stop(in1, in2, enA, in3, in4, enB);
                Serial.println("STOP");
                break;

        }
    }
    
}
