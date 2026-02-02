<<<<<<< HEAD
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

}

void loop() {
    // put your main code here, to run repeatedly:

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



=======
/\*ESP Recovery sketch\*/
#include <Wire.h>
#include <UnoWiFiDevEd.h>

#define BAUD 9600
WifiData ESPSerial;

void setup()
{
Serial.begin(BAUD);
pinMode(13, OUTPUT);
digitalWrite(13, LOW);

ESPSerial.begin(BAUD);
}

void loop()
{
while (Serial.available()) {
char inChar = (char)Serial.read();
ESPSerial.write(inChar);
}

while (ESPSerial.available()) {
char inChar = (char)ESPSerial.read();
Serial.write(inChar);
}

}
>>>>>>> 5fe247cf053b24d3115716488d721a1db4f39346
