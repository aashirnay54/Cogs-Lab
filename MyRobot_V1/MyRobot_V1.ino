

#include <WiFiS3.h>  // Correct library for Uno R4 WiFi

const char* ssid     = "nhu";
const char* password = "44444444";

int enA = 9;   // PWM pin for Motor A
int in1 = 8;
int in2 = 7;

int enB = 5;   // PWM pin for Motor B
int in3 = 4;
int in4 = 2;

// --- Motor speed ---
int motorSpeed = 200; // 0-255

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);

  // --- Motor pins ---
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  // --- Connect to Wi-Fi ---
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
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
                stop(in1, in2, 3, 4, enA, enB);
                Serial.println("STOP");
                break;
         
        }

    }



    
}

