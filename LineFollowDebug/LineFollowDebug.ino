// ============================================
// LINE FOLLOWING DEBUG SKETCH
// Simple code to test and debug line following
// ============================================

// Motor A pins
const int enA = 11;  // PWM
const int in1 = 13;
const int in2 = 12;

// Motor B pins
const int enB = 9;   // PWM
const int in3 = 8;
const int in4 = 10;

// IR sensors (0 = on tape/black, 1 = off tape/white)
const int irLeft   = A2;
const int irRight  = A1;
const int irMiddle = A3;

// Timing
unsigned long lastPrint = 0;
const int PRINT_INTERVAL = 100;

void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // IR sensor pins
    pinMode(irLeft, INPUT);
    pinMode(irRight, INPUT);
    pinMode(irMiddle, INPUT);

    stop();

    Serial.println("========================================");
    Serial.println("LINE FOLLOWING DEBUG");
    Serial.println("========================================");
    Serial.println("Commands:");
    Serial.println("  s = Start line following");
    Serial.println("  e = Stop");
    Serial.println("  w = Test forward");
    Serial.println("  a = Test turn left");
    Serial.println("  d = Test turn right");
    Serial.println("========================================");
    Serial.println("IR Sensors: 0 = ON tape (black), 1 = OFF tape (white)");
    Serial.println("========================================");
}

bool followingLine = false;

void loop() {
    unsigned long now = millis();

    // Read IR sensors
    int irL = digitalRead(irLeft);
    int irM = digitalRead(irMiddle);
    int irR = digitalRead(irRight);

    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 's') {
            followingLine = true;
            Serial.println("# STARTING LINE FOLLOW");
        }
        else if (cmd == 'e') {
            followingLine = false;
            stop();
            Serial.println("# STOPPED");
        }
        else if (cmd == 'w') {
            followingLine = false;
            Serial.println("# TEST: Forward");
            moveForward();
            delay(500);
            stop();
        }
        else if (cmd == 'a') {
            followingLine = false;
            Serial.println("# TEST: Turn Left");
            turnLeft();
            delay(500);
            stop();
        }
        else if (cmd == 'd') {
            followingLine = false;
            Serial.println("# TEST: Turn Right");
            turnRight();
            delay(500);
            stop();
        }
    }

    // Print sensor values periodically
    if (now - lastPrint >= PRINT_INTERVAL) {
        Serial.print("IR: L="); Serial.print(irL);
        Serial.print(" M="); Serial.print(irM);
        Serial.print(" R="); Serial.print(irR);
        Serial.print("  |  ");

        // Show what state this represents
        if (irL == 0 && irM == 0 && irR == 0) {
            Serial.print("[ALL ON TAPE]");
        } else if (irL == 1 && irM == 0 && irR == 1) {
            Serial.print("[CENTERED]");
        } else if (irL == 0 && irM == 0 && irR == 1) {
            Serial.print("[DRIFT RIGHT]");
        } else if (irL == 1 && irM == 0 && irR == 0) {
            Serial.print("[DRIFT LEFT]");
        } else if (irL == 0 && irM == 1 && irR == 1) {
            Serial.print("[FAR RIGHT]");
        } else if (irL == 1 && irM == 1 && irR == 0) {
            Serial.print("[FAR LEFT]");
        } else if (irL == 1 && irM == 1 && irR == 1) {
            Serial.print("[OFF TAPE]");
        }

        Serial.println();
        lastPrint = now;
    }

    // Line following logic
    if (followingLine) {
        if (irM == 0 && irL == 1 && irR == 1) {
            // Centered - go straight
            moveForward();
        }
        else if (irM == 0 && irL == 0 && irR == 1) {
            // Drifted right, turn left
            turnLeftSlow();
        }
        else if (irM == 0 && irR == 0 && irL == 1) {
            // Drifted left, turn right
            turnRightSlow();
        }
        else if (irL == 0 && irM == 1 && irR == 1) {
            // Far right, hard left
            turnLeft();
        }
        else if (irR == 0 && irM == 1 && irL == 1) {
            // Far left, hard right
            turnRight();
        }
        else if (irL == 0 && irM == 0 && irR == 0) {
            // All on tape - go straight
            turnRight();
        }
        
        
        else {
            // Off tape - move backward a tiny bit then stop
            moveBackward();
            delay(250);  // Small backward movement
            stop();
        }
    }
}

// ============================================
// MOTOR FUNCTIONS - From Motor.ino
// ============================================

// void stop() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 0);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 0);
// }

// void moveForward() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 57);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 77);
// }

// void moveBackward() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 57);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 77);
// }

// // Gentle correction (SWAPPED)
// void turnLeftSlow() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 70);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 0);
// }

// void turnRightSlow() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 0);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 80);
// }

// // Aggressive correction (SWAPPED)
// void turnLeft() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 70);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 0);
// }

// void turnRight() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 0);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 80);
// }

// void turnRightShimmy() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 0);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 57);
// }

// void turnLeftShimmy() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 60);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 0);

// }