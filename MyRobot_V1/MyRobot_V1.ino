// #include <WiFiS3.h>  // Commented out for Serial Monitor control
#include <Servo.h>

// // ── Access Point credentials ──────────────────────────────────────
// char ssid[] = "RobotAP";
// char pass[] = "robot1234";
// WiFiServer server(80);
// // ─────────────────────────────────────────────────────────────────

// Motor A pins
const int enA = 11;  // PWM ~
const int in1 = 13;
const int in2 = 12;

// Motor B pins
const int enB =  9;  // PWM ~
const int in3 =  8;
const int in4 = 10;

// Encoder pins (interrupt-capable: 2, 3)
const int encA = 3;
const int encB = 2;

// Ultrasonic sensor pins
const int trigPin = 6;
const int echoPin = 7;

// Servo for object detection
Servo sonarServo;
const int servoPin = A5;  // Using A5 for servo

// IR sensors (0 = on tape, 1 = off tape)
const int irLeft   = A2;
const int irRight  = A1;
const int irMiddle = A3;

// Moving average filter for ultrasonic
const int US_NUM_READINGS = 5;
float usReadings[US_NUM_READINGS];
int   usReadIndex = 0;
float usTotal     = 0.0;

// P-Controller parameters
const float SET_POINT = 25.0;
const float Kp        = 10.0;
const int   DEAD_ZONE = 2;
const int   MIN_PWM   = 60;
const int   MAX_PWM   = 255;
const float MAX_RANGE = 200.0;
const float MIN_RANGE = 2.0;

// State
bool autonomousMode = false;
bool followMode     = false;
bool tapeFollowMode = false;
bool objectDetectMode = false;
int state = 0;
char currentCommand = 'e';

// Encoder
volatile long encoderACount = 0;
volatile long encoderBCount = 0;
int lastEncAState = 0;
int lastEncBState = 0;

// Recording/Replay
bool recording = false;
bool replaying  = false;
unsigned long recordStart = 0;
unsigned long replayStart = 0;
String recordedCommands   = "";
int replayPos = 0;

// Follow mode telemetry
float lastFilteredDistance = 0.0;
float lastError  = 0.0;
float lastOutput = 0.0;

// Tape follow
char lastTurnDir = 'S';
unsigned long deadZoneStart       = 0;
const unsigned long TAPE_END_TIMEOUT = 500;

// Object detection with depth map
const int NUM_ANGLES = 9;                     // 0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180 degrees
const int ANGLE_STEP = 180 / (NUM_ANGLES - 1); // 22.5 degrees
float depthMap[NUM_ANGLES];                   // stores distance at each angle
int angleMap[NUM_ANGLES];                     // stores the actual angles
int targetAngle = 90;                         // angle to turn towards (0-180, 90 is center)
float minObjectDistance = 200.0;              // for tracking closest object
const float WALL_DISTANCE_THRESHOLD = 50.0;   // distances > this are likely the wall
const float OBJECT_DETECTION_RANGE = 100.0;   // max range to consider for objects

// Telemetry
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL    = 500;

void setup() {
    Serial.begin(115200);

    pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(irLeft,   INPUT);
    pinMode(irRight,  INPUT);
    pinMode(irMiddle, INPUT);

    // Initialize servo for object detection
    sonarServo.attach(servoPin, 500, 2400);
    sonarServo.write(90);  // center position
    delay(1000);

    // Initialize angle map
    for (int i = 0; i < NUM_ANGLES; i++) {
        angleMap[i] = i * ANGLE_STEP;
        depthMap[i] = 0.0;
    }

    lastEncAState = digitalRead(encA);
    lastEncBState = digitalRead(encB);

    for (int i = 0; i < US_NUM_READINGS; i++) usReadings[i] = SET_POINT;
    usTotal = SET_POINT * US_NUM_READINGS;

    // // ── Access Point Setup ────────────────────────────────────────
    // Serial.println("Creating Access Point...");
    // if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    //     Serial.println("AP failed!");
    //     while (true);
    // }
    // Serial.println("AP created!");
    // Serial.print("SSID: "); Serial.println(ssid);
    // Serial.print("IP:   "); Serial.println(WiFi.localIP());
    // server.begin();
    // Serial.println("Server started on port 80");
    // // ─────────────────────────────────────────────────────────────

    Serial.println("# Serial Monitor Control Ready");
    Serial.println("# Commands: o=object detect, v=test servo, f=follow, t=tape, e=stop, w/s/a/d=move");
}

void loop() {
    unsigned long now = millis();

    // Read encoders
    int encAState = digitalRead(encA);
    int encBState = digitalRead(encB);
    if (encAState != lastEncAState && encAState == HIGH) encoderACount++;
    if (encBState != lastEncBState && encBState == HIGH) encoderBCount++;
    lastEncAState = encAState;
    lastEncBState = encBState;

    // Read ultrasonic every 50ms
    static unsigned long lastUsReadTime = 0;
    if (now - lastUsReadTime >= 50) {
        float rawDistance = readUltrasonic();
        if (rawDistance >= MIN_RANGE && rawDistance <= MAX_RANGE) {
            usTotal -= usReadings[usReadIndex];
            usReadings[usReadIndex] = rawDistance;
            usTotal += usReadings[usReadIndex];
            usReadIndex = (usReadIndex + 1) % US_NUM_READINGS;
        }
        lastFilteredDistance = usTotal / US_NUM_READINGS;
        lastUsReadTime = now;
    }

    // // ── Handle HTTP client ────────────────────────────────────────
    // WiFiClient client = server.available();
    // if (client) {
    //     String request = client.readStringUntil('\r');
    //     client.flush();

    //     char cmd = 0;
    //     int slashIdx = request.indexOf('/');
    //     if (slashIdx >= 0 && slashIdx + 1 < request.length()) {
    //         cmd = request.charAt(slashIdx + 1);
    //     }

    //     client.println("HTTP/1.1 200 OK");
    //     client.println("Content-Type: text/plain");
    //     client.println("Connection: close");
    //     client.println();
    //     client.println("OK");
    //     client.stop();

    //     if (cmd != 0 && !replaying) {
    //         currentCommand = cmd;

    // ── Handle Serial commands ────────────────────────────────────
    char cmd = 0;
    if (Serial.available() && !replaying) {
        cmd = Serial.read();
        // Skip newlines and carriage returns
        while (cmd == '\n' || cmd == '\r') {
            if (Serial.available()) {
                cmd = Serial.read();
            } else {
                cmd = 0;
                break;
            }
        }

        if (cmd != 0) {
            currentCommand = cmd;

            if (cmd == 'x') {
                recording = true;
                recordStart = now;
                recordedCommands = "";
                encoderACount = 0;
                encoderBCount = 0;
                Serial.println("# RECORDING STARTED");
            }
            else if (cmd == 'z') {
                recording = false;
                Serial.println("# RECORDING STOPPED");
                Serial.print("# Commands: ");
                Serial.println(recordedCommands);
            }
            else if (cmd == 'p') {
                replaying = true;
                replayStart = now;
                replayPos = 0;
                encoderACount = 0;
                encoderBCount = 0;
                Serial.println("# REPLAYING");
            }
            else if (cmd == 'v') {
                stop();
                Serial.println("# Testing servo...");
                testServo();
            }
            else if (cmd == 'f') {
                stop();
                followMode = true;
                autonomousMode = false;
                tapeFollowMode = false;
                objectDetectMode = false;
                Serial.println("# FOLLOW MODE");
            }
            else if (cmd == 't') {
                stop();
                tapeFollowMode = true;
                autonomousMode = false;
                followMode = false;
                objectDetectMode = false;
                lastTurnDir = 'S';
                deadZoneStart = 0;
                Serial.println("# TAPE FOLLOW MODE");
            }
            else if (cmd == 'o') {
                stop();
                objectDetectMode = true;
                autonomousMode = false;
                followMode = false;
                tapeFollowMode = false;
                state = 0;
                sonarServo.write(90);  // center servo
                Serial.println("# OBJECT DETECTION MODE STARTED");
            }
            else if (cmd == 'e') {
                autonomousMode = false;
                followMode = false;
                tapeFollowMode = false;
                objectDetectMode = false;
                state = 0;
                stop();
                Serial.println("# MANUAL MODE");
            }
            else if (!autonomousMode && !followMode && !tapeFollowMode && !objectDetectMode) {
                executeCommand(cmd);
                if (recording) {
                    unsigned long elapsed = now - recordStart;
                    recordedCommands += String(elapsed) + ":" + cmd + ",";
                }
            }
        }
    }
    // // ─────────────────────────────────────────────────────────────

    // Replay logic
    if (replaying) {
        unsigned long elapsed = now - replayStart;
        int nextColon = recordedCommands.indexOf(':', replayPos);
        if (nextColon > 0) {
            unsigned long cmdTime = recordedCommands.substring(replayPos, nextColon).toInt();
            if (elapsed >= cmdTime) {
                char cmd = recordedCommands.charAt(nextColon + 1);
                executeCommand(cmd);
                currentCommand = cmd;
                replayPos = recordedCommands.indexOf(',', nextColon) + 1;
                if (replayPos >= recordedCommands.length() || replayPos == 0) {
                    replaying = false;
                    stop();
                    Serial.println("# REPLAY COMPLETE");
                }
            }
        }
    }

    // Follow-Me P-Controller
    if (followMode) {
        lastError  = SET_POINT - lastFilteredDistance;
        lastOutput = Kp * lastError;

        if (abs(lastError) < DEAD_ZONE) {
            stop();
        } else if (lastOutput > 0) {
            int pwm = constrain((int)abs(lastOutput), MIN_PWM, MAX_PWM);
            moveForwardPWM(pwm);
        } else {
            int pwm = constrain((int)abs(lastOutput), MIN_PWM, MAX_PWM);
            moveBackwardPWM(pwm);
        }
    }

    // Object detection mode
    if (objectDetectMode) {

    // ── State 0: Sweep ────────────────────────────────────────────
    if (state == 0) {
        Serial.println("# STATE 0: Performing sweep");
        stop();
        performSweep();
        state = 1;
    }

    // ── State 1: Analyze depth map ────────────────────────────────
    else if (state == 1) {
        Serial.println("# STATE 1: Analyzing depth map");
        int objectIdx = detectObject();
        if (objectIdx >= 0) {
            Serial.println("# Object found! Proceeding to turn");
            state = 2;
        } else {
            Serial.println("# No object found — rotating to explore");
            // Turn right in place (both motors active)
            digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(enA, 120);
            digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); analogWrite(enB, 120);
            delay(300);
            stop();
            delay(400);
            state = 0;
        }
    }

    // ── State 2: Turn to face object ──────────────────────────────
    else if (state == 2) {
        Serial.println("# STATE 2: Turning towards object");
        int turnDir = calculateTurnDirection();  // +ve = left (servo>90°), -ve = right (servo<90°)

        if (abs(turnDir) < 15) {
            Serial.println("# Aligned — moving forward");
            state = 3;
        } else {
            Serial.print("# Turning ");
            Serial.print(turnDir > 0 ? "LEFT " : "RIGHT ");
            Serial.print(abs(turnDir)); Serial.println("°");

            if (turnDir > 0) {
                // Object left of center — turn left in place
                digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(enA, 120);
                digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  analogWrite(enB, 120);
            } else {
                // Object right of center — turn right in place
                digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(enA, 120);
                digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); analogWrite(enB, 120);
            }

            delay(abs(turnDir) * 8);
            stop();
            delay(400);
            state = 0;  // re-sweep to verify alignment
        }
    }

    // ── State 3: Drive toward object ──────────────────────────────
    else if (state == 3) {
        // Center servo so sensor looks forward while driving
        sonarServo.write(90);
        delay(300);

        float liveDist = lastFilteredDistance;

        Serial.print("# STATE 3: Live distance = ");
        Serial.print(liveDist, 1); Serial.println(" cm");

        if (liveDist > 0 && liveDist <= 15.0) {
            // Reached the object
            stop();
            Serial.println("# ========================================");
            Serial.println("# OBJECT REACHED! Mission complete!");
            Serial.println("# ========================================");
            objectDetectMode = false;

        } else if (liveDist > 15.0 && liveDist <= 40.0) {
            // Close — creep forward then re-scan to stay aligned
            moveForwardPWM(100);
            delay(300);
            stop();
            delay(200);
            state = 0;

        } else if (liveDist > 40.0) {
            // Far — drive confidently then re-scan
            moveForwardPWM(180);
            delay(600);
            stop();
            delay(200);
            state = 0;

        } else {
            // No valid reading — re-scan
            stop();
            Serial.println("# No valid distance — re-scanning");
            state = 0;
        }
    }
}

    // Tape follow mode
    if (tapeFollowMode) {
        int irL = digitalRead(irLeft);
        int irM = digitalRead(irMiddle);
        int irR = digitalRead(irRight);

        if (irM == 0 && irL == 1 && irR == 1) {
            moveForward();
            lastTurnDir = 'S';
            deadZoneStart = 0;
        } else if (irM == 0 && irL == 0) {
            turnLeftSlow();
            lastTurnDir = 'L';
            deadZoneStart = 0;
        } else if (irM == 0 && irR == 0) {
            turnRightSlow();
            lastTurnDir = 'R';
            deadZoneStart = 0;
        } else if (irL == 0) {
            turnLeft();
            lastTurnDir = 'L';
            deadZoneStart = 0;
        } else if (irR == 0) {
            turnRight();
            lastTurnDir = 'R';
            deadZoneStart = 0;
        } else {
            if (deadZoneStart == 0) deadZoneStart = now;
            if (now - deadZoneStart < TAPE_END_TIMEOUT) {
                if      (lastTurnDir == 'L') turnLeft();
                else if (lastTurnDir == 'R') turnRight();
                else                         moveForward();
            } else {
                tapeFollowMode = false;
                stop();
                Serial.println("# TAPE ENDED");
            }
        }
    }

    // Telemetry
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print("Cmd: ");    Serial.print(currentCommand);
        Serial.print(" | EncA: ");Serial.print(encoderACount);
        Serial.print(" | EncB: ");Serial.print(encoderBCount);
        Serial.print(" | Dist: ");Serial.print(lastFilteredDistance, 1);
        Serial.print("cm | Err: ");Serial.print(lastError, 1);
        Serial.print(" | Out: "); Serial.println(lastOutput, 1);
        lastPrintTime = now;
    }
}

// ──────────────────── Ultrasonic ─────────────────────────────────
float readUltrasonic() {
    digitalWrite(trigPin, LOW);  delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);  delayMicroseconds(2);
    long duration = pulseIn(echoPin, HIGH, 11600);
    if (duration == 0 || duration < 150) return -1;
    float distance = (duration * 0.0343) / 2.0;
    if (distance < MIN_RANGE || distance > MAX_RANGE) return -1;
    return distance;
}

// ──────────────────── Object Detection ───────────────────────────
// Test servo full range of motion
void testServo() {
    Serial.println("# Testing servo range...");

    Serial.println("# Moving to 0 degrees (full right)");
    sonarServo.write(0);
    delay(2000);

    Serial.println("# Moving to 45 degrees");
    sonarServo.write(45);
    delay(2000);

    Serial.println("# Moving to 90 degrees (center)");
    sonarServo.write(90);
    delay(2000);

    Serial.println("# Moving to 135 degrees");
    sonarServo.write(135);
    delay(2000);

    Serial.println("# Moving to 180 degrees (full left)");
    sonarServo.write(180);
    delay(2000);

    Serial.println("# Returning to center (90)");
    sonarServo.write(90);
    delay(2000);
    Serial.println("# Servo test complete");
}

// Perform a 180-degree sweep and populate the depth map
void performSweep() {
    Serial.println("# Starting sweep...");

    for (int i = 0; i < NUM_ANGLES; i++) {
        int angle = angleMap[i];
        sonarServo.write(angle);
        delay(500);  // wait for servo to reach position

        // Take multiple readings and average them for better accuracy
        float sum = 0;
        int validReadings = 0;
        for (int j = 0; j < 3; j++) {
            float dist = readUltrasonic();
            if (dist > 0) {
                sum += dist;
                validReadings++;
            }
            delay(50);
        }

        if (validReadings > 0) {
            depthMap[i] = sum / validReadings;
        } else {
            depthMap[i] = MAX_RANGE;  // no valid reading, assume max range
        }

        Serial.print("Angle: "); Serial.print(angle);
        Serial.print("° | Distance: "); Serial.print(depthMap[i]);
        Serial.println(" cm");
    }

    // Return servo to center
    sonarServo.write(90);
    delay(300);
    Serial.println("# Sweep complete");
}

// Analyze depth map to find object (looks for closest point that's not the wall)
int detectObject() {
    minObjectDistance = MAX_RANGE;
    int objectIndex = -1;

    Serial.println("# Analyzing depth map...");

    // Find the minimum distance (closest object)
    for (int i = 0; i < NUM_ANGLES; i++) {
        if (depthMap[i] < minObjectDistance && depthMap[i] > MIN_RANGE) {
            minObjectDistance = depthMap[i];
            objectIndex = i;
        }
    }

    if (objectIndex == -1) {
        Serial.println("# No object detected");
        return -1;
    }

    // Check if this is a "bump" (object) vs linear wall
    // Look for local minimum - object should be closer than neighbors
    bool isObject = false;

    if (objectIndex > 0 && objectIndex < NUM_ANGLES - 1) {
        // Check if it's significantly closer than neighbors (bump detection)
        float leftDist = depthMap[objectIndex - 1];
        float rightDist = depthMap[objectIndex + 1];
        float centerDist = depthMap[objectIndex];

        // Object should be at least 10cm closer than average of neighbors
        float avgNeighbor = (leftDist + rightDist) / 2.0;
        if (avgNeighbor - centerDist > 10.0) {
            isObject = true;
        }
    } else {
        // Edge case - if at boundary and significantly closer than next point
        isObject = true;
    }

    if (isObject && minObjectDistance < OBJECT_DETECTION_RANGE) {
        targetAngle = angleMap[objectIndex];
        Serial.print("# Object detected at angle: "); Serial.print(targetAngle);
        Serial.print("°, distance: "); Serial.print(minObjectDistance);
        Serial.println(" cm");
        return objectIndex;
    } else {
        Serial.println("# No distinct object found (likely wall)");
        return -1;
    }
}

// Calculate how much to turn the robot to face the target angle
// Servo angles: 0° = right, 90° = forward, 180° = left
// Returns: negative = turn right, positive = turn left
int calculateTurnDirection() {
    int turnAmount = targetAngle - 90;  // 0 = no turn, + = left, - = right
    Serial.print("# Turn amount: "); Serial.print(turnAmount);
    Serial.println("°");
    return turnAmount;
}

// ──────────────────── Commands ────────────────────────────────────
void executeCommand(char c) {
    switch (c) {
        case 'w': moveForward();      break;
        case 's': moveBackward();     break;
        case 'a': moveLeft();         break;
        case 'd': moveRight();        break;
        case 'q': turnRobotInPlace(); break;
        case 'r': encoderACount = 0; encoderBCount = 0; break;
        case 'e': stop();             break;
    }
}

// ──────────────────── Motor Control ──────────────────────────────
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
//     analogWrite(enA, 150);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 150);
// }

// void moveForwardPWM(int pwm) {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, pwm);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, pwm);
// }

// void moveBackward() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 150);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 150);
// }

// void moveBackwardPWM(int pwm) {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, pwm);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, pwm);
// }

// void moveLeft() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 150);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 150);
// }

// void moveRight() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 150);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 150);
// }

// void turnLeft() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 150);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 150);
// }

// void turnRight() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 150);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 150);
// }

// void turnLeftSlow() {
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 100);
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 100);
// }

// void turnRightSlow() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 100);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 100);
// }

// void turnRobotInPlace() {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 150);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 150);
// }
