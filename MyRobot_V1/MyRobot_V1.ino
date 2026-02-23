// bug: not a persistent WiFi client (connection reset)

#include "WiFiS3.h"

// ── WiFi credentials ──────────────────────────────────────────────
char ssid[] = "Meli";
char pass[] = "melmel12";
int wifiStatus = WL_IDLE_STATUS;
WiFiServer server(3333);
WiFiClient persistentClient; // persistent — stays connected across loop iterations
// ─────────────────────────────────────────────────────────────────

// Motor A pins
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Motor B pins
const int enB = 5;
const int in3 = 4;
const int in4 = 2;

// Encoder pins
const int encA = 3;
const int encB = 11;

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 10;

// Photocell
const int photocellPin = A0;
int photocellValue = 0;
int Lightthreshold = 140;
int darkThreshold = 125;
int hysteresis = 10;

// Moving average filter for photocell
const int NUM_READINGS = 10;
int photocellReadings[NUM_READINGS];
int readIndex = 0;
long photocellTotal = 0;

// Moving average filter for ultrasonic
const int US_NUM_READINGS = 10;
float usReadings[US_NUM_READINGS];
int   usReadIndex  = 0;
float usTotal      = 0.0;

// P-Controller parameters
const float SET_POINT = 10.0;
const float Kp        = 3.0;
const int   DEAD_ZONE = 3;
const int   MIN_PWM   = 120;
const int   MAX_PWM   = 255;
const float MAX_RANGE = 200.0;
const float MIN_RANGE = 2.0;

// Calibration
int minReading = 1023;
int maxReading = 0;

// State
int state = 0;
bool autonomousMode = false;
bool followMode = false;

// Telemetry
unsigned long lastPrintTime = 0;
const int PRINT_INTERVAL = 1023;
char currentCommand = 'e';

// Encoder
volatile long encoderACount = 0;
volatile long encoderBCount = 0;
int lastEncAState = 0;
int lastEncBState = 0;

// Recording/Replay
bool recording = false;
bool replaying = false;
unsigned long recordStart = 0;
unsigned long replayStart = 0;
String recordedCommands = "";
int replayPos = 0;

// Photocell edge detection
int prev = 0;

// Follow mode telemetry
float lastFilteredDistance = 0.0;
float lastError = 0.0;
float lastOutput = 0.0;

// Wall-following 
bool wallFollowMode = false;
const float WALL_SETPOINT = 5.0;   // cm from wall
const float Kp_wall = 1.0;          // tune this
const int BASE_SPEED = 100;         // straight-line base PWM

//fixing motor imbalance
const int MOTOR_TRIM = 15;


void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    lastEncAState = digitalRead(encA);
    lastEncBState = digitalRead(encB);

    for (int i = 0; i < NUM_READINGS; i++) photocellReadings[i] = 0;

    for (int i = 0; i < US_NUM_READINGS; i++) usReadings[i] = SET_POINT;
    usTotal = SET_POINT * US_NUM_READINGS;

    Serial.println("time_ms,cmd,encA_count,encB_count,photocell,state,distance_cm,error,output");

    // ── WiFi Setup ───────────────────────────────────────────────
    // if (WiFi.status() == WL_NO_MODULE) {
    //     Serial.println("WiFi module not found!");
    //     while (true);
    // }

    // while (wifiStatus != WL_CONNECTED) {
    //     Serial.print("Connecting to: ");
    //     Serial.println(ssid);
    //     wifiStatus = WiFi.begin(ssid, pass);
    //     delay(10000);
    // }

    // Serial.println("Connected to WiFi!");
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.localIP());

    // server.begin();
    // Serial.println("TCP server started on port 3333");
    // ─────────────────────────────────────────────────────────────
}

void loop() {
    unsigned long now = millis();

    // ── WiFi Reconnect ───────────────────────────────────────────
    // if (WiFi.status() != WL_CONNECTED) {
    //     Serial.println("WiFi disconnected! Retrying...");
    //     wifiStatus = WiFi.begin(ssid, pass);
    //     delay(10000);
    //     if (wifiStatus == WL_CONNECTED) {
    //         Serial.println("Reconnected!");
    //         Serial.print("IP Address: ");
    //         Serial.println(WiFi.localIP());
    //     }
    //     return;
    // }
    // ─────────────────────────────────────────────────────────────

    // Read encoders
    int encAState = digitalRead(encA);
    int encBState = digitalRead(encB);
    if (encAState != lastEncAState && encAState == HIGH) encoderACount++;
    if (encBState != lastEncBState && encBState == HIGH) encoderBCount++;
    lastEncAState = encAState;
    lastEncBState = encBState;

    // Read photocell with moving average filter
    photocellTotal -= photocellReadings[readIndex];
    photocellReadings[readIndex] = analogRead(photocellPin);
    photocellTotal += photocellReadings[readIndex];
    readIndex = (readIndex + 1) % NUM_READINGS;
    photocellValue = photocellTotal / NUM_READINGS;

    // Read ultrasonic every 50ms
    static unsigned long lastUsReadTime = 0;
    if (now - lastUsReadTime >= 50) {
        float rawDistance = readUltrasonic();

        float toAdd = (rawDistance >= MIN_RANGE && rawDistance <= MAX_RANGE) ? rawDistance : MAX_RANGE;
        usTotal -= usReadings[usReadIndex];
        usReadings[usReadIndex] = toAdd;
        usTotal += usReadings[usReadIndex];
        usReadIndex = (usReadIndex + 1) % US_NUM_READINGS;
        lastFilteredDistance = usTotal / US_NUM_READINGS;

        lastUsReadTime = now;
    }

    // ── Accept new client only if we don't have one ───────────────
    if (!persistentClient || !persistentClient.connected()) {
        persistentClient = server.available();
        if (persistentClient) {
            Serial.println("# Client connected");
        }
    }

    // ── Read command from persistent WiFi client OR Serial ────────
    // Drain entire buffer, keep only the LAST real command character
    // Skip \n \r and spaces so they don't overwrite the actual command
    char c = 0;
    if (persistentClient && persistentClient.connected() && persistentClient.available()) {
        while (persistentClient.available()) {
            char incoming = (char)persistentClient.read();
            if (incoming != '\n' && incoming != '\r' && incoming != ' ') {
                c = incoming;
            }
        }
    } else if (Serial.available()) {
        while (Serial.available()) {
            char incoming = (char)Serial.read();
            if (incoming != '\n' && incoming != '\r' && incoming != ' ') {
                c = incoming;
            }
        }
    }
    // ─────────────────────────────────────────────────────────────

    // Handle commands (only if not replaying)
    if (c != 0 && !replaying) {
        currentCommand = c;

        if (c == 'x') {
            recording = true;
            recordStart = now;
            recordedCommands = "";
            encoderACount = 0;
            encoderBCount = 0;
            Serial.println("# RECORDING STARTED");
        }
        else if (c == 'z') {
            recording = false;
            Serial.println("# RECORDING STOPPED");
            Serial.print("# Commands: ");
            Serial.println(recordedCommands);
        }
        else if (c == 'p') {
            replaying = true;
            replayStart = now;
            replayPos = 0;
            encoderACount = 0;
            encoderBCount = 0;
            Serial.println("# REPLAYING");
        }
        else if (c == 'c') {
            calibratePhotocell();
        }
        else if (c == 'm') {
            stop();
            autonomousMode = true;
            followMode = false;
            state = 0;
            prev = photocellValue;
            Serial.print("# AUTONOMOUS MODE STARTED - Initial photocell: ");
            Serial.println(prev);
        }
        else if (c == 'f') {
            stop();
            followMode = true;
            autonomousMode = false;
            state = 0;
            Serial.print("# FOLLOW MODE STARTED - Set point: ");
            Serial.print(SET_POINT);
            Serial.println(" cm");
        }
        else if (c == 'l') {
            stop();
            wallFollowMode = true;
            autonomousMode = false;
            followMode = false;
            state = 0;
            Serial.println("# WALL FOLLOW MODE STARTED");
        }
        else if (c == 'e') {
            autonomousMode = false;
            followMode = false;
            wallFollowMode = false;  
            state = 0;
            stop();
            Serial.println("# MANUAL MODE");
        }
        else if (!autonomousMode && !followMode) {
            executeCommand(c);
            if (recording) {
                unsigned long elapsed = now - recordStart;
                recordedCommands += String(elapsed) + ":" + c + ",";
            }
        }
    }

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

    // Photocell edge detection
    int diff = photocellValue - prev;

    // Autonomous photocell behavior
    if (autonomousMode) {
        Serial.print("# State: "); Serial.print(state);
        Serial.print(", Photo: "); Serial.print(photocellValue);
        Serial.print(", Prev: ");  Serial.print(prev);
        Serial.print(", Diff: ");  Serial.println(diff);

        if (state == 0) {
            if (diff >= 20) {
                moveForward();
                state = 1;
                prev = photocellValue;
                Serial.println("# TRIGGERED: light->dark, MOVING FORWARD");
            }
        } else if (state == 1) {
            if (diff <= -20) {
                stop();
                state = 0;
                prev = photocellValue;
                Serial.println("# TRIGGERED: dark->light, STOPPING");
            }
        }
    }

    // Follow-Me P-Controller
    if (followMode) {
        lastError  = lastFilteredDistance - SET_POINT;  // flipped: positive = too far
        lastOutput = Kp * lastError;

        if (lastFilteredDistance > 40.0) {
            // out of range, stop
            stop();
            Serial.println("# OUT OF RANGE");
        } else if (abs(lastError) < DEAD_ZONE) {
            // within 10cm of setpoint, chill
            stop();
        } else if (lastError > 0) {
            // too far, speed up forward
            int pwm = constrain((int)abs(lastOutput), MIN_PWM, MAX_PWM);
            moveForwardPWM(pwm);
        } else {
            // too close, back up
            int pwm = constrain((int)abs(lastOutput), MIN_PWM, MAX_PWM);
            moveBackwardPWM(pwm);
        }
    }

    if (wallFollowMode) {
            if (lastFilteredDistance > 20.0) {
            // wall gone, turn left to find it
                digitalWrite(in1, LOW);
                digitalWrite(in2, HIGH);
                analogWrite(enA, BASE_SPEED - 30);   // left motor slower

                digitalWrite(in3, LOW);
                digitalWrite(in4, HIGH);
                analogWrite(enB, BASE_SPEED - MOTOR_TRIM);  // right motor faster = turns left
        } else {
            float error = WALL_SETPOINT - lastFilteredDistance;
            float correction = constrain(Kp_wall * error, -15, 15);  // max 15 PWM difference

            int leftSpeed  = constrain(BASE_SPEED + (int)correction, MIN_PWM, MAX_PWM);
            int rightSpeed = constrain(BASE_SPEED - (int)correction, MIN_PWM, MAX_PWM);

            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            analogWrite(enA, leftSpeed);

            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(enB, rightSpeed);
    }
    }

    // Telemetry
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print("Time: ");   Serial.print(now);
        Serial.print(" | Cmd: "); Serial.print(currentCommand);
        Serial.print(" | EncA: ");Serial.print(encoderACount);
        Serial.print(" | EncB: ");Serial.print(encoderBCount);
        Serial.print(" | Photo: ");Serial.print(photocellValue);
        Serial.print(" | State: ");Serial.print(state);
        Serial.print(" | Dist: "); Serial.print(lastFilteredDistance, 1);
        Serial.print("cm | Err: ");Serial.print(lastError, 1);
        Serial.print(" | Out: ");  Serial.println(lastOutput, 1);
        lastPrintTime = now;
    }
}

// ──────────────────── Ultrasonic Sensor ──────────────────────────
float readUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    //long duration = pulseIn(echoPin, HIGH, 60000);

    long duration = pulseIn(echoPin, HIGH, 30000);
    // Serial.print("# RAW DURATION: ");
    // Serial.println(duration);

    if (duration == 0 || duration < 150) return -1;
    float distance = (duration * 0.0343) / 2.0;
    if (distance < MIN_RANGE || distance > MAX_RANGE) return -1;
    return distance;
}

// ──────────────────── Command Execution ──────────────────────────
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

// ──────────────────── Motor Functions ────────────────────────────


// Variable-speed functions (used by P-controller)
void moveForwardPWM(int speed) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, speed);

}

void moveBackwardPWM(int speed) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);

}

// ──────────────────── Calibration ────────────────────────────────
void calibratePhotocell() {
    Serial.println("# Calibrating photocell...");
    Serial.println("# Move robot over dark and light areas");

    minReading = 1023;
    maxReading = 0;

    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        int val = analogRead(photocellPin);
        if (val < minReading) minReading = val;
        if (val > maxReading) maxReading = val;
        delay(50);
    }

    Lightthreshold = (minReading + maxReading) / 2;
    hysteresis = (maxReading - minReading) / 10;

    Serial.print("# Min: ");        Serial.print(minReading);
    Serial.print(", Max: ");        Serial.print(maxReading);
    Serial.print(", Threshold: ");  Serial.print(Lightthreshold);
    Serial.print(", Hysteresis: "); Serial.println(hysteresis);
    Serial.println("# Calibration complete");
}
