// MyRobot-V2.ino
// Added: Ultrasonic sensor + P-controller "Follow Me" mode (press 'f')

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
const int US_NUM_READINGS = 5;
float usReadings[US_NUM_READINGS];
int   usReadIndex  = 0;
float usTotal      = 0.0;

// P-Controller parameters
const float SET_POINT = 25;   // Target distance in cm
const float Kp        = 5.0;    // Proportional gain — tune this!
const int   DEAD_ZONE = 3;      // ±3 cm window where robot holds still
const int   MIN_PWM   = 60;     // Minimum PWM to overcome motor friction
const int   MAX_PWM   = 255;    // Maximum PWM output
const float MAX_RANGE  = 200.0; // Ignore readings beyond 200 cm
const float MIN_RANGE  = 2.0;   // Ignore readings below 2 cm

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

void setup() {
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
    
    // Initialize photocell moving average
    for (int i = 0; i < NUM_READINGS; i++) {
        photocellReadings[i] = 0;
    }
    
    // Initialize ultrasonic moving average
    for (int i = 0; i < US_NUM_READINGS; i++) {
        usReadings[i] = SET_POINT;
    }
    usTotal = SET_POINT * US_NUM_READINGS;
    
    Serial.begin(115200);
    Serial.println("time_ms,cmd,encA_count,encB_count,photocell,state,distance_cm,error,output");
}

void loop() {
    unsigned long now = millis();
    
    // Read encoders
    int encAState = digitalRead(encA);
    int encBState = digitalRead(encB);
    
    if (encAState != lastEncAState && encAState == HIGH) {
        encoderACount++;
    }
    if (encBState != lastEncBState && encBState == HIGH) {
        encoderBCount++;
    }
    lastEncAState = encAState;
    lastEncBState = encBState;
    
    // Read photocell with moving average filter
    photocellTotal -= photocellReadings[readIndex];
    photocellReadings[readIndex] = analogRead(photocellPin);
    photocellTotal += photocellReadings[readIndex];
    readIndex = (readIndex + 1) % NUM_READINGS;
    photocellValue = photocellTotal / NUM_READINGS;
    
    // Read ultrasonic with moving average filter (always, so telemetry has data)
    // Read ultrasonic every 50ms (not every loop) to avoid pin/timing conflicts
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
    // Handle commands (only if not replaying)
    if (Serial.available() && !replaying) {
        char c = Serial.read();
        currentCommand = c;
        
        if (c == 'x') {
            // Start recording
            recording = true;
            recordStart = now;
            recordedCommands = "";
            encoderACount = 0;
            encoderBCount = 0;
            Serial.println("# RECORDING STARTED");
        }
        else if (c == 'z') {
            // Stop recording
            recording = false;
            Serial.println("# RECORDING STOPPED");
            Serial.print("# Commands: ");
            Serial.println(recordedCommands);
        }
        else if (c == 'p') {
            // Start replay
            replaying = true;
            replayStart = now;
            replayPos = 0;
            encoderACount = 0;
            encoderBCount = 0;
            Serial.println("# REPLAYING");
        }
        else if (c == 'c') {
            // Calibrate photocell
            calibratePhotocell();
        }
        else if (c == 'm') {
            // Start autonomous photocell mode
            stop();
            autonomousMode = true;
            followMode = false;
            state = 0;
            prev = photocellValue;
            Serial.print("# AUTONOMOUS MODE STARTED - Initial photocell: ");
            Serial.println(prev);
        }
        else if (c == 'f') {
            // Start follow-me mode
            stop();
            followMode = true;
            autonomousMode = false;
            state = 0;
            Serial.print("# FOLLOW MODE STARTED - Set point: ");
            Serial.print(SET_POINT);
            Serial.println(" cm");
        }
        else if (c == 'e') {
            // Stop and return to manual
            autonomousMode = false;
            followMode = false;
            state = 0;
            stop();
            Serial.println("# MANUAL MODE");
        }
        else if (!autonomousMode && !followMode) {
            // Manual controls
            executeCommand(c);
            
            // Record command with timestamp
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
    
    // Calculate difference for photocell edge detection
    int diff = photocellValue - prev;
    
    // Autonomous photocell behavior with edge detection
    if (autonomousMode) {
        Serial.print("# State: ");
        Serial.print(state);
        Serial.print(", Photo: ");
        Serial.print(photocellValue);
        Serial.print(", Prev: ");
        Serial.print(prev);
        Serial.print(", Diff: ");
        Serial.println(diff);
        
        if (state == 0) {
            if (diff >= 20) {
                moveForward();
                state = 1;
                prev = photocellValue;
                Serial.println("# TRIGGERED: light→dark, MOVING FORWARD");
            }
        }
        else if (state == 1) {
            if (diff <= -20) {
                stop();
                state = 0;
                prev = photocellValue;
                Serial.println("# TRIGGERED: dark→light, STOPPING");
            }
        }
    }
    
    // ── Follow-Me P-Controller behavior ──
    if (followMode) {
        lastError  = SET_POINT - lastFilteredDistance;
        lastOutput = Kp * lastError;
        
        if (abs(lastError) < DEAD_ZONE) {
            // Within dead zone — hold still
            stop();
        }
        else if (lastOutput > 0) {
            // Object is farther than set point → drive forward
            int pwm = constrain((int)abs(lastOutput), 40, 100);
            moveForwardPWM(pwm);
        }
        else {
            // Object is closer than set point → drive backward
            int pwm = constrain((int)abs(lastOutput), 40, 100);
            moveBackwardPWM(pwm);
        }
    }
    
    // Print telemetry
    // Print telemetry
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print("Time: ");
        Serial.print(now);
        Serial.print(" | Cmd: ");
        Serial.print(currentCommand);
        Serial.print(" | EncA: ");
        Serial.print(encoderACount);
        Serial.print(" | EncB: ");
        Serial.print(encoderBCount);
        Serial.print(" | Photo: ");
        Serial.print(photocellValue);
        Serial.print(" | State: ");
        Serial.print(state);
        Serial.print(" | Dist: ");
        Serial.print(lastFilteredDistance, 1);
        Serial.print("cm | Err: ");
        Serial.print(lastError, 1);
        Serial.print(" | Out: ");
        Serial.println(lastOutput, 1);
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
    
    long duration = pulseIn(echoPin, HIGH, 38000);
    
    if (duration == 0) {
        return MAX_RANGE + 1;  // No echo → out of range
    }
    
    // Speed of sound ≈ 0.0343 cm/µs, divide by 2 for round trip
    float distance = (duration * 0.0343) / 2.0;
    return distance;
}

// ──────────────────── Command Execution ──────────────────────────
void executeCommand(char c) {
    switch (c) {
        case 'w': moveForward(); break;
        case 's': moveBackward(); break;
        case 'a': moveLeft(); break;
        case 'd': moveRight(); break;
        case 'q': turnRobotInPlace(); break;
        case 'r': encoderACount = 0; encoderBCount = 0; break;
        case 'e': stop(); break;
    }
}

// ──────────────────── Motor Functions ────────────────────────────


// Variable-speed functions (used by P-controller)
void moveForwardPWM(int speed) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);
}

void moveBackwardPWM(int speed) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
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
    
    Serial.print("# Min: ");
    Serial.print(minReading);
    Serial.print(", Max: ");
    Serial.print(maxReading);
    Serial.print(", Threshold: ");
    Serial.print(Lightthreshold);
    Serial.print(", Hysteresis: ");
    Serial.println(hysteresis);
    Serial.println("# Calibration complete");
}