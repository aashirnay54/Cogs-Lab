// MyRobot-V1.ino

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

// Calibration
int minReading = 1023;
int maxReading = 0;

// State
int state = 0;
bool autonomousMode = false;

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

void setup() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(encA, INPUT_PULLUP);
    pinMode(encB, INPUT_PULLUP);
    
    lastEncAState = digitalRead(encA);
    lastEncBState = digitalRead(encB);
    
    // Initialize moving average array
    for (int i = 0; i < NUM_READINGS; i++) {
        photocellReadings[i] = 0;
    }
    
    Serial.begin(115200);
    Serial.println("time_ms,cmd,encA_count,encB_count,photocell,state");
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
            // Start autonomous mode
            stop();
            autonomousMode = true;
            state = 0;
            prev = photocellValue;  // Initialize prev to current reading
            Serial.print("# AUTONOMOUS MODE STARTED - Initial photocell: ");
            Serial.println(prev);
        }
        else if (c == 'e') {
            // Stop and return to manual
            autonomousMode = false;
            state = 0;
            stop();
            Serial.println("# MANUAL MODE");
        }
        else if (!autonomousMode) {
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
        
        // Parse and execute next command if it's time
        int nextColon = recordedCommands.indexOf(':', replayPos);
        if (nextColon > 0) {
            unsigned long cmdTime = recordedCommands.substring(replayPos, nextColon).toInt();
            
            if (elapsed >= cmdTime) {
                char cmd = recordedCommands.charAt(nextColon + 1);
                executeCommand(cmd);
                currentCommand = cmd;
                
                // Move to next command
                replayPos = recordedCommands.indexOf(',', nextColon) + 1;
                
                // Check if done
                if (replayPos >= recordedCommands.length() || replayPos == 0) {
                    replaying = false;
                    stop();
                    Serial.println("# REPLAY COMPLETE");
                }
            }
        }
    }
    
    // Calculate difference for edge detection
    int diff = photocellValue - prev;
    
    // Autonomous behavior with edge detection
    if (autonomousMode) {
        // Debug output
        Serial.print("# State: ");
        Serial.print(state);
        Serial.print(", Photo: ");
        Serial.print(photocellValue);
        Serial.print(", Prev: ");
        Serial.print(prev);
        Serial.print(", Diff: ");
        Serial.println(diff);
        
        if (state == 0) {
            // Waiting on light surface - detect transition TO dark
            if (diff >= 20) {
                moveForward();
                state = 1;
                prev = photocellValue;
                Serial.println("# TRIGGERED: light→dark, MOVING FORWARD");
            }
        }
        else if (state == 1) {
            // Driving on dark surface - detect transition TO light
            if (diff <= -20) {
                stop();
                state = 0;
                prev = photocellValue;
                Serial.println("# TRIGGERED: dark→light, STOPPING");
            }
        }
    }
    
    // Print telemetry
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print(now);
        Serial.print(',');
        Serial.print(currentCommand);
        Serial.print(',');
        Serial.print(encoderACount);
        Serial.print(',');
        Serial.print(encoderBCount);
        Serial.print(',');
        Serial.print(photocellValue);
        Serial.print(',');
        Serial.println(state);
        lastPrintTime = now;
    }
}

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