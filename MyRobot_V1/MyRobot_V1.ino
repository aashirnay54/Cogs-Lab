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
int threshold = 58;

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
    
    Serial.begin(115200);
    Serial.println("time_ms,cmd,encA_count,encB_count,photocell,state");
}

void loop() {

    // Serial.println(analogRead(A0));
    // delay(300);
    // return;

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
    
    // Read photocell
    long sum = 0;

    for (int i = 0; i<10; i++) {
        sum += analogRead(A0);
        delayMicroseconds(300);
    }

    photocellValue = sum / 10;
    
    delay(300);

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
        else if (c == 'm') {
            // Start autonomous mode
            autonomousMode = true;
            state = 0;
        }
        else if (c == 'e') {
            // Stop and return to manual
            autonomousMode = false;
            state = 0;
            stop();
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
    
    // Autonomous behavior (photocell)
    if (autonomousMode) {
        if (state == 0) {
            // Waiting on white tape
            if (photocellValue > threshold) {
                state = 1;
                moveForward();
            }
        }
        else if (state == 1) {
            // Driving on dark floor
            if (photocellValue < threshold) {
                state = 0;
                stop();
            }
        }

        
    }
    
    // Print telemetry
    if (now - lastPrintTime >= PRINT_INTERVAL) {
        Serial.print(now);
        Serial.print(',');
        Serial.print(currentCommand);
        Serial.print(',');
        // Serial.print("Encoder A: ");
        Serial.print(encoderACount);
        Serial.print(',');
        // Serial.print("Encoder B: ");
        Serial.print(encoderBCount);
        Serial.print(',');
        // Serial.print("Photocell: ");
        Serial.print(photocellValue);
        Serial.print(',');
        Serial.println(state); 
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