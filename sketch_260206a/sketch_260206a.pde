import processing.serial.*;

Serial port;
boolean wPressed = false;
boolean aPressed = false;
boolean sPressed = false;
boolean dPressed = false;
boolean qPressed = false;

void setup() {
    size(600, 400);
    
    // Find your Arduino port
    printArray(Serial.list());
    port = new Serial(this, Serial.list()[0], 115200);  // Change [0] to your port index
    
    textAlign(CENTER, CENTER);
}

void draw() {
    background(40);
    
    // Draw title
    fill(255);
    textSize(24);
    text("Robot Remote Control", width/2, 40);
    
    // Draw instructions
    textSize(16);
    text("W = Forward | S = Backward", width/2, 80);
    text("A = Left | D = Right | Q = Turn", width/2, 110);
    text("E = Stop | R = Reset Encoders", width/2, 140);
    text("X = Start Recording | Z = Stop | P = Replay", width/2, 170);
    
    // Draw key indicators
    drawKey('W', wPressed, width/2, 220);
    drawKey('A', aPressed, width/2 - 80, 270);
    drawKey('S', sPressed, width/2, 270);
    drawKey('D', dPressed, width/2 + 80, 270);
    drawKey('Q', qPressed, width/2, 320);
    
    // Draw telemetry area
    fill(60);
    rect(20, height - 80, width - 40, 60);
    fill(200);
    textSize(12);
    text("Telemetry will appear in Processing console below", width/2, height - 50);
}

void drawKey(char key, boolean pressed, float x, float y) {
    if (pressed) {
        fill(0, 255, 0);  // Green when pressed
    } else {
        fill(100);
    }
    rect(x - 25, y - 25, 50, 50, 5);
    
    fill(255);
    textSize(20);
    text(key, x, y);
}

void keyPressed() {
    char k = Character.toLowerCase(key);
    
    // Send command to Arduino
    if (k == 'w' || k == 'a' || k == 's' || k == 'd' || 
        k == 'q' || k == 'e' || k == 'r' ||
        k == 'x' || k == 'z' || k == 'p') {
        port.write(k);
        println("Sent: " + k);
    }
    
    // Update visual feedback
    if (k == 'w') wPressed = true;
    if (k == 'a') aPressed = true;
    if (k == 's') sPressed = true;
    if (k == 'd') dPressed = true;
    if (k == 'q') qPressed = true;
}

void keyReleased() {
    char k = Character.toLowerCase(key);
    
    // Auto-stop when key released (optional - comment out if you don't want this)
    if (k == 'w' || k == 'a' || k == 's' || k == 'd' || k == 'q') {
        port.write('e');
        println("Auto-stop");
    }
    
    // Update visual feedback
    if (k == 'w') wPressed = false;
    if (k == 'a') aPressed = false;
    if (k == 's') sPressed = false;
    if (k == 'd') dPressed = false;
    if (k == 'q') qPressed = false;
}

void serialEvent(Serial p) {
    String data = p.readStringUntil('\n');
    if (data != null) {
        print(data);  // Print telemetry to console
    }
}
