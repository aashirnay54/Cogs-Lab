// Sonar Radar Visualization for Object Detection
// Adapted to work with Arduino robot object detection code

import processing.serial.*;

Serial myPort;
String portName = "/dev/cu.usbmodemB081849857F02";  // Your Arduino UNO R4 WiFi port

String ang = "";
String distance = "";
String data = "";

int angle, dist;
boolean objectDetected = false;
int detectedAngle = -1;
float detectedDistance = -1;

// Radar display settings
float radarRadius = 350;  // Radar radius in pixels
int centerX, centerY;
int maxDistance = 200;  // Max range in cm

void setup() {
  size(1600, 900);  // Adjusted window size for better proportions

  // Position radar more centered
  centerX = width / 2;
  centerY = height - 200;  // Moved up from bottom to show more

  try {
    myPort = new Serial(this, portName, 115200);  // Match Arduino baud rate
    myPort.bufferUntil('\n');  // Read until newline
    println("Connected to: " + portName);
  } catch (Exception e) {
    println("Error opening port. Available ports:");
    printArray(Serial.list());
  }

  background(0);
}

void draw() {
  fill(0, 5);
  noStroke();
  rect(0, 0, width, height - 100);  // Main area with fade effect
  noStroke();
  fill(0, 255);
  rect(0, height - 100, width, 100);  // Bottom info bar (100px high)
  drawRadar();
  drawLine();
  drawObject();
  drawText();
}

void serialEvent(Serial myPort) {
  data = myPort.readStringUntil('\n');
  if (data != null) {
    data = trim(data);
    println(data);  // Debug output

    // Parse "Angle: 45° | Distance: 123.4 cm"
    if (data.contains("Angle:") && data.contains("Distance:")) {
      String[] parts = split(data, '|');
      if (parts.length >= 2) {
        // Parse angle
        String anglePart = trim(parts[0]);
        int angleStart = anglePart.indexOf(':') + 1;
        int angleEnd = anglePart.indexOf('°');
        if (angleStart > 0 && angleEnd > angleStart) {
          ang = trim(anglePart.substring(angleStart, angleEnd));
          angle = int(float(ang));
        }

        // Parse distance
        String distPart = trim(parts[1]);
        int distStart = distPart.indexOf(':') + 1;
        int distEnd = distPart.indexOf("cm");
        if (distStart > 0 && distEnd > distStart) {
          distance = trim(distPart.substring(distStart, distEnd));
          dist = int(float(distance));
        }
      }
    }

    // Parse detected object
    if (data.contains("Object detected at angle:")) {
      objectDetected = true;
      int angleIdx = data.indexOf("angle:") + 6;
      int angleEnd = data.indexOf("°", angleIdx);
      int distIdx = data.indexOf("distance:") + 9;
      int distEnd = data.indexOf("cm", distIdx);

      if (angleIdx > 0 && angleEnd > angleIdx) {
        detectedAngle = int(float(trim(data.substring(angleIdx, angleEnd))));
      }
      if (distIdx > 0 && distEnd > distIdx) {
        detectedDistance = float(trim(data.substring(distIdx, distEnd)));
      }
    }

    // Clear detection if no object
    if (data.contains("No distinct object") || data.contains("No object detected")) {
      objectDetected = false;
      detectedAngle = -1;
      detectedDistance = -1;
    }
  }
}

void drawRadar() {
  pushMatrix();
  noFill();
  strokeWeight(2);
  stroke(10, 255, 10);
  translate(centerX, centerY);

  // Base line
  line(-radarRadius, 0, radarRadius, 0);

  // Distance rings (50cm, 100cm, 150cm, 200cm)
  strokeWeight(1);
  arc(0, 0, radarRadius * 0.5, radarRadius * 0.5, PI, TWO_PI);
  arc(0, 0, radarRadius, radarRadius, PI, TWO_PI);
  arc(0, 0, radarRadius * 1.5, radarRadius * 1.5, PI, TWO_PI);
  arc(0, 0, radarRadius * 2, radarRadius * 2, PI, TWO_PI);

  // Major angle lines (30° increments)
  strokeWeight(1.5);
  line(0, 0, radarRadius * cos(radians(30)), -radarRadius * sin(radians(30)));
  line(0, 0, radarRadius * cos(radians(60)), -radarRadius * sin(radians(60)));
  line(0, 0, radarRadius * cos(radians(90)), -radarRadius * sin(radians(90)));
  line(0, 0, radarRadius * cos(radians(120)), -radarRadius * sin(radians(120)));
  line(0, 0, radarRadius * cos(radians(150)), -radarRadius * sin(radians(150)));

  // Minor angle lines (15° increments) - lighter color
  stroke(175, 255, 175, 100);
  strokeWeight(0.5);
  line(0, 0, radarRadius * cos(radians(15)), -radarRadius * sin(radians(15)));
  line(0, 0, radarRadius * cos(radians(45)), -radarRadius * sin(radians(45)));
  line(0, 0, radarRadius * cos(radians(75)), -radarRadius * sin(radians(75)));
  line(0, 0, radarRadius * cos(radians(105)), -radarRadius * sin(radians(105)));
  line(0, 0, radarRadius * cos(radians(135)), -radarRadius * sin(radians(135)));
  line(0, 0, radarRadius * cos(radians(165)), -radarRadius * sin(radians(165)));

  popMatrix();
}

void drawLine() {
  pushMatrix();
  strokeWeight(4);
  stroke(0, 255, 0, 180);  // Green scanning line with transparency
  translate(centerX, centerY);
  line(0, 0, radarRadius * cos(radians(angle)), -radarRadius * sin(radians(angle)));
  popMatrix();
}

void drawObject() {
  pushMatrix();
  translate(centerX, centerY);

  // Draw distance reading as red line from edge to object
  if (dist <= maxDistance && dist > 0) {
    stroke(255, 0, 0, 200);
    strokeWeight(6);

    // Map distance to radar radius (200cm = radarRadius pixels)
    float pixleDist = map(dist, 0, maxDistance, 0, radarRadius);
    float x = pixleDist * cos(radians(angle));
    float y = -pixleDist * sin(radians(angle));

    // Draw line from edge to object point
    float edgeX = radarRadius * cos(radians(angle));
    float edgeY = -radarRadius * sin(radians(angle));
    line(edgeX, edgeY, x, y);

    // Draw point at object location
    fill(255, 0, 0);
    noStroke();
    ellipse(x, y, 8, 8);
  }

  // Draw detected object highlight
  if (objectDetected && detectedAngle >= 0 && detectedDistance > 0) {
    float detPixDist = map(detectedDistance, 0, maxDistance, 0, radarRadius);
    float detX = detPixDist * cos(radians(detectedAngle));
    float detY = -detPixDist * sin(radians(detectedAngle));

    // Pulsing effect
    float pulse = sin(frameCount * 0.15) * 8 + 20;
    stroke(255, 50, 50);
    strokeWeight(3);
    noFill();
    ellipse(detX, detY, pulse, pulse);

    // Bright center point
    fill(255, 50, 50);
    noStroke();
    ellipse(detX, detY, 12, 12);

    // Line to detected object
    stroke(255, 50, 50, 200);
    strokeWeight(4);
    line(0, 0, detX, detY);
  }

  popMatrix();
}

void drawText() {
  pushMatrix();

  // Top status area
  fill(100, 200, 255);
  textSize(20);
  textAlign(LEFT);
  text("Arduino Sonar Radar - Object Detection", 20, 30);

  // Distance scale on right side
  textSize(18);
  textAlign(RIGHT);
  text("50cm", width - 20, 100);
  text("100cm", width - 20, 150);
  text("150cm", width - 20, 200);
  text("200cm", width - 20, 250);

  // Bottom info bar - current readings
  textAlign(LEFT);
  textSize(22);
  if (dist <= 200 && dist > 0) {
    text("Angle: " + angle + "°", 20, height - 60);
    text("Distance: " + dist + " cm", 20, height - 30);
  }

  // Object detection status - center bottom
  if (objectDetected) {
    fill(255, 50, 50);
    textAlign(CENTER);
    textSize(28);
    text("⚠ OBJECT DETECTED!", width / 2, height - 60);
    fill(100, 200, 255);
    textSize(20);
    text("At " + detectedAngle + "° | " + nf(detectedDistance, 0, 1) + " cm", width / 2, height - 30);
  }

  // Controls info - right bottom
  fill(100, 200, 255);
  textSize(16);
  textAlign(RIGHT);
  text("'o' scan | 'v' test | 'e' stop", width - 20, height - 30);

  // Angle labels on radar
  translate(centerX, centerY);
  textSize(20);
  textAlign(CENTER);
  fill(100, 200, 255);
  float labelRadius = radarRadius * 1.15;
  text("30°", labelRadius * cos(radians(30)), -labelRadius * sin(radians(30)));
  text("60°", labelRadius * cos(radians(60)), -labelRadius * sin(radians(60)));
  text("90°", labelRadius * cos(radians(90)), -labelRadius * sin(radians(90)) - 10);
  text("120°", labelRadius * cos(radians(120)), -labelRadius * sin(radians(120)));
  text("150°", labelRadius * cos(radians(150)), -labelRadius * sin(radians(150)));
  text("0°", labelRadius * cos(radians(0)) + 20, -labelRadius * sin(radians(0)));
  text("180°", labelRadius * cos(radians(180)) - 20, -labelRadius * sin(radians(180)));

  popMatrix();
}

// Keyboard controls
void keyPressed() {
  if (myPort != null) {
    if (key == 'o') {
      myPort.write('o');
      println("Sent: Start object detection");
      objectDetected = false;
    } else if (key == 'e') {
      myPort.write('e');
      println("Sent: Stop");
      objectDetected = false;
    } else if (key == 'v') {
      myPort.write('v');
      println("Sent: Test servo");
    }
  }
}
