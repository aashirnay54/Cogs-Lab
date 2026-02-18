import processing.net.*;

// ── Connection ────────────────────────────────────────────────────
String arduinoIP = "10.85.234.126";
int arduinoPort  = 3333;
Client c;
boolean connected = false;

// ── State ─────────────────────────────────────────────────────────
char currentMode = 'e';
char lastCmd     = ' ';
String statusMsg = "Connecting...";
String modeLabel = "MANUAL";

// ── Key hold tracking ─────────────────────────────────────────────
boolean keyW, keyS, keyA, keyD, keyQ;
char    lastSentMove = 0;

// ── Rate limiting ─────────────────────────────────────────────────
int lastSendTime   = 0;
int SEND_INTERVAL  = 50; // faster since Arduino drains buffer now // ms between sends — raise if still dropping

// ── Colors ────────────────────────────────────────────────────────
color BG         = #0D0D0D;
color PANEL      = #141414;
color ACCENT     = #00FF88;
color ACCENT2    = #FF4466;
color ACCENT3    = #FFD700;
color TEXTDIM    = #444444;
color TEXTMID    = #888888;
color TEXTBRIGHT = #EEEEEE;

// ── Button class ──────────────────────────────────────────────────
class Btn {
  float x, y, w, h;
  String label;
  String sub;
  char cmd;
  color col;
  boolean held = false;

  Btn(float x, float y, float w, float h, String label, String sub, char cmd, color col) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.label = label; this.sub = sub; this.cmd = cmd; this.col = col;
  }

  boolean over() { return mouseX > x && mouseX < x+w && mouseY > y && mouseY < y+h; }

  void draw() {
    boolean hot = over() || held;
    noStroke();
    fill(hot ? col : color(red(col)*0.15, green(col)*0.15, blue(col)*0.15));
    rect(x, y, w, h, 6);
    noFill();
    strokeWeight(1);
    stroke(hot ? col : color(red(col)*0.4, green(col)*0.4, blue(col)*0.4));
    rect(x, y, w, h, 6);
    noStroke();
    fill(hot ? BG : col);
    textAlign(CENTER, CENTER);
    textSize(13);
    text(label, x + w/2, y + h/2 - (sub.length() > 0 ? 8 : 0));
    if (sub.length() > 0) {
      textSize(10);
      fill(hot ? color(150) : color(red(col)*0.7, green(col)*0.7, blue(col)*0.7));
      text(sub, x + w/2, y + h/2 + 10);
    }
  }
}

// ── Buttons ───────────────────────────────────────────────────────
Btn[] moveBtns;
Btn[] modeBtns;
Btn[] utilBtns;

void setup() {
  size(620, 560);
  smooth();
  textFont(createFont("Arial", 16, true)); // plain font, no missing glyph warnings

  // Plain text labels -- no special characters
  moveBtns = new Btn[] {
    new Btn(260, 175, 80, 70, "FWD",   "W",  'w', ACCENT),
    new Btn(260, 255, 80, 70, "BACK",  "S",  's', ACCENT),
    new Btn(175, 255, 80, 70, "LEFT",  "A",  'a', ACCENT),
    new Btn(345, 255, 80, 70, "RIGHT", "D",  'd', ACCENT),
    new Btn(175, 175, 80, 70, "SPIN",  "Q",  'q', ACCENT2),
    new Btn(345, 175, 80, 70, "STOP",  "E",  'e', ACCENT2),
  };

  modeBtns = new Btn[] {
    new Btn(50, 175, 110, 55, "MANUAL",  "key: E", 'e', TEXTMID),
    new Btn(50, 240, 110, 55, "AUTO",    "key: M", 'm', ACCENT3),
    new Btn(50, 305, 110, 55, "FOLLOW",  "key: F", 'f', ACCENT2),
  };

  utilBtns = new Btn[] {
    new Btn(460, 175, 110, 55, "RECORD",   "key: X", 'x', ACCENT3),
    new Btn(460, 240, 110, 55, "STOP REC", "key: Z", 'z', ACCENT3),
    new Btn(460, 305, 110, 55, "REPLAY",   "key: P", 'p', ACCENT),
    new Btn(460, 370, 110, 55, "CALIB",    "key: C", 'c', TEXTMID),
  };

  frameRate(60);
}

void draw() {
  background(BG);

  if (!connected && frameCount == 60) tryConnect();

  drawGrid();
  drawHeader();
  drawModeIndicator();
  drawMovePanel();
  drawSidePanels();
  drawKeyHints();
  drawFooter();

  handleHeldKeys();
}

void tryConnect() {
  try {
    c = new Client(this, arduinoIP, arduinoPort);
    connected = true;
    statusMsg = "Connected  " + arduinoIP + ":" + arduinoPort;
  } catch (Exception e) {
    statusMsg = "Failed -- check IP & Arduino";
  }
}

void send(char cmd) {
  // Auto-reconnect if connection dropped
  if (connected && (c == null || !c.active())) {
    println("Connection lost -- reconnecting...");
    connected = false;
    statusMsg = "Reconnecting...";
    try {
      c = new Client(this, arduinoIP, arduinoPort);
      connected = true;
      statusMsg = "Connected  " + arduinoIP + ":" + arduinoPort;
      println("Reconnected!");
    } catch (Exception e) {
      statusMsg = "Reconnect failed -- retrying next send";
      return;
    }
  }

  // Rate limit -- don't flood the Arduino
  if (millis() - lastSendTime < SEND_INTERVAL) return;
  lastSendTime = millis();

  if (connected && c != null && c.active()) {
    c.write(cmd + "\n");
    lastCmd = cmd;
  }
}

void handleHeldKeys() {
  char move = 0;
  if (keyW) move = 'w';
  else if (keyS) move = 's';
  else if (keyA) move = 'a';
  else if (keyD) move = 'd';
  else if (keyQ) move = 'q';

  if (move != 0 && move != lastSentMove) {
    send(move);
    lastSentMove = move;
  } else if (move == 0 && lastSentMove != 0) {
    send('e');
    lastSentMove = 0;
  }
}

// ── Drawing ───────────────────────────────────────────────────────
void drawGrid() {
  strokeWeight(1);
  stroke(25);
  for (int x = 0; x < width; x += 40) line(x, 0, x, height);
  for (int y = 0; y < height; y += 40) line(0, y, width, y);
}

void drawHeader() {
  noStroke();
  fill(PANEL);
  rect(0, 0, width, 55);

  fill(ACCENT);
  textAlign(LEFT, CENTER);
  textSize(22);
  text("ROBOT CTRL", 20, 27);

  fill(TEXTDIM);
  textSize(11);
  text("v4 -- WiFi", 165, 27);

  float dotX = width - 20;
  float dotY = 27;
  fill(connected ? ACCENT : ACCENT2);
  ellipse(dotX, dotY, 10, 10);
  fill(connected ? ACCENT : ACCENT2);
  textAlign(RIGHT, CENTER);
  textSize(11);
  text(statusMsg, dotX - 16, dotY);
}

void drawModeIndicator() {
  noStroke();
  color mc = currentMode == 'm' ? ACCENT3 : currentMode == 'f' ? ACCENT2 : TEXTMID;
  fill(red(mc)*0.12, green(mc)*0.12, blue(mc)*0.12);
  rect(20, 70, width - 40, 38, 5);
  noFill();
  strokeWeight(1);
  stroke(mc);
  rect(20, 70, width - 40, 38, 5);
  noStroke();

  fill(mc);
  textAlign(LEFT, CENTER);
  textSize(13);
  text("MODE", 35, 89);

  fill(TEXTBRIGHT);
  textSize(15);
  text(modeLabel, 90, 89);

  fill(TEXTMID);
  textAlign(RIGHT, CENTER);
  textSize(11);
  text("Last cmd: " + (lastCmd == ' ' ? "--" : str(lastCmd)), width - 35, 89);
}

void drawMovePanel() {
  noStroke();
  fill(PANEL);
  rect(160, 160, 290, 185, 8);
  noFill();
  strokeWeight(1);
  stroke(TEXTDIM);
  rect(160, 160, 290, 185, 8);
  noStroke();

  fill(TEXTDIM);
  textAlign(CENTER, CENTER);
  textSize(10);
  text("MOVEMENT", 305, 172);

  for (Btn b : moveBtns) b.draw();
}

void drawSidePanels() {
  fill(TEXTDIM);
  textAlign(CENTER);
  textSize(10);
  text("MODES", 105, 165);
  for (Btn b : modeBtns) b.draw();

  text("UTILS", 515, 165);
  for (Btn b : utilBtns) b.draw();
}

void drawKeyHints() {
  fill(TEXTDIM);
  textAlign(CENTER);
  textSize(10);
  text("Hold W/A/S/D keys for continuous movement", width/2, 380);
}

void drawFooter() {
  noStroke();
  fill(PANEL);
  rect(0, height - 45, width, 45);
  stroke(TEXTDIM);
  strokeWeight(1);
  line(0, height - 45, width, height - 45);
  noStroke();

  fill(TEXTDIM);
  textAlign(LEFT, CENTER);
  textSize(10);
  text("W fwd  S back  A left  D right  Q spin  E stop  M auto  F follow  X rec  Z stop-rec  P replay  C calib", 20, height - 22);
}

// ── Mouse ─────────────────────────────────────────────────────────
void mousePressed() {
  for (Btn b : moveBtns) {
    if (b.over()) { b.held = true; send(b.cmd); }
  }
  for (Btn b : modeBtns) {
    if (b.over()) { b.held = true; setMode(b.cmd); send(b.cmd); }
  }
  for (Btn b : utilBtns) {
    if (b.over()) { b.held = true; send(b.cmd); }
  }
}

void mouseReleased() {
  for (Btn b : moveBtns)  b.held = false;
  for (Btn b : modeBtns)  b.held = false;
  for (Btn b : utilBtns)  b.held = false;
  send('e');
}

// ── Keyboard ──────────────────────────────────────────────────────
void keyPressed() {
  char k = Character.toLowerCase(key);
  if      (k == 'w') keyW = true;
  else if (k == 's') keyS = true;
  else if (k == 'a') keyA = true;
  else if (k == 'd') keyD = true;
  else if (k == 'q') keyQ = true;
  else if (k == 'm') { setMode('m'); send('m'); }
  else if (k == 'f') { setMode('f'); send('f'); }
  else if (k == 'e') { setMode('e'); send('e'); }
  else if (k == 'x') send('x');
  else if (k == 'z') send('z');
  else if (k == 'p') send('p');
  else if (k == 'c') send('c');
  else if (k == 'r') send('r');
}

void keyReleased() {
  char k = Character.toLowerCase(key);
  if      (k == 'w') keyW = false;
  else if (k == 's') keyS = false;
  else if (k == 'a') keyA = false;
  else if (k == 'd') keyD = false;
  else if (k == 'q') keyQ = false;
}

void setMode(char m) {
  currentMode = m;
  if      (m == 'm') modeLabel = "AUTONOMOUS (photocell)";
  else if (m == 'f') modeLabel = "FOLLOW ME (ultrasonic)";
  else               modeLabel = "MANUAL";
}
