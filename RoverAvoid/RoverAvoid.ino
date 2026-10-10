// 2WD obstacle-avoiding rover: 2x 28BYJ-48 + ULN2003, HC-SR04 on an SG-90.
// Dashboard: http://192.168.137.50
//
// Wiring
//   Battery (+) -> switch -> VIN, (-) -> GND
//   Arduino 5V/GND -> breadboard rails -> both ULN2003 (+/-), SG-90, HC-SR04
//   Left  ULN2003 IN1..IN4 -> D2..D5
//   Right ULN2003 IN1..IN4 -> D6..D9
//   SG-90 signal -> D10, HC-SR04 Trig -> D11, Echo -> D12
#include <AccelStepper.h>
#include <Servo.h>
#include <WiFiS3.h>
#include "FspTimer.h"
#include "dashboard.h"

const char WIFI_SSID[] = "laphot";
const char WIFI_PASS[] = "einstein";

// HALF4WIRE needs pin order IN1, IN3, IN2, IN4
AccelStepper leftM (AccelStepper::HALF4WIRE, 2, 4, 3, 5);
AccelStepper rightM(AccelStepper::HALF4WIRE, 6, 8, 7, 9);
Servo scanner;
FspTimer stepTimer;
WiFiServer server(80);

const int TRIG = 11, ECHO = 12;

const float WHEEL_D    = 6.8;   // cm
const float TRACK      = 21.6;  // cm, wheel centre to wheel centre
const int   LEFT_DIR   = 1;
const int   RIGHT_DIR  = -1;    // right motor is mirrored
float cruiseSpeed = 1100;       // half-steps/s, adjustable from the dashboard
float turnSpeed   = 1000;
const float ACCEL      = 600;   // half-steps/s^2
const int   SAFE_CM    = 20;
const int   SCAN_CENTER = 90;
const int   SCAN_SPAN   = 30;   // servo 60..120
const int   SCAN_STEP   = 10;
const int   BACKUP_CM   = 15;
const float TURN_DEG    = 45;
const float TURN_RADIUS = 25;   // cm
// Stuck: the steps say the robot moved STUCK_TRAVEL_CM but the distance ahead
// changed by less than STUCK_CHANGE_CM.
const float STUCK_TRAVEL_CM = 10;
const int   STUCK_CHANGE_CM = 3;
const long  STEPS_REV   = 4096;  // half-steps per wheel turn
const long  FAR_AWAY    = 100000000L;
const float CM_PER_STEP = PI * WHEEL_D / STEPS_REV;

long cmToSteps(float cm)   { return cm / CM_PER_STEP; }
long degToSteps(float deg) { return cmToSteps(PI * TRACK * deg / 360.0); }

volatile bool motorsRunning = false;
bool enabled = false;      // start/stop from the web page
bool driving = false;      // currently cruising forward
bool wifiOk = false;
long lastDistance = 999;
const char *state = "stopped";

// Manual drive: the browser repeats the command while a button is held.
char manualCmd = 0;        // 'f', 'b', 'l', 'r' or 0
char manualActive = 0;     // direction the motors are currently running in
unsigned long manualUntil = 0;
const unsigned long MANUAL_HOLD_MS = 600;

// odometry: x, y in cm, heading in radians (0 = start direction, + = left)
float odoX = 0, odoY = 0, odoH = 0, travelled = 0;
long lastL = 0, lastR = 0;
int avoidedCount = 0;

// Visited cells: 6 x 6 m grid of 10 cm cells around the start point.
const int   GRID_N  = 60;
const float CELL_CM = 10;
const int   VISITED_PENALTY = 20;    // cm of free space one visited cell is worth
uint8_t visited[GRID_N][GRID_N];
int lastCellX = -1000, lastCellY = -1000;

int cellOf(float v) { return (int)floor(v / CELL_CM) + GRID_N / 2; }

void markVisited() {
  int cx = cellOf(odoX), cy = cellOf(odoY);
  if (cx == lastCellX && cy == lastCellY) return;
  lastCellX = cx; lastCellY = cy;
  const int nb[5][2] = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (auto &d : nb) {
    int x = cx + d[0], y = cy + d[1];
    if (x >= 0 && x < GRID_N && y >= 0 && y < GRID_N) visited[x][y] = 1;
  }
}

bool isVisited(float x, float y) {
  int cx = cellOf(x), cy = cellOf(y);
  return cx >= 0 && cx < GRID_N && cy >= 0 && cy < GRID_N && visited[cx][cy];
}

void clearVisited() {
  memset(visited, 0, sizeof(visited));
  lastCellX = lastCellY = -1000;
}

const int MAX_SCAN = 2 * SCAN_SPAN / SCAN_STEP + 1;
int scanAngle[MAX_SCAN];
long scanDist[MAX_SCAN];
int scanCount = 0, scanId = 0;

char eventText[96] = "";
int eventId = 0;

void setEvent(const char *msg) {
  strncpy(eventText, msg, sizeof(eventText) - 1);
  eventId++;
  Serial.println(msg);
}

void stepISR(timer_callback_args_t *) {
  if (motorsRunning) {
    leftM.run();
    rightM.run();
  }
}

void updateOdometry() {
  long l = leftM.currentPosition(), r = rightM.currentPosition();
  float dl = LEFT_DIR  * (l - lastL) * CM_PER_STEP;
  float dr = RIGHT_DIR * (r - lastR) * CM_PER_STEP;
  lastL = l; lastR = r;
  float ds = (dl + dr) / 2, dh = (dr - dl) / TRACK;
  float mid = odoH + dh / 2;
  odoX -= ds * sin(mid);
  odoY += ds * cos(mid);
  odoH += dh;
  travelled += fabs(ds);
  markVisited();
}

void motorsOn()  { leftM.enableOutputs();  rightM.enableOutputs(); }
void motorsOff() { leftM.disableOutputs(); rightM.disableOutputs(); }

void startMove(long l, long r, float speed) {   // robot frame, + = forward
  motorsRunning = false;
  motorsOn();
  leftM.setMaxSpeed(speed);
  rightM.setMaxSpeed(speed);
  leftM.setAcceleration(ACCEL);
  rightM.setAcceleration(ACCEL);
  leftM.move(LEFT_DIR * l);
  rightM.move(RIGHT_DIR * r);
  motorsRunning = true;
}

void halt() {
  motorsRunning = false;
  leftM.setCurrentPosition(leftM.currentPosition());     // also resets speed to 0
  rightM.setCurrentPosition(rightM.currentPosition());
  motorsOff();
  updateOdometry();
  driving = false;
}

bool moving() { return leftM.distanceToGo() != 0 || rightM.distanceToGo() != 0; }

void handleWeb();

void moveSteps(long l, long r) {
  startMove(l, r, turnSpeed);
  while (moving()) {
    updateOdometry();
    handleWeb();
    if (!enabled) break;
  }
  halt();
}

int servoAngle = SCAN_CENTER;

void servoTo(int target) {             // move slowly, never past the limits
  target = constrain(target, SCAN_CENTER - SCAN_SPAN, SCAN_CENTER + SCAN_SPAN);
  while (servoAngle != target) {
    servoAngle += (target > servoAngle) ? 1 : -1;
    scanner.write(servoAngle);
    delay(8);
    handleWeb();
  }
  delay(150);                          // let it settle before measuring
}

long pingOnce() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseInLong(ECHO, HIGH, 25000);
  return us == 0 ? 999 : us / 58;
}

long medianDistance() {
  long a = pingOnce(); delay(30);
  long b = pingOnce(); delay(30);
  long c = pingOnce();
  return max(min(a, b), min(max(a, b), c));
}

// Arc of TURN_RADIUS: heading change deg (+ = left), dir 1 forward, -1 backward.
void arc(float deg, int dir) {
  float th = deg * PI / 180;
  float s  = dir * fabs(th) * TURN_RADIUS;
  long l = cmToSteps(s - th * TRACK / 2);
  long r = cmToSteps(s + th * TRACK / 2);
  float big = max(labs(l), labs(r));
  motorsRunning = false;
  motorsOn();
  leftM.setMaxSpeed(turnSpeed * labs(l) / big);
  rightM.setMaxSpeed(turnSpeed * labs(r) / big);
  leftM.setAcceleration(ACCEL * labs(l) / big);
  rightM.setAcceleration(ACCEL * labs(r) / big);
  leftM.move(LEFT_DIR * l);
  rightM.move(RIGHT_DIR * r);
  motorsRunning = true;
  state = "turning";
  unsigned long lastCheck = 0;
  while (moving()) {
    updateOdometry();
    handleWeb();
    if (!enabled) break;
    if (dir > 0 && millis() - lastCheck > 60) {
      lastCheck = millis();
      long d = pingOnce();
      lastDistance = d;
      if (d < SAFE_CM / 2) { setEvent("Curve stopped early: obstacle very close"); break; }
    }
  }
  halt();
}

int visitedAlong(float x, float y, float h) {
  int n = 0;
  for (float d = 25; d <= 85; d += 10)
    if (isVisited(x - d * sin(h), y + d * cos(h))) n++;
  return n;
}

// Visited cells ahead after a curve to this side (+1 left, -1 right).
int visitedAfterCurve(int side, int dir) {
  float th = side * TURN_DEG * PI / 180, s = dir * fabs(th) * TURN_RADIUS;
  float x = odoX, y = odoY;
  for (int i = 0; i < 10; i++) {
    float h = odoH + th * (i + 0.5) / 10;
    x -= s / 10 * sin(h);
    y += s / 10 * cos(h);
  }
  float h = odoH + th;
  return visitedAlong(x, y, h) + visitedAlong(x, y, h + th) / 2;
}

void avoid(bool stuck = false) {
  halt();
  avoidedCount++;
  char msg[96];
  if (stuck)
    snprintf(msg, sizeof(msg), "Stuck: wheels turned %d cm but distance stayed %ld cm, backing up %d cm",
             (int)STUCK_TRAVEL_CM, lastDistance, BACKUP_CM);
  else
    snprintf(msg, sizeof(msg), "Obstacle at %ld cm, backing up %d cm", lastDistance, BACKUP_CM);
  setEvent(msg);

  state = "backing";
  moveSteps(-cmToSteps(BACKUP_CM), -cmToSteps(BACKUP_CM));
  if (!enabled) return;

  state = "scanning";
  scanId++;
  scanCount = 0;
  long leftMin = 999, rightMin = 999, leftSum = 0, rightSum = 0;
  for (int a = SCAN_CENTER - SCAN_SPAN; a <= SCAN_CENTER + SCAN_SPAN; a += SCAN_STEP) {
    servoTo(a);
    long d = medianDistance();
    scanAngle[scanCount] = a;
    scanDist[scanCount] = d;
    scanCount++;
    long dc = min(d, 400L);                       // "no echo" (999) counts as 4 m
    if (a > SCAN_CENTER) { leftMin = min(leftMin, d);   leftSum += dc; }
    if (a < SCAN_CENTER) { rightMin = min(rightMin, d); rightSum += dc; }
    handleWeb();
    if (!enabled) { servoTo(SCAN_CENTER); return; }
  }
  servoTo(SCAN_CENTER);

  // score = free space minus a penalty for visited cells
  bool bothBlocked = leftMin < SAFE_CM && rightMin < SAFE_CM;
  int dir = bothBlocked ? -1 : 1;
  int visL = visitedAfterCurve(1, dir), visR = visitedAfterCurve(-1, dir);
  long scoreL = min(leftMin, 100L)  - (long)VISITED_PENALTY * visL;
  long scoreR = min(rightMin, 100L) - (long)VISITED_PENALTY * visR;
  if (!bothBlocked) {
    if (leftMin  < SAFE_CM) scoreL = -1000;
    if (rightMin < SAFE_CM) scoreR = -1000;
  }
  // +1 left, -1 right; ties go by total free space, then alternate
  static int tieSide = 1;
  int side;
  if (labs(scoreL - scoreR) >= 10)  side = scoreL > scoreR ? 1 : -1;
  else if (leftSum != rightSum)     side = leftSum > rightSum ? 1 : -1;
  else                              { side = tieSide; tieSide = -tieSide; }
  const char *name = side > 0 ? "left" : "right";

  if (bothBlocked) {
    snprintf(msg, sizeof(msg), "Both sides blocked (L %ld / R %ld cm, visited L %d / R %d): reverse-curve %s",
             leftMin, rightMin, visL, visR, name);
    setEvent(msg);
    arc(side * TURN_DEG, -1);
  } else {
    snprintf(msg, sizeof(msg), "Curve %s %d deg (free L %ld / R %ld cm, visited L %d / R %d)",
             name, (int)TURN_DEG, leftMin, rightMin, visL, visR);
    setEvent(msg);
    arc(side * TURN_DEG, 1);
  }
}

void sendHeader(WiFiClient &c, const char *type, size_t len) {
  c.println("HTTP/1.1 200 OK");
  c.print("Content-Type: "); c.println(type);
  c.print("Content-Length: "); c.println(len);
  c.println("Cache-Control: no-store");
  c.println("Connection: close");
  c.println();
}

void sendDashboard(WiFiClient &c) {
  size_t len = strlen(DASHBOARD_HTML);
  sendHeader(c, "text/html", len);
  for (size_t i = 0; i < len; i += 1024)
    c.write((const uint8_t *)DASHBOARD_HTML + i, min((size_t)1024, len - i));
}

void sendData(WiFiClient &c) {
  updateOdometry();
  String j = "{\"run\":";
  j += enabled ? 1 : 0;
  j += ",\"st\":\"";  j += state;
  j += "\",\"d\":";   j += lastDistance;
  j += ",\"a\":";     j += servoAngle;
  j += ",\"x\":";     j += String(odoX, 1);
  j += ",\"y\":";     j += String(odoY, 1);
  j += ",\"h\":";     j += String(odoH, 4);
  j += ",\"tr\":";    j += String(travelled, 1);
  j += ",\"av\":";    j += avoidedCount;
  j += ",\"sf\":";    j += SAFE_CM;
  j += ",\"cs\":";    j += (int)cruiseSpeed;
  j += ",\"ts\":";    j += (int)turnSpeed;
  j += ",\"up\":";    j += millis() / 1000;
  j += ",\"sid\":";   j += scanId;
  j += ",\"scan\":[";
  for (int i = 0; i < scanCount; i++) {
    if (i) j += ",";
    j += "["; j += scanAngle[i]; j += ","; j += scanDist[i]; j += "]";
  }
  j += "],\"eid\":";  j += eventId;
  j += ",\"ev\":\"";  j += eventText;
  j += "\"}";
  sendHeader(c, "application/json", j.length());
  c.print(j);
}

void sendOk(WiFiClient &c) {
  sendHeader(c, "application/json", 8);
  c.print("{\"ok\":1}");
}

void handleWeb() {
  static unsigned long lastCheck = 0;
  if (!wifiOk || millis() - lastCheck < 50) return;
  lastCheck = millis();

  WiFiClient c = server.available();
  if (!c) return;

  String line = "";
  unsigned long t0 = millis();
  while (c.connected() && millis() - t0 < 300) {
    if (!c.available()) continue;
    char ch = c.read();
    if (ch == '\n') break;
    line += ch;
  }
  while (c.available()) c.read();          // drop the rest of the request

  if (line.startsWith("GET /data")) {
    sendData(c);
  } else if (line.startsWith("GET /start")) {
    if (!enabled) setEvent("Started (automatic)");
    manualCmd = manualActive = 0;
    halt();
    enabled = true;
    sendOk(c);
  } else if (line.startsWith("GET /stop")) {
    if (enabled) setEvent("Stopped");
    enabled = false;
    manualCmd = manualActive = 0;
    halt();
    state = "stopped";
    sendOk(c);
  } else if (line.startsWith("GET /m?d=")) {
    char d = line.charAt(9);
    if (enabled) {
      enabled = false;
      halt();
      setEvent("Manual control");
    }
    if (d == 'f' || d == 'b' || d == 'l' || d == 'r') {
      manualCmd = d;
      manualUntil = millis() + MANUAL_HOLD_MS;
    } else {
      manualCmd = 0;
    }
    sendOk(c);
  } else if (line.startsWith("GET /set?")) {
    int ci = line.indexOf("c="), ti = line.indexOf("t=");
    if (ci > 0) cruiseSpeed = constrain(line.substring(ci + 2).toInt(), 400, 2000);
    if (ti > 0) turnSpeed   = constrain(line.substring(ti + 2).toInt(), 400, 2000);

    if (driving) {
      motorsRunning = false;
      leftM.setMaxSpeed(cruiseSpeed);
      rightM.setMaxSpeed(cruiseSpeed);
      motorsRunning = true;
    }
    char msg[96];
    snprintf(msg, sizeof(msg), "Speed set: drive %d, turn %d", (int)cruiseSpeed, (int)turnSpeed);
    setEvent(msg);
    sendOk(c);
  } else if (line.startsWith("GET /reset")) {
    updateOdometry();
    odoX = odoY = odoH = travelled = 0;
    clearVisited();
    avoidedCount = 0;
    scanCount = 0;
    setEvent("Map reset");
    sendOk(c);
  } else if (line.startsWith("GET / ")) {
    sendDashboard(c);
  } else {
    c.println("HTTP/1.1 404 Not Found");
    c.println("Connection: close");
    c.println();
  }
  delay(2);
  c.stop();
}

void connectWiFi() {
  Serial.print("Connecting to "); Serial.println(WIFI_SSID);
  // static IP, the hotspot gave no DHCP address
  WiFi.config(IPAddress(192, 168, 137, 50), IPAddress(192, 168, 137, 1),
              IPAddress(192, 168, 137, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    delay(1000);
    server.begin();
    wifiOk = true;
    Serial.print("Open http://"); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed, retrying in 5 s");
  }
}

void setup() {
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  Serial.begin(115200);

  leftM.setAcceleration(ACCEL);
  rightM.setAcceleration(ACCEL);
  motorsOff();

  scanner.write(SCAN_CENTER);
  scanner.attach(10);

  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) ch = FspTimer::get_available_timer(type, true);
  stepTimer.begin(TIMER_MODE_PERIODIC, type, ch, 10000.0f, 0.0f, stepISR);
  stepTimer.setup_overflow_irq();
  stepTimer.open();
  stepTimer.start();

  connectWiFi();
}

void loop() {
  static unsigned long lastPing = 0;
  static unsigned long lastWifiTry = 0;
  static long  stuckRefDist = -1;
  static float stuckRefTrav = 0;

  if (!wifiOk && millis() - lastWifiTry > 5000) {
    lastWifiTry = millis();
    connectWiFi();
  }
  handleWeb();
  updateOdometry();

  if (!enabled) {
    if (manualCmd && millis() > manualUntil) manualCmd = 0;
    if (servoAngle != SCAN_CENTER) servoTo(SCAN_CENTER);
    if (millis() - lastPing > 60) {
      lastPing = millis();
      lastDistance = pingOnce();
      if (manualCmd == 'f' && lastDistance < SAFE_CM) {
        manualCmd = 0;
        setEvent("Manual forward blocked: obstacle ahead");
      }
    }
    if (manualCmd != manualActive) {
      halt();
      manualActive = manualCmd;
      if      (manualCmd == 'f') startMove( FAR_AWAY,  FAR_AWAY, cruiseSpeed);
      else if (manualCmd == 'b') startMove(-FAR_AWAY, -FAR_AWAY, cruiseSpeed);
      else if (manualCmd == 'l') startMove(-FAR_AWAY,  FAR_AWAY, turnSpeed);
      else if (manualCmd == 'r') startMove( FAR_AWAY, -FAR_AWAY, turnSpeed);
    }
    state = manualCmd ? "manual" : "stopped";
    return;
  }

  if (!driving) {
    startMove(FAR_AWAY, FAR_AWAY, cruiseSpeed);
    driving = true;
    state = "driving";
    stuckRefDist = -1;
  }

  if (millis() - lastPing > 60) {
    lastPing = millis();
    long d = pingOnce();
    lastDistance = d;
    bool blocked = false;
    if (d < SAFE_CM) {
      delay(30);
      blocked = pingOnce() < SAFE_CM + 5;             // confirm
    }
    if (blocked) {
      avoid();
      return;
    }

    // stuck check
    if (d >= 300) {
      stuckRefDist = -1;
    } else if (stuckRefDist < 0) {
      stuckRefDist = d;
      stuckRefTrav = travelled;
    } else if (travelled - stuckRefTrav >= STUCK_TRAVEL_CM) {
      if (labs(d - stuckRefDist) < STUCK_CHANGE_CM) {
        avoid(true);
        return;
      }
      stuckRefDist = d;
      stuckRefTrav = travelled;
    }
  }
}
