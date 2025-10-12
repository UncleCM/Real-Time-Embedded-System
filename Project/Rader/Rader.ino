#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 13

Servo servo;
TFT_eSPI tft = TFT_eSPI();

#define SCREEN_W 240
#define SCREEN_H 320
#define CENTER_X (SCREEN_W / 2)
#define CENTER_Y (SCREEN_H / 2 + 20)  // center radar vertically
#define MAX_DISTANCE 100
#define RADAR_RADIUS 140  // bigger radar (almost full screen)

// Store detected objects
struct Blip {
  float angle;
  int distance;
  uint32_t fadeTime;
};
Blip blips[20];
int blipCount = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  drawRadarBase();

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawCentreString("ESP32 Submarine Radar", SCREEN_W / 2, 5, 2);
}

void loop() {
  for (int angle = 0; angle <= 180; angle += 3) {
    updateRadar(angle);
  }
  for (int angle = 180; angle >= 0; angle -= 3) {
    updateRadar(angle);
  }
}

// ---- Measure distance ----
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
  return distance;
}

// ---- Draw radar base grid ----
void drawRadarBase() {
  tft.fillScreen(TFT_BLACK);

  // Circular grid
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS * 2 / 3, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS / 3, TFT_DARKGREEN);

  // Cross lines
  tft.drawLine(CENTER_X - RADAR_RADIUS, CENTER_Y, CENTER_X + RADAR_RADIUS, CENTER_Y, TFT_DARKGREEN);
  tft.drawLine(CENTER_X, CENTER_Y - RADAR_RADIUS, CENTER_X, CENTER_Y + RADAR_RADIUS, TFT_DARKGREEN);
}

// ---- Polar → Cartesian ----
int mapX(float angleDeg, int dist) {
  float rad = radians(angleDeg);
  return CENTER_X - dist * cos(rad);
}
int mapY(float angleDeg, int dist) {
  float rad = radians(angleDeg);
  return CENTER_Y - dist * sin(rad);
}

// ---- Radar update ----
void updateRadar(int angle) {
  servo.write(angle);
  long distance = measureDistance();

  // Erase previous sweep
  static int lastAngle = 0;
  drawSweepLine(lastAngle, TFT_BLACK);

  // Draw new sweep
  drawSweepLine(angle, TFT_GREEN);
  lastAngle = angle;

  // Add blip if something detected
  if (distance < MAX_DISTANCE && distance > 5) {
    addBlip(angle, distance);
  }

  drawBlips();

  // Text info
  tft.fillRect(0, SCREEN_H - 20, SCREEN_W, 20, TFT_BLACK);
  tft.setCursor(10, SCREEN_H - 18);
  tft.print("Angle: ");
  tft.print(angle);
  tft.print("°  Dist: ");
  tft.print(distance);
  tft.println(" cm");

  delay(80);
}

// ---- Sweep line ----
void drawSweepLine(int angle, uint16_t color) {
  int x = mapX(angle, RADAR_RADIUS);
  int y = mapY(angle, RADAR_RADIUS);
  tft.drawLine(CENTER_X, CENTER_Y, x, y, color);
}

// ---- Add blip ----
void addBlip(float angle, int distance) {
  if (blipCount < 20) {
    blips[blipCount++] = {angle, distance, millis()};
  } else {
    for (int i = 1; i < 20; i++) blips[i - 1] = blips[i];
    blips[19] = {angle, distance, millis()};
  }
}

// ---- Fading blips ----
void drawBlips() {
  uint32_t now = millis();
  for (int i = 0; i < blipCount; i++) {
    uint32_t age = now - blips[i].fadeTime;
    if (age > 2000) continue;

    int alpha = map(age, 0, 2000, 255, 0);
    uint16_t color = tft.color565(alpha / 4, alpha, alpha / 8);
    int distPix = map(blips[i].distance, 0, MAX_DISTANCE, 0, RADAR_RADIUS);
    int bx = mapX(blips[i].angle, distPix);
    int by = mapY(blips[i].angle, distPix);
    tft.fillCircle(bx, by, 3, color);
  }
}
