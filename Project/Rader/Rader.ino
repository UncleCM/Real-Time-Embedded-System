#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// --- Pins ---
#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 13

// --- Display setup ---
#define SCREEN_W 240
#define SCREEN_H 320
#define CENTER_X (SCREEN_W / 2)
#define CENTER_Y (SCREEN_H / 2 + 20)
#define MAX_DISTANCE 100
#define RADAR_RADIUS 140

Servo servo;
TFT_eSPI tft = TFT_eSPI();

// Shared data
struct RadarData {
  int angle;
  int distance;
  bool updated;
};
RadarData radarData;
SemaphoreHandle_t dataMutex;

// Blip tracking
struct Blip {
  float angle;
  int distance;
  uint32_t fadeTime;
};
Blip blips[20];
int blipCount = 0;

// --- Task handles ---
TaskHandle_t taskScanHandle;
TaskHandle_t taskDisplayHandle;
TaskHandle_t taskLoggerHandle;

// --- Function declarations ---
long measureDistance();
void drawRadarBase();
void addBlip(float angle, int distance);
void drawBlips();
int mapX(float angleDeg, int dist);
int mapY(float angleDeg, int dist);

// ============================================================
//  Task: Scan (servo + distance measurement)
// ============================================================
void TaskScan(void *pvParameters) {
  int direction = 1;
  int angle = 0;

  while (true) {
    servo.write(angle);               // keep normal rotation
    long distance = measureDistance();

    // Lock and update shared data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    radarData.angle = angle;
    radarData.distance = distance;
    radarData.updated = true;
    xSemaphoreGive(dataMutex);

    angle += 3 * direction;
    if (angle >= 180 || angle <= 0) direction = -direction;

    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

// ============================================================
//  Task: Display (draw radar sweep + blips)
// ============================================================
void TaskDisplay(void *pvParameters) {
  int lastAngle = 0;

  drawRadarBase();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawCentreString("ESP32 Submarine Radar", SCREEN_W / 2, 5, 2);

  while (true) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    bool hasUpdate = radarData.updated;
    int rawAngle = radarData.angle;
    int distance = radarData.distance;
    radarData.updated = false;
    xSemaphoreGive(dataMutex);

    if (hasUpdate) {
      // 🔁 invert angle for display only
      int displayAngle = 180 - rawAngle;

      // Erase previous sweep
      int oldX = mapX(180 - lastAngle, RADAR_RADIUS);
      int oldY = mapY(180 - lastAngle, RADAR_RADIUS);
      tft.drawLine(CENTER_X, CENTER_Y, oldX, oldY, TFT_BLACK);

      // Draw new sweep
      int newX = mapX(displayAngle, RADAR_RADIUS);
      int newY = mapY(displayAngle, RADAR_RADIUS);
      tft.drawLine(CENTER_X, CENTER_Y, newX, newY, TFT_GREEN);
      lastAngle = rawAngle;

      // Add and draw blips
      if (distance > 5 && distance < MAX_DISTANCE) {
        addBlip(displayAngle, distance);
      }
      drawBlips();

      // Info text
      tft.fillRect(0, SCREEN_H - 20, SCREEN_W, 20, TFT_BLACK);
      tft.setCursor(10, SCREEN_H - 18);
      tft.printf("Angle: %3d°  Dist: %3d cm", rawAngle, distance);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
//  Task: Logger (print info to Serial)
// ============================================================
void TaskLogger(void *pvParameters) {
  while (true) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int angle = radarData.angle;
    int distance = radarData.distance;
    xSemaphoreGive(dataMutex);

    Serial.printf("Angle: %3d°, Distance: %3d cm\n", angle, distance);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  dataMutex = xSemaphoreCreateMutex();

  // Create tasks pinned to cores
  xTaskCreatePinnedToCore(TaskScan, "Scan", 4096, NULL, 2, &taskScanHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, &taskDisplayHandle, 1);
  xTaskCreatePinnedToCore(TaskLogger, "Logger", 2048, NULL, 1, &taskLoggerHandle, 1);
}

void loop() {
  vTaskDelete(NULL);
}

// ============================================================
//  Utility Functions
// ============================================================
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

void drawRadarBase() {
  tft.fillScreen(TFT_BLACK);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS * 2 / 3, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS / 3, TFT_DARKGREEN);
  tft.drawLine(CENTER_X - RADAR_RADIUS, CENTER_Y, CENTER_X + RADAR_RADIUS, CENTER_Y, TFT_DARKGREEN);
  tft.drawLine(CENTER_X, CENTER_Y - RADAR_RADIUS, CENTER_X, CENTER_Y + RADAR_RADIUS, TFT_DARKGREEN);
}

int mapX(float angleDeg, int dist) {
  float rad = radians(angleDeg);
  return CENTER_X - dist * cos(rad);
}
int mapY(float angleDeg, int dist) {
  float rad = radians(angleDeg);
  return CENTER_Y - dist * sin(rad);
}

void addBlip(float angle, int distance) {
  if (blipCount < 20) {
    blips[blipCount++] = {angle, distance, millis()};
  } else {
    for (int i = 1; i < 20; i++) blips[i - 1] = blips[i];
    blips[19] = {angle, distance, millis()};
  }
}

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
