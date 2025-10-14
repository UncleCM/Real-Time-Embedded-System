#include <ESP32Servo.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 13
#define BUZZER_PIN 25

#define SCREEN_W 240
#define SCREEN_H 320
#define CENTER_X (SCREEN_W / 2)
#define CENTER_Y (SCREEN_H / 2 + 20)
#define MAX_DISTANCE 100
#define RADAR_RADIUS 140

#define LOCK_THRESHOLD 60    // cm
#define UNLOCK_TIMEOUT 2000  // ms

Servo servo;
TFT_eSPI tft = TFT_eSPI();

struct RadarData {
  int angle;
  int distance;
  bool updated;
  bool lockedOn;
};
RadarData radarData;
SemaphoreHandle_t dataMutex;

struct Blip {
  float angle;
  int distance;
  uint32_t fadeTime;
};
Blip blips[20];
int blipCount = 0;

TaskHandle_t taskScanHandle;
TaskHandle_t taskDisplayHandle;
TaskHandle_t taskLoggerHandle;
TaskHandle_t taskBuzzerHandle;

bool targetLocked = false;
uint32_t lastDetectionTime = 0;

// ====================== Measure Distance =======================
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

// ====================== Scan Task ==============================
void TaskScan(void *pvParameters) {
  int direction = 1;
  int angle = 0;

  while (true) {
    if (!targetLocked) servo.write(angle);
    long distance = measureDistance();

    bool lockedNow = (distance > 5 && distance <= LOCK_THRESHOLD);
    if (lockedNow) {
      targetLocked = true;
      lastDetectionTime = millis();
    }

    // Unlock after timeout
    if (targetLocked && (millis() - lastDetectionTime > UNLOCK_TIMEOUT))
      targetLocked = false;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    radarData.angle = angle;
    radarData.distance = distance;
    radarData.lockedOn = targetLocked;
    radarData.updated = true;
    xSemaphoreGive(dataMutex);

    if (!targetLocked) {
      angle += 3 * direction;
      if (angle >= 180 || angle <= 0) direction = -direction;
    }

    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

// ====================== Display Task ===========================
void TaskDisplay(void *pvParameters) {
  int lastAngle = 0;
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Draw base
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS * 2 / 3, TFT_DARKGREEN);
  tft.drawCircle(CENTER_X, CENTER_Y, RADAR_RADIUS / 3, TFT_DARKGREEN);
  tft.drawLine(CENTER_X - RADAR_RADIUS, CENTER_Y, CENTER_X + RADAR_RADIUS, CENTER_Y, TFT_DARKGREEN);
  tft.drawLine(CENTER_X, CENTER_Y - RADAR_RADIUS, CENTER_X, CENTER_Y + RADAR_RADIUS, TFT_DARKGREEN);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawCentreString("ESP32 Submarine Radar", SCREEN_W / 2, 5, 2);

  while (true) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    bool hasUpdate = radarData.updated;
    int rawAngle = radarData.angle;
    int distance = radarData.distance;
    bool locked = radarData.lockedOn;
    radarData.updated = false;
    xSemaphoreGive(dataMutex);

    if (hasUpdate) {
      int displayAngle = 180 - rawAngle;
      int oldX = CENTER_X - RADAR_RADIUS * cos(radians(180 - lastAngle));
      int oldY = CENTER_Y - RADAR_RADIUS * sin(radians(180 - lastAngle));
      tft.drawLine(CENTER_X, CENTER_Y, oldX, oldY, TFT_BLACK);

      int newX = CENTER_X - RADAR_RADIUS * cos(radians(displayAngle));
      int newY = CENTER_Y - RADAR_RADIUS * sin(radians(displayAngle));
      tft.drawLine(CENTER_X, CENTER_Y, newX, newY, locked ? TFT_RED : TFT_GREEN);
      lastAngle = rawAngle;

      if (distance > 5 && distance < MAX_DISTANCE)
        blips[blipCount++ % 20] = {float(displayAngle), distance, millis()};

      for (int i = 0; i < blipCount; i++) {
        uint32_t age = millis() - blips[i].fadeTime;
        if (age > 2000) continue;
        int dPix = map(blips[i].distance, 0, MAX_DISTANCE, 0, RADAR_RADIUS);
        int bx = CENTER_X - dPix * cos(radians(blips[i].angle));
        int by = CENTER_Y - dPix * sin(radians(blips[i].angle));
        uint16_t color = locked ? TFT_RED : TFT_GREENYELLOW;
        tft.fillCircle(bx, by, 3, color);
      }

      tft.fillRect(0, SCREEN_H - 25, SCREEN_W, 25, TFT_BLACK);
      tft.setCursor(10, SCREEN_H - 20);
      if (locked)
        tft.printf("LOCKED ON!  Angle: %3d  Dist: %3d cm", rawAngle, distance);
      else
        tft.printf("Angle: %3d°  Dist: %3d cm", rawAngle, distance);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ====================== Logger Task ============================
void TaskLogger(void *pvParameters) {
  while (true) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int angle = radarData.angle;
    int distance = radarData.distance;
    bool locked = radarData.lockedOn;
    xSemaphoreGive(dataMutex);

    Serial.printf("[Radar] Angle: %3d°, Dist: %3d cm %s\n",
                  angle, distance, locked ? " <LOCKED>" : "");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ====================== Buzzer Task ============================
void TaskBuzzer(void *pvParameters) {
  pinMode(BUZZER_PIN, OUTPUT);
  bool wasLocked = false;
  while (true) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    bool locked = radarData.lockedOn;
    xSemaphoreGive(dataMutex);

    if (locked) {
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
      wasLocked = true;
    } else {
      if (wasLocked) {
        digitalWrite(BUZZER_PIN, LOW);
        wasLocked = false;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ====================== Setup ================================
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(TaskScan, "Scan", 4096, NULL, 2, &taskScanHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, &taskDisplayHandle, 1);
  xTaskCreatePinnedToCore(TaskLogger, "Logger", 2048, NULL, 1, &taskLoggerHandle, 1);
  xTaskCreatePinnedToCore(TaskBuzzer, "Buzzer", 2048, NULL, 1, &taskBuzzerHandle, 1);
}

void loop() { vTaskDelete(NULL); }
