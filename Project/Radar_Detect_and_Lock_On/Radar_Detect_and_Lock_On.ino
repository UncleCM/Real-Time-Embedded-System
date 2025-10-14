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

#define LOCK_THRESHOLD 30     // cm - detection range for tracking
#define MIN_DISTANCE 5        // cm - minimum valid distance
#define UNLOCK_TIMEOUT 2000   // ms - time before losing lock
#define TRACK_ZONE 10         // degrees - narrow tracking zone for precise lock

Servo servo;
TFT_eSPI tft = TFT_eSPI();

struct RadarData {
  int angle;
  int distance;
  bool updated;
  bool lockedOn;
  bool objectMoving;
  int movementDirection; // -1=left, 0=stationary, 1=right
};
RadarData radarData;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t modeChangeSemaphore; // Signal mode changes
QueueHandle_t detectionQueue;          // Pass detection data between tasks

struct DetectionEvent {
  int angle;
  int distance;
  uint32_t timestamp;
};

struct Blip {
  float angle;
  int distance;
  uint32_t fadeTime;
};
Blip blips[20];
int blipCount = 0;

TaskHandle_t taskSearchHandle;
TaskHandle_t taskTrackHandle;
TaskHandle_t taskDisplayHandle;
TaskHandle_t taskLoggerHandle;
TaskHandle_t taskBuzzerHandle;

enum RadarMode { SEARCHING, TRACKING };
volatile RadarMode currentMode = SEARCHING;

// Shared tracking data
volatile int targetAngle = 90;
volatile uint32_t lastDetectionTime = 0;

// Movement detection variables
int lastTrackedAngle = 90;
int lastTrackedDistance = 0;
uint32_t lastTrackUpdate = 0;
int angleHistory[5] = {90, 90, 90, 90, 90};
int historyIndex = 0;

// ====================== Detect Movement =======================
void detectMovement(int currentAngle, int currentDistance) {
  // Update angle history
  angleHistory[historyIndex] = currentAngle;
  historyIndex = (historyIndex + 1) % 5;
  
  // Calculate average angle change
  int angleSum = 0;
  for (int i = 0; i < 5; i++) {
    angleSum += angleHistory[i];
  }
  int avgAngle = angleSum / 5;
  
  // Detect angle change (horizontal movement)
  int angleDiff = avgAngle - lastTrackedAngle;
  
  // Detect distance change (approaching/receding)
  int distDiff = currentDistance - lastTrackedDistance;
  
  bool isMoving = false;
  int moveDir = 0;
  
  // Movement threshold: >3° angle change or >5cm distance change
  if (abs(angleDiff) > 3) {
    isMoving = true;
    moveDir = (angleDiff > 0) ? 1 : -1;  // 1=moving right, -1=moving left
  } else if (abs(distDiff) > 5) {
    isMoving = true;
    moveDir = 0;  // Moving toward/away (not left/right)
  }
  
  // Update tracking data
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  radarData.objectMoving = isMoving;
  radarData.movementDirection = moveDir;
  xSemaphoreGive(dataMutex);
  
  // Update last known position
  if (millis() - lastTrackUpdate > 200) {
    lastTrackedAngle = avgAngle;
    lastTrackedDistance = currentDistance;
    lastTrackUpdate = millis();
  }
}
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return MAX_DISTANCE;
  long distance = duration * 0.034 / 2;
  if (distance > MAX_DISTANCE) distance = MAX_DISTANCE;
  return distance;
}

// ====================== Search Task ==============================
void TaskSearch(void *pvParameters) {
  int sweepAngle = 0;
  int direction = 1;

  servo.write(0); // Start at 0 degrees

  while (true) {
    if (currentMode == SEARCHING) {
      // Active search mode
      long distance = measureDistance();
      bool detected = (distance > MIN_DISTANCE && distance <= LOCK_THRESHOLD);

      // Update radar data
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      radarData.angle = sweepAngle;
      radarData.distance = distance;
      radarData.lockedOn = false;
      radarData.updated = true;
      xSemaphoreGive(dataMutex);

      if (detected) {
        // Target found! Switch to tracking mode
        targetAngle = sweepAngle;
        lastDetectionTime = millis();
        currentMode = TRACKING;
        
        Serial.print("[SEARCH] Target acquired at ");
        Serial.print(sweepAngle);
        Serial.print("° - Distance: ");
        Serial.print(distance);
        Serial.println("cm - LOCKING ON");
        
        // Send detection event via queue
        DetectionEvent evt = {sweepAngle, (int)distance, millis()};
        xQueueSend(detectionQueue, &evt, 0);
        
        // Signal mode change
        xSemaphoreGive(modeChangeSemaphore);
        
        // Wake up tracking task
        vTaskResume(taskTrackHandle);
        
        // This task will sleep until tracking is done
        vTaskDelay(pdMS_TO_TICKS(100));
      } else {
        // Continue sweeping
        sweepAngle += 3 * direction;
        if (sweepAngle >= 180) {
          sweepAngle = 180;
          direction = -1;
        }
        if (sweepAngle <= 0) {
          sweepAngle = 0;
          direction = 1;
        }
        
        servo.write(sweepAngle);
        vTaskDelay(pdMS_TO_TICKS(50));
      }
    } else {
      // Tracking mode active - this task sleeps
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}

// ====================== Track Task ==============================
void TaskTrack(void *pvParameters) {
  int currentAngle = 90;
  int minAngle, maxAngle;
  const int trackStep = 1;  // Very small steps for precise tracking
  int trackDirection = 1;
  int stableCount = 0;
  int lastGoodAngle = 90;
  int lastGoodDistance = 0;

  // Start suspended, will be resumed by search task
  vTaskSuspend(NULL);

  while (true) {
    if (currentMode == TRACKING) {
      // Define narrow tracking zone around target
      minAngle = max(0, targetAngle - TRACK_ZONE);
      maxAngle = min(180, targetAngle + TRACK_ZONE);
      
      // Initialize if just started tracking
      if (currentAngle < minAngle || currentAngle > maxAngle) {
        currentAngle = targetAngle;
        trackDirection = 1;
        stableCount = 0;
      }
      
      // Small sweep around target position
      currentAngle += trackStep * trackDirection;
      if (currentAngle >= maxAngle) {
        currentAngle = maxAngle;
        trackDirection = -1;
      }
      if (currentAngle <= minAngle) {
        currentAngle = minAngle;
        trackDirection = 1;
      }
      
      servo.write(currentAngle);
      vTaskDelay(pdMS_TO_TICKS(40));
      
      long distance = measureDistance();
      bool detected = (distance > MIN_DISTANCE && distance <= LOCK_THRESHOLD);
      
      if (detected) {
        // Target found at this angle
        lastDetectionTime = millis();
        stableCount++;
        lastGoodAngle = currentAngle;
        lastGoodDistance = distance;
        
        // Detect if object is moving
        detectMovement(currentAngle, distance);
        
        // If object is stable at this position, center on it
        if (stableCount >= 3) {
          targetAngle = lastGoodAngle;
          
          // For very stable stationary objects, stay put
          if (abs(currentAngle - targetAngle) <= 2) {
            currentAngle = targetAngle;
            vTaskDelay(pdMS_TO_TICKS(100)); // Hold position longer
          }
        }
        
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        radarData.angle = currentAngle;
        radarData.distance = distance;
        radarData.lockedOn = true;
        radarData.updated = true;
        xSemaphoreGive(dataMutex);
      } else {
        // No detection at this angle
        stableCount = max(0, stableCount - 1);
        
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        radarData.angle = currentAngle;
        radarData.distance = distance;
        radarData.lockedOn = false;
        radarData.updated = true;
        xSemaphoreGive(dataMutex);
      }
      
      // Check if target completely lost
      if (millis() - lastDetectionTime > UNLOCK_TIMEOUT) {
        // Lost target - switch back to search
        currentMode = SEARCHING;
        stableCount = 0;
        Serial.println("[TRACK] Target lost - switching to search mode");
        
        // Signal mode change
        xSemaphoreGive(modeChangeSemaphore);
        
        // Suspend this task, let search take over
        vTaskSuspend(NULL);
      }
    } else {
      // Not in tracking mode - suspend
      vTaskSuspend(NULL);
    }
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
  
  // Draw angle markers
  tft.setTextFont(1);
  tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
  tft.drawString("0", CENTER_X + RADAR_RADIUS - 15, CENTER_Y + 5, 1);
  tft.drawString("90", CENTER_X - 10, CENTER_Y - RADAR_RADIUS + 5, 1);
  tft.drawString("180", CENTER_X - RADAR_RADIUS + 5, CENTER_Y + 5, 1);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawCentreString("ESP32 Submarine Radar", SCREEN_W / 2, 5, 2);
  
  // Draw distance labels on radar circles
  tft.setTextFont(1);
  tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
  
  // Inner circle - 33cm
  tft.drawString("33cm", CENTER_X + RADAR_RADIUS / 3 + 5, CENTER_Y - 5, 1);
  
  // Middle circle - 67cm
  tft.drawString("67cm", CENTER_X + (RADAR_RADIUS * 2 / 3) + 5, CENTER_Y - 5, 1);
  
  // Outer circle - 100cm
  tft.drawString("100cm", CENTER_X + RADAR_RADIUS + 5, CENTER_Y - 5, 1);
  
  tft.setTextFont(2); // Reset to normal font

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
      
      // Color based on mode
      uint16_t sweepColor = (currentMode == TRACKING) ? TFT_RED : TFT_GREEN;
      tft.drawLine(CENTER_X, CENTER_Y, newX, newY, sweepColor);
      lastAngle = rawAngle;

      // Add blips for detected objects
      if (distance > MIN_DISTANCE && distance < MAX_DISTANCE) {
        blips[blipCount++ % 20] = {float(displayAngle), distance, millis()};
      }

      // Draw all active blips with fade effect
      for (int i = 0; i < blipCount && i < 20; i++) {
        uint32_t age = millis() - blips[i].fadeTime;
        if (age > 2000) continue; // Skip old blips
        
        int dPix = map(blips[i].distance, 0, MAX_DISTANCE, 0, RADAR_RADIUS);
        int bx = CENTER_X - dPix * cos(radians(blips[i].angle));
        int by = CENTER_Y - dPix * sin(radians(blips[i].angle));
        
        // Erase old blip first
        tft.fillCircle(bx, by, 5, TFT_BLACK);
        
        // Draw new blip with fade
        if (age < 1500) {
          uint16_t color = (currentMode == TRACKING) ? TFT_RED : TFT_GREENYELLOW;
          int size = (currentMode == TRACKING) ? 4 : 3;
          tft.fillCircle(bx, by, size, color);
        }
      }
      
      // Clean up expired blips from array
      if (blipCount > 20) {
        // Reset and keep only recent blips
        int newCount = 0;
        for (int i = 0; i < 20; i++) {
          if (millis() - blips[i].fadeTime <= 2000) {
            blips[newCount++] = blips[i];
          }
        }
        blipCount = newCount;
      }

      // Get movement status
      bool isMoving = false;
      int moveDir = 0;
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      isMoving = radarData.objectMoving;
      moveDir = radarData.movementDirection;
      xSemaphoreGive(dataMutex);

      // Display status with movement indicator
      tft.fillRect(0, SCREEN_H - 25, SCREEN_W, 25, TFT_BLACK);
      tft.setCursor(10, SCREEN_H - 20);
      if (currentMode == TRACKING) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.printf("TRACK A:%3d D:%2dcm ", rawAngle, distance);
        
        // Movement indicator
        if (isMoving) {
          if (moveDir == 1) tft.print(">>>");      // Moving right
          else if (moveDir == -1) tft.print("<<<"); // Moving left
          else tft.print("<>");                      // Moving toward/away
        } else {
          tft.print("[*]");                          // Stationary
        }
      } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("SEARCH A:%3d D:%3dcm [<30cm]", rawAngle, distance);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// ====================== Logger Task ============================
void TaskLogger(void *pvParameters) {
  while (true) {
    int angle, distance;
    bool isMoving;
    int moveDir;
    
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    angle = radarData.angle;
    distance = radarData.distance;
    isMoving = radarData.objectMoving;
    moveDir = radarData.movementDirection;
    xSemaphoreGive(dataMutex);

    const char* mode = (currentMode == TRACKING) ? "TRACK" : "SEARCH";
    
    if (currentMode == TRACKING) {
      const char* movement = isMoving ? 
        (moveDir == 1 ? "MOVING RIGHT >>>" : 
         moveDir == -1 ? "MOVING LEFT <<<" : 
         "APPROACHING/RECEDING") : "STATIONARY";
      Serial.printf("[%s] Angle: %3d°, Dist: %3d cm - %s\n", mode, angle, distance, movement);
    } else {
      Serial.printf("[%s] Angle: %3d°, Dist: %3d cm\n", mode, angle, distance);
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ====================== Buzzer Task ============================
void TaskBuzzer(void *pvParameters) {
  pinMode(BUZZER_PIN, OUTPUT);
  bool wasTracking = false;
  
  while (true) {
    bool isTracking = (currentMode == TRACKING);

    if (isTracking) {
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(100));
      wasTracking = true;
    } else {
      if (wasTracking) {
        digitalWrite(BUZZER_PIN, LOW);
        wasTracking = false;
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
  
  // Create synchronization primitives
  dataMutex = xSemaphoreCreateMutex();
  modeChangeSemaphore = xSemaphoreCreateBinary();
  detectionQueue = xQueueCreate(10, sizeof(DetectionEvent));

  if (dataMutex == NULL || modeChangeSemaphore == NULL || detectionQueue == NULL) {
    Serial.println("Failed to create RTOS primitives!");
    while(1);
  }

  // Create tasks - Track starts suspended
  xTaskCreatePinnedToCore(TaskSearch, "Search", 3072, NULL, 2, &taskSearchHandle, 0);
  xTaskCreatePinnedToCore(TaskTrack, "Track", 3072, NULL, 2, &taskTrackHandle, 0);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, &taskDisplayHandle, 1);
  xTaskCreatePinnedToCore(TaskLogger, "Logger", 2048, NULL, 0, &taskLoggerHandle, 1);
  xTaskCreatePinnedToCore(TaskBuzzer, "Buzzer", 2048, NULL, 0, &taskBuzzerHandle, 1);

  Serial.println("=================================");
  Serial.println("Radar System Initialized");
  Serial.println("Mode: SEARCH (0-180° sweep)");
  Serial.println("Tracking Range: 30cm");
  Serial.println("RTOS: FreeRTOS on ESP32");
  Serial.println("Tasks: 5 (2 cores)");
  Serial.println("=================================");
}

void loop() { 
  vTaskDelete(NULL); 
}