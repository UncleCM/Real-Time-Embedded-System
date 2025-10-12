#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <DHT.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

const int RED_BUTTON_PIN = 4;
const int BLUE_BUTTON_PIN = 16;
const int WHITE_BUTTON_PIN = 17;
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const int DHT_PIN = 14;
const int RELAY_LED1 = 33;  // Fan
const int RELAY_LED2 = 32;  // Light
const int LIGHT_SENSOR_PIN = 35;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define DHT_TYPE DHT11

#define TEMP_HYSTERESIS 1.0
#define LDR_HYSTERESIS 80

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHT_PIN, DHT_TYPE);

volatile uint8_t display_value = 1;
volatile float humidity = 0.0;
volatile float temperature = 0.0;
volatile int light = 0;
volatile int temp_set_value = 25;  // Default temp setpoint
volatile int ldr_set_value = 500;  // Default LDR setpoint
volatile bool relay1_status = 0;
volatile bool relay2_status = 0;
volatile bool sub_display = 0;

volatile bool in_setting_mode = false;
volatile unsigned long last_button_time = 0;
const unsigned long DEBOUNCE_DELAY = 200;

void IRAM_ATTR handleRedButtonInterrupt() {
    unsigned long current_time = millis();
    if (current_time - last_button_time > DEBOUNCE_DELAY) {
        if (in_setting_mode) {
            in_setting_mode = false;
            sub_display = false;
        } else {
            display_value++;
            if (display_value > 3) {
                display_value = 1;
            }
        }
        last_button_time = current_time;
    }
}

void IRAM_ATTR handleBlueButtonInterrupt() {
    unsigned long current_time = millis();
    if (current_time - last_button_time > DEBOUNCE_DELAY) {
        if (in_setting_mode) {
            if (display_value == 1) {
                temp_set_value++;
                if (temp_set_value > 50) temp_set_value = 50;
            } else if (display_value == 2) {
                ldr_set_value += 50;
                if (ldr_set_value > 4095) ldr_set_value = 4095;
            }
        } else {
            if (display_value == 1 || display_value == 2) {
                in_setting_mode = true;
                sub_display = true;
            }
        }
        last_button_time = current_time;
    }
}

void IRAM_ATTR handleWhiteButtonInterrupt() {
    unsigned long current_time = millis();
    if (current_time - last_button_time > DEBOUNCE_DELAY) {
        if (in_setting_mode) {
            if (display_value == 1) {
                temp_set_value--;
                if (temp_set_value < 0) temp_set_value = 0;
            } else if (display_value == 2) {
                ldr_set_value -= 50;
                if (ldr_set_value < 0) ldr_set_value = 0;
            }
        }
        last_button_time = current_time;
    }
}

void dhtTask(void *pvParameters) {
    dht.begin();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        
        if (!isnan(h) && !isnan(t)) {
            humidity = h;
            temperature = t;
        } else {
            Serial.println("Failed to read from DHT sensor!");
        }
    }
}

void ldrTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        light = analogRead(LIGHT_SENSOR_PIN);
    }
}

void relayControlTask(void *pvParameters) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (temperature > temp_set_value) {
            digitalWrite(RELAY_LED1, LOW);  
            relay1_status = 1;
        } else if (temperature < (temp_set_value - TEMP_HYSTERESIS)) {
            digitalWrite(RELAY_LED1, HIGH);
            relay1_status = 0;
        }
        
          if (light < ldr_set_value) {
            digitalWrite(RELAY_LED2, LOW); 
            relay2_status = 1;
        } else if (light > (ldr_set_value + LDR_HYSTERESIS)) {
            digitalWrite(RELAY_LED2, HIGH);
            relay2_status = 0;
        }
    }
}

void displayTask(void *pvParameters) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    while (true) {
        display.clearDisplay();
        
        if (display_value == 1) {
            // Display 1: Temperature and Humidity
            display.setTextSize(1);
            display.setCursor(0, 0);
            
            if (in_setting_mode && sub_display) {
                // Setting mode for temperature
                display.println("** SETTING MODE **");
                display.println();
                display.print("Temp Set: ");
                display.print(temp_set_value);
                display.println(" C");
                display.println();
                display.println("White: Decrease");
                display.println("Blue:  Increase");
                display.println("Red:   Save & Exit");
            } else {
                // Normal display
                display.println("=== TEMP & HUMI ===");
                display.println();
                display.print("Temp: ");
                display.print(temperature, 1);
                display.println(" C");
                display.print("Humi: ");
                display.print(humidity, 1);
                display.println(" %");
                display.println();
                display.print("Set:  ");
                display.print(temp_set_value);
                display.println(" C");
            }

        } else if (display_value == 2) {
            // Display 2: Light Sensor (LDR)
            display.setTextSize(1);
            display.setCursor(0, 0);
            
            if (in_setting_mode && sub_display) {
                // Setting mode for LDR
                display.println("** SETTING MODE **");
                display.println();
                display.print("LDR Set: ");
                display.println(ldr_set_value);
                display.println();
                display.println("White: Decrease");
                display.println("Blue:  Increase");
                display.println("Red:   Save & Exit");
            } else {
                // Normal display
                display.println("===== LIGHT =====");
                display.println();
                display.print("LDR Val: ");
                display.println(light);
                display.println();
                display.print("Set Val: ");
                display.println(ldr_set_value);
                display.println();
                // Show light level description
                if (light < 40) {
                    display.println("Status: Dark");
                } else if (light < 800) {
                    display.println("Status: Dim");
                } else if (light < 2000) {
                    display.println("Status: Light");
                } else {
                    display.println("Status: Bright");
                }
            }

        } else if (display_value == 3) {
            // Display 3: Relay Status
            display.setTextSize(1);
            display.setCursor(0, 0);
            display.println("RELAY");
            display.println();
            
            display.print("Fan:  ");
            display.println(relay1_status ? "ON" : "OFF");
            display.println();
            
            display.print("Light:");
            display.println(relay2_status ? "ON" : "OFF");
        }
        
        display.display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);

    Wire.begin(I2C_SDA, I2C_SCL);

    pinMode(RED_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BLUE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(WHITE_BUTTON_PIN, INPUT_PULLUP);

    pinMode(RELAY_LED1, OUTPUT);
    pinMode(RELAY_LED2, OUTPUT);

    digitalWrite(RELAY_LED1, HIGH);
    digitalWrite(RELAY_LED2, HIGH);

    attachInterrupt(digitalPinToInterrupt(RED_BUTTON_PIN), handleRedButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(BLUE_BUTTON_PIN), handleBlueButtonInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(WHITE_BUTTON_PIN), handleWhiteButtonInterrupt, FALLING);

    xTaskCreatePinnedToCore(dhtTask, "DHT Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(ldrTask, "LDR Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(relayControlTask, "Relay Control", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, NULL, 0);
}

void loop() {
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("C | Humi: ");
    Serial.print(humidity);
    Serial.print("% | LDR: ");
    Serial.print(light);
    Serial.print(" | Fan: ");
    Serial.print(relay1_status ? "ON" : "OFF");
    Serial.print(" | Light: ");
    Serial.println(relay2_status ? "ON" : "OFF");
    
    vTaskDelay(pdMS_TO_TICKS(2000));
}