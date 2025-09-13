#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- Pin Definitions ---
const int BUTTON_PIN = 27;  // GPIO pin for the button
const int I2C_SDA = 21;     // I2C SDA pin
const int I2C_SCL = 22;     // I2C SCL pin

// --- OLED Display Parameters ---
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset)
#define SCREEN_ADDRESS 0x3C ///< I2C address for the OLED

// --- Global Variables ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
volatile uint8_t display_value = 1;

// --- Interrupt Service Routine (ISR) for the button ---
// This function must be short, fast, and not use blocking operations.
// It is called directly by the hardware when the button state changes.
void IRAM_ATTR handleButtonInterrupt() {
    // Basic debouncing by checking if the button is still pressed
    // This is not a full debounce, but it's effective for simple cases.
    unsigned long current_time = millis();
    static unsigned long last_interrupt_time = 0;
    if (current_time - last_interrupt_time > 200) { // 200ms debounce time
        display_value++;
        if (display_value > 3) {
            display_value = 1;
        }
        last_interrupt_time = current_time;
    }
}

// --- Task Function: Updates the OLED Display ---
void displayTask(void *pvParameters) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) {
            // Lock up if display fails to initialize
        }
    }

    display.clearDisplay();
    display.setTextSize(4);
    display.setTextColor(SSD1306_WHITE);

    while (true) {
        // Clear and update the display with the current value
        display.clearDisplay();
        display.setCursor(35, 15);
        display.print(display_value);
        display.display();

        // This task runs periodically to update the screen.
        vTaskDelay(pdMS_TO_TICKS(100)); // Update every 100ms
    }
}

// --- Arduino Setup Function ---
void setup() {
    Serial.begin(115200);

    // Initialize I2C communication
    Wire.begin(I2C_SDA, I2C_SCL);

    // Configure the button pin with internal pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Attach the interrupt to the button pin
    // The ISR will be triggered on a falling edge (button press)
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);

    // Create the FreeRTOS display task
    // xTaskCreatePinnedToCore(TaskFunction, TaskName, StackSize, Parameters, Priority, TaskHandle, CoreID)
    xTaskCreatePinnedToCore(
        displayTask,         // Task function
        "Display Task",      // Name of the task
        4096,                // Stack size in words (larger for display)
        NULL,                // Task parameters
        1,                   // Task priority
        NULL,                // Task handle
        1);                  // Core to run on
}

// --- Arduino Loop Function (empty as tasks and ISR handle all logic) ---
void loop() {
    // This is an empty loop because all program logic is handled by FreeRTOS tasks and the ISR.
}
