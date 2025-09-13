#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Pins
static const int led_pin = 2; //INTERNAL LED Pin
TaskHandle_t blinkTaskHandle = NULL;

// Our Task: blink an LED
void toggleLED(void *parameter){
  while(1){
    digitalWrite(led_pin, HIGH);
    vTaskDelay(500/ portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(500/ portTICK_PERIOD_MS);
  }

}

void testTouchPin(void *parameter) {
  while (1) {
    int touchValue = touchRead(T0);

    if (touchValue < 30) { 
      if (blinkTaskHandle != NULL) {
        vTaskSuspend(blinkTaskHandle);  
        digitalWrite(led_pin, LOW);   
      }
    } else {
      if (blinkTaskHandle != NULL) {
        vTaskResume(blinkTaskHandle);    
      }
    }

    vTaskDelay(200 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  // put your setup code here, to run once:
  // Config LED pin
  pinMode(led_pin, OUTPUT);

  // Task to run forever
  xTaskCreatePinnedToCore(  //Use xTaskCreate () in vanilla FreeRTOS
              toggleLED,    // Function to be Called
              "Toggle LED", // Name of Task
              1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass the function
              1,            // Task priority (o to config MAX_PRIORITIES(24) - 1)
              &blinkTaskHandle,         // Task handle
              app_cpu);     // Run on one core for deme purposes (ESP32 Only)

  xTaskCreatePinnedToCore(
              testTouchPin,
              "Test TouchPin",
              2048,
              NULL,
              1,
              NULL,
              app_cpu);
}
void loop() {
  // put your main code here, to run repeatedly:

}