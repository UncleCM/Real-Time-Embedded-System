#include <ESP32Servo.h>

// Define the servo motor's GPIO pin
const int servoPin = 18;

// Create a Servo object
Servo myservo;

// Variables for servo position
int pos = 0;

void setup() {
  // Attach the servo object to the pin
  // The min and max pulse widths can be adjusted for different servos
  myservo.attach(servoPin, 500, 2500); 
}

void loop() {
  // Sweep the servo from 0 to 180 degrees
  for (pos = 0; pos <= 180; pos += 1) { 
    myservo.write(pos);
   vTaskDelay(20 / portTICK_PERIOD_MS);              // use  dealy is fine, since we have only one supperloop
  }

  // Sweep the servo back from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) { 
    myservo.write(pos);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}