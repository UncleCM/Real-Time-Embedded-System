#include <ESP32Servo.h>

Servo servo;

void setup() {
  Serial.begin(115200);
  
  // On ESP32, the servo library needs a PWM channel allocation
  servo.attach(13);  // GPIO 13 connected to servo signal
  Serial.println("ESP32 + MG995 Servo Test Start");
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos += 5) {
    servo.write(pos);
    delay(200);
  }

  // Sweep back
  for (int pos = 180; pos >= 0; pos -= 5) {
    servo.write(pos);
    delay(200);
  }
}
