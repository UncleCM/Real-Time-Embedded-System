#include <ESP32Servo.h>

Servo servo;

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 13

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);
  Serial.println("ESP32 Rotating Ultrasonic Radar Test");
}

void loop() {
  // Sweep left to right
  for (int angle = 0; angle <= 180; angle += 5) {
    servo.write(angle);
    delay(100);
    long distance = measureDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("°  Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Sweep right to left
  for (int angle = 180; angle >= 0; angle -= 5) {
    servo.write(angle);
    delay(100);
    long distance = measureDistance();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print("°  Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

// Measure distance using HC-SR04
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout after 30 ms
  long distance = duration * 0.034 / 2; // convert to cm
  return distance;
}
