// Define the digital pin the PIR sensor is connected to
const int pirPin = 13;

// Variable to store the state of the sensor
int pirState = 0;

void setup() {
  // Initialize the serial monitor for debugging
  Serial.begin(115200);

  // Set the PIR sensor pin as an input
  pinMode(pirPin, INPUT);
}

void loop() {
  // Read the state of the PIR sensor
  // The state will be HIGH if motion is detected, LOW otherwise
  pirState = digitalRead(pirPin);

  // Check if motion is detected
  if (pirState == HIGH) {
    // If motion is detected, print a message to the serial monitor
    Serial.println("Motion detected!");
  } else {
    // If no motion is detected, print a different message
    Serial.println("No motion.");
  }
  
  // Wait for a short period before checking again
  vTaskDelay(500 / portTICK_PERIOD_MS);
}
