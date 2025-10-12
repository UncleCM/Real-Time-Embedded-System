// Pull-Down resister with LDR
#define LIGHT_SENSOR_PIN 34 // ESP32 pin GIOP34 (ADC1)

void setup() {

 Serial.begin(115200);
 // set the ADC attenuation to 11 dB (up to ~3.3V input)
 analogSetAttenuation(ADC_11db); //0-3.3v or ADC_11db is a default, you may not have to setup using this line.

}

void loop() {
 int analogValue = analogRead(LIGHT_SENSOR_PIN);

 Serial.print("Analog Value = ");
 Serial.print(analogValue);


 if (analogValue < 40) {
 Serial.println(" => Dark");
 } else if (analogValue < 800) {
 Serial.println(" => Dim");
 } else if (analogValue < 2000) {
 Serial.println(" => Light");
 } else if (analogValue < 3200) {
 Serial.println(" => Bright");
 } else {
 Serial.println(" => Very bright");
 }

 vTaskDelay(1000);
}