#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 16
Adafruit_SSD1306 display(OLED_RESET);
void setup() {
display.begin(SSD1306_SWITCHCAPVCC, 0x3c); // Initializes the OLED screen at address 0x3C
display.clearDisplay(); // Clears everything from the screen
display.setTextSize(1); // Sets the text size to 1
display.setTextColor(WHITE); // Sets the text color to white
display.setCursor(0,0); // Sets the cursor position at x,y
display.println(" OLED 0.96 TESTER ");
display.setCursor(0,10);
display.setTextSize(2);
display.setTextColor(BLACK, WHITE); // Sets the text color to white on a black background
display.println(" Myarduino");
display.setCursor(0,32);
display.setTextSize(1);
display.setTextColor(WHITE);
display.println("128 x 64 Pixels 0.96");
display.setCursor(0,48);
display.setTextSize(1);
display.setTextColor(WHITE);
display.println(" www.myarduino.net "); // Display text www.Myarduino.net
display.display();
}
void loop() {
}