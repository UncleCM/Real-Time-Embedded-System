#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define PIN_LED 10      // LED pin
#define PIN_LDR 26      // LDR pin (ADC0)
#define ADC_CHANNEL 0   // GPIO26 corresponds to ADC channel 0
#define LIGHT_THRESHOLD 2000  // Threshold value (adjust based on your LDR)
                              // Lower values = darker conditions

int main() 
{
    stdio_init_all();
    sleep_ms(3000);

    // Initialize LED pin as output
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0); // Start with LED off

    // Initialize ADC
    adc_init();

    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(PIN_LDR);

    // Select ADC input channel 0 (GPIO26)
    adc_select_input(ADC_CHANNEL);

    printf("LDR Light Sensor Started\n");
    printf("Light threshold: %d\n", LIGHT_THRESHOLD);
    printf("Reading from GPIO%d (ADC channel %d)\n\n", PIN_LDR, ADC_CHANNEL);

    while (true) {
        // Read raw ADC value (0-4095 for 12-bit ADC)
        uint16_t adc_value = adc_read();

        // Convert to voltage (3.3V reference)
        float voltage = adc_value * 3.3f / 4095.0f;

        printf("ADC: %4d, Voltage: %.2fV", adc_value, voltage);

        // Check if it's dark (ADC value below threshold)
        if (adc_value < LIGHT_THRESHOLD) {
            gpio_put(PIN_LED, 0); // Turn LED ON (dark condition)
            printf(" - DARK: LED ON\n");
        } else {
            gpio_put(PIN_LED, 1); // Turn LED OFF (light condition)
            printf(" - LIGHT: LED OFF\n");
        }

        sleep_ms(500); // Read every 500ms
    }
}