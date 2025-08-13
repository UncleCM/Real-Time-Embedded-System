#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define PIN_LED 10          // GPIO15 - reliable PWM pin
#define PIN_VOL_ADJ 27      // GPIO27 - match your working ADC setup
#define ADC_CHANNEL 1       // GPIO27 = ADC channel 1

int main() 
{
    stdio_init_all();
    sleep_ms(2000);
    
    printf("=== SIMPLE LED BRIGHTNESS CONTROL ===\n");
    printf("LED: GPIO%d, Potentiometer: GPIO%d\n\n", PIN_LED, PIN_VOL_ADJ);
    
    // Initialize ADC (same as your working Part3_1)
    adc_init();
    adc_gpio_init(PIN_VOL_ADJ);
    adc_select_input(ADC_CHANNEL);
    printf("ADC initialized\n");
    
    // Test LED with simple on/off first
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    printf("Testing LED connection...\n");
    
    // Quick blink test
    for (int i = 0; i < 3; i++) {
        gpio_put(PIN_LED, 0);
        printf("LED ON\n");
        sleep_ms(500);
        gpio_put(PIN_LED, 1);
        printf("LED OFF\n");
        sleep_ms(500);
    }
    
    // Now setup PWM
    gpio_set_function(PIN_LED, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_LED);
    uint channel_num = pwm_gpio_to_channel(PIN_LED);
    
    printf("PWM: slice=%d, channel=%d\n", slice_num, channel_num);
    
    // Simple PWM setup
    pwm_set_wrap(slice_num, 1000);  // Simple wrap value
    pwm_set_clkdiv(slice_num, 125.0f);  // Slow down clock
    pwm_set_enabled(slice_num, true);
    
    printf("PWM enabled. Starting control loop...\n\n");
    
    while (true) {
        // Read potentiometer (same as your working code)
        uint16_t adc_value = adc_read();
        
        // Simple conversion: 0-4095 ADC -> 0-1000 PWM level
        uint16_t pwm_level = (adc_value * 1000) / 4095;
        
        // Set PWM
        pwm_set_chan_level(slice_num, channel_num, pwm_level);
        
        // Debug output
        float voltage = adc_value * 3.3f / 4095.0f;
        float percent = (pwm_level * 100.0f) / 1000.0f;
        
        printf("ADC=%4d (%.2fV) -> PWM=%4d (%.1f%%)\n", 
               adc_value, voltage, pwm_level, percent);
        
        sleep_ms(200);
    }
}