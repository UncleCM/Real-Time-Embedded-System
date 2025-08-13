#include <stdio.h> 
#include "pico/stdlib.h" 
#include "hardware/pwm.h" 
#include "hardware/clocks.h" 
#include "hardware/structs/pll.h" 
#include "hardware/structs/clocks.h" 
#define PIN_LED 10 // LED pin as PWM output  
#define F_PWM 5000.0f // Set PWM frequency (e.g., 10 kHz)
#define DUTY_CYCLE 0.45f // duty cycle, range from 0-1 as 0%-100% 
 
int main() 
{ 
    stdio_init_all(); 
    sleep_ms(3000); 
 
    // Get current PWM's clock source: clk_sys 
    // In RP2040, PWM use the different clock source from the Timer 
    uint32_t f_sys = clock_get_hz(clk_sys); 
     
    // Set GPIO at LED module as PWM function 
    gpio_set_function(PIN_LED, GPIO_FUNC_PWM); 
 
    // Get the PWM slice and channel for the chosen pin 
    uint slice_num = pwm_gpio_to_slice_num(PIN_LED); 
    uint channel_num = pwm_gpio_to_channel(PIN_LED); 
 
    // Calculate PWM TOP register to set the target PWM frequency 
    uint32_t top = (f_sys / F_PWM) - 1; 
    pwm_set_wrap(slice_num, top); // For a 125MHz clock, wrap = (125MHz / 10kHz) - 1 
    pwm_set_clkdiv(slice_num, 1.0f); // No clock divider, PWM use exact clk_ref as the its clock speed. 
 
    // Set duty cycle (e.g., 50%) 
    // By setting the threshold level, from 0 to TOP value 
    // Example, 6250 for 50% duty cycle at 10kHz 
    uint32_t threshold_level = top * DUTY_CYCLE; 
    pwm_set_chan_level(slice_num, channel_num, threshold_level); 
 
    // Enable the PWM channel 
    pwm_set_enabled(slice_num, true); 
 
    printf("f_sys=%d\n", f_sys); 
    printf("f_pwm=%f\n", F_PWM); 
    printf("TOP=%d\n", top); 
    printf("threshold_level=%d\n\n", threshold_level); 
     
    while (true) { 
        printf("f_sys=%d\n", f_sys); 
        printf("f_pwm=%f\n", F_PWM); 
        printf("TOP=%d\n", top); 
        printf("threshold_level=%d\n\n", threshold_level); 
        sleep_ms(5000); 
    } 
} 