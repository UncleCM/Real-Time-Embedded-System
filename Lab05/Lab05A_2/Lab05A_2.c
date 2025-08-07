#include <stdio.h> 
#include "pico/stdlib.h" 
#include "hardware/pll.h" 
#include "hardware/clocks.h" 
#include "hardware/structs/pll.h" 
#include "hardware/structs/clocks.h" 
#include "hardware/structs/systick.h" 
#include <hardware/exception.h>
#include "hardware/gpio.h" 
#include "pico/cyw43_arch.h"
#define BIT_ENABLE 0 
#define BIT_TICKINT 1 
#define BIT_CLKSOURCE 2 
#define GPIO_PIN_RED_LED 10
#define PICO_DEFAULT_LED_PIN 25
#define PERIOD_MS_TASK1 2000 
#define PERIOD_MS_TASK2 5000 
volatile uint64_t curr_tick=0; 
uint64_t prev_tick_t1=0; 
uint64_t prev_tick_t2=0;
bool green_led_state = false;
bool red_led_state = false; 
// SysTick IRQ Handler 
void systick_handler() { 
curr_tick++; 
} 
int main() { 
uint32_t f_sys; 
// system initialise 
stdio_init_all(); 
f_sys = clock_get_hz(clk_sys); 
sleep_ms(5000); 
printf("f_sys=%d\n", f_sys);

// Initialize GPIO pins for LEDs
gpio_init(PICO_DEFAULT_LED_PIN);
gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
gpio_put(PICO_DEFAULT_LED_PIN, 0); // Start with LED off

gpio_init(GPIO_PIN_RED_LED);
gpio_set_dir(GPIO_PIN_RED_LED, GPIO_OUT);
gpio_put(GPIO_PIN_RED_LED, 0); // Start with LED off

// SysTick initialize 
exception_set_exclusive_handler (SYSTICK_EXCEPTION, &systick_handler); 
// Set SysTick to use clk_sys clock, enable interrupt and start counting 
systick_hw->rvr = (uint32_t)(f_sys / 1000 - 1); 
systick_hw->csr |= (1 << BIT_ENABLE) | (1 << BIT_TICKINT) | (1 << 
BIT_CLKSOURCE); 
while (1) { 
     if (curr_tick - prev_tick_t1 >= PERIOD_MS_TASK1) { 
         // run task1 - blink green LED (Pico's onboard LED)
         green_led_state = !green_led_state;
         gpio_put(PICO_DEFAULT_LED_PIN, green_led_state);
         printf("Task 1: Green LED %s (Period: %dms)\n", green_led_state ? "ON" : "OFF", PERIOD_MS_TASK1); 
          
         // update tick for task1 
         prev_tick_t1 = curr_tick; 
     } 
 
     if (curr_tick - prev_tick_t2 >= PERIOD_MS_TASK2) { 
         // run task2 - blink red LED
         red_led_state = !red_led_state;
         gpio_put(GPIO_PIN_RED_LED, red_led_state);
         printf("Task 2: Red LED %s (Period: %dms)\n", red_led_state ? "ON" : "OFF", PERIOD_MS_TASK2); 
          
         // update tick for task2 
         prev_tick_t2 = curr_tick; 
     } 
 } 
 return 0; 
}