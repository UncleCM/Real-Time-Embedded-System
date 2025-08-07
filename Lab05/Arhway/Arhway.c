#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include <hardware/exception.h>
#include <stdint.h>
#include "hardware/structs/systick.h" 
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"

#define BIT_ENABLE 0
#define BIT_TICKINT 1
#define BIT_CLKSOURCE 2
#define GPIO_PIN_LED 10
#define GPIO_PIN_KEY 3

// Global variables - moved to top
volatile uint64_t curr_tick = 0;
uint64_t prev_tick_t1 = 0;
uint64_t prev_tick_t2 = 0;

// Task2 definitions
#define PERIOD_MS_TASK1 2000
#define PERIOD_MS_TASK2 5000  // Fixed typo: was PREIOD_MS_TASK2

// Task4 definitions
#define ALARM_DURATION_MS 5000
#define PERIOD_TASK1 500
#define PERIOD_TASK2 1500
volatile uint8_t task1_ready = 0;
volatile uint8_t task2_ready = 0;

// SysTick interrupt handler
void systick_handler(){
    curr_tick++;
}

// Software delay function using SysTick
void software_delay_us(uint32_t delay_us, uint32_t f_clk){
    int zero_flag = 0;
    uint32_t reload;

    // Fixed calculation - when CLKSOURCE bit is set, SysTick runs at processor speed
    reload = (uint32_t)(delay_us * (f_clk/1000000.0f));

    // Disable SysTick and clear interrupt flag
    systick_hw->csr &= ~((1 << BIT_ENABLE) | (1 << BIT_TICKINT));
    systick_hw->cvr = 0;
    systick_hw->rvr = reload;
    systick_hw->csr |= 1 << BIT_ENABLE;

    // Wait for COUNTFLAG to be set (bit 16)
    while(!zero_flag){
        zero_flag = systick_hw->csr & (1<<16);
    }
}

// Alarm callback function
int64_t alarm_callback(alarm_id_t id, void *user_data){
    printf("Alarm Occurred! Alarm %d fired!\n", (int)id);
    return 0;  // Don't repeat the alarm
}

// Timer callbacks
bool timer1_callback(struct repeating_timer *t){
    task1_ready = 1;
    return true;
}

bool timer2_callback(struct repeating_timer *t){
    task2_ready = 1;
    return true;    
}

// Task 1: LED blinking with software delay
void task1() {
    uint32_t f_sys = clock_get_hz(clk_sys);
    printf("System clock (f_sys) = %u Hz\n", f_sys);

    systick_hw->csr |= (1 << BIT_CLKSOURCE); 
    gpio_set_function(GPIO_PIN_LED, GPIO_FUNC_SIO);
    gpio_set_dir(GPIO_PIN_LED, GPIO_OUT);

    while (1) {
        gpio_put(GPIO_PIN_LED, 1);
        software_delay_us(1000000, f_sys);  // 5 seconds
        printf("System clock (f_sys) = %u Hz\n", f_sys); 
        gpio_put(GPIO_PIN_LED, 0);
        software_delay_us(200000, f_sys);  // 8 seconds
    }
}

// Task 2: Using SysTick for periodic tasks with WiFi LED toggle
void task2(){
    uint32_t f_sys;
    stdio_init_all();  // Fixed typo: was stdio_iit_all()
    f_sys = clock_get_hz(clk_sys);
    printf("f_sys=%d\n", f_sys);  // Fixed format string
    
    // Initialize CYW43 for WiFi LED control
    if (cyw43_arch_init() != 0) {
        printf("CYW43 initialization failed!\n");
        return;
    }
    
    gpio_set_function(GPIO_PIN_LED, GPIO_FUNC_SIO);
    gpio_set_dir(GPIO_PIN_LED, GPIO_OUT);
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, &systick_handler);

    // Configure SysTick to interrupt every 1ms for precise timing
    systick_hw->rvr = (uint32_t)(f_sys / 1000) - 1;  // 1ms period
    systick_hw->csr = (1 << BIT_ENABLE) | (1 << BIT_TICKINT) | (1 << BIT_CLKSOURCE);

    while(1){
        if(curr_tick - prev_tick_t1 >= PERIOD_MS_TASK1){
            printf("Run Task1\n");
            prev_tick_t1 = curr_tick;
            
            // Toggle onboard WiFi LED (Pico W)
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN)); 
        }
        if(curr_tick - prev_tick_t2 >= PERIOD_MS_TASK2){
            printf("Run Task2\n");
            prev_tick_t2 = curr_tick;
            
            // Toggle external LED on GPIO pin
            gpio_put(GPIO_PIN_LED, !gpio_get(GPIO_PIN_LED));
        }   
    }
}

// Task 3: Display current time
void task3(){
    uint32_t f_ref;
    stdio_init_all();
    sleep_ms(3000);
    f_ref = clock_get_hz(clk_ref);
    printf("f_ref=%d\n", f_ref);  // Fixed format string

    while(1){
        printf("current time(us): %lld\n", time_us_64());
        sleep_ms(1000);
    }
    // Removed incorrect return statement
}

// Task 4: Clock configuration and timers
void task4(){
    uint32_t f_ref;
    stdio_init_all();
    sleep_ms(3000);
    f_ref = clock_get_hz(clk_ref);
    printf("f_ref=%d\n", f_ref);

    clock_configure(clk_ref, 0, 0, 12*MHZ, 4*MHZ);

    stdio_init_all();
    sleep_ms(2000);
    f_ref = clock_get_hz(clk_ref);
    printf("f_ref=%d\n", f_ref);

    add_alarm_in_ms(ALARM_DURATION_MS, alarm_callback, NULL, true);

    struct repeating_timer timer1, timer2;
    add_repeating_timer_ms(PERIOD_TASK1, timer1_callback, NULL, &timer1);
    add_repeating_timer_ms(PERIOD_TASK2, timer2_callback, NULL, &timer2);

    while(1){
        if(task1_ready){
            printf("Run Task1\n");
            task1_ready = 0;
        }
        if(task2_ready){
            printf("Run Task2\n");
            task2_ready = 0;
        }
    }
}

// Task 5 helper functions
void task_blinking(){
    gpio_put(GPIO_PIN_LED, 1);
    sleep_ms(500);
    gpio_put(GPIO_PIN_LED, 0);
    sleep_ms(500);
}

void task_button(){
    while (gpio_get(GPIO_PIN_KEY));
}

// Task 5: Watchdog demonstration
void task5() {
    stdio_init_all();
    sleep_ms(1000);
    
    if(watchdog_caused_reboot()){
        printf("Rebooted by Watchdog!\n");
    }
    
    gpio_set_function(GPIO_PIN_LED, GPIO_FUNC_SIO);
    gpio_set_function(GPIO_PIN_KEY, GPIO_FUNC_SIO);
    gpio_set_dir(GPIO_PIN_LED, GPIO_OUT);
    gpio_set_dir(GPIO_PIN_KEY, GPIO_IN);
    gpio_pull_up(GPIO_PIN_KEY);

    watchdog_enable(5000, false);  // 5 second watchdog
    
    while (1) {
        task_blinking();
        task_button();
        watchdog_update();  // Pet the watchdog
    }
}

int main()
{   
    stdio_init_all();
    sleep_ms(8000); 
    
    // Uncomment one of the following tasks to run:
    // task1();    // LED blinking with software delay
    // task2();  // Periodic tasks with SysTick and WiFi LED
    // task3();  // Display current time
    // task4();  // Hardware timers and alarms
    task5();  // Watchdog demonstration with button
    
    return 0;
}