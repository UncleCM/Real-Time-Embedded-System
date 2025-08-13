#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#define PIN_KEY 3
#define PIN_PULSE1 10
#define PIN_PULSE2 11
#define PRIORITY_GPIO 3
#define PRIORITY_TIMER 1

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    gpio_put(PIN_PULSE1, 1);
    busy_wait_us_32(3000000);
    gpio_put(PIN_PULSE1, 0);
    return 0;
}

void gpio_callback(uint gpio, uint32_t events) {
    gpio_put(PIN_PULSE2, 1);
    busy_wait_us_32(5000000);
    gpio_put(PIN_PULSE2, 0);
}

int main() {
    stdio_init_all();
    gpio_set_function(PIN_PULSE1, GPIO_FUNC_SIO);
    gpio_set_function(PIN_PULSE2, GPIO_FUNC_SIO);
    gpio_set_function(PIN_KEY, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_PULSE1, GPIO_OUT);
    gpio_set_dir(PIN_PULSE2, GPIO_OUT);
    gpio_set_dir(PIN_KEY, GPIO_IN);

    gpio_pull_up(PIN_KEY);

    gpio_put(PIN_PULSE1, 0);
    gpio_put(PIN_PULSE2, 0);
    sleep_ms(2500);

    printf("priority at default\n");
    printf("Priority Timer3=%d\n", irq_get_priority(TIMER_IRQ_3));
    printf("Priority GPIO1=%d\n", irq_get_priority(IO_IRQ_BANK0));
    sleep_ms(2500);

    irq_set_priority (TIMER_IRQ_3, PRIORITY_TIMER << 6); 
    irq_set_priority (IO_IRQ_BANK0, PRIORITY_GPIO << 6); 

    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(PIN_KEY, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(&gpio_callback);
    add_alarm_in_ms(5000, alarm_callback, NULL, false);

    while (true) {
        printf("Hello, world!\n");
        printf("priority at default\n");
        printf("Priority Timer3=%d\n", irq_get_priority(TIMER_IRQ_3));
        printf("Priority GPIO1=%d\n", irq_get_priority(IO_IRQ_BANK0));
        sleep_ms(1000);
    }
}