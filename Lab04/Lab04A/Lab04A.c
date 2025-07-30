#include <stdio.h>
#include "pico/stdlib.h"

int button_pressed = 1;
void isr_gpio(uint gpio, uint32_t events){
        button_pressed = 0;
}
int main()
{
    stdio_init_all();

    gpio_init(10);
    gpio_init(3);
    gpio_set_dir(10, GPIO_OUT);
    gpio_set_dir(3, GPIO_IN);
    gpio_pull_up(3);

    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_set_irq_enabled(3, GPIO_IRQ_LEVEL_HIGH, true);
    gpio_set_irq_callback(&isr_gpio);

    while (true) {
        gpio_put(10, !gpio_get(10));
        sleep_ms(1000);
        printf("Loop\n");

        if (button_pressed == 0) {
            printf("button pressed\n");
            button_pressed = 1;
            sleep_ms(500);
        }
    }
}
