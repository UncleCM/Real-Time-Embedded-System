#include <stdio.h>
#include "pico/stdlib.h"


int main()
{
    stdio_init_all();

    gpio_init(3);
    gpio_init(10);
    gpio_set_dir(3, GPIO_IN);
    gpio_set_dir(10, GPIO_OUT);

    gpio_pull_up(3);
    while (true) {
        if (!gpio_get(3)) {
            gpio_put(10, !gpio_get(10));
            sleep_ms(500);
        }
    }

}
