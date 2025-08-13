#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#define PIN_VOLADJ 27
#define ADC_IN_VOLADJ 1

int main() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(PIN_VOLADJ);
    adc_select_input(ADC_IN_VOLADJ);
    uint16_t adc_value;

    while (true) {
        adc_value = adc_read();
        printf("ADC val=%d\n", adc_value);
        sleep_ms(1000);
    }
}