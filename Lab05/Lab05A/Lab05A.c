#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include <hardware/exception.h>
#include <stdint.h>
#include "hardware/structs/systick.h" // Add this header for systick_hw
#define BIT_ENABLE 0
#define BIT_TICKINT 1
#define BIT_CLKSOURCE 2
#define GPIO_PIN_LED 10

void software_delay_us(uint32_t delay_us, uint32_t f_clk){
    int zero_flag = 0;
    uint32_t reload;

    reload = (uint32_t)(delay_us * (f_clk/1000000.0f));

    /*
    
    
    */
   systick_hw -> csr &= ~((1 << BIT_ENABLE) | (1 << BIT_TICKINT));
   systick_hw -> cvr = 0;
   systick_hw -> rvr = reload;
   systick_hw -> csr |= 1 << BIT_ENABLE;

   while(!zero_flag){
        zero_flag = systick_hw -> csr & (1<<16);
   }

}

int main()
{   
    uint32_t f_sys;
    stdio_init_all();
    sleep_ms(3000);
    f_sys = clock_get_hz(clk_sys);
    systick_hw -> csr |= (1 << BIT_CLKSOURCE);

    gpio_set_function(GPIO_PIN_LED, GPIO_FUNC_SIO);
    gpio_set_dir(GPIO_PIN_LED, GPIO_OUT);

    while (1)
    {
    gpio_put(GPIO_PIN_LED, 1);
    software_delay_us(10000000,f_sys);
    gpio_put(GPIO_PIN_LED, 0);
    software_delay_us(2000000, f_sys);
    printf("System clock frequency: %d Hz\n", f_sys);
    }
    return 0;
}