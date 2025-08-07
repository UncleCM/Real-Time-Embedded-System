#include <stdio.h> 
#include "pico/stdlib.h" 
#include "hardware/pll.h" 
#include "hardware/clocks.h" 
#include "hardware/structs/pll.h" 
#include "hardware/structs/clocks.h" 
#include "hardware/structs/systick.h" 
#include <hardware/exception.h> 
int main() { 
uint32_t f_ref; 
stdio_init_all(); 
sleep_ms(3000); 
f_ref = clock_get_hz(clk_ref); 
printf("f_ref=%d\n", f_ref); 
while (1) { 
printf("current time (us) : %lld\n", time_us_64()); 
sleep_ms(1000); 
} 
return 0; 
} 