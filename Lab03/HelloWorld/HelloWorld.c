#include <stdio.h>
#include "pico/stdlib.h"

// #include "pico/cyw43_arch.h"
void _func_asm();

int main()
{
    stdio_init_all();

   printf("Hello, World!\n");
    while(true){
        _func_asm();
    }
}
