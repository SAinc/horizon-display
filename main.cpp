#include <stdio.h>
#include <VescUartPico.h>
#include <pico/stdlib.h>

#include <hagl_hal.h>
#include <hagl.h>

int main() {
    stdio_init_all();

    display = hagl_init();

    hagl_clear(display);
}