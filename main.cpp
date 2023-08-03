#include <stdio.h>
#include <pico/stdlib.h>
#include <cmath>

#include <hagl_hal.h>
#include <hagl.h>

#include "Background.h"
#include "VescUartPico.h"
#include "hagl/char.h"
#include "hagl_hal_color.h"

#include <mipi_display.h>
#include "font10x20-ISO8859-1.h"
#include "play_12px.h"
//#include "roboto_12.h"
#include "roboto_12.fnt.h"
#include "play_90px_numbers.h"

#define DEBUG true

#define PIN_TX                  28                  // UART tx pin
#define PIN_RX                  29                  // UART rx pin
#define UART_INST               uart0               // UART inst to use
#define TIMEOUT_MS              100                 // timeout for recv
#define BAUD                    115200              // baudrate for UART
#define NUM_CELLS               14                  // number of battery cells
#define V_MAX                   NUM_CELLS * 4.2     // max battery voltage
#define V_MIN                   NUM_CELLS * 3.3     // min battery voltage
#define WHEEL_DIA               200                 // wheel diameter in mm

static const double CONVERSION_FACTOR = M_PI * WHEEL_DIA * 0.0000372902;   // obtained by mm/min -> mi/hr

// init vesc controller object
static VescUartPico vesc = VescUartPico(TIMEOUT_MS, UART_INST, BAUD, PIN_TX, PIN_RX, DEBUG);

// Init display and background objects
static hagl_backend_t *display;

// init globals
float batteryPercentage;
float tripCounter;
float odometer;
float power;
float speed_mph;

// Define colors
// white
hagl_color_t white = 0xffff;

/**
 * @brief Redraw entire display screen
 * 
 * @param display hagl display surface to use
 *
 */
void drawDisplay(const hagl_backend_t* surface) {
    // draw background image
    mipi_display_write_xywh(0, 0, MIPI_DISPLAY_WIDTH, MIPI_DISPLAY_HEIGHT, (uint8_t*)BackgroundImage);
    // draw odometer (text)
    hagl_put_text(surface, L"ODO: 1000 mi", 5, 220, white, play_12px_fnt);
    hagl_put_text(surface, L"25", 60, 74, white, play_90px_numbers_fnt);
    // draw trip (text)
    // draw speedometer indicator
    // draw speedometer value (text)
    // draw power indicator
    // draw power value (text)
    // draw battery indicator
    // draw battery percent (text)
    // draw battery voltage (text)
    // draw temps (text)
}

/**
 * @brief Draws an indicator needle
 * 
 * @param 
 */
 void drawIndicator() {
    // position
    // value/angle
    // color
    // size
    // thickness
 }

/**
 * @brief Get and process telemetry data from the connected VESC
 * 
 * @return int return code
 */
 int getProcessTelemData() {
    // only recalculate if we have an update
    if (vesc.getVescTelemetry()){
        // calculate battery percent
        batteryPercentage = (vesc.data.inpVoltage - V_MIN) / (V_MAX - V_MIN) * 100;
        // calculate motor power
        power = vesc.data.inpVoltage * vesc.data.avgMotorCurrent;
        // calculate speedometer value
        speed_mph = vesc.data.rpm * CONVERSION_FACTOR;
        // TODO calculate and store odometer every 1mi
        // TODO calculate trip distance
        // do unit conversions?
        return 1;
    }
    return 0;
 }

/**
 * @brief Polls connected VESC for telemetry data and updates display
 * 
 * @return int return code
 */
int main() {
    // init display
    stdio_init_all();
    sleep_ms(5000);
    vesc.init();
    display = hagl_init();
    hagl_clear(display);
    drawDisplay(display);

    // continuously try to get VESC data and update display
    while (true) {
        //printf("Got data");
        if (getProcessTelemData()){    // get VESC data
            drawDisplay(display);     // only update display if data has changed
            printf("%f", vesc.data.inpVoltage);
        }   // TODO: timeout
        sleep_ms(50);
    }
}