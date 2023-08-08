#include <TFT_eSPI.h>
#include <SPI.h>
#include <VescUart.h>
#include <EEPROM.h>

// images and fonts
#include "Background.h"
#include "Play_Regular16.h"
#include "Play_Regular18.h"
#include "Play_Regular20.h"
#include "Play_Regular48.h"
#include "Play_Regular86.h"

//#define DEBUG
//#define TEST

// DEFINES
#define LOOP_PERIOD             35                  // loop period in ms
#define PIN_TX                  28                  // UART tx pin
#define PIN_RX                  29                  // UART rx pin
#define PIN_POWER               
#define TIMEOUT_MS              100                 // timeout for recv
#define BAUD                    115200              // baudrate for VESC UART
#define NUM_CELLS               14                  // number of battery cells
#define V_MAX                   NUM_CELLS * 4.2     // max battery voltage
#define V_MIN                   NUM_CELLS * 3.3     // min battery voltage
#define WHEEL_DIA               200                 // wheel diameter in mm
#define NUM_MAGNETS             30                  // number of magnets on the stator

// COLORS
#define COLOR_BACKGROUND        0x18c3      // 0x1a1a1a
#define COLOR_NEEDLE            0xfa80      // 0xff5000
#define COLOR_WARNING           0xfbc5      // 0xff7a2a
#define COLOR_TEXT_SECONDARY    0xcc79      // 0xcccccc

#define SPEEDOMETER_CENTER_X    105         // center of speedometer x coord
#define SPEEDOMETER_CENTER_Y    105         // center of speedometer y coord
#define POWER_CENTER_X          245         // center of power meter x coord
#define POWER_CENTER_Y          140         // center of power meter y coord
#define BATTERY_CENTER_X        198         // center of battery meter x coord
#define BATTERY_CENTER_Y        54          // center of battery meter y coord

#define EEPROM_ADDR             0           // address to store odometer value
#define NUM_SAMPLES_POWER       16          // number of samples to use for rolling average


// CONSTANTS
static const double CONVERSION_FACTOR_MPH = PI * WHEEL_DIA * 0.0000372902;   // obtained by mm/min -> mi/hr
static const double CONVERSION_FACTOR_MI = PI * WHEEL_DIA / 1.609e+6;

// Initialize TFT and sprites
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite speedometerNeedle = TFT_eSprite(&tft);
TFT_eSprite powerNeedle = TFT_eSprite(&tft);
TFT_eSprite batteryNeedle = TFT_eSprite(&tft);

// Initialize VESC connection
VescUart vesc;

// globl vars
uint32_t updateTime = 0;    // time for next update
float batteryPercentage = 0;
float tripCounter = 0;
float odometer = 772.5;
float power = 0;
float speed_mph = 0;
float vbat = 0;
float temp = 0;

float powerSamples[NUM_SAMPLES_POWER] = {0};
uint16_t index = 0;

// initialize as NAN so that we always draw values the first execution
float odometerPrev = NAN;
float tripCounterPrev = NAN;
float speedPrev = NAN;
float powerPrev = NAN;
float batteryPercentagePrev = NAN;
float vbatPrev = NAN;
float tempPrev = NAN;

// buffers to store image sections for erasing needles
uint16_t* speedometerBuffer;
uint16_t* powerBuffer;
uint16_t* batteryBuffer;

bool speedometerBufferLoaded = false;
bool powerBufferLoaded = false;
bool batteryBufferLoaded = false;

int16_t SpMin_x;
int16_t SpMin_y;
int16_t SpMax_x;
int16_t SpMax_y;

int16_t PwMin_x;
int16_t PwMin_y;
int16_t PwMax_x;
int16_t PwMax_y;

int16_t BtMin_x;
int16_t BtMin_y;
int16_t BtMax_x;
int16_t BtMax_y;


void setup(void) {
    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    Serial1.setRX(PIN_RX);
    Serial1.setTX(PIN_TX);
    Serial1.begin(BAUD);    // start uart0

    EEPROM.begin(256);  // start emulated eeprom with 256 byte length

    vesc.setSerialPort(&Serial1);    // set serial port to use

    #ifdef DEBUG
        vesc.setDebugPort(&Serial);
    #endif

    tft.begin();
    tft.setRotation(1);     // set orientation to landscape
    tft.fillScreen(TFT_BLACK);
    // draw ON screen
    tft.loadFont(Play_Regular20);
    tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
    tft.setCursor(150, 110);
    tft.printf("ON");
    tft.unloadFont();

    #ifndef TEST
        while (!vesc.getFWversion()) {
            #ifdef DEBUG
                Serial.println("Error: Could not connect to VESC!");
            #endif
        }
    #endif

    #ifdef DEBUG
        delay(5000);
    #endif

    // draw initial background image
    tft.pushImage(0, 0, BACKGROUND_WIDTH, BACKGROUND_HEIGHT, BackgroundImage);

    tft.setPivot(SPEEDOMETER_CENTER_X, SPEEDOMETER_CENTER_Y);   // set pivot for speedometer
    createSpeedometerNeedle(COLOR_NEEDLE);

    tft.setPivot(POWER_CENTER_X, POWER_CENTER_Y);   // set pivot for power meter
    createPowerNeedle(COLOR_NEEDLE);

    tft.setPivot(BATTERY_CENTER_X, BATTERY_CENTER_Y);   // set pivot for battery meter
    createBatteryNeedle(COLOR_NEEDLE);

    // restore odometer value
    //EEPROM.get(EEPROM_ADDR, odometer);
}

void loop(void) {
    // if (!digitalRead(PIN_POWER)) {
    //     shutdown();
    // }

    if (updateTime <= millis()) {
        updateTime = millis() + LOOP_PERIOD;

        #ifdef TEST
            // update values without needing VESC connection
            updateDisplay();

            if (++batteryPercentage > 100)
                batteryPercentage = 0;
            if (++tripCounter > 15)
                tripCounter = 0;
            if ((odometer += 10) > 1000)
                odometer = 0;
            if ((power += 10) > 2500)
                power = -1000;
            if (++speed_mph > 35)
                speed_mph = 0;
            if (++vbat > 58.8)
                vbat = 0;
            if (++temp > 45)
                temp = 12;
        #endif
        #ifndef TEST
            if (getProcessTelemData()) {
                updateDisplay();
            }
        #endif
    }
}

/**
 * @brief Get and process telemetry data from the connected VESC
 * 
 * @return int return code
 */
int getProcessTelemData() {
    // only recalculate if we have an update
    if (vesc.getVescValues()){
        // calculate battery percent
        batteryPercentage = (vesc.data.inpVoltage - V_MIN) / (V_MAX - V_MIN) * 100;
        vbat = vesc.data.inpVoltage;
        // calculate motor power
        float powerRaw = vesc.data.inpVoltage * vesc.data.avgMotorCurrent;

        // take running average of power to smooth it out
        powerSamples[index] = powerRaw;
        // wrap index at sample size
        if (++index <= NUM_SAMPLES_POWER) {
            index = 0;
        }
        power = 0;
        for (int i = 0; i < NUM_SAMPLES_POWER; i++) {
            power += powerSamples[i];
        }
        power /= NUM_SAMPLES_POWER;

        // calculate speedometer value
        speed_mph = vesc.data.rpm/(NUM_MAGNETS/2) * CONVERSION_FACTOR_MPH;
        // calculate trip distance
        tripCounter = vesc.data.tachometer/(3 * (NUM_MAGNETS/2)) * CONVERSION_FACTOR_MI;
        // calculate and store odometer every 0.1mi
        EEPROM.put(EEPROM_ADDR, odometer);
        temp = vesc.data.tempMosfet;
        return 1;
    }
    return 0;
}

void updateDisplay() {
    // check if odometer has changed
    if (!equalFloat(odometer, odometerPrev, 0.1)) {
        // draw odometer
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.loadFont(Play_Regular20);
        tft.setCursor(3, 222);
        tft.printf("ODO: %.1f mi      ", round(odometer, 1) );
        tft.unloadFont();
        // store current odometer value
        odometerPrev = odometer;
    }
    // check if trip counter has changed
    if (!equalFloat(tripCounter, tripCounterPrev, 0.1)) {
        // draw trip (text)
        odometer += tripCounter;
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.loadFont(Play_Regular20);
        tft.setCursor(196, 222);
        tft.printf("TRIP: %.1f mi    ", round(tripCounter, 1) );
        tft.unloadFont();
        // store current trip value
        tripCounterPrev = tripCounter;
    }
    // check if speedometer value has changed
    if (!equalFloat(speed_mph, speedPrev, 0.1)) {
        // draw speedometer indicator
        // draw speedometer value (text)
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.loadFont(Play_Regular86);
        tft.setCursor(55, 71);
        tft.printf("%2.0f ", round(std::abs(speed_mph), 0) );
        tft.unloadFont();
        tft.setPivot(SPEEDOMETER_CENTER_X, SPEEDOMETER_CENTER_Y);   // set pivot for speedometer
        drawSpeedometerNeedle(std::abs(speed_mph));
        // store current speed value
        speedPrev = speed_mph;
    }
    // check if power has changed
    if (!equalFloat(power, powerPrev, 10)) {
        // draw power indicator
        // draw power value (text)
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.loadFont(Play_Regular48);
        tft.setCursor(217, 121);
        tft.printf("%2.0f  ", round(std::abs(power)/100, 0) );
        tft.unloadFont();
        tft.setPivot(POWER_CENTER_X, POWER_CENTER_Y);   // set pivot for power meter
        drawPowerMeterNeedle(power/100);
        // store current power value
        powerPrev = power;
    }
    // check if battery has changed
    if (!equalFloat(batteryPercentage, batteryPercentagePrev, 1)) {
        // draw battery indicator
        // draw battery percent (text)
        tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
        tft.loadFont(Play_Regular18);
        tft.setCursor(190, 34);
        tft.printf("%3.0f   ", round(batteryPercentage, 0) );
        tft.unloadFont();
        tft.setPivot(BATTERY_CENTER_X, BATTERY_CENTER_Y);   // set pivot for battery meter
        drawBatteryMeterNeedle(batteryPercentage);
        // store current battery percentage
        batteryPercentagePrev = batteryPercentage;
    }
    // check if vbat has changed
    if (!equalFloat(vbat, vbatPrev, 0.1)) {
        // draw battery voltage (text)
        tft.setTextColor(TFT_WHITE, COLOR_BACKGROUND, true);
        tft.loadFont(Play_Regular16);
        tft.setCursor(248, 30);
        tft.printf("V: %3.1f V   ", round(vbat, 1) );
        tft.unloadFont();
        // store current vbat value
        vbatPrev = vbat;
    }
    // check if temp has changed
    if (!equalFloat(temp, tempPrev, 0.1)) {
        // draw temps (text)
        tft.setTextColor(TFT_WHITE, COLOR_BACKGROUND, true);
        tft.loadFont(Play_Regular16);
        tft.setCursor(265, 10);
        tft.printf("%3.1f C  ", round(temp, 1) );
        tft.unloadFont();
        // store current temp value
        tempPrev = temp;
    }
}

/**
 * @brief Draws an indicator needle for the speedometer
 * 
 * @param 
 */
void drawSpeedometerNeedle(float val) {
    // draw image where needle was
    // draw needle in new poition
    // save position as old position
    if (val < 0)
        val = 0;
    if (val > 35)
        val = 35;

    int angle = (45 + (val/35) * (220-45)) - 180;

    if (speedometerBufferLoaded) {
        tft.pushImage(SpMin_x, SpMin_y, 1 + SpMax_x - SpMin_x, 1 + SpMax_y - SpMin_y, speedometerBuffer);
    }

    if (speedometerNeedle.getRotatedBounds(angle, &SpMin_x, &SpMin_y, &SpMax_x, &SpMax_y)) {
        // get copy of image where needle will be
        getImageSelection(speedometerBuffer, SpMin_x, SpMin_y, 1 + SpMax_x - SpMin_x, 1 + SpMax_y - SpMin_y, 
                            BACKGROUND_WIDTH, BackgroundImage);
        speedometerBufferLoaded = true;
    }

    // draw rotated needle in new position
    speedometerNeedle.pushRotated(angle, TFT_WHITE);
}

/**
 * @brief Draws an indicator needle for the power meter
 * 
 * @param 
 */
void drawPowerMeterNeedle(float val) {
    // draw image where needle was
    // draw needle in new poition
    // save position as old position
    if (val < -10)
        val = -10;
    if (val > 25)
        val = 25;

    int angle;

    if (val >= 0)
        angle = (95 + (val/25) * (220-95)) - 180;
    else
        angle = (95 + (val/10) * (95-45)) - 180;

    if (powerBufferLoaded) {
        tft.pushImage(PwMin_x, PwMin_y, 1 + PwMax_x - PwMin_x, 1 + PwMax_y - PwMin_y, powerBuffer);
    }

    if (powerNeedle.getRotatedBounds(angle, &PwMin_x, &PwMin_y, &PwMax_x, &PwMax_y)) {
        // get copy of image where needle will be
        getImageSelection(powerBuffer, PwMin_x, PwMin_y, 1 + PwMax_x - PwMin_x, 1 + PwMax_y - PwMin_y, 
                            BACKGROUND_WIDTH, BackgroundImage);
        powerBufferLoaded = true;
    }

    // draw rotated needle in new position
    powerNeedle.pushRotated(angle, TFT_WHITE);
}

/**
 * @brief Draws an indicator needle for the battery meter
 * 
 * @param 
 */
void drawBatteryMeterNeedle(float val) {
    // draw image where needle was
    // draw needle in new poition
    // save position as old position
    if (val < 0)
        val = 0;
    if (val > 100)
        val = 100;

    int angle = (135 + (val/100) * (290-135)) - 180;

    if (batteryBufferLoaded) {
        tft.pushImage(BtMin_x, BtMin_y, 1 + BtMax_x - BtMin_x, 1 + BtMax_y - BtMin_y, batteryBuffer);
    }

    if (batteryNeedle.getRotatedBounds(angle, &BtMin_x, &BtMin_y, &BtMax_x, &BtMax_y)) {
        // get copy of image where needle will be
        getImageSelection(batteryBuffer, BtMin_x, BtMin_y, 1 + BtMax_x - BtMin_x, 1 + BtMax_y - BtMin_y, 
                            BACKGROUND_WIDTH, BackgroundImage);
        batteryBufferLoaded = true;
    }

    // draw rotated needle in new position
    batteryNeedle.pushRotated(angle, TFT_WHITE);
}

/**
 * @brief Create Speedometer Needle
 * 
 * @param color color of needle
 */
void createSpeedometerNeedle(uint16_t color) {
    speedometerNeedle.setColorDepth(16);
    speedometerNeedle.createSprite(5, 35);

    speedometerNeedle.fillSprite(TFT_WHITE);

    speedometerNeedle.setPivot(5/2, 90);    // set needle pivot point

    // draw needle image
    speedometerNeedle.drawWedgeLine(5/2, 0, 5/2, 35, 1, 5, color, TFT_BLACK);

    // bounding box
    int16_t min_x;
    int16_t min_y;
    int16_t max_x;
    int16_t max_y;

    // calculate max area that must be stored (occurs at 45deg)
    speedometerNeedle.getRotatedBounds(45, &min_x, &min_y, &max_x, &max_y);
    // allocate space for image
    speedometerBuffer = (uint16_t*)malloc( ((max_x - min_x) + 10) * ((max_y - min_y) + 10) * 2 );
}

 /**
 * @brief Create Power Meter Needle
 * 
 * @param color color of needle
 */
void createPowerNeedle(uint16_t color) {
    powerNeedle.setColorDepth(16);
    powerNeedle.createSprite(3, 30);

    powerNeedle.fillSprite(TFT_WHITE);

    powerNeedle.setPivot(3/2, 61);    // set needle pivot point

    // draw needle image
    powerNeedle.drawWedgeLine(3/2, 0, 3/2, 30, 1, 3, color, TFT_BLACK);

    // bounding box
    int16_t min_x;
    int16_t min_y;
    int16_t max_x;
    int16_t max_y;

    // calculate max area that must be stored (occurs at 45deg)
    powerNeedle.getRotatedBounds(45, &min_x, &min_y, &max_x, &max_y);
    // allocate space for image
    powerBuffer = (uint16_t*)malloc( ((max_x - min_x) + 10) * ((max_y - min_y) + 10) * 2 );
}

/**
 * @brief Create battery Meter Needle
 * 
 * @param color color of needle
 */
void createBatteryNeedle(uint16_t color) {
    batteryNeedle.setColorDepth(16);
    batteryNeedle.createSprite(3, 15);

    batteryNeedle.fillSprite(TFT_WHITE);

    batteryNeedle.setPivot(3/2, 40);    // set needle pivot point

    // draw needle image
    batteryNeedle.drawWedgeLine(3/2, 0, 3/2, 15, 1, 3, color, TFT_BLACK);

    // bounding box
    int16_t min_x;
    int16_t min_y;
    int16_t max_x;
    int16_t max_y;

    // calculate max area that must be stored (occurs at 45deg)
    batteryNeedle.getRotatedBounds(45, &min_x, &min_y, &max_x, &max_y);
    // allocate space for image
    batteryBuffer = (uint16_t*)malloc( ((max_x - min_x) + 10) * ((max_y - min_y) + 10) * 2 );
}

/**
 * @brief Compares two floats with tolerance
 * 
 * @param n1 
 * @param n2 
 * @param epsilon tolerance to check
 * @return true if floats are within tolerance specified by epsilon
 * @return false if not
 */
bool equalFloat(float n1, float n2, float epsilon) {
    return std::abs(n1-n2) < epsilon;
}

/**
 * @brief Rounds float to specified decimal places
 * 
 * @param val value to round
 * @param places number of places to round to
 * @return float rounded value
 */
float round(float val, int places) {
    float temp = (int)(val * pow(10, places) + 0.5);
    return (float)(temp / pow(10, places));
}

/**
 * @brief Gets a subselection of an image
 * 
 * @param dst destination image pointer
 * @param x0 starting x coord
 * @param y0 starting y coord
 * @param w_dst width of dst image
 * @param h_dst height of dst image
 * @param w_src width of src image
 * @param src source image pointer
 */
void getImageSelection(uint16_t* dst, int16_t x0, int16_t y0, int16_t w_dst, 
                    int16_t h_dst, int16_t w_src, const uint16_t* src) {
    const uint16_t* srcPtr = src + ( ((y0) * w_src) + x0 );    // start pointer at starting pixel coords
    uint16_t* dstPtr = dst;

    // loop through dst image size, reset ptr to start of next line after each iteration
    for (uint16_t i = 0; i < h_dst; i++) {
        for (uint16_t j = 0; j < w_dst; j++) {
            *(dst++) = *(srcPtr + j);     // copy pixel from src to dst
        }
        srcPtr += w_src;   // move ptr to next line
    }
}

/**
 * @brief called when power is lost
 * 
 */
void shutdown() {
    EEPROM.commit();
}