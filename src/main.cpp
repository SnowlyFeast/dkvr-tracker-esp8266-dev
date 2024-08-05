#include <Arduino.h>
#include <Esp.h>

#include "common/dkvr_core.h"

#include "common/adc_interface.h"
#include "common/gpio_interface.h"
#include "common/i2c_interface.h"
#include "common/udp_interface.h"
#include "common/wifi_interface.h"

#include "tracker/tracker_main.h"

#define SERIAL_SPEED        115200


static void init_framework();


void IRAM_ATTR interrupt_callback()
{
    tracker_main_set_gpio_interrupted();
}

void setup()
{
#ifdef DKVR_DEBUG_ENABLE
    Serial.begin(SERIAL_SPEED);
#endif
    pinMode(DKVR_HARDWARE_LED_GPIO_NUM, OUTPUT);

    init_framework();
    tracker_main_init();

    if (tracker_main_init())
    {
        PRINTLN("Tracker initialization failed.");
    }
    else
    {
        attachInterrupt(digitalPinToInterrupt(DKVR_HARDWARE_INT_GPIO_NUM), interrupt_callback, RISING);
    }
}

void loop()
{
    tracker_main_update();
}


static void init_framework()
{
    dkvr_err result;
    int failed = 0;

    result = dkvr_adc_init();
    if (result != DKVR_OK)
    {
        PRINTLN("ADC init failed : 0x", (result));
        failed = 1;
    }

    result = dkvr_gpio_init();
    if (result != DKVR_OK)
    {
        PRINTLN("GPIO init failed : 0x", (result));
        failed = 1;
    }

    result = dkvr_i2c_init();
    if (result != DKVR_OK)
    {
        PRINTLN("I2C init failed : 0x", (result));
        failed = 1;
    }

    result = dkvr_wifi_init();
    if (result != DKVR_OK)
    {
        PRINTLN("WiFi init failed : 0x", (result));
        failed = 1;
    }

    result = dkvr_udp_init();
    if (result != DKVR_OK)
    {
        PRINTLN("UDP init failed : 0x", (result));
        failed = 1;
    }

    if (failed)
    {
        PRINTLN("Framework init failed. Aborting program...");
        delay(5000);
        ESP.reset();
    }
}