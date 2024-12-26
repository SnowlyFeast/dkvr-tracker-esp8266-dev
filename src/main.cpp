#include <Arduino.h>
#include <Esp.h>

#include "common/adc_interface.h"
#include "common/gpio_interface.h"
#include "common/i2c_interface.h"
#include "common/system_interface.h"
#include "common/udp_interface.h"
#include "common/wifi_interface.h"

#include "tracker/tracker_main.h"


int check_init_result(dkvr_err result, const char* msg)
{
    if (result != DKVR_OK)
    {
        dkvr_serial_print_str(msg);
        dkvr_serial_print_str(PSTR(" : 0x"));
        dkvr_serial_print_hex(result);
        dkvr_serial_print_ln();
        return 1;
    }
    return 0;
}

bool init_framework()
{
    // not gonna happen on Arduino framework cuz it always returns DKVR_OK
    if (dkvr_system_init() != DKVR_OK)
        return 1;   // if happens, it's critical.

    int failed = 0;
    failed |= check_init_result(dkvr_adc_init(),  PSTR("ADC init failed"));
    failed |= check_init_result(dkvr_gpio_init(), PSTR("GPIO init failed"));
    failed |= check_init_result(dkvr_i2c_init(),  PSTR("I2C init failed"));
    failed |= check_init_result(dkvr_wifi_init(), PSTR("WiFi init failed"));
    failed |= check_init_result(dkvr_udp_init(),  PSTR("UDP init failed"));
    return failed;
}

void IRAM_ATTR interrupt_callback()
{
    tracker_main_set_gpio_interrupted();
}

/* -------------------------------------------------------------------------- */
/*                                 ENTRY POINT                                */
/* -------------------------------------------------------------------------- */
void setup()
{
    // dkvr base framework initialization
    if (init_framework())
    {
        dkvr_serial_print_str(PSTR("Framework init failed. Aborting program..."));
        delay(5000);
        ESP.reset();
    }
    
    // tracker main system initialization
    if (tracker_main_init())
    {
        dkvr_serial_print_str(PSTR("Tracker initialization failed."));
        // just keep going on but no tracker data is available as IRS is not attached
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
