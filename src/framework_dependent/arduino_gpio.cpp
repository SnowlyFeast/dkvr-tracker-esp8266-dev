#include "common/gpio_interface.h"

#include <Arduino.h>

dkvr_err dkvr_gpio_init()
{
    return DKVR_OK;
}

void dkvr_gpio_write(uint8_t gpio_num, uint32_t level)
{
    digitalWrite(gpio_num, level);
}