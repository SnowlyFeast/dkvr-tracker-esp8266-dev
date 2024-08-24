#include "common/gpio_interface.h"

#include <Arduino.h>

dkvr_err dkvr_gpio_init()
{
    return DKVR_OK;
}

void dkvr_gpio_mode(uint8_t gpio_num, uint8_t mode)
{
    if (mode == DKVR_GPIO_MODE_INPUT)
        pinMode(gpio_num, INPUT);
    else if (mode == DKVR_GPIO_MODE_OUTPUT)
        pinMode(gpio_num, OUTPUT);
}

void dkvr_gpio_write(uint8_t gpio_num, uint32_t level)
{
    digitalWrite(gpio_num, level);
}