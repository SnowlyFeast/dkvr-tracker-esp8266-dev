#include "common/adc_interface.h"

#include <Arduino.h>

dkvr_err_t dkvr_adc_init()
{
    return DKVR_OK;
}

uint16_t dkvr_adc_read()
{
    return static_cast<uint16_t>(analogRead(A0));
}