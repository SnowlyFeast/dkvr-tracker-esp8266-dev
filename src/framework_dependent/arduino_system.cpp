#include "common/system_interface.h"

#include <cstdint>

#include <Arduino.h>

uint32_t dkvr_approximate_time = 0;

uint32_t dkvr_get_time()
{
    dkvr_approximate_time = millis();
    return dkvr_approximate_time;
}

void dkvr_delay(uint32_t milli) 
{
    delay(milli);
}

void dkvr_serial_print(const char* msg)
{
    Serial.print(msg);
}

void dkvr_serial_print_float(float f)
{
    Serial.print(f);
    Serial.print(" ");
}