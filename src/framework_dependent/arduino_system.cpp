#include "common/system_interface.h"

#include <cstdint>

#include <Arduino.h>

#include "common/dkvr_core.h"

#define SERIAL_SPEED    115200


uint32_t dkvr_approximate_time = 0;

dkvr_err dkvr_system_init()
{
#ifdef DKVR_SYSTEM_ENABLE_SERIAL
    Serial.begin(SERIAL_SPEED);
#endif
    return DKVR_OK;
}

uint32_t dkvr_get_time()
{
    dkvr_approximate_time = millis();
    return dkvr_approximate_time;
}

void dkvr_delay(uint32_t milli) 
{
    delay(milli);
}

void dkvr_serial_print_str(const char* msg)
{
#ifdef DKVR_SYSTEM_ENABLE_SERIAL
    Serial.print(msg);
#endif
}

void dkvr_serial_print_float(float f)
{
#ifdef DKVR_SYSTEM_ENABLE_SERIAL
    Serial.print(f);
#endif
}

void dkvr_serial_print_hex(unsigned char val)
{
#ifdef DKVR_SYSTEM_ENABLE_SERIAL
    Serial.printf("%02X", val);
#endif
}

void dkvr_serial_print_ln()
{
#ifdef DKVR_SYSTEM_ENABLE_SERIAL
    Serial.println();
#endif
}