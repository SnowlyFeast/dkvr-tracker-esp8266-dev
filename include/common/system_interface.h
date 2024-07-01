#pragma once 

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// value from last dkvr_get_time() called
extern uint32_t dkvr_approximate_time;

uint32_t dkvr_get_time();
void dkvr_delay(uint32_t milli);
void dkvr_serial_print(const char* msg);
void dkvr_serial_print_float(float f); // TODO: test tool

#ifdef __cplusplus
}
#endif