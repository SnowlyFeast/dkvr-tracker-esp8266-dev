#pragma once 

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

// value from last dkvr_get_time() called
extern uint32_t dkvr_approximate_time;

dkvr_err dkvr_system_init();
uint32_t dkvr_get_time();
void dkvr_delay(uint32_t milli);
void dkvr_serial_print_str(const char* msg);
void dkvr_serial_print_float(float f);
void dkvr_serial_print_hex(unsigned char val);
void dkvr_serial_print_ln();

#ifdef __cplusplus
}
#endif