#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#define ESP8266_ADC_RESOLUTION  1024

#ifdef __cplusplus
extern "C" {
#endif

dkvr_err_t dkvr_adc_init();
uint16_t dkvr_adc_read();

#ifdef __cplusplus
}
#endif