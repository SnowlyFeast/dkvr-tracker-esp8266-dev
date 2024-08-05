#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

dkvr_err dkvr_adc_init();
uint16_t dkvr_adc_read();

#ifdef __cplusplus
}
#endif