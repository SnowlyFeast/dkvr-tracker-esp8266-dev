#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#define DKVR_HIGH   1
#define DKVR_LOW    0

#ifdef __cplusplus
extern "C" {
#endif

dkvr_err_t dkvr_gpio_init();
void dkvr_gpio_write(uint8_t gpio_num, uint32_t level);

#ifdef __cplusplus
}
#endif