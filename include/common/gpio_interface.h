#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#define DKVR_HIGH   1
#define DKVR_LOW    0
#define DKVR_GPIO_MODE_INPUT    0x00
#define DKVR_GPIO_MODE_OUTPUT   0x01

#ifdef __cplusplus
extern "C" {
#endif

dkvr_err dkvr_gpio_init();
void dkvr_gpio_mode(uint8_t gpio_num, uint8_t mode);
void dkvr_gpio_write(uint8_t gpio_num, uint32_t level);

#ifdef __cplusplus
}
#endif