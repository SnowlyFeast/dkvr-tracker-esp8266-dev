#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

dkvr_err_t dkvr_i2c_init();
uint8_t dkvr_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
uint8_t dkvr_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);

#ifdef __cplusplus
}
#endif