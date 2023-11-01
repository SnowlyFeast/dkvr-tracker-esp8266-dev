#ifndef DKVR_DRIVER_I2C
#define DKVR_DRIVER_I2C

#include <Wire.h>
#include <Arduino.h>
#include "configuration.h"

namespace DKVR
{
    namespace Driver
    {

        class I2C
        {
        public:
            static uint8_t ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint32_t timeout = 0);
            static uint8_t WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t *data, uint32_t timeout = 0);
            static uint8_t WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t pos, uint8_t data, uint32_t timeout = 0);
        };

    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_I2C
