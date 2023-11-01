#include "ExtGY271QMC.h"

namespace DKVR
{
    namespace Driver
    {
        ExtGY271QMC::ExtGY271QMC(const uint8_t parentAddress) : parentAddress(parentAddress){}

        uint8_t ExtGY271QMC::InitializeMag()
        {
            // enable bypass & disable I2C master
            uint8_t result = 0;
            result |= I2C::WriteBit(parentAddress, MPU_REG_USER_CTRL, 5, 0x0);
            result |= I2C::WriteBit(parentAddress, MPU_REG_INT_PIN_CFG, 1, 0x1);
            if (result)
                return IMU_INIT_NACK_RECEIVED;

            // GY271 soft reset
            uint8_t data = GY271_SOFT_RESET_BIT;
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_CONTROL_2, 1, &data);
            delay(50);

            // GY271 setup
            data = GY271_MODE_CONTINUOUS | GY271_ODR_100HZ | GY271_FSR_8G | GY271_OSR_256;
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_CONTROL_1, 1, &data);
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_CONTROL_2, 1, &(data = 0x00));
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_PERIOD_FBR, 1, &(data = GY271_PERIOD_FBR_VAL));
            if (result)
                return IMU_INIT_EXT_NACK_RECEIVED;

            // disable bypass & enable I2C master
            result |= I2C::WriteBit(parentAddress, MPU_REG_INT_PIN_CFG, 1, 0x0);
            data = MPU_WAIT_FOR_ES_BIT | MPU_I2C_MST_CLK_400_BIT;
            result |= I2C::WriteBytes(parentAddress, MPU_REG_I2C_MST_CTRL, 1, &data);
            result |= I2C::WriteBit(parentAddress, MPU_REG_USER_CTRL, 5, 0x1); // enable I2C_MST_EN

            // setup I2C read mode
            data = GY271_DEVICE_ADDRESS | MPU_I2C_SLV0_R_MODE_BIT;
            result |= I2C::WriteBytes(parentAddress, MPU_REG_I2C_SLV0_ADDR, 1, &data);
            data = GY271_REG_DATA_BEGIN;
            result |= I2C::WriteBytes(parentAddress, MPU_REG_I2C_SLV0_REG, 1, &data);
            data = MPU_I2C_SLV0_EN_BIT | GY271_DATA_SIZE;
            result |= I2C::WriteBytes(parentAddress, MPU_REG_I2C_SLV0_CTRL, 1, &data);

            magSensitivity = GY271_FSR_8_RESOLUTION;
            return result ? IMU_INIT_NACK_RECEIVED : IMU_SUCCESS;
        }

        uint8_t ExtGY271QMC::ReadRawMag(DataType::Vector3s &mag)
        {
            uint8_t buffer[GY271_DATA_SIZE];
            if (I2C::ReadBytes(parentAddress, MPU_REG_EXT_SENS_DATA, GY271_DATA_SIZE, buffer))
                return IMU_MAG_READ_FAIL;

            mag.x = ((int16_t)buffer[1] << 8) | buffer[0];
            mag.y = ((int16_t)buffer[3] << 8) | buffer[2];
            mag.z = ((int16_t)buffer[5] << 8) | buffer[4];
            return IMU_SUCCESS;
        }

    } // namespace driver

} // namespace dkvr
