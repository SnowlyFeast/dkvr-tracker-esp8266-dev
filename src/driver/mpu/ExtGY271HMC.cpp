#include "ExtGY271HMC.h"

namespace DKVR
{
    namespace Driver
    {
        ExtGY271HMC::ExtGY271HMC(const uint8_t parentAddress) : parentAddress(parentAddress){}

        uint8_t ExtGY271HMC::InitializeMag()
        {
            // enable bypass & disable I2C master
            uint8_t result = 0;
            result |= I2C::WriteBit(parentAddress, MPU_REG_USER_CTRL, 5, 0x0);
            result |= I2C::WriteBit(parentAddress, MPU_REG_INT_PIN_CFG, 1, 0x1);
            if (result)
                return IMU_INIT_NACK_RECEIVED;

            // GY271 setup
            uint8_t data = GY271_MA_4 | GY271_DO_75HZ | GY271_MS_NORMAL;
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_CONF_A, 1, &data);

            data = GY271_GN_8P1G;
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_CONF_B, 1, &data);

            data = GY271_MD_CONTINUOUS;
            result |= I2C::WriteBytes(GY271_DEVICE_ADDRESS, GY271_REG_MODE, 1, &data);

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


            magSensitivity = GY271_GN_8P1G_RESOLUTION;
            return result ? IMU_INIT_NACK_RECEIVED : IMU_SUCCESS;
        }

        uint8_t ExtGY271HMC::ReadRawMag(DataType::Vector3s &mag)
        {
            uint8_t buffer[GY271_DATA_SIZE];
            if (I2C::ReadBytes(parentAddress, MPU_REG_EXT_SENS_DATA, GY271_DATA_SIZE, buffer))
                return IMU_MAG_READ_FAIL;

            mag.x = ((int16_t)buffer[0] << 8) | buffer[1];
            mag.y = ((int16_t)buffer[4] << 8) | buffer[5];
            mag.z = ((int16_t)buffer[2] << 8) | buffer[3];
            return IMU_SUCCESS;
        }

    } // namespace driver

} // namespace dkvr
