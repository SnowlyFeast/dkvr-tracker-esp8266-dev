#include "MPU6050.h"

namespace DKVR
{
    namespace Driver
    {

        MPU6050::MPU6050(bool a0, const MPU6050Configurations &arg)
            : deviceAddress(a0 ? MPU6050_DEVICE_ADDRESS_A0_HIGH : MPU6050_DEVICE_ADDRESS_A0_LOW), configuration(arg)
        {
        }

        uint8_t MPU6050::InitializeGyro()
        {
            // test communication
            uint8_t data = 0x0;
            if (I2C::WriteBytes(deviceAddress, MPU_REG_POWER_MGMT_1, 1, &data))
                return IMU_INIT_NACK_RECEIVED;

            // self-test
#ifdef DKVR_IMU_RUN_SELF_TEST
            uint8_t selftest = RunSelfTest();
            if (selftest)
            {
                PRINTLN("Self-test failed.");
                PRINTLN("(MSB) XG, YG, ZG, XA, YA, ZA (LSB)");
                PRINT("result : 0b");
                PRINTLN(String(selftest, BIN));
                return IMU_INIT_SELF_TEST_FAIL;
            }
            else
                PRINTLN("Self-test passed.");
#endif

            // reset
            data = MPU_DEVICE_RESET_BIT;
            uint8_t result = 0x0;
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_POWER_MGMT_1, 1, &data);
            delay(100);

            // setup
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_POWER_MGMT_1,    1, &(data = configuration.clksel));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_SMPRT_DIV,       1, &(data = configuration.smprtDiv));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_CONFIG,          1, &(data = configuration.dlpf));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_GYRO_CONFIG,     1, &(data = configuration.gyroFsr));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_ACCEL_CONFIG,    1, &(data = configuration.accelFsr));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_FIFO_EN,         1, &(data = configuration.fifoEn));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_INT_PIN_CFG,     1, &(data = configuration.intConfig));
            result |= I2C::WriteBytes(deviceAddress, MPU_REG_INT_ENABLE,      1, &(data = configuration.intEnable));

            if (result)
                return IMU_INIT_NACK_RECEIVED;

            // set sensitivity
            switch (configuration.gyroFsr)
            {
            default:
            case MPU_GYRO_FSR_250_BIT:
                gyroSensitivity = MPU_GYRO_FSR_250_RESOLUTION;
                break;
            case MPU_GYRO_FSR_500_BIT:
                gyroSensitivity = MPU_GYRO_FSR_500_RESOLUTION;
                break;
            case MPU_GYRO_FSR_1000_BIT:
                gyroSensitivity = MPU_GYRO_FSR_1000_RESOLUTION;
                break;
            case MPU_GYRO_FSR_2000_BIT:
                gyroSensitivity = MPU_GYRO_FSR_2000_RESOLUTION;
                break;
            }

            switch (configuration.accelFsr)
            {
            default:
            case MPU_ACCEL_FSR_2_BIT:
                accelSensitivity = MPU_ACCEL_FSR_2_RESOLUTION;
                break;
            case MPU_ACCEL_FSR_4_BIT:
                accelSensitivity = MPU_ACCEL_FSR_4_RESOLUTION;
                break;
            case MPU_ACCEL_FSR_8_BIT:
                accelSensitivity = MPU_ACCEL_FSR_8_RESOLUTION;
                break;
            case MPU_ACCEL_FSR_16_BIT:
                accelSensitivity = MPU_ACCEL_FSR_16_RESOLUTION;
                break;
            }
            
            return IMU_SUCCESS;
        }

        uint8_t MPU6050::InitializeAccel()
        {
            return IMU_SUCCESS;
        }

        uint8_t MPU6050::ReadRawGyro(DataType::Vector3s &gyro)
        {
            if (I2C::ReadBytes(deviceAddress, MPU_REG_GYRO_XOUT_H, 6, sharedBuffer))
                return IMU_GYRO_READ_FAIL;
            
            gyro.x = ((int16_t)sharedBuffer[0] << 8) | sharedBuffer[1];
            gyro.y = ((int16_t)sharedBuffer[2] << 8) | sharedBuffer[3];
            gyro.z = ((int16_t)sharedBuffer[4] << 8) | sharedBuffer[5];
            return IMU_SUCCESS;
        }

        uint8_t MPU6050::ReadRawAccel(DataType::Vector3s &accel)
        {
            if (I2C::ReadBytes(deviceAddress, MPU_REG_ACCEL_XOUT_H, 6, sharedBuffer))
                return IMU_ACCEL_READ_FAIL;

            accel.x = ((int16_t)sharedBuffer[0] << 8) | sharedBuffer[1];
            accel.y = ((int16_t)sharedBuffer[2] << 8) | sharedBuffer[3];
            accel.z = ((int16_t)sharedBuffer[4] << 8) | sharedBuffer[5];
            return IMU_SUCCESS;
        }

        uint8_t MPU6050::ReadInterrupt(uint8_t &intout)
        {
            intout = 0x0;
            return I2C::ReadBytes(deviceAddress, MPU_REG_INT_STATUS, 1, &intout) ? IMU_INT_READ_FAIL : IMU_SUCCESS;
        }

        void MPU6050::HandleInterrupt(uint8_t interrupt)
        {
            dataReady = false;
            if (interrupt & MPU_INT_DATA_RDY)
            {
                dataReady = true;
            }

            if (interrupt & MPU_INT_I2C_MST)
            {
                PRINTLN("I2C Master interrupt raised.");
            }

            // Not gonna happen
            // MPU_INT_DMP
            // MPU_INT_FIFO_OFLOW
        }

        uint8_t MPU6050::RunSelfTest()
        {
            // reset
            uint8_t data = MPU_DEVICE_RESET_BIT;
            I2C::WriteBytes(deviceAddress, MPU_REG_POWER_MGMT_1, 1, &data);
            delay(200);

            // setup
            I2C::WriteBytes(deviceAddress, MPU_REG_POWER_MGMT_1, 1, &(data = MPU_CLK_GYRO_Z_BIT));
            I2C::WriteBytes(deviceAddress, MPU_REG_SMPRT_DIV,    1, &(data = 9));
            I2C::WriteBytes(deviceAddress, MPU_REG_CONFIG,       1, &(data = MPU_DLPF_42_HZ_BIT));
            I2C::WriteBytes(deviceAddress, MPU_REG_GYRO_CONFIG,  1, &(data = MPU_GYRO_SELFTEST_BIT | MPU_GYRO_FSR_250_BIT));
            I2C::WriteBytes(deviceAddress, MPU_REG_ACCEL_CONFIG, 1, &(data = MPU_ACCEL_SELFTEST_BIT | MPU_ACCEL_FSR_8_BIT));
            delay(50);

            // get self-test enabled value
            DataType::Vector3s stGyro, stAccel, gyro, accel;
            ReadRawGyro(stGyro);
            ReadRawAccel(stAccel);

            // get self-test disabled value
            I2C::WriteBytes(deviceAddress, MPU_REG_GYRO_CONFIG, 1, &(data = MPU_GYRO_FSR_250_BIT));
            I2C::WriteBytes(deviceAddress, MPU_REG_ACCEL_CONFIG, 1, &(data = MPU_ACCEL_FSR_8_BIT));
            delay(50);
            ReadRawGyro(gyro);
            ReadRawAccel(accel);
            stGyro -= gyro;
            stAccel -= accel;

            // get Factory Trim
            uint8_t testGyro[3], testAccel[3];
            I2C::ReadBytes(deviceAddress, MPU_REG_SELF_TEST_X, 4, sharedBuffer);
            for (int i = 0; i < 3; i++)
            {
                testGyro[i] = sharedBuffer[i] & 0x1F;
                testAccel[i] = ((sharedBuffer[i] >> 3) & 0x1C) | ((sharedBuffer[3] >> (4 - i * 2)) & 0x03);
            }

            double ftGryo[3], ftAccel[3]; // FT
            for (int i = 0; i < 3; i++)
            {
                ftGryo[i] = testGyro[i] ? (25.0 * 131.0 * pow(1.046, testGyro[i])) * (i % 2 ? -1 : 1) : 0.0;
                ftAccel[i] = testAccel[i] ? (4096.0 * 0.34 * pow((0.92 / 0.34), ((testAccel[i] - 1.0) / 30.0))) : 0.0;
            }

            // change from FT of the STR
            uint8_t result = 0;
            for (int i = 0; i < 3; i++)
            {
                ftGryo[i] = fabs(stGyro[i] / ftGryo[i] - 1.0);
                if (ftGryo[i] > 0.14)
                    result |= (0x20 >> i);

                ftAccel[i] = fabs(stAccel[i] / ftAccel[i] - 1.0);
                if (ftAccel[i] > 0.14)
                    result |= (0x04 >> i);
            }

            return result;
        }

    } // namespace driver

} // namespace dkvr
