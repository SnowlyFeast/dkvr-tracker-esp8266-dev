#include "IMUFactory.h"

namespace DKVR
{
    namespace Driver
    {

        IMU* IMUFactory::CreateAutoConfiguredIMU()
        {
            Gyroscope *gyro = nullptr;
            Accelerometer *accel = nullptr;
            Magnetometer *mag = nullptr;
            Interruptible *interrupt = nullptr;


/* -------------------------------------------------------------------------- */
/*                                   MPU6050                                  */
/* -------------------------------------------------------------------------- */
#ifdef DKVR_IMU_BOARD_MPU6050
            MPU6050Configurations arg{
                arg.clksel = MPU_CLK_GYRO_Z_BIT,
                arg.smprtDiv = (1000 / DKVR_IMU_SAMPLING_RATE) - 1,
                arg.dlpf = MPU_DLPF_42_HZ_BIT,
                arg.gyroFsr = MPU_GYRO_FSR_2000_BIT,
                arg.accelFsr = MPU_ACCEL_FSR_2_BIT,
                arg.fifoEn = 0x0,
                arg.intConfig = 0x0,
                // arg.intConfig = MPU_LATCH_INT_EN_BIT,
                arg.intEnable = MPU_I2C_MST_INT_EN_BIT | MPU_DATA_RDY_EN_BIT};
            MPU6050 *mpu6050 = new MPU6050(false, arg);

            gyro = mpu6050;
            accel = mpu6050;
            interrupt = mpu6050;

            // external mag selection
#if defined DKVR_IMU_MAG_GY271HMC
            mag = new ExtGY271HMC(mpu6050->deviceAddress);
#elif defined DKVR_IMU_MAG_GY271QMC
            mag = new ExtGY271QMC(mpu6050->deviceAddress);
#endif  // MPU6050 mag selection

#endif  // MPU6050


            // end of setup
            return new IMU(gyro, accel, mag, interrupt);
        }

    } // namespace Driver
    
} // namespace DKVR
