#ifndef DKVR_DRIVER_IMUFACTORY
#define DKVR_DRIVER_IMUFACTORY

#include "Driver/IMU.h"

#ifdef DKVR_IMU_BOARD_MPU6050
#include "Driver/MPU/MPU6050.h"
#endif

#ifdef DKVR_IMU_MAG_GY271HMC
#include "Driver/MPU/ExtGY271HMC.h"
#elif defined DKVR_IMU_MAG_GY271QMC
#include "driver/mpu/ExtGY271QMC.h"
#endif

namespace DKVR
{
    namespace Driver
    {

        class IMUFactory
        {
        public:
            static IMU* CreateAutoConfiguredIMU();
        };

    } // namespace Driver
    
} // namespace DKVR


#endif // DKVR_DRIVER_IMUFACTORY
