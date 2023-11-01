#include "OrientationFixer.h"

namespace DKVR
{
    namespace Service
    {
        void OrientationFixer::FixGyro(DataType::Vector3f &vec)
        {
            // gyro X
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_X)
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_NEGATIVE)
            float x = -vec.x;
#else
            float x = vec.x;
#endif
#elif (DKVR_IMU_GYRO_Y_HEADING & AXIS_X)
#if (DKVR_IMU_GYRO_Y_HEADING & AXIS_NEGATIVE)
            float x = -vec.y;
#else
            float x = vec.y;
#endif
#else
#if (DKVR_IMU_GYRO_Z_HEADING & AXIS_NEGATIVE)
            float x = -vec.z;
#else
            float x = vec.z;
#endif
#endif

            // gyro Y
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_Y)
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_NEGATIVE)
            float y = -vec.x;
#else
            float y = vec.x;
#endif
#elif (DKVR_IMU_GYRO_Y_HEADING & AXIS_Y)
#if (DKVR_IMU_GYRO_Y_HEADING & AXIS_NEGATIVE)
            float y = -vec.y;
#else
            float y = vec.y;
#endif
#else
#if (DKVR_IMU_GYRO_Z_HEADING & AXIS_NEGATIVE)
            float y = -vec.z;
#else
            float y = vec.z;
#endif
#endif

            // gyro Z
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_Z)
#if (DKVR_IMU_GYRO_X_HEADING & AXIS_NEGATIVE)
            float z = -vec.x;
#else
            float z = vec.x;
#endif
#elif (DKVR_IMU_GYRO_Y_HEADING & AXIS_Z)
#if (DKVR_IMU_GYRO_Y_HEADING & AXIS_NEGATIVE)
            float z = -vec.y;
#else
            float z = vec.y;
#endif
#else
#if (DKVR_IMU_GYRO_Z_HEADING & AXIS_NEGATIVE)
            float z = -vec.z;
#else
            float z = vec.z;
#endif
#endif
            // fix rotation
            vec = DataType::Vector3f(x, y, z);
        }

        void OrientationFixer::FixAccel(DataType::Vector3f &vec)
        {
            // accel X
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_X)
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_NEGATIVE)
            float x = -vec.x;
#else
            float x = vec.x;
#endif
#elif (DKVR_IMU_ACCEL_Y_HEADING & AXIS_X)
#if (DKVR_IMU_ACCEL_Y_HEADING & AXIS_NEGATIVE)
            float x = -vec.y;
#else
            float x = vec.y;
#endif
#else
#if (DKVR_IMU_ACCEL_Z_HEADING & AXIS_NEGATIVE)
            float x = -vec.z;
#else
            float x = vec.z;
#endif
#endif

            // accel Y
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_Y)
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_NEGATIVE)
            float y = -vec.x;
#else
            float y = vec.x;
#endif
#elif (DKVR_IMU_ACCEL_Y_HEADING & AXIS_Y)
#if (DKVR_IMU_ACCEL_Y_HEADING & AXIS_NEGATIVE)
            float y = -vec.y;
#else
            float y = vec.y;
#endif
#else
#if (DKVR_IMU_ACCEL_Z_HEADING & AXIS_NEGATIVE)
            float y = -vec.z;
#else
            float y = vec.z;
#endif
#endif

            // accel Z
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_Z)
#if (DKVR_IMU_ACCEL_X_HEADING & AXIS_NEGATIVE)
            float z = -vec.x;
#else
            float z = vec.x;
#endif
#elif (DKVR_IMU_ACCEL_Y_HEADING & AXIS_Z)
#if (DKVR_IMU_ACCEL_Y_HEADING & AXIS_NEGATIVE)
            float z = -vec.y;
#else
            float z = vec.y;
#endif
#else
#if (DKVR_IMU_ACCEL_Z_HEADING & AXIS_NEGATIVE)
            float z = -vec.z;
#else
            float z = vec.z;
#endif
#endif
            // fix rotation
            vec = DataType::Vector3f(x, y, z);
        }

        void OrientationFixer::FixMag(DataType::Vector3f &vec)
        {
            // mag X
#if (DKVR_IMU_MAG_X_HEADING & AXIS_X)
#if (DKVR_IMU_MAG_X_HEADING & AXIS_NEGATIVE)
            float x = -vec.x;
#else
            float x = vec.x;
#endif
#elif (DKVR_IMU_MAG_Y_HEADING & AXIS_X)
#if (DKVR_IMU_MAG_Y_HEADING & AXIS_NEGATIVE)
            float x = -vec.y;
#else
            float x = vec.y;
#endif
#else
#if (DKVR_IMU_MAG_Z_HEADING & AXIS_NEGATIVE)
            float x = -vec.z;
#else
            float x = vec.z;
#endif
#endif

            // mag Y
#if (DKVR_IMU_MAG_X_HEADING & AXIS_Y)
#if (DKVR_IMU_MAG_X_HEADING & AXIS_NEGATIVE)
            float y = -vec.x;
#else
            float y = vec.x;
#endif
#elif (DKVR_IMU_MAG_Y_HEADING & AXIS_Y)
#if (DKVR_IMU_MAG_Y_HEADING & AXIS_NEGATIVE)
            float y = -vec.y;
#else
            float y = vec.y;
#endif
#else
#if (DKVR_IMU_MAG_Z_HEADING & AXIS_NEGATIVE)
            float y = -vec.z;
#else
            float y = vec.z;
#endif
#endif

            // mag Z
#if (DKVR_IMU_MAG_X_HEADING & AXIS_Z)
#if (DKVR_IMU_MAG_X_HEADING & AXIS_NEGATIVE)
            float z = -vec.x;
#else
            float z = vec.x;
#endif
#elif (DKVR_IMU_MAG_Y_HEADING & AXIS_Z)
#if (DKVR_IMU_MAG_Y_HEADING & AXIS_NEGATIVE)
            float z = -vec.y;
#else
            float z = vec.y;
#endif
#else
#if (DKVR_IMU_MAG_Z_HEADING & AXIS_NEGATIVE)
            float z = -vec.z;
#else
            float z = vec.z;
#endif
#endif
            // fix rotation
            vec = DataType::Vector3f(x, y, z);
        }
    } // namespace Service

} // namespace DKVR
