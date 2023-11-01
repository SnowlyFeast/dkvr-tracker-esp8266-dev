#ifndef DKVR_SERVICE_ORIENTATIONFIXER
#define DKVR_SERVICE_ORIENTATIONFIXER

#include "driver/DataType.h"
#include "configuration.h"

#define VEC_SIGN(X, Y) ((((X & AXIS_X) && (Y & AXIS_Z)) | ((X & AXIS_Y) && (Y & AXIS_X)) | ((X & AXIS_Z) && (Y & AXIS_Y))) ? 1 : 0)
#define VEC_CROSS(X, Y) ((~(X | Y) & 0b1110) | ((X ^ Y ^ VEC_SIGN(X, Y)) & 0b0001))

#define DKVR_IMU_GYRO_Z_HEADING     VEC_CROSS(DKVR_IMU_GYRO_X_HEADING,  DKVR_IMU_GYRO_Y_HEADING)
#define DKVR_IMU_ACCEL_Z_HEADING    VEC_CROSS(DKVR_IMU_ACCEL_X_HEADING, DKVR_IMU_ACCEL_Y_HEADING)
#define DKVR_IMU_MAG_Z_HEADING      VEC_CROSS(DKVR_IMU_MAG_X_HEADING,   DKVR_IMU_MAG_Y_HEADING)

namespace DKVR
{
    namespace Service
    {

        class OrientationFixer
        {
        public:
            static void FixGyro(DataType::Vector3f &vec);
            static void FixAccel(DataType::Vector3f &vec);
            static void FixMag(DataType::Vector3f &vec);
        };

    } // namespace Service

} // namespace DKVR

#endif